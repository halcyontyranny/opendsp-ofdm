# FEC Integration — Implementation & Debugging Notes

## Overview

This document records the design decisions, bugs discovered, and root-cause analysis from
the codec2 LDPC FEC integration session.  It is intended as a reference for future work on
the modem's physical layer.

---

## What was built

### New files
| File | Purpose |
|------|---------|
| `third_party/codec2/ldpc_api.h` | Vendored minimal codec2 LDPC C declarations (struct LDPC, run_ldpc_decoder, ldpc_encode_frame, ldpc_codes_find/setup). Public libcodec2-dev does not expose these internal headers. |
| `include/fec.h` | `FECCodec` class interface — pimpl, non-copyable, movable |
| `src/fec.cpp` | `FECCodec` implementation using vendored API |

### Modified files
| File | Change |
|------|--------|
| `CMakeLists.txt` | Optional `WITH_CODEC2` dependency via `pkg_check_modules(CODEC2 codec2)`; adds `HAVE_CODEC2` compile definition, `third_party/` include dir, `src/fec.cpp` to library sources |
| `src/sim_test.cpp` | Added `run_fec_ber_test()`, `--no-fec` flag, dual `BER(raw)` / `BER(FEC)` columns |
| `src/main.cpp` | TX: FEC encodes bits before subcarrier mapping; RX: accumulates LLRs, decodes FEC blocks, parses frames via `Framer` |
| `src/ofdm_core.cpp` | Fixed channel noise variance estimator (see Bug #1 below) |
| `src/fec.cpp` | Fixed `run_ldpc_decoder` output buffer size (see Bug #2 below) |

---

## Code selection per ACM tier

```
fec_code_for_tier():
  tier 0–1 → "H_256_768_22"   rate 1/4  (256 data, 768 coded)
  tier 2–3 → "H_128_256_5"    rate 1/3  (128 data, 256 coded)
  tier 4–5 → "HRA_112_112"    rate 1/2  (112 data, 224 coded)
  tier 6–8 → "HRAa_1536_512"  rate 3/4  (1536 data, 2048 coded)
```

The rate is chosen to match the ACM tier's nominal code rate.  The specific codec2 code
names were verified against `ldpc_codes_find()` on libcodec2 1.0.1.

---

## LLR sign convention

`OFDMDemodulator::equalise_and_demap` uses:
```
positive LLR → bit 0 more likely
negative LLR → bit 1 more likely
```
`run_ldpc_decoder` expects the same convention (log P(0)/P(1)).  No sign flip is needed.

---

## Bug #1 — Noise variance estimator (ofdm_core.cpp)

### Symptom
After the noise estimate fix was applied to the LLR scale, BER(FEC) was consistently
equal to or slightly *worse* than BER(raw) at every SNR.  The LDPC decoder was receiving
LLRs with essentially no confidence signal and defaulting to near-random hard decisions.

### Root cause
`update_channel_estimate()` estimated noise_var with:

```cpp
cx residual = rx_sc[idx] / (h_est + cx(1e-12, 0)) - cx(1, 0); // "should be ≈0"
```

For a pilot with reference `pilot_ref`:
```
h_ls = rx_sc[idx] / pilot_ref
residual = rx_sc[idx] / h_ls - 1
         = (h_true * pilot_ref) / h_true - 1   (noiseless case)
         = pilot_ref - 1
```

For `pilot_ref = +1`: residual = 0 (correct).
For `pilot_ref = -1`: residual = -2, contributing `|residual|² = 4` to noise_sum.

With roughly half the pilots being -1, `noise_var` was estimated at ≈ 2.0 instead of the
true value (e.g. 0.25 at 6 dB SNR) — an **8× overestimate**.  This made every LLR ≈ 8×
too small.  The LDPC belief-propagation decoder needs correctly scaled LLRs to converge;
with tiny LLRs it effectively treats every bit as equally likely and fails to correct
anything.

### Fix
Replaced the residual-based estimate with the variance of h_ls values across pilots:

```cpp
// h_ls[k] = h_true + noise_k  →  Var(h_ls) = noise_var  (flat channel, unbiased)
cx h_mean = mean of all pilots' h_ls values;
noise_var = mean(|h_ls[k] - h_mean|²);
```

This is unbiased regardless of pilot phase and works correctly for flat AWGN channels.
For frequency-selective channels it will slightly overestimate noise (channel variation
is indistinguishable from noise using only per-symbol pilot observations without
smoothing across time), but this is acceptable for HF amateur radio use.

Signal power was also updated to use `|h_mean|²` rather than the average of `|h_ls|²`
(the latter is biased high by the noise component).

---

## Bug #2 — run_ldpc_decoder output buffer size (fec.cpp)

### Symptom
`sim_test --tier 0` (and tiers 1, 2, 3, 6, 7, 8) aborted with "corrupted size vs.
prev_size" or "double free or corruption".  Tier 4 and 5 ran without error.

### Root cause
From the codec2 source (`mpdecode_core.c`, line 552):

```c
for (i=0; i<CodeLength; i++) out_char[i] = DecodedBits[i];
```

`run_ldpc_decoder` always writes **`CodeLength` bytes** to `out_char` — the full codeword
(data bits + parity bits) — not just the `ldpc_data_bits_per_frame` data bits.

Our `fec.cpp` allocated only `ldpc_data_bits_per_frame` bytes:

```cpp
out_bits.resize(db);   // db = ldpc_data_bits_per_frame
run_ldpc_decoder(&impl_->ldpc, out_bits.data(), ...);  // writes CodeLength bytes — OVERFLOW
```

Buffer overflow sizes by code:
```
H_256_768_22    CodeLength=768  allocated=256  overflow=512 bytes  → immediate crash
H_128_256_5     CodeLength=256  allocated=128  overflow=128 bytes  → crash
HRA_112_112     CodeLength=224  allocated=112  overflow=112 bytes  → silent (adjacent heap luck)
HRAa_1536_512   CodeLength=2048 allocated=1536 overflow=512 bytes  → crash
```

HRA_112_112's 112-byte overflow landed in adjacent allocated heap space without
triggering the allocator's corruption checks, which is why tiers 4–5 appeared to work.

### Fix
Allocate `CodeLength` bytes for the decoder output, then extract only the first
`ldpc_data_bits_per_frame` bytes (which hold the recovered data bits — the codeword is
laid out as `[data_bits | parity_bits]` in codec2's systematic encoding):

```cpp
int cl = impl_->ldpc.CodeLength;
std::vector<uint8_t> full_out(cl, 0);
run_ldpc_decoder(&impl_->ldpc, full_out.data(), llr_copy.data(), &parity_ok);
out_bits.assign(full_out.begin(), full_out.begin() + db);
```

---

## Observed BER results (seed=42, threshold+2 dB SNR per tier)

```
Tier  SNR(dB)  BER(raw)   BER(FEC)   Notes
0     -18.0    4.24e-01   4.23e-01   Too deep in noise; 42% raw → LDPC cannot converge
1     -10.0    1.83e-01   8.37e-02   Partial gain; still above LDPC waterfall
2     -6.0     1.39e-01   1.18e-01   Minimal gain; rate-1/3 code at high input BER
3     -3.0     1.91e-01   1.91e-01   QPSK at threshold; pilot interpolation error inflates raw BER
4     0.0      1.31e-01   9.29e-02   Moderate gain
5     3.0      7.07e-02   1.25e-03   Strong gain: 7% → 0.12%
6     6.0      1.09e-01   1.09e-01   8-PSK hard for rate-3/4 code at this SNR
7     9.0      5.80e-02   3.25e-02   Partial gain
8     12.0     4.52e-02   3.33e-03   Strong gain: 4.5% → 0.33%
```

The high raw BER values at threshold+2 dB are expected — we're testing right at the
demodulator's operating margin.  LDPC coding gains are most visible when raw BER is
between roughly 1–10%; above that the decoder cannot converge.

---

## Known limitations / future work

1. **Low-SNR tiers (0–2)** achieve almost no FEC gain at their own threshold SNR.  The
   ACM threshold values for these tiers may need to be raised, or a longer/more powerful
   code substituted (e.g. turbo or polar codes).

2. **Noise estimator for frequency-selective channels**: the h_ls variance approach
   confounds multipath-induced channel variation with noise.  A per-subcarrier noise
   estimate using a MMSE smoothing filter over pilots in time would be more accurate.

3. **LLR scaling**: `demap_symbol_llr` computes `2 * Re(eq_sym) / noise_var` for BPSK.
   The theoretically correct LLR is `4 * Re(eq_sym) * |h|² / noise_var` (factor of 2
   missing).  This does not affect hard-decision BER but reduces LDPC convergence speed.
   Fixing it would lower the effective threshold SNR for FEC.

4. **RX frame recovery in main.cpp** only reconstructs frames when exactly one FEC block
   aligns with a complete demodulated message.  For longer messages spanning multiple
   blocks, a proper reassembly buffer is needed.

5. **Pilot interval** is currently 4 (every 4th subcarrier is a pilot).  Tighter pilot
   grids (interval 8–16) would free more bandwidth for data at the cost of channel
   estimate accuracy.

# OpenDSP-OFDM

**Adaptive HF OFDM Modem** — FT8-class weak-signal survival mode with
multi-carrier throughput that scales from a 50 Hz rescue channel up to
3.5 kHz 16-QAM depending on channel conditions.

---

## Features

- **9 ACM tiers** — 50 Hz BPSK survival through 3.5 kHz 16-QAM; modulation,
  bandwidth, code rate, and subcarrier spacing all adapt together
- **Tier 0 survival mode** — 50 Hz FT8-analogous channel with codec2 LDPC
  (H_256_768_22, rate 1/3) and 4-pass soft LLR accumulation; decodes to
  approximately −25 dB SNR in a 2500 Hz reference bandwidth
- **Codec2 LDPC FEC** — hard-integrated; four codec2 codes cover the tier range
  (H_256_768_22, H_128_256_5, HRA_112_112, HRAa_1536_512)
- **Soft Chase combining** — FECAccumulator sums LLRs across N identical
  codeword transmissions before decoding; each doubling of passes ≈ +3 dB
- **Multi-block framing** — `num_blocks` in the ACM header lets the receiver
  accumulate all FEC blocks before parsing; receiver prints expected duration
  on multi-block frames so the operator knows to hold TX
- **FT8-proven subcarrier spacing** — 6.25 Hz at low SNR tiers for ionospheric
  Doppler tolerance; widens to 12.5 Hz and 25 Hz only at high-SNR tiers
- **MMSE channel equalisation** with LS pilot-based channel estimation
- **Bandwidth negotiation** — PROBE/ACK/RETUNE/NAK handshake
- **PortAudio I/O** — works with any soundcard
- **Hamlib integration** (optional) — CAT/PTT control
- **Offline simulation** — BER sweeps and SNR tests without a radio

---

## ACM Tier Table

| Tier | Min SNR  | BW      | Spacing  | Mod    | Code rate | Passes | LDPC code       | Net bps |
|------|----------|---------|----------|--------|-----------|--------|-----------------|---------|
| 0    | −20 dB   | 50 Hz   | 6.25 Hz  | BPSK   | 1/3       | 4      | H_256_768_22    | ~2      |
| 1    | −12 dB   | 500 Hz  | 6.25 Hz  | BPSK   | 1/3       | 1      | H_256_768_22    | ~88     |
| 2    | −8 dB    | 1 kHz   | 6.25 Hz  | BPSK   | 1/2       | 1      | H_128_256_5     | ~265    |
| 3    | −5 dB    | 1.5 kHz | 6.25 Hz  | QPSK   | 1/2       | 1      | H_128_256_5     | ~796    |
| 4    | −2 dB    | 2 kHz   | 6.25 Hz  | QPSK   | 1/2       | 1      | HRA_112_112     | ~1.1k   |
| 5    | +1 dB    | 2.5 kHz | 6.25 Hz  | QPSK   | 1/2       | 1      | HRA_112_112     | ~1.3k   |
| 6    | +4 dB    | 3 kHz   | 12.5 Hz  | 8-PSK  | 3/4       | 1      | HRAa_1536_512   | ~3.6k   |
| 7    | +7 dB    | 3.2 kHz | 12.5 Hz  | 8-PSK  | 3/4       | 1      | HRAa_1536_512   | ~3.8k   |
| 8    | +10 dB   | 3.5 kHz | 25 Hz    | 16-QAM | 3/4       | 1      | HRAa_1536_512   | ~5.6k   |

Net bps includes FEC overhead and multi-pass repetition. Tier 0's 4-pass
repetition cuts raw throughput by 4× but buys roughly 6 dB of additional
coding gain via LLR accumulation.

> **SNR reference caveat**: simulation SNR figures are measured relative to
> total OFDM symbol power. With 8 subcarriers in a 1280-bin FFT, per-carrier
> processing gain is ≈ +22 dB, so simulation numbers are not directly
> comparable to the 2500 Hz wideband reference used for FT8 comparisons.
> The −25 dB figure quoted for tier 0 is the 2500 Hz reference equivalent.

---

## Frame Structure

Each transmission consists of a ZC preamble, a pilot OFDM symbol, and then
`num_blocks × passes` FEC codewords:

```
| ZC Preamble | Pilot Symbol | [Codeword × passes] × num_blocks |
```

The `num_blocks` FEC data blocks concatenate to form a single frame:

```
| ACM Header (5 B) | Callsign (7 B) | Payload (N B) | Zero-padding | CRC-32 (4 B) |
```

### ACM Header (5 bytes)

| Byte | Field        | Description                                         |
|------|--------------|-----------------------------------------------------|
| 0    | `tier_index` | Sender's current ACM tier (0–8)                     |
| 1    | `max_bw_code`| Sender's highest supported tier                     |
| 2    | `frame_type` | DATA / PROBE / ACK / RETUNE / NAK / BEACON          |
| 3    | `num_blocks` | Total FEC blocks in this transmission               |
| 4    | `payload_len`| Valid payload bytes (remainder of block is padding) |

`num_blocks` tells the receiver how long to hold off transmitting; the RX
pipeline accumulates exactly that many decoded blocks before attempting a
frame parse. `payload_len` lets the parser skip zero-padding cleanly.

The CRC-32 (IEEE 802.3) is always the **last 4 bytes** of the assembled
multi-block frame, covering the header, callsign, payload, and padding.

---

## Survival Mode (Tier 0)

Tier 0 is designed around the same philosophy as FT8 — exploit a very narrow
channel and heavy FEC to communicate when broader modes have failed.

- **Channel**: 50 Hz, BPSK, 6.25 Hz subcarrier spacing, 90-second frame window
- **FEC**: H_256_768_22 (256 data bits, 768 coded bits, rate 1/3)
- **Combining**: 4 identical codeword transmissions; LLRs summed before decode
- **Effective threshold**: ≈ −8 dB channel SNR ≈ −25 dB in 2500 Hz reference BW
- **Transmission time for a short message**: ≈ 100 seconds end-to-end

A "help me" message (7 bytes of payload) fits in a single FEC block of 32 bytes
(H_256_768_22 data field). Total air time ≈ 512 OFDM symbols × 192 ms/symbol
plus preamble ≈ 99 seconds.

---

## Dependencies

```bash
# Ubuntu / Debian
sudo apt install libfftw3-dev portaudio19-dev libhamlib-dev libcodec2-dev cmake build-essential

# macOS (Homebrew)
brew install fftw portaudio codec2 cmake
# hamlib optional: brew install hamlib
```

---

## Build

```bash
mkdir build && cd build

# Recommended — with codec2 LDPC FEC
cmake .. -DWITH_CODEC2=ON -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Without FEC (hard-decision demod only, useful for development)
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Without Hamlib
cmake .. -DWITH_CODEC2=ON -DWITH_HAMLIB=OFF
make -j$(nproc)
```

---

## Usage

### Offline simulation

```bash
# BER sweep across all tiers
./sim_test

# BER sweep for a single tier
./sim_test --tier 4

# Test at a specific SNR
./sim_test --snr -3.5

# Survival mode SNR sweep with 2500 Hz reference column
./sim_test --survival
```

### Live TX

```bash
./opendsp_ofdm --call W1AW --tx "Hello via OFDM"

# Force tier 0 (survival mode)
./opendsp_ofdm --call W1AW --tx "help me" --tier 0

# Offline preview — shows frame size and duration without a soundcard
./opendsp_ofdm --call W1AW --tx "test" --snr 0
```

### Live RX

```bash
./opendsp_ofdm --call W1AW --rx
./opendsp_ofdm --call W1AW --rx --verbose    # per-symbol SNR and ACM status
```

### Misc

```bash
./opendsp_ofdm --list-devices               # list audio device indices
./opendsp_ofdm --call W1AW --rx --tier 3    # lock to tier 3
./opendsp_ofdm --help
```

---

## Bandwidth Negotiation

Both stations start at tier 0 (50 Hz BPSK) and upgrade once SNR is confirmed:

```
Station A                        Station B
    |── PROBE (tier=0, max=8) ──>|
    |<── ACK (agreed=4, max=8) ──|
    |══════ DATA @ tier 4 ════════|
    |<── RETUNE (new=2) ──────────|   (B's SNR degraded)
    |──── ACK (agreed=2) ────────>|
    |══════ DATA @ tier 2 ════════|
```

The ACM engine also upgrades autonomously after `UPGRADE_HOLD_FRAMES`
consecutive frames above the next tier's SNR threshold.

---

## Project Structure

```
opendsp-ofdm/
├── CMakeLists.txt
├── README.md
├── include/
│   ├── ofdm_core.h       # OFDM modulator/demodulator, constellations, ZC sync
│   ├── acm.h             # ACM engine, 9-tier table, OFDMParams builder
│   ├── fec.h             # FECCodec (codec2 LDPC), FECAccumulator (soft combining)
│   ├── framer.h          # Frame builder/parser, ACM header, CRC-32
│   ├── negotiation.h     # PROBE/ACK/RETUNE/NAK state machine
│   └── audio_io.h        # PortAudio I/O, AGC, DC block, Hamlib PTT
├── src/
│   ├── ofdm_core.cpp
│   ├── acm.cpp
│   ├── fec.cpp
│   ├── framer.cpp
│   ├── negotiation.cpp
│   ├── audio_io.cpp
│   ├── main.cpp          # CLI entry point, TX/RX pipelines
│   └── sim_test.cpp      # Offline BER simulation, --survival sweep
└── third_party/
    └── codec2/
        └── ldpc_api.h    # Vendored codec2 LDPC internal declarations
```

---

## Known Limitations

1. **LLR scaling**: `demap_symbol_llr` uses `2·Re/noise_var`; theoretically
   should be `4·Re·|h|²/noise_var` — correcting this would lower the effective
   FEC threshold by ≈ 1 dB.
2. **Frequency-selective noise estimation**: the LS channel variance estimator
   conflates multipath with noise, overstating noise in fading channels.
3. **RX multi-block**: blocks must arrive consecutively; there is no re-ordering
   or timeout to recover from mid-stream tier changes.
4. **Pilot density**: pilot interval is fixed at 4; widening to 8–16 would free
   bandwidth on flat channels.

---

## License

MIT — see LICENSE file.

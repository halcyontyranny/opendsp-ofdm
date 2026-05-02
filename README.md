# OpenDSP-OFDM

**Adaptive Bandwidth HF OFDM Modem** — FT8-class weak-signal sensitivity with
multi-carrier throughput scaling from 500 Hz to 3500 Hz based on channel quality.

---

## Features

- **Adaptive OFDM**: 9 ACM tiers from BPSK 1/8 @ 500 Hz to 16-QAM 3/4 @ 3500 Hz
- **FT8-proven subcarrier spacing**: 6.25 Hz at low SNR for ionospheric Doppler tolerance
- **Bandwidth negotiation**: PROBE/ACK/RETUNE handshake — always starts at 500 Hz
- **MMSE channel equalisation** with LS pilot-based channel estimation
- **Soft-decision demodulation** (LLR output, ready for LDPC)
- **PortAudio I/O**: works with any soundcard, full-duplex
- **Hamlib integration** (optional): CAT/PTT control for transceivers
- **Offline simulation mode**: test without a radio

---

## ACM Tier Table

| Tier | SNR (dB) | BW (Hz) | Spacing | Modulation | Code Rate | ~Throughput |
|------|----------|---------|---------|-----------|-----------|------------|
| 0    | −20      | 500     | 6.25 Hz | BPSK      | 1/8       | ~30 bps    |
| 1    | −12      | 500     | 6.25 Hz | BPSK      | 1/4       | ~60 bps    |
| 2    | −8       | 1000    | 6.25 Hz | BPSK      | 1/3       | ~160 bps   |
| 3    | −5       | 1500    | 6.25 Hz | QPSK      | 1/3       | ~480 bps   |
| 4    | −2       | 2000    | 6.25 Hz | QPSK      | 1/2       | ~960 bps   |
| 5    | +1       | 2500    | 6.25 Hz | QPSK      | 2/3       | ~1.6 kbps  |
| 6    | +4       | 3000    | 12.5 Hz | 8-PSK     | 2/3       | ~2.4 kbps  |
| 7    | +7       | 3200    | 12.5 Hz | 8-PSK     | 3/4       | ~3.0 kbps  |
| 8    | +10      | 3500    | 25 Hz   | 16-QAM    | 3/4       | ~3.8 kbps  |

> **Note**: Throughputs are pre-FEC estimates. With LDPC applied, effective
> throughput will be lower but reliability substantially higher.

---

## Dependencies

```bash
# Ubuntu / Debian
sudo apt install libfftw3-dev portaudio19-dev libhamlib-dev cmake build-essential

# macOS (Homebrew)
brew install fftw portaudio hamlib cmake
```

---

## Build

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)
```

To build without Hamlib:
```bash
cmake .. -DWITH_HAMLIB=OFF
```

---

## Usage

### Offline simulation (no radio needed)
```bash
# Run BER sweep across all tiers
./sim_test

# Test a specific tier
./sim_test --tier 4

# Test at a specific SNR
./sim_test --snr -3.5
```

### Live TX
```bash
./opendsp_ofdm --call W1AW --tx "Hello via OFDM"
```

### Live RX
```bash
./opendsp_ofdm --call W1AW --rx --verbose
```

### List audio devices
```bash
./opendsp_ofdm --list-devices
```

### Force a specific ACM tier
```bash
./opendsp_ofdm --call W1AW --rx --tier 3
```

### Offline TX preview (no soundcard)
```bash
./opendsp_ofdm --call W1AW --tx "test" --snr 0
```

---

## Frame Structure

```
| ZC Preamble | Pilot Symbol | ACM Header (3B) | Payload | CRC-32 (4B) |
```

- **ZC Preamble**: Zadoff-Chu sequence for timing sync and coarse frequency offset
- **Pilot Symbol**: full OFDM symbol with known pilots for LS channel estimation
- **ACM Header**: tier index, max supported tier, frame type (DATA/PROBE/ACK/RETUNE/NAK)
- **CRC-32**: covers header + callsign + payload

---

## Bandwidth Negotiation

Both stations always start at **tier 0 (500 Hz BPSK)**:

```
Station A                    Station B
    |── PROBE (tier=0, max=8) ──>|
    |<── ACK (agreed=4, max=8) ──|
    |══════ DATA @ tier 4 ═══════|
    |<── RETUNE (new=2) ─────────|  (B's SNR degraded)
    |──── ACK (agreed=2) ────────>|
    |══════ DATA @ tier 2 ═══════|
```

---

## Project Structure

```
opendsp-ofdm/
├── CMakeLists.txt
├── README.md
├── include/
│   ├── ofdm_core.h       # OFDM modulator/demodulator, constellations, ZC sync
│   ├── acm.h             # Adaptive Coding & Modulation engine
│   ├── framer.h          # Frame builder/parser, CRC-32
│   ├── negotiation.h     # PROBE/ACK/RETUNE state machine
│   └── audio_io.h        # PortAudio I/O, AGC, DC block, Hamlib PTT
└── src/
    ├── ofdm_core.cpp
    ├── acm.cpp
    ├── framer.cpp
    ├── negotiation.cpp
    ├── audio_io.cpp
    ├── main.cpp           # CLI entry point
    └── sim_test.cpp       # Offline BER simulation, no radio needed
```

---

## Next Steps (Phase 2)

- [ ] Integrate codec2 LDPC for true FEC (currently LLR output is ready for it)
- [ ] Rayleigh fading channel model in sim_test
- [ ] Frequency offset estimation and correction (CFO)
- [ ] Per-subcarrier loading (water-filling) for known channel response
- [ ] WSJT-X compatible UDP interface for integration with logging software
- [ ] OTA testing on 40m / 20m amateur HF bands

---

## License

MIT — see LICENSE file.

## Author

Generated as an open-source HF modem PoC. Contributions welcome.

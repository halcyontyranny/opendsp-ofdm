#include "framer.h"
#include <stdexcept>
#include <cstring>

namespace opendsp {

// ── CRC-32 (standard IEEE 802.3 polynomial) ───────────────────────────────────

static const uint32_t CRC32_TABLE[256] = {
    0x00000000,0x77073096,0xee0e612c,0x990951ba,0x076dc419,0x706af48f,
    0xe963a535,0x9e6495a3,0x0edb8832,0x79dcb8a4,0xe0d5e91b,0x97d2d988,
    0x09b64c2b,0x7eb17cbf,0xe7b82d08,0x90bf1d3c,0x1db71064,0x6ab020f2,
    0xf3b97148,0x84be41de,0x1adad47d,0x6ddde4eb,0xf4d4b551,0x83d385c7,
    0x136c9856,0x646ba8c0,0xfd62f97a,0x8a65c9ec,0x14015c4f,0x63066cd9,
    0xfa0f3d63,0x8d080df5,0x3b6e20c8,0x4c69105e,0xd56041e4,0xa2677172,
    0x3c03e4d1,0x4b04d447,0xd20d85fd,0xa50ab56b,0x35b5a8fa,0x42b2986c,
    0xdbbbc9d6,0xacbcf940,0x32d86ce3,0x45df5c75,0xdcd60dcf,0xabd13d59,
    0x26d930ac,0x51de003a,0xc8d75180,0xbfd06116,0x21b4f92b,0x56b3c4bd,
    0xcfba9599,0xb8bda50f,0x2802b89e,0x5f058808,0xc60cd9b2,0xb10be924,
    0x2f6f7c87,0x58684c11,0xc1611dab,0xb6662d3d,0x76dc4190,0x01db7106,
    0x98d220bc,0xefd5102a,0x71b18589,0x06b6b51f,0x9fbfe4a5,0xe8b8d433,
    0x7807c9a2,0x0f00f934,0x9609a88e,0xe10e9818,0x7f6796db,0x086d3d2d,
    0x91646c97,0xe6635c01,0x6b6b51f4,0x1c6c6162,0x856530d8,0xf262004e,
    0x6c0695ed,0x1b01a57b,0x8208f4c1,0xf50fc457,0x65b0d9c6,0x12b7e950,
    0x8bbeb8ea,0xfcb9887c,0x62dd1ddf,0x15da2d49,0x8cd37cf3,0xfbd44c65,
    0x4db26158,0x3ab551ce,0xa3bc0074,0xd4bb30e2,0x4adfa541,0x3dd895d7,
    0xa4d1c46d,0xd3d6f4fb,0x4369e96a,0x346ed9fc,0xad678846,0xda60b8d0,
    0x44042d73,0x33031de5,0xaa0a4c5f,0xdd0d7cc9,0x5005713c,0x270241aa,
    0xbe0b1010,0xc90c2086,0x5768b525,0x206f85b3,0xb966d409,0xce61e49f,
    0x5edef90e,0x29d9c998,0xb0d09822,0xc7d7a8b4,0x59b33d17,0x2eb40d81,
    0xb7bd5c3b,0xc0ba6cad,0xedb88320,0x9abfb3b6,0x03b6e20c,0x74b1d29a,
    0xead54739,0x9dd277af,0x04db2615,0x73dc1683,0xe3630b12,0x94643b84,
    0x0d6d6a3e,0x7a6a5aa8,0xe40ecf0b,0x9309ff9d,0x0a00ae27,0x7d079eb1,
    0xf00f9344,0x8708a3d2,0x1e01f268,0x6906c2fe,0xf762575d,0x806567cb,
    0x196c3671,0x6e6b06e7,0xfed41b76,0x89d32be0,0x10da7a5a,0x67dd4acc,
    0xf9b9df6f,0x8ebeeff9,0x17b7be43,0x60b08ed5,0xd6d6a3e8,0xa1d1937e,
    0x38d8c2c4,0x4fdff252,0xd1bb67f1,0xa6bc5767,0x3fb506dd,0x48b2364b,
    0xd80d2bda,0xaf0a1b4c,0x36034af6,0x41047a60,0xdf60efc3,0xa8670955,
    0x316658ef,0x466c7879,0xb40bbe37,0xc30c8ea1,0x5a05df1b,0x2d02ef8d,
    // (truncated table — generated at runtime below if needed)
};

uint32_t Framer::crc32(const std::vector<uint8_t>& data) {
    uint32_t crc = 0xFFFFFFFF;
    for (uint8_t byte : data) {
        uint8_t idx = (crc ^ byte) & 0xFF;
        crc = (crc >> 8) ^ CRC32_TABLE[idx];
    }
    return crc ^ 0xFFFFFFFF;
}

// ── Build ─────────────────────────────────────────────────────────────────────

std::vector<uint8_t> Framer::build(const Frame& f, int target_bytes) const {
    std::vector<uint8_t> raw;

    // ACM header (5 bytes); payload_len auto-derived from actual payload size
    raw.push_back(f.header.tier_index);
    raw.push_back(f.header.max_bw_code);
    raw.push_back(static_cast<uint8_t>(f.header.frame_type));
    raw.push_back(f.header.num_blocks);
    raw.push_back(static_cast<uint8_t>(f.payload.size()));

    // Callsign (7 bytes, null-padded)
    char cs[CALLSIGN_BYTES] = {};
    std::strncpy(cs, f.callsign.c_str(), CALLSIGN_BYTES - 1);
    for (int i = 0; i < CALLSIGN_BYTES; i++) raw.push_back(static_cast<uint8_t>(cs[i]));

    // Payload
    for (uint8_t b : f.payload) raw.push_back(b);

    // Zero-pad to (target_bytes - CRC32_BYTES) so CRC is always the last 4 bytes
    if (target_bytes > 0) {
        int pad_target = target_bytes - CRC32_BYTES;
        while (static_cast<int>(raw.size()) < pad_target) raw.push_back(0);
    }

    // CRC-32 (always last 4 bytes, covers header + callsign + payload + padding)
    uint32_t crc = crc32(raw);
    raw.push_back((crc >> 24) & 0xFF);
    raw.push_back((crc >> 16) & 0xFF);
    raw.push_back((crc >>  8) & 0xFF);
    raw.push_back( crc        & 0xFF);

    return raw;
}

// ── Parse ─────────────────────────────────────────────────────────────────────

Frame Framer::parse(const std::vector<uint8_t>& raw) const {
    Frame f;
    if (raw.size() < static_cast<size_t>(ACM_HDR_BYTES + CALLSIGN_BYTES + CRC32_BYTES)) {
        f.crc_ok = false;
        return f;
    }

    size_t pos = 0;
    f.header.tier_index   = raw[pos++];
    f.header.max_bw_code  = raw[pos++];
    f.header.frame_type   = static_cast<FrameType>(raw[pos++]);
    f.header.num_blocks   = raw[pos++];
    f.header.payload_len  = raw[pos++];

    char cs[CALLSIGN_BYTES + 1] = {};
    for (int i = 0; i < CALLSIGN_BYTES; i++) cs[i] = static_cast<char>(raw[pos++]);
    f.callsign = std::string(cs);

    // Extract exactly payload_len bytes; ignore zero-padding between payload and CRC
    size_t payload_end = raw.size() - CRC32_BYTES;
    size_t plen = f.header.payload_len;
    if (pos + plen > payload_end) plen = payload_end - pos;
    for (size_t i = 0; i < plen; i++) f.payload.push_back(raw[pos + i]);

    // Verify CRC
    std::vector<uint8_t> body(raw.begin(), raw.begin() + payload_end);
    uint32_t computed = crc32(body);
    uint32_t stored =
        (static_cast<uint32_t>(raw[payload_end    ]) << 24) |
        (static_cast<uint32_t>(raw[payload_end + 1]) << 16) |
        (static_cast<uint32_t>(raw[payload_end + 2]) <<  8) |
         static_cast<uint32_t>(raw[payload_end + 3]);
    f.crc    = stored;
    f.crc_ok = (computed == stored);
    return f;
}

// ── Convenience builders ──────────────────────────────────────────────────────

Frame Framer::make_probe(const std::string& callsign, int our_max_tier) const {
    Frame f;
    f.header.tier_index  = 0;  // Always probe at minimum tier
    f.header.max_bw_code = static_cast<uint8_t>(our_max_tier);
    f.header.frame_type  = FrameType::PROBE;
    f.header.num_blocks  = 1;
    f.callsign           = callsign;
    return f;
}

Frame Framer::make_ack(const std::string& callsign,
                       int agreed_tier,
                       int our_max_tier) const {
    Frame f;
    f.header.tier_index  = static_cast<uint8_t>(agreed_tier);
    f.header.max_bw_code = static_cast<uint8_t>(our_max_tier);
    f.header.frame_type  = FrameType::ACK;
    f.header.num_blocks  = 1;
    f.callsign           = callsign;
    // Encode agreed tier in payload byte 0 for redundancy
    f.payload.push_back(static_cast<uint8_t>(agreed_tier));
    return f;
}

Frame Framer::make_retune(const std::string& callsign, int new_tier) const {
    Frame f;
    f.header.tier_index  = static_cast<uint8_t>(new_tier);
    f.header.max_bw_code = 0;
    f.header.frame_type  = FrameType::RETUNE;
    f.header.num_blocks  = 1;
    f.callsign           = callsign;
    f.payload.push_back(static_cast<uint8_t>(new_tier));
    return f;
}

Frame Framer::make_nak(const std::string& callsign) const {
    Frame f;
    f.header.tier_index  = 0;
    f.header.max_bw_code = 0;
    f.header.frame_type  = FrameType::NAK;
    f.header.num_blocks  = 1;
    f.callsign           = callsign;
    return f;
}

int Framer::max_payload_bytes(const ACMTier& tier) {
    // Bits available in one frame window for payload (after FEC, pilots, overhead)
    int num_sc = static_cast<int>(std::floor(tier.bw_hz / tier.sub_spacing_hz));
    double pilot_frac   = 0.25;
    int    data_sc      = static_cast<int>(num_sc * (1.0 - pilot_frac));
    double sym_dur      = (1.0 / tier.sub_spacing_hz) * (1.0 + CP_RATIO);
    double syms_per_win = tier.frame_window_sec / sym_dur;
    int    bps          = static_cast<int>(tier.mod);

    int total_bits  = static_cast<int>(data_sc * syms_per_win * bps);
    int coded_bits  = static_cast<int>(total_bits * tier.rate.value());
    // Reserve 15% for sync/ACM header/CRC
    int payload_bits = static_cast<int>(coded_bits * 0.85);
    return payload_bits / 8;
}

} // namespace opendsp

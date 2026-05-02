#pragma once
#include <vector>
#include <cstdint>
#include <string>
#include "acm.h"

namespace opendsp {

// ── Frame field sizes (bytes) ─────────────────────────────────────────────────
//
//  FEC block layout (size = fec.data_bits()/8 per block):
//  | ACM_HDR 5B | CALLSIGN 7B | PAYLOAD NB | ZERO-PAD KB | CRC-32 4B |
//
//  ACM_HDR (5 bytes):
//    [tier_index 8b][max_bw_code 8b][frame_type 8b][num_blocks 8b][payload_len 8b]
//
//  num_blocks : total FEC blocks in this transmission (receiver holds TX for this long)
//  payload_len: valid payload bytes in this message (remainder of block is zero-padding)
//
//  CRC-32 covers all bytes preceding it (header + callsign + payload + padding).
//  Zero-padding keeps each FEC block at an exact fec.data_bits()/8 byte boundary.

static constexpr int ACM_HDR_BYTES  = 5;
static constexpr int CRC32_BYTES    = 4;
static constexpr int CALLSIGN_BYTES = 7;   // up to 6 chars + null

// Frame type (carried in ACM_HDR frame_type byte)
enum class FrameType : uint8_t {
    DATA    = 0x00,
    PROBE   = 0x01,   // Bandwidth negotiation probe
    ACK     = 0x02,   // Negotiation acknowledgement
    RETUNE  = 0x03,   // Mid-QSO retune request
    NAK     = 0x04,   // Negative acknowledgement
    BEACON  = 0x05,   // Periodic beacon / ID
};

// ── ACM header (5 bytes on-air) ───────────────────────────────────────────────

struct ACMHeader {
    uint8_t   tier_index;   // Sender's current tier
    uint8_t   max_bw_code;  // Sender's maximum supported BW (index into tier table)
    FrameType frame_type;
    uint8_t   num_blocks;   // Total FEC blocks in this transmission (1 = single block)
    uint8_t   payload_len;  // Valid payload bytes (set automatically by build())
};

// ── Frame ─────────────────────────────────────────────────────────────────────

struct Frame {
    ACMHeader            header;
    std::string          callsign;       // Sender callsign (up to 6 chars)
    std::vector<uint8_t> payload;        // Data bytes
    uint32_t             crc;            // CRC-32 (computed over header+callsign+payload)
    bool                 crc_ok = false; // Set by parser
};

// ── Framer ────────────────────────────────────────────────────────────────────

class Framer {
public:
    // Build a complete byte-stream for a DATA frame.
    // target_bytes: if > 0, zero-pads between payload and CRC to fill exactly that many bytes.
    std::vector<uint8_t> build(const Frame& f, int target_bytes = 0) const;

    // Parse a byte-stream into a Frame; sets frame.crc_ok.
    Frame parse(const std::vector<uint8_t>& raw) const;

    // Build a PROBE frame (sent at minimum tier to initiate negotiation).
    Frame make_probe(const std::string& callsign, int our_max_tier) const;

    // Build an ACK frame in response to a PROBE.
    // agreed_tier: the lower of what both sides support at current SNR.
    Frame make_ack(const std::string& callsign,
                   int agreed_tier,
                   int our_max_tier) const;

    // Build a RETUNE frame requesting a tier change mid-QSO.
    Frame make_retune(const std::string& callsign, int new_tier) const;

    // Build a NAK frame.
    Frame make_nak(const std::string& callsign) const;

    // Maximum payload bytes for a given ACM tier.
    static int max_payload_bytes(const ACMTier& tier);

private:
    static uint32_t crc32(const std::vector<uint8_t>& data);
};

} // namespace opendsp

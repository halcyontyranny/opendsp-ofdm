#pragma once

#ifdef HAVE_CODEC2

#include <vector>
#include <cstdint>
#include <string>

namespace opendsp {

// LDPC forward error correction wrapper around codec2.
//
// Works entirely in "flat bit" space: every element of a bit vector is a
// single uint8_t with value 0 or 1.  LLRs use the same sign convention as
// OFDMDemodulator::equalise_and_demap: positive → bit 0, negative → bit 1.
class FECCodec {
public:
    explicit FECCodec(const std::string& code_name);
    ~FECCodec();

    FECCodec(const FECCodec&)            = delete;
    FECCodec& operator=(const FECCodec&) = delete;
    FECCodec(FECCodec&&)                 = default;

    // Bits of user data per encoder input block.
    int data_bits()  const;

    // Bits out of encoder per block (data + parity).
    int coded_bits() const;

    // Encode one block.  payload_bits must be exactly data_bits() elements
    // (each 0 or 1).  Returns coded_bits() elements.
    std::vector<uint8_t> encode(const std::vector<uint8_t>& payload_bits) const;

    // Decode one block.  llr must be exactly coded_bits() floats.
    // Writes data_bits() decoded bits into out_bits.
    // Returns true if all parity checks passed.
    bool decode(const std::vector<float>& llr,
                std::vector<uint8_t>& out_bits) const;

    // Pad a bit vector to the next multiple of data_bits() (zero-fill).
    std::vector<uint8_t> pad_to_block(const std::vector<uint8_t>& bits) const;

    const std::string& code_name() const;

private:
    struct Impl;
    Impl* impl_;
};

// Return the codec2 LDPC code name best matched to ACM tier_index (0–8).
std::string fec_code_for_tier(int tier_index);

// Byte ↔ flat-bit conversions (MSB first).
std::vector<uint8_t> bytes_to_bits(const std::vector<uint8_t>& bytes);
std::vector<uint8_t> bits_to_bytes(const std::vector<uint8_t>& bits);

} // namespace opendsp

#endif // HAVE_CODEC2

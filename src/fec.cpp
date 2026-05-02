#include "fec.h"

#ifdef HAVE_CODEC2

#include "../third_party/codec2/ldpc_api.h"
#include <cstring>
#include <stdexcept>
#include <vector>

namespace opendsp {

// ── Impl ──────────────────────────────────────────────────────────────────────

struct FECCodec::Impl {
    struct LDPC ldpc;
    std::string code_name;
};

// ── Constructor / destructor ──────────────────────────────────────────────────

FECCodec::FECCodec(const std::string& code_name) : impl_(new Impl) {
    impl_->code_name = code_name;
    std::memset(&impl_->ldpc, 0, sizeof(impl_->ldpc));

    char buf[64];
    std::strncpy(buf, code_name.c_str(), sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    if (ldpc_codes_find(buf) < 0)
        throw std::runtime_error("FEC: codec2 LDPC code not found: " + code_name);

    ldpc_codes_setup(&impl_->ldpc, buf);
    impl_->ldpc.max_iter = 100;
}

FECCodec::~FECCodec() { delete impl_; }

// ── Accessors ─────────────────────────────────────────────────────────────────

int FECCodec::data_bits()  const { return impl_->ldpc.ldpc_data_bits_per_frame;  }
int FECCodec::coded_bits() const { return impl_->ldpc.ldpc_coded_bits_per_frame; }
const std::string& FECCodec::code_name() const { return impl_->code_name; }

// ── Encode ────────────────────────────────────────────────────────────────────

std::vector<uint8_t> FECCodec::encode(const std::vector<uint8_t>& payload_bits) const {
    int db = data_bits();
    int cb = coded_bits();

    if (static_cast<int>(payload_bits.size()) != db)
        throw std::invalid_argument(
            "FEC encode: expected " + std::to_string(db) +
            " bits, got " + std::to_string(payload_bits.size()));

    std::vector<unsigned char> in(db);
    for (int j = 0; j < db; j++)
        in[j] = payload_bits[j] & 1;

    std::vector<int> codeword(cb);
    ldpc_encode_frame(&impl_->ldpc, codeword.data(), in.data());

    std::vector<uint8_t> out(cb);
    for (int j = 0; j < cb; j++)
        out[j] = static_cast<uint8_t>(codeword[j] & 1);
    return out;
}

// ── Decode ────────────────────────────────────────────────────────────────────

bool FECCodec::decode(const std::vector<float>& llr,
                      std::vector<uint8_t>& out_bits) const {
    int db = data_bits();
    int cb = coded_bits();

    if (static_cast<int>(llr.size()) != cb)
        throw std::invalid_argument(
            "FEC decode: expected " + std::to_string(cb) +
            " LLRs, got " + std::to_string(llr.size()));

    // codec2 run_ldpc_decoder modifies the input buffer, so give it a copy.
    std::vector<float> llr_copy(llr);

    // run_ldpc_decoder writes CodeLength bytes (full codeword: data + parity).
    // Allocate that much even though we only return the first db data bits.
    int cl = impl_->ldpc.CodeLength;
    std::vector<uint8_t> full_out(cl, 0);
    int parity_ok = 0;
    run_ldpc_decoder(&impl_->ldpc, full_out.data(), llr_copy.data(), &parity_ok);

    // Data bits occupy the first ldpc_data_bits_per_frame positions.
    out_bits.assign(full_out.begin(), full_out.begin() + db);

    return parity_ok == impl_->ldpc.NumberParityBits;
}

// ── pad_to_block ──────────────────────────────────────────────────────────────

std::vector<uint8_t> FECCodec::pad_to_block(const std::vector<uint8_t>& bits) const {
    int db  = data_bits();
    int rem = static_cast<int>(bits.size()) % db;
    if (rem == 0) return bits;
    std::vector<uint8_t> padded = bits;
    padded.resize(bits.size() + (db - rem), 0);
    return padded;
}

// ── Code selection ────────────────────────────────────────────────────────────

std::string fec_code_for_tier(int tier_index) {
    if (tier_index <= 1) return "H_256_768_22";   // rate 1/4: 256 data, 1024 coded
    if (tier_index <= 3) return "H_128_256_5";    // rate 1/3: 128 data,  384 coded
    if (tier_index <= 5) return "HRA_112_112";    // rate 1/2: 112 data,  224 coded
    return                      "HRAa_1536_512";  // rate 3/4: 1536 data, 2048 coded
}

// ── Byte ↔ bit helpers ────────────────────────────────────────────────────────

std::vector<uint8_t> bytes_to_bits(const std::vector<uint8_t>& bytes) {
    std::vector<uint8_t> bits;
    bits.reserve(bytes.size() * 8);
    for (auto b : bytes)
        for (int i = 7; i >= 0; i--)
            bits.push_back((b >> i) & 1);
    return bits;
}

std::vector<uint8_t> bits_to_bytes(const std::vector<uint8_t>& bits) {
    std::vector<uint8_t> bytes((bits.size() + 7) / 8, 0);
    for (size_t i = 0; i < bits.size(); i++)
        bytes[i / 8] |= (bits[i] & 1) << (7 - (i % 8));
    return bytes;
}

} // namespace opendsp

#endif // HAVE_CODEC2

/*
 * Minimal vendored declarations for the codec2 LDPC encode/decode API.
 *
 * Sourced from drowe67/codec2 (main branch):
 *   src/mpdecode_core.h  — struct LDPC, run_ldpc_decoder
 *   src/ldpc_codes.h     — ldpc_codes_find, ldpc_codes_setup
 *   src/interldpc.h      — ldpc_encode_frame
 *
 * Only the subset needed by opendsp-ofdm is declared here; transitive
 * codec2 headers (comp.h, ofdm_internal.h) are not required for this subset.
 *
 * Compatible with libcodec2 >= 1.0.
 */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct LDPC {
    char     name[32];
    int      max_iter;
    int      dec_type;
    int      q_scale_factor;
    int      r_scale_factor;
    int      CodeLength;
    int      NumberParityBits;
    int      NumberRowsHcols;
    int      max_row_weight;
    int      max_col_weight;
    uint16_t *H_rows;
    uint16_t *H_cols;
    int      ldpc_data_bits_per_frame;
    int      ldpc_coded_bits_per_frame;
    int      protection_mode;
    int      data_bits_per_frame;
    int      coded_bits_per_frame;
};

/* mpdecode_core.h */
int  run_ldpc_decoder(struct LDPC *ldpc, uint8_t out_char[], float input[],
                      int *parityCheckCount);
void ldpc_print_info(struct LDPC *ldpc);

/* ldpc_codes.h */
int  ldpc_codes_num(void);
void ldpc_codes_list(void);
int  ldpc_codes_find(char name[]);
void ldpc_codes_setup(struct LDPC *ldpc, char name[]);

/* interldpc.h */
void ldpc_encode_frame(struct LDPC *ldpc, int codeword[],
                       unsigned char tx_bits_char[]);

#ifdef __cplusplus
}
#endif

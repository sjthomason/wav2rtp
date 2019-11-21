/*
 * $Id$
 *
 * Copyright (c) 2010, R.Imankulov, Yu Jiang
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the R.Imankulov nor the names of its contributors may
 *  be used to endorse or promote products derived from this software without
 *  specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "g729a_codec.h"

#include "options.h"

#define G729_BYTES_PER_FRAME  10
#define PCM_SAMPLES_PER_FRAME 80

/* ENCODER */

wr_encoder_t *wr_g729a_encoder_init(wr_encoder_t *pcodec)
{
    wr_g729a_encoder_state *state = malloc(sizeof(wr_g729a_encoder_state));
    if (!state)
        return NULL;
    bzero(state, sizeof(wr_g729a_encoder_state));
    state->buffer_size = iniparser_getpositiveint(wr_options.codecs_options, "g729a:buffer_size", 640);
    state->encoder_object = initBcg729EncoderChannel(0);

    pcodec->state = (void *) state;
    pcodec->payload_type = iniparser_getnonnegativeint(wr_options.codecs_options, "g729a:payload_type", 18);
    return pcodec;
}

void wr_g729a_encoder_destroy(wr_encoder_t *pcodec)
{
    wr_g729a_encoder_state *state = (wr_g729a_encoder_state *) pcodec->state;
    closeBcg729EncoderChannel(state->encoder_object);
    free(pcodec->state);
}

int wr_g729a_encoder_get_input_buffer_size(void *state)
{
    wr_g729a_encoder_state *s = (wr_g729a_encoder_state *) state;
    return s->buffer_size;
}

int wr_g729a_encoder_get_output_buffer_size(void *state)
{
    wr_g729a_encoder_state *s = (wr_g729a_encoder_state *) state;
    int frames = s->buffer_size / PCM_SAMPLES_PER_FRAME;
    return frames * G729_BYTES_PER_FRAME;
}

int wr_g729a_encode(void *state, const short *input, char *output)
{
    wr_g729a_encoder_state *s = (wr_g729a_encoder_state *) state;
    int size = s->buffer_size;
    int16_t *in_buf = (int16_t *) input;
    int encoded_data_len = 0;

    if (size % PCM_SAMPLES_PER_FRAME == 0) {
        while (size >= PCM_SAMPLES_PER_FRAME) {
            uint8_t olen;
            bcg729Encoder(s->encoder_object, in_buf, (uint8_t *) output, &olen);

            size -= PCM_SAMPLES_PER_FRAME;
            in_buf += PCM_SAMPLES_PER_FRAME;

            output += olen;
            encoded_data_len += olen;
        }
    }
    return encoded_data_len;
}

/* DECODER */

wr_decoder_t *wr_g729a_decoder_init(wr_decoder_t *pcodec)
{
    wr_g729a_decoder_state *state = malloc(sizeof(wr_g729a_decoder_state));
    if (!state)
        return NULL;
    bzero(state, sizeof(wr_g729a_decoder_state));
    state->buffer_size = iniparser_getpositiveint(wr_options.codecs_options, "g729a:buffer_size", 640);
    state->decoder_object = initBcg729DecoderChannel();

    pcodec->state = (void *) state;
    pcodec->payload_type = iniparser_getnonnegativeint(wr_options.codecs_options, "g729a:payload_type", 18);

    return pcodec;
}

void wr_g729a_decoder_destroy(wr_decoder_t *pcodec)
{
    wr_g729a_decoder_state *state = (wr_g729a_decoder_state *) pcodec->state;
    closeBcg729DecoderChannel(state->decoder_object);
    free(pcodec->state);
}

int wr_g729a_decoder_get_input_buffer_size(void *state)
{
    wr_g729a_decoder_state *s = (wr_g729a_decoder_state *) state;
    int frames = s->buffer_size / PCM_SAMPLES_PER_FRAME;
    return frames * G729_BYTES_PER_FRAME;
}

int wr_g729a_decoder_get_output_buffer_size(void *state)
{
    wr_g729a_decoder_state *s = (wr_g729a_decoder_state *) state;
    return s->buffer_size;
}

int wr_g729a_decode(void *state, const char *input, size_t size, short *output)
{
    wr_g729a_decoder_state *s = (wr_g729a_decoder_state *) state;
    uint8_t *in_buf = (uint8_t *) input;
    unsigned int decoded_data_len = 0;

    if (size % G729_BYTES_PER_FRAME != 0) {
        while (size >= G729_BYTES_PER_FRAME) {
            /* Decode a frame  */
            bcg729Decoder(s->decoder_object, in_buf, G729_BYTES_PER_FRAME, 0 /* no erasure */, 0 /* not SID */,
                          0 /* not RFC3389 */, (int16_t *) output);

            size -= G729_BYTES_PER_FRAME;
            in_buf += G729_BYTES_PER_FRAME;

            output += PCM_SAMPLES_PER_FRAME;
            decoded_data_len += PCM_SAMPLES_PER_FRAME;
        }
    }

    return decoded_data_len;
}

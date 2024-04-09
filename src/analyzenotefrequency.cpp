/* Audio Library Note Frequency Detection & Guitar/Bass Tuner
 * Copyright (c) 2015, Colin Duffy
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// Code taken from AudioAnalyzeNoteFrequency, part of the Teensy Audio library.
// See https://github.com/duff2013/AudioTuner/blob/master/AudioTuner.h for original.

#include "analyzenotefrequency.h"

#include "arm_math.h"
#include <DebugLog.h>

// this is provided by "utility/dspinst.h" but including it causes compilation errors
// in the teensy audio library.
static inline int64_t anf_multiply_accumulate_16tx16t_add_16bx16b(int64_t sum, uint32_t a, uint32_t b)
{
	asm volatile("smlald %Q0, %R0, %1, %2" : "+r" (sum) : "r" (a), "r" (b));
	return sum;
}

// __SIMD32 defined in "arm_math.h", but needs may_alias attribute otherwise it violates
// strict aliasing rules.
#define ANF_SIMD32(addr) (( int32_t __attribute__((__may_alias__)) *) (addr))

void AnalyzeNoteFrequency::update(int16_t sample) {
    if (!enabled) {
        return;
    }

    samples[state++] = sample;

    if (state == SAMPLES_TO_ANALYZE) {
        process();
        state = 0;
    }
}

/**
 *  Start the Yin algorithm
 *
 *  TODO: Significant speed up would be to use spectral domain to find fundamental frequency.
 *  This paper explains: https://aubio.org/phd/thesis/brossier06thesis.pdf -> Section 3.2.4
 *  page 79. Might have to downsample for low fundmental frequencies because of fft buffer
 *  size limit.
 */
void AnalyzeNoteFrequency::process( void ) {
    const uint16_t inner_cycles = SAMPLES_TO_ANALYZE >> 1;
    uint16_t outer_cycles = OUTER_CYCLES;
    uint16_t tau = 1;
    uint8_t yin_idx = 1;
    uint32_t running_sum = 0;
    do {
        uint64_t sum = 0;
        int32_t  a1, a2, b1, b2, c1, c2, d1, d2;
        int32_t  out1, out2, out3, out4;
        uint16_t blkCnt;
        int16_t __attribute__((__may_alias__)) * cur = samples;
        int16_t __attribute__((__may_alias__)) * lag = samples + tau;
        // unrolling the inner loop by 8
        blkCnt = inner_cycles >> 3;
        do {
            // a(n), b(n), c(n), d(n) each hold two samples
            a1 = *ANF_SIMD32( cur ); cur += 2;
            a2 = *ANF_SIMD32( cur ); cur += 2;
            b1 = *ANF_SIMD32( lag ); lag += 2;
            b2 = *ANF_SIMD32( lag ); lag += 2;
            c1 = *ANF_SIMD32( cur ); cur += 2;
            c2 = *ANF_SIMD32( cur ); cur += 2;
            d1 = *ANF_SIMD32( lag ); lag += 2;
            d2 = *ANF_SIMD32( lag ); lag += 2;
            // subract two samples at a time
            out1 = __QSUB16( a1, b1 );
            out2 = __QSUB16( a2, b2 );
            out3 = __QSUB16( c1, d1 );
            out4 = __QSUB16( c2, d2 );
            // square the difference
            sum = anf_multiply_accumulate_16tx16t_add_16bx16b( sum, out1, out1 );
            sum = anf_multiply_accumulate_16tx16t_add_16bx16b( sum, out2, out2 );
            sum = anf_multiply_accumulate_16tx16t_add_16bx16b( sum, out3, out3 );
            sum = anf_multiply_accumulate_16tx16t_add_16bx16b( sum, out4, out4 );

        } while( --blkCnt );

        //LOG_INFO("anf loop", tau, cur-samples,lag-samples);
        running_sum += sum >> SUM_DIVISOR_BITS;
        yin_buffer[yin_idx] = sum*tau >> SUM_DIVISOR_BITS;
        rs_buffer[yin_idx] = running_sum;
        yin_idx = ( ++yin_idx >= 5 ) ? 0 : yin_idx;
        tau = estimate( yin_idx, tau );
        
        if ( tau == 0 ) {
            new_output = true;
            return;
        }
    } while ( --outer_cycles );
    
    periodicity = 0.0f;
    new_output = true;
}

/**
 *  check the sampled data for fundamental frequency
 *
 *  @param yin  buffer to hold sum*tau value
 *  @param rs   buffer to hold running sum for sampled window
 *  @param head buffer index
 *  @param tau  lag we are currently working on gets incremented
 *
 *  @return tau
 */
uint16_t AnalyzeNoteFrequency::estimate( uint16_t head, uint16_t tau ) {
    if ( tau > 4 ) {
        uint16_t idx0, idx1, idx2;
        idx0 = head;
        idx1 = head + 1;
        idx1 = ( idx1 >= 5 ) ? 0 : idx1;
        idx2 = head + 2;
        idx2 = ( idx2 >= 5 ) ? idx2 - 5 : idx2;
        
        // maybe fixed point would be better here? But how?
        float s0, s1, s2;
        s0 = (float) yin_buffer[idx0] / (float) rs_buffer[idx0];
        s1 = (float) yin_buffer[idx1] / (float) rs_buffer[idx1];
        s2 = (float) yin_buffer[idx2] / (float) rs_buffer[idx2];
        
        if ( s1 < yin_threshold && s1 < s2 ) {
            uint16_t period = tau - 3;
            periodicity = 1 - s1;
            data = period + 0.5f * ( s0 - s2 ) / ( s0 - 2.0f * s1 + s2 );
            return 0;
        }
    }

    return tau + 1;
}

/**
 *  Initialise
 *
 */
void AnalyzeNoteFrequency::begin() {
    periodicity         = 0.0f;
    enabled             = true;
    state               = 0;
    data                = 0.0f;
}

void AnalyzeNoteFrequency::stop() {
    enabled = false;
}

/**
 *  available
 *
 *  @return true if processing is complete. resets internal flag
 */
bool AnalyzeNoteFrequency::available( void ) {
    bool flag = new_output;
    new_output = false;
    return flag;
}

/**
 *  validResult
 *
 *  @return true if valid frequency found
 */
bool AnalyzeNoteFrequency::validResult( void ) {
    return periodicity != 0.0f;
}

/**
 *  read processes the data samples for the Yin algorithm.
 *
 *  @return frequency in hertz
 */
float AnalyzeNoteFrequency::read( void ) {
    return sample_rate / data;
}

/**
 *  Periodicity of the sampled signal from Yin algorithm from read function.
 *
 *  @return periodicity
 */
float AnalyzeNoteFrequency::probability( void ) {
    return periodicity;
}

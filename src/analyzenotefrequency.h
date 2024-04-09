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

#pragma once

#include "Arduino.h"

// These parameters define the number of samples to measure and the maximum lag offset to measure up to.
#define SAMPLES_TO_ANALYZE 200 // the larger the number, the more precise the reading
#define OUTER_CYCLES 32 // sets min detectable frequency (fs / OUTER_CYCLES). max (SAMPLES_TO_ANALYZE / 2) to test up to the last sample.
#define SUM_DIVISOR_BITS 12 // perf optimization: divide autocorrelation sums to fit within 32 bits. 12 bits is sufficient to ensure no overflow with 200 samples.

class AnalyzeNoteFrequency {
public:
    /**
     *  constructor to setup Audio Library and initialize
     *
     *  @return none
     */
    AnalyzeNoteFrequency(float sample_rate, float threshold)
        : sample_rate(sample_rate)
        , yin_threshold(threshold)
        , enabled(false)
        , new_output(false) {
    }
    
    /**
     *  initialize variables and start conversion
     *
     *  @return none
     */
    void begin();
    
    /**
     *  stop conversion, any following calls to update() will be ignored
     */
    void stop();

    /**
     *  triggers true when processing a block is complete
     *
     *  @return flag to indicate valid frequency is found
     */
    bool available( void );

    /**
     *  valid frequency is found
     *
     *  @return flag to indicate 'read' and 'probability' results are valid
     */
    bool validResult( void );

    /**
     *  get frequency
     *
     *  @return frequency in hertz
     */
    float read( void );
    
    /**
     *  get predicitity
     *
     *  @return probability of frequency found
     */
    float probability( void );
    
    /**
     *  Audio Library calls this update function ~2.9ms
     *
     *  @return none
     */
    void update(int16_t sample);
    
private:
    /**
     *  check the sampled data for fundamental frequency
     *
     *  @param yin  buffer to hold sum*tau value
     *  @param rs   buffer to hold running sum for sampled window
     *  @param head buffer index
     *  @param tau  lag we are currently working on this gets incremented
     *
     *  @return tau
     */
    uint16_t estimate( uint16_t head, uint16_t tau );
    
    /**
     *  process audio data
     *
     *  @return none
     */
    void process( void );
    
    /**
     *  Variables
     */
    float    sample_rate;
    float    periodicity, yin_threshold, data;
    uint32_t yin_buffer[5], rs_buffer[5];
    int16_t  samples[SAMPLES_TO_ANALYZE] __attribute__ ((aligned(4)));
    uint32_t state;
    bool     enabled, new_output;
};

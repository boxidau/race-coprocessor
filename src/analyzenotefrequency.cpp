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

#define HALF_BLOCKS (SAMPLES_PER_BLOCK / 2)

void AnalyzeNoteFrequency::update(int16_t sample) {
    if ( next_buffer ) {
        samples1[state++] = sample;
        if ( !first_run && process_buffer ) process( );
    } else {
        samples2[state++] = sample;
        if ( !first_run && process_buffer ) process( );
    }
    
    if ( state >= SAMPLES_PER_BLOCK ) {
        if ( next_buffer ) {
            if ( !first_run && process_buffer ) process( );
            samples = samples1;
            next_buffer = false;
        } else {
            if ( !first_run && process_buffer ) process( );
            samples = samples2;
            next_buffer = true;
        }
        process_buffer = true;
        first_run = false;
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
    
    const int16_t *p = samples;
    
    uint16_t cycles = 64;
    uint16_t tau = tau_global;
    do {
        uint16_t x   = 0;
        uint64_t  sum = 0;
        do {
            int16_t current, lag, delta;
            lag = *( ( int16_t * )p + ( x+tau ) );
            current = *( ( int16_t * )p+x );
            delta = ( current-lag );
            sum += delta * delta;
            x += 4;
            
            lag = *( ( int16_t * )p + ( x+tau ) );
            current = *( ( int16_t * )p+x );
            delta = ( current-lag );
            sum += delta * delta;
            x += 4;
            
            lag = *( ( int16_t * )p + ( x+tau ) );
            current = *( ( int16_t * )p+x );
            delta = ( current-lag );
            sum += delta * delta;
            x += 4;
            
            lag = *( ( int16_t * )p + ( x+tau ) );
            current = *( ( int16_t * )p+x );
            delta = ( current-lag );
            sum += delta * delta;
            x += 4;
        } while ( x < HALF_BLOCKS );
        
        uint64_t rs = running_sum;
        rs += sum;
        yin_buffer[yin_idx] = sum*tau;
        rs_buffer[yin_idx] = rs;
        running_sum = rs;
        yin_idx = ( ++yin_idx >= 5 ) ? 0 : yin_idx;
        tau = estimate( yin_buffer, rs_buffer, yin_idx, tau );
        
        if ( tau == 0 ) {
            process_buffer  = false;
            new_output      = true;
            yin_idx         = 1;
            running_sum     = 0;
            tau_global      = 1;
            return;
        }
    } while ( --cycles );
    //digitalWriteFast(10, LOW);
    if ( tau >= HALF_BLOCKS ) {
        process_buffer  = false;
        new_output      = false;
        yin_idx         = 1;
        running_sum     = 0;
        tau_global      = 1;
        return;
    }
    tau_global = tau;
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
uint16_t AnalyzeNoteFrequency::estimate( uint64_t *yin, uint64_t *rs, uint16_t head, uint16_t tau ) {
    const uint64_t *y = ( uint64_t * )yin;
    const uint64_t *r = ( uint64_t * )rs;
    uint16_t _tau, _head;
    const float thresh = yin_threshold;
    _tau = tau;
    _head = head;
    
    if ( _tau > 4 ) {
        
        uint16_t idx0, idx1, idx2;
        idx0 = _head;
        idx1 = _head + 1;
        idx1 = ( idx1 >= 5 ) ? 0 : idx1;
        idx2 = head + 2;
        idx2 = ( idx2 >= 5 ) ? 0 : idx2;
        
        float s0, s1, s2;
        s0 = ( ( float )*( y+idx0 ) / *( r+idx0 ) );
        s1 = ( ( float )*( y+idx1 ) / *( r+idx1 ) );
        s2 = ( ( float )*( y+idx2 ) / *( r+idx2 ) );
        
        if ( s1 < thresh && s1 < s2 ) {
            uint16_t period = _tau - 3;
            periodicity = 1 - s1;
            data = period + 0.5f * ( s0 - s2 ) / ( s0 - 2.0f * s1 + s2 );
            return 0;
        }
    }
    return _tau + 1;
}

/**
 *  Initialise
 *
 *  @param threshold Allowed uncertainty
 */
void AnalyzeNoteFrequency::begin( float threshold ) {
    __disable_irq( );
    process_buffer = false;
    yin_threshold  = threshold;
    periodicity    = 0.0f;
    next_buffer    = true;
    running_sum    = 0;
    tau_global     = 1;
    first_run      = true;
    yin_idx        = 1;
    enabled        = true;
    state          = 0;
    data           = 0.0f;
    __enable_irq( );
}

/**
 *  available
 *
 *  @return true if data is ready else false
 */
bool AnalyzeNoteFrequency::available( void ) {
    __disable_irq( );
    bool flag = new_output;
    if ( flag ) new_output = false;
    __enable_irq( );
    return flag;
}

/**
 *  read processes the data samples for the Yin algorithm.
 *
 *  @return frequency in hertz
 */
float AnalyzeNoteFrequency::read( void ) {
    __disable_irq( );
    float d = data;
    __enable_irq( );
    return sample_rate / d;
}

/**
 *  Periodicity of the sampled signal from Yin algorithm from read function.
 *
 *  @return periodicity
 */
float AnalyzeNoteFrequency::probability( void ) {
    __disable_irq( );
    float p = periodicity;
    __enable_irq( );
    return p;
}

/**
 *  Initialise parameters.
 *
 *  @param thresh    Allowed uncertainty
 */
void AnalyzeNoteFrequency::threshold( float p ) {
    __disable_irq( );
    yin_threshold = p;
    __enable_irq( );
}

/*
 *  (C) 2019 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of State Variable Filter VST plugin.
 *
 *  State Variable Filter VST plugin is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  State Variable Filter VST plugin is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with State Variable Filter VST plugin.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cmath>
#include "fir.h"

// constructor
FIRLowpass::FIRLowpass(double newSamplerate, double newCutoff, int newOrder)
{
  // initialize filter design parameters
  samplerate = newSamplerate;
  cutoff = newCutoff;
  order = newOrder;
  
  // allocate dsp vectors
  h_d = new double[order];
  h = new double[order];
  w = new double[order];

  // initialize ring buffer delay line
  ringBufferIndex = 0;
  ringBuffer = new double[order];

  for(int n=0; n<order; n++){
    ringBuffer[n] = 0.0;
  }

  // compute impulse response
  ComputeImpulseResponse();
}

// default constructor
FIRLowpass::FIRLowpass()
{
  // set default design parameters
  samplerate=(double)(44100.0);
  cutoff=(double)(440.0);
  order=128;
  
  // allocate dsp vectors
  h_d = new double[order];
  h = new double[order];
  w = new double[order];

  // initialize ring buffer delay line
  ringBufferIndex = 0;
  ringBuffer = new double[order];

  for(int n=0; n<order; n++){
    ringBuffer[n] = 0.0;
  }

  // compute impulse response
  ComputeImpulseResponse();
}

// destructor
FIRLowpass::~FIRLowpass(){
  // free dsp vectors
  delete[] h_d;
  delete[] h;
  delete[] w;
}

void FIRLowpass::SetFilterOrder(int newOrder){
  order = newOrder;

  // free dsp vectors
  delete[] h_d;
  delete[] h;
  delete[] w;
  
  // allocate dsp vectors
  h_d = new double[order];
  h = new double[order];
  w = new double[order];

  // compute new impulse response
  ComputeImpulseResponse();

  // initialize ring buffer delay line
  ringBufferIndex = 0;
  ringBuffer = new double[order];

  for(int n=0; n<order; n++){
    ringBuffer[n] = 0.0;
  }
}

void FIRLowpass::SetFilterSamplerate(double newSamplerate){
  samplerate = newSamplerate;

  // compute new impulse response
  ComputeImpulseResponse();

  // initialize ring buffer delay line
  ringBufferIndex = 0;
  ringBuffer = new double[order];

  for(int n=0; n<order; n++){
    ringBuffer[n] = 0.0;
  }
}

void FIRLowpass::SetFilterCutoff(double newCutoff){
  cutoff = newCutoff;

  // compute new impulse response
  ComputeImpulseResponse();

  // initialize ring buffer delay line
  ringBufferIndex = 0;
  ringBuffer = new double[order];

  for(int n=0; n<order; n++){
    ringBuffer[n] = 0.0;
  }
}

double* FIRLowpass::GetImpulseResponse(){
  return h;
}

double FIRLowpass::FIRfilter(double input){
  // update delay line
  ringBuffer[ringBufferIndex++] = input;
  if(ringBufferIndex > (order - 1)){
    ringBufferIndex -= order;
  }

  // compute convolution
  double output = 0.0;
  int ii;
  for(int n = 0; n < order; n++){
    // wrap around index
    ii = (ringBufferIndex - (n + 1));
    if(ii < 0)
      ii += order;
    
    // multiply and accumulate
    output += h[n] * ringBuffer[ii]; 
  }

  return output;
}

void FIRLowpass::ComputeImpulseResponse(){
  // index as -M..M
  double ii;
  
  // set cutoff frequency in radians
  omega_c = (double)(cutoff/samplerate) * 2.0 * M_PI;

  // compute truncated ideal impulse response
  for(int n=0; n<order; n++){
    // compute index as -M..M and avoid NaN at impulse peak
    ii = (double)(n) - 1.0 - (double)(floor((double)(order)/2.0)) + 1.0e-9;

    // sample sinc function
    h_d[n] = omega_c * std::sin((double)(omega_c * ii))/(double)(omega_c * ii);
  }

  // compute windowing function
  for(int n=0; n<order; n++){
    // compute index as -M..M and avoid NaN at impulse peak
    ii = (double)(n) - 1.0 - (double)(floor((double)(order) / 2.0)) + 1.0e-9;

    // hanning window function
    w[n] = std::cos(M_PI * ii/(double)(order));
    w[n] *= w[n];
  }

  // compute windowed ideal impulse function
  for(int n=0; n<order; n++){
    // window truncated ideal impulse response
    h[n] = w[n] * h_d[n];
  }
}


/*
 *  (C) 2019 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of Finite Impulse Response Filter VCV Rack plugin.
 *
 *  Finite Impulse Response Filter VCV Rack plugin is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Finite Impulse Response Filter VCV Rack plugin is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Finite Impulse Response Filter VCV Rack plugin.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __dspfirh__
#define __dspfirh__

class FIRLowpass{
public:
  // constructor/destructor
  FIRLowpass(double newSamplerate, double newCutoff, int newOrder);
  FIRLowpass();
  ~FIRLowpass();

  // set filter parameters
  void SetFilterOrder(int newOrder);
  void SetFilterSamplerate(double newSamplerate);
  void SetFilterCutoff(double newCutoff);
  
  // FIR filter signal 
  double FIRfilter(double input);

  // get impulse response
  double* GetImpulseResponse();
  
private:
  // compute windowed ideal impulse response
  void ComputeImpulseResponse();

  // filter design variables
  double samplerate;
  double cutoff;
  int order;
  
  // dsp variables
  double omega_c;
  double *h_d;
  double *h;
  double *w;

  // ring buffer delay line
  double *ringBuffer;
  int ringBufferIndex;
};

#endif

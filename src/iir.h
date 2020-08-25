/*
 *  (C) 2019 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of Infinite Impulse Response Filter VCV Rack plugin.
 *
 *  Infinite Impulse Response Filter VCV Rack plugin is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Infinite Impulse Response Filter VCV Rack plugin is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Infinite Impulse Response Filter VCV Rack plugin.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __dspiirh__
#define __dspiirh__

class IIRLowpass{
public:
  // constructor/destructor
  IIRLowpass(double newSamplerate, double newCutoff, int newOrder);
  IIRLowpass();
  ~IIRLowpass();

  // set filter parameters
  void SetFilterOrder(int newOrder);
  void SetFilterSamplerate(double newSamplerate);
  void SetFilterCutoff(double newCutoff);

  // initialize biquad cascade delayline
  void InitializeBiquadCascade();
  
  // IIR filter signal 
  double IIRfilter(double input);

  // get filter coefficients
  double* GetFilterCoeffA1();
  double* GetFilterCoeffA2();
  double* GetFilterCoeffK();
  
private:
  // compute biquad cascade coefficients
  void ComputeCoefficients();

  // filter design variables
  double samplerate;
  double cutoff;
  int order;
  
  // dsp variables
  double *a1;
  double *a2;
  double *K;
  double *pa_real;
  double *pa_imag;
  double *p_real;
  double *p_imag;
  
  // cascaded biquad buffers
  double *z;
};

#endif

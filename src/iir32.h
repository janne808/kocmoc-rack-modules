/*
 *  (C) 2024 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of Kocmoc VCV Rack plugin.
 *
 *  Kocmoc VCV Rack plugin is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Kocmoc VCV Rack plugin is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Kocmoc VCV Rack plugin.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __dspiir32h__
#define __dspiir32h__

class IIRLowpass32{
public:
  // constructor/destructor
  IIRLowpass32(double newSamplerate, double newCutoff, int newOrder);
  IIRLowpass32();
  ~IIRLowpass32();

  // set filter parameters
  void SetFilterOrder(int newOrder);
  void SetFilterSamplerate(double newSamplerate);
  void SetFilterCutoff(double newCutoff);

  // initialize biquad cascade delayline
  void InitializeBiquadCascade();
  
  // IIR filter signal 
  float IIRfilter32(float input);

  // get filter coefficients
  float* GetFilterCoeffA1();
  float* GetFilterCoeffA2();
  float* GetFilterCoeffK();
  
private:
  // compute biquad cascade coefficients
  void ComputeCoefficients();

  // filter design variables
  double samplerate;
  double cutoff;
  int order;
  
  // dsp variables
  float *a1;
  float *a2;
  float *K;
  double *pa_real;
  double *pa_imag;
  double *p_real;
  double *p_imag;
  
  // cascaded biquad buffers
  float *z;
};

#endif
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

#ifndef __dspsvfh__
#define __dspsvfh__

#ifdef FLOATDSP
#include "iir32.h"
#else
#include "iir.h"
#endif

// filter modes
enum SVFFilterMode {
   SVF_LOWPASS_MODE,
   SVF_BANDPASS_MODE,
   SVF_HIGHPASS_MODE
};

// integration methods
enum SVFIntegrationMethod {
   SVF_SEMI_IMPLICIT_EULER,
   SVF_PREDICTOR_CORRECTOR,
   SVF_TRAPEZOIDAL,
   SVF_INV_TRAPEZOIDAL
};

class SVFilter{
public:
  // constructor/destructor
  SVFilter(double newCutoff, double newResonance, int newOversamplingFactor,
	   SVFFilterMode newFilterMode, double newSampleRate,
	   SVFIntegrationMethod newIntegrationMethod, int newDecimatorOrder);
  SVFilter();
  ~SVFilter();

  // set filter parameters
  void SetFilterCutoff(double newCutoff);
  void SetFilterResonance(double newResonance);
  void SetFilterMode(SVFFilterMode newFilterMode);
  void SetFilterSampleRate(double newSampleRate);
  void SetFilterIntegrationMethod(SVFIntegrationMethod method);
  void SetFilterOversamplingFactor(int newOversamplingFactor);
  void SetFilterDecimatorOrder(int decimatorOrder);
    
  // get filter parameters
  double GetFilterCutoff();
  double GetFilterResonance();
  SVFFilterMode GetFilterMode();  
  double GetFilterSampleRate();
  SVFIntegrationMethod GetFilterIntegrationMethod();
  int GetFilterOversamplingFactor();  
  int GetFilterDecimatorOrder();
  
  // tick filter state
#ifdef FLOATDSP
  void filter(float input);
#else
  void filter(double input);
#endif
  
  // get filter responses
  double GetFilterLowpass();
  double GetFilterBandpass();
  double GetFilterHighpass();

  // get filter output
#ifdef FLOATDSP
  float GetFilterOutput();
#else
  double GetFilterOutput();
#endif
  
  // reset state
  void ResetFilterState();
  
private:
  // set integration rate
  void SetFilterIntegrationRate();

  // pade approximant functions for hyperbolic functions
  // filter parameters
  double cutoffFrequency;
  double Resonance;
  SVFFilterMode filterMode;
  SVFIntegrationMethod integrationMethod;
  double dt;
  double sampleRate;
  int oversamplingFactor;
  int decimatorOrder;
  
  // filter state
#ifdef FLOATDSP
  float lp;
  float bp;
  float hp;
  float u_t1;
#else
  double lp;
  double bp;
  double hp;
  double u_t1;
#endif
  
  // filter output
#ifdef FLOATDSP
  float out;
#else
  double out;
#endif
  
  // IIR downsampling filter
#ifdef FLOATDSP
  IIRLowpass32 *iir;
#else
  IIRLowpass *iir;
#endif
};

#endif

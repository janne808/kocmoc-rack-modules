/*
 *  (C) 2020 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of State Variable Filter VCV Rack plugin.
 *
 *  State Variable Filter VCV Rack plugin is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  State Variable Filter VCV Rack plugin is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with State Variable Filter VCV Rack plugin.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __dspsvfh__
#define __dspsvfh__

#include "iir.h"

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
	   SVFFilterMode newFilterMode, double newSampleRate, SVFIntegrationMethod newIntegrationMethod);
  SVFilter();
  ~SVFilter();

  // set filter parameters
  void SetFilterCutoff(double newCutoff);
  void SetFilterResonance(double newResonance);
  void SetFilterOversamplingFactor(int newOversamplingFactor);
  void SetFilterMode(SVFFilterMode newFilterMode);
  void SetFilterSampleRate(double newSampleRate);
  void SetFilterIntegrationMethod(SVFIntegrationMethod method);
  
  // get filter parameters
  double GetFilterCutoff();
  double GetFilterResonance();
  int GetFilterOversamplingFactor();  
  SVFFilterMode GetFilterMode();  
  double GetFilterSampleRate();
  SVFIntegrationMethod GetFilterIntegrationMethod();
  
  // tick filter state
  void filter(double input);

  // get filter responses
  double GetFilterLowpass();
  double GetFilterBandpass();
  double GetFilterHighpass();

  // get filter output
  double GetFilterOutput();

  // reset state
  void ResetFilterState();
  
private:
  // set integration rate
  void SetFilterIntegrationRate();

  // pade approximant functions for hyperbolic functions
  // filter parameters
  double cutoffFrequency;
  double Resonance;
  int oversamplingFactor;
  SVFFilterMode filterMode;
  double sampleRate;
  double dt;
  SVFIntegrationMethod integrationMethod;
  
  // filter state
  double lp;
  double bp;
  double hp;
  double u_t1;
  
  // filter output
  double out;

  // IIR downsampling filter
  IIRLowpass *iir;
};

#endif

/*
 *  (C) 2020 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of Sallen-Key Filter VCV Rack plugin.
 *
 *  Sallen-Key Filter VCV Rack plugin is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Sallen-Key Filter VCV Rack plugin is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Sallen-Key Filter VCV Rack plugin.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __dspskfh__
#define __dspskfh__

#include "iir.h"

// filter modes
enum SKFilterMode {
   SK_LOWPASS_MODE,
   SK_BANDPASS_MODE,
   SK_HIGHPASS_MODE
};

// integration methods
enum SKIntegrationMethod {
   SK_SEMI_IMPLICIT_EULER,
   SK_PREDICTOR_CORRECTOR,
   SK_TRAPEZOIDAL
};

class SKFilter{
public:
  // constructor/destructor
  SKFilter(double newCutoff, double newResonance, int newOversamplingFactor,
	   SKFilterMode newFilterMode, double newSampleRate, SKIntegrationMethod newIntegrationMethod);
  SKFilter();
  ~SKFilter();

  // set filter parameters
  void SetFilterCutoff(double newCutoff);
  void SetFilterResonance(double newResonance);
  void SetFilterOversamplingFactor(int newOversamplingFactor);
  void SetFilterMode(SKFilterMode newFilterMode);
  void SetFilterSampleRate(double newSampleRate);
  void SetFilterIntegrationMethod(SKIntegrationMethod method);
  
  // get filter parameters
  double GetFilterCutoff();
  double GetFilterResonance();
  int GetFilterOversamplingFactor();  
  SKFilterMode GetFilterMode();  
  double GetFilterSampleRate();
  SKIntegrationMethod GetFilterIntegrationMethod();
  
  // tick filter state
  void filter(double input);

  // set filter inputs
  void SetFilterLowpassInput(double input);
  void SetFilterBandpassInput(double input);
  void SetFilterHighpassInput(double input);

  // get filter output
  double GetFilterOutput();

  // reset state
  void ResetFilterState();
  
private:
  // set integration rate
  void SetFilterIntegrationRate();

  // filter parameters
  double cutoffFrequency;
  double Resonance;
  int oversamplingFactor;
  SKFilterMode filterMode;
  double sampleRate;
  double dt;
  SKIntegrationMethod integrationMethod;
  
  // filter state
  double p0;
  double p1;

  // filter input
  double input_lp;
  double input_bp;
  double input_hp;
  double input_lp_t1;
  double input_bp_t1;
  double input_hp_t1;
  
  // filter output
  double out;

  // IIR downsampling filter
  IIRLowpass *iir;
};

#endif

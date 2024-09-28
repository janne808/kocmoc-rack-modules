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

#ifndef __dspladderh__
#define __dspladderh__

#ifdef FLOATDSP
#include "iir32.h"
#else
#include "iir.h"
#endif

// filter modes
enum LadderFilterMode {
   LADDER_LOWPASS_MODE,
   LADDER_BANDPASS_MODE,
   LADDER_HIGHPASS_MODE
};

// integration methods
enum LadderIntegrationMethod {
   LADDER_EULER_FULL_TANH,
   LADDER_PREDICTOR_CORRECTOR_FULL_TANH,
   LADDER_PREDICTOR_CORRECTOR_FEEDBACK_TANH,
   LADDER_TRAPEZOIDAL_FEEDBACK_TANH
};

class Ladder{
public:
  // constructor/destructor
  Ladder(double newCutoff, double newResonance, int newOversamplingFactor,
	 LadderFilterMode newFilterMode, double newSampleRate,
	 LadderIntegrationMethod newIntegrationMethod, int newDecimatorOrder);
  Ladder();
  ~Ladder();

  // set filter parameters
  void SetFilterCutoff(double newCutoff);
  void SetFilterResonance(double newResonance);
  void SetFilterMode(LadderFilterMode newFilterMode);
  void SetFilterSampleRate(double newSampleRate);
  void SetFilterIntegrationMethod(LadderIntegrationMethod method);
  void SetFilterOversamplingFactor(int newOversamplingFactor);
  void SetFilterDecimatorOrder(int decimatorOrder);
  
  // get filter parameters
  double GetFilterCutoff();
  double GetFilterResonance();
  LadderFilterMode GetFilterMode();  
  double GetFilterSampleRate();
  LadderIntegrationMethod GetFilterIntegrationMethod();
  int GetFilterOversamplingFactor();  
  int GetFilterDecimatorOrder();
  
  // tick filter state
#ifdef FLOATDSP
  void LadderFilter(float input);
#else
  void LadderFilter(double input);
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

  // filter parameters
  double cutoffFrequency;
  double Resonance;
  LadderFilterMode filterMode;
  double sampleRate;
  double dt;
  LadderIntegrationMethod integrationMethod;
  int oversamplingFactor;
  int decimatorOrder;
  
  // filter state
#ifdef FLOATDSP
  float p0, p1, p2, p3;
  float ut_1;
#else
  double p0, p1, p2, p3;
  double ut_1;
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

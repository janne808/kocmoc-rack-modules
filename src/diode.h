/*
 *  (C) 2025 Janne Heikkarainen <janne808@radiofreerobotron.net>
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

#ifndef __dspdiodeh__
#define __dspdiodeh__

#ifdef FLOATDSP
#include "iir32.h"
#else
#include "iir.h"
#endif

// filter modes
enum DiodeFilterMode {
   DIODE_LOWPASS4_MODE,
   DIODE_LOWPASS2_MODE,
};

// integration methods
enum DiodeIntegrationMethod {
   DIODE_EULER_FULL_TANH,
   DIODE_PREDICTOR_CORRECTOR_FULL_TANH,
};

class Diode{
public:
  // constructor/destructor
  Diode(double newCutoff, double newResonance, int newOversamplingFactor,
	 DiodeFilterMode newFilterMode, double newSampleRate,
	 DiodeIntegrationMethod newIntegrationMethod, int newDecimatorOrder);
  Diode();
  ~Diode();

  // set filter parameters
  void SetFilterCutoff(double newCutoff);
  void SetFilterResonance(double newResonance);
  void SetFilterMode(DiodeFilterMode newFilterMode);
  void SetFilterSampleRate(double newSampleRate);
  void SetFilterIntegrationMethod(DiodeIntegrationMethod method);
  void SetFilterOversamplingFactor(int newOversamplingFactor);
  void SetFilterDecimatorOrder(int decimatorOrder);
  
  // get filter parameters
  double GetFilterCutoff();
  double GetFilterResonance();
  DiodeFilterMode GetFilterMode();  
  double GetFilterSampleRate();
  DiodeIntegrationMethod GetFilterIntegrationMethod();
  int GetFilterOversamplingFactor();  
  int GetFilterDecimatorOrder();
  
  // tick filter state
#ifdef FLOATDSP
  void DiodeFilter(float input);
#else
  void DiodeFilter(double input);
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
  DiodeFilterMode filterMode;
  double sampleRate;
  double dt;
  double dt_hp, dt_hp2;
  DiodeIntegrationMethod integrationMethod;
  int oversamplingFactor;
  int decimatorOrder;
  
  // filter state
#ifdef FLOATDSP
  float p0, p1, p2, p3;
  float ut_1;
  float hp0, hp1, hp2, hp3, hp4, hp5, hp6, hp7;
#else
  double p0, p1, p2, p3;
  double ut_1;
  double hp0, hp1, hp2, hp3, hp4, hp5, hp6, hp7;
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

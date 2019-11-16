/*
 *  (C) 2019 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of Ladder Filter VCV Rack plugin.
 *
 *  Ladder Filter VCV Rack plugin is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Ladder Filter VCV Rack plugin is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Ladder Filter VCV Rack plugin.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __dspladderh__
#define __dspladderh__

#include "fir.h"

class Ladder{
public:
  // constructor/destructor
  Ladder(double newCutoff, double newResonance, int newOversamplingFactor,
      int newFilterMode, double newSampleRate, int newIntegrationMethod);
  Ladder();
  ~Ladder();

  // set filter parameters
  void SetFilterCutoff(double newCutoff);
  void SetFilterResonance(double newResonance);
  void SetFilterOversamplingFactor(int newOversamplingFactor);
  void SetFilterMode(int newFilterMode);
  void SetFilterSampleRate(double newSampleRate);
  void SetFilterIntegrationMethod(int method);
  
  // get filter parameters
  double GetFilterCutoff();
  double GetFilterResonance();
  int GetFilterOversamplingFactor();  
  int GetFilterMode();  
  double GetFilterSampleRate();
  int GetFilterIntegrationMethod();
  
  // tick filter state
  void LadderFilter(double input);

  // get filter responses
  double GetFilterLowpass();
  double GetFilterBandpass();
  double GetFilterHighpass();

  // get filter output
  double GetFilterOutput();

  // reset state
  void ResetFilterState();
  
private:
  // methods
  void SetFilterIntegrationRate();

  // filter parameters
  double cutoffFrequency;
  double Resonance;
  int oversamplingFactor;
  int filterMode;
  double sampleRate;
  double dt;
  int integrationMethod;
  
  // filter state
  double p0, p1, p2, p3;
  double ut_1;
  
  // filter output
  double out;

  // FIR downsampling filter
  FIRLowpass *fir;
};

#endif

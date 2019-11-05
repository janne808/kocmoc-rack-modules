/*
 *  (C) 2019 Janne Heikkarainen <janne808@radiofreerobotron.net>
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

#include "fir.h"

class SVF{
public:
  // constructor/destructor
  SVF(double newCutoff, double newResonance, int newOversamplingFactor, int newFilterMode, double newSampleRate);
  SVF();
  ~SVF();

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
  void SVFfilter(double input);

  // get filter responses
  double GetFilterLowpass();
  double GetFilterBandpass();
  double GetFilterHighpass();

  // get filter output
  double GetFilterOutput();
  
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
  double lp;
  double bp;
  double hp;
  double u_t1;
  
  // filter output
  double out;

  // FIR downsampling filter
  FIRLowpass *fir;
};

#endif

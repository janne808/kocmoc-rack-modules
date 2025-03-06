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

#include <cstdlib>
#include <cmath>
#include "diode.h"

#ifdef FLOATDSP
#include "iir32.h"
#else
#include "iir.h"
#endif

#include "fastmath.h"

// steepness of downsample filter response
#define IIR_DOWNSAMPLE_ORDER 16

// downsampling passthrough bandwidth
#define IIR_DOWNSAMPLING_BANDWIDTH 0.9

// maximum newton-raphson iteration steps
#define LADDER_MAX_NEWTON_STEPS 8

// check for newton-raphson breaking limit
#define DIODE_NEWTON_BREAKING_LIMIT 1

// constructor
Diode::Diode(double newCutoff, double newResonance, int newOversamplingFactor,
	     DiodeFilterMode newFilterMode, double newSampleRate,
	     DiodeIntegrationMethod newIntegrationMethod, int newDecimatorOrder){
  // initialize filter parameters
  cutoffFrequency = newCutoff;
  Resonance = newResonance;
  filterMode = newFilterMode;
  sampleRate = newSampleRate;
  oversamplingFactor = newOversamplingFactor;
  decimatorOrder = newDecimatorOrder;
  
  SetFilterIntegrationRate();

  // initialize filter state
  p0 = p1 = p2 = p3 = out = ut_1 = 0.0f;
  
  integrationMethod = newIntegrationMethod;
  
  // instantiate downsampling filter
#ifdef FLOATDSP
  iir = new IIRLowpass32(sampleRate * oversamplingFactor, IIR_DOWNSAMPLING_BANDWIDTH * sampleRate / 2.0, decimatorOrder);
#else
  iir = new IIRLowpass(sampleRate * oversamplingFactor, IIR_DOWNSAMPLING_BANDWIDTH * sampleRate / 2.0, decimatorOrder);
#endif
}

// default constructor
Diode::Diode(){
  // initialize filter parameters
  cutoffFrequency = 0.25;
  Resonance = 0.5;
  filterMode = DIODE_LOWPASS4_MODE;
  sampleRate = 44100.0;
  oversamplingFactor = 2;
  decimatorOrder = IIR_DOWNSAMPLE_ORDER;
  
  SetFilterIntegrationRate();
  
  // initialize filter state
  p0 = p1 = p2 = p3 = out = ut_1 = 0.0f;
  
  integrationMethod = DIODE_PREDICTOR_CORRECTOR_FULL_TANH;
  
  // instantiate downsampling filter
#ifdef FLOATDSP
  iir = new IIRLowpass32(sampleRate * oversamplingFactor, IIR_DOWNSAMPLING_BANDWIDTH * sampleRate / 2.0, decimatorOrder);
#else
  iir = new IIRLowpass(sampleRate * oversamplingFactor, IIR_DOWNSAMPLING_BANDWIDTH * sampleRate / 2.0, decimatorOrder);
#endif
}

// default destructor
Diode::~Diode(){
  delete iir;
}

void Diode::ResetFilterState(){
  // initialize filter parameters
  cutoffFrequency = 0.25;
  Resonance = 0.0;

  SetFilterIntegrationRate();
  
  // initialize filter state
  p0 = p1 = p2 = p3 = out = ut_1 = 0.0f;
  
  // set oversampling
  iir->SetFilterSamplerate(sampleRate * oversamplingFactor);
  iir->SetFilterCutoff(IIR_DOWNSAMPLING_BANDWIDTH * sampleRate / 2.0);
  iir->SetFilterOrder(decimatorOrder);  
}

void Diode::SetFilterCutoff(double newCutoff){
  cutoffFrequency = newCutoff;

  SetFilterIntegrationRate();
}

void Diode::SetFilterResonance(double newResonance){
  Resonance = newResonance;
}

void Diode::SetFilterMode(DiodeFilterMode newFilterMode){
  filterMode = newFilterMode;
}

void Diode::SetFilterSampleRate(double newSampleRate){
  sampleRate = newSampleRate;
  iir->SetFilterSamplerate(sampleRate * (double)(oversamplingFactor));
  iir->SetFilterCutoff(IIR_DOWNSAMPLING_BANDWIDTH*sampleRate / 2.0);
  iir->SetFilterOrder(decimatorOrder);

  SetFilterIntegrationRate();
}

void Diode::SetFilterIntegrationMethod(DiodeIntegrationMethod method){
  integrationMethod = method;
}

void Diode::SetFilterOversamplingFactor(int newOversamplingFactor){
  oversamplingFactor = newOversamplingFactor;
  iir->SetFilterSamplerate(sampleRate * oversamplingFactor);
  iir->SetFilterCutoff(IIR_DOWNSAMPLING_BANDWIDTH * sampleRate / 2.0);
  iir->SetFilterOrder(decimatorOrder);

  SetFilterIntegrationRate();
}

void Diode::SetFilterDecimatorOrder(int newDecimatorOrder){
  decimatorOrder = newDecimatorOrder;
  iir->SetFilterOrder(decimatorOrder);
}

void Diode::SetFilterIntegrationRate(){
  // normalize cutoff freq to samplerate
  dt = 44100.0f / (sampleRate * oversamplingFactor) * cutoffFrequency;

  // clamp integration rate
  if(dt < 0.0f){
    dt = 0.0f;
  }
  else if(dt > 0.865f){
    dt = 0.865f;
  }
}

double Diode::GetFilterCutoff(){
  return cutoffFrequency;
}

double Diode::GetFilterResonance(){
  return Resonance;
}

int Diode::GetFilterOversamplingFactor(){
  return oversamplingFactor;
}

int Diode::GetFilterDecimatorOrder(){
  return decimatorOrder;
}

#ifdef FLOATDSP
float Diode::GetFilterOutput(){
  return out;
}
#else
double Diode::GetFilterOutput(){
  return out;
}
#endif

DiodeFilterMode Diode::GetFilterMode(){
  return filterMode;
}

double Diode::GetFilterSampleRate(){
  return sampleRate;
}

DiodeIntegrationMethod Diode::GetFilterIntegrationMethod(){
  return integrationMethod;
}

#ifdef FLOATDSP
void Diode::DiodeFilter(float input){
  // noise term
  float noise;

  // feedback amount
  float fb = 48.0 * Resonance;

  // update noise terms
  noise = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
  noise = 1.0e-6 * 2.0 * (noise - 0.5);

  input += noise;
  
  // integrate filter state
  // with oversampling
  for(int nn = 0; nn < oversamplingFactor; nn++){
    // switch integration method
    switch(integrationMethod){
    case DIODE_EULER_FULL_TANH:
      // semi-implicit euler integration
      // with full tanh stages
      {
	p0 = p0 + dt * (TanhPade32(input - fb * p3) - TanhPade32(p0 - p1));
	p1 = p1 + 0.5 * dt * (TanhPade32(p0 - p1) - TanhPade32(p1 - p2));
	p2 = p2 + 0.5 * dt * (TanhPade32(p1 - p2) - TanhPade32(p2 - p3));
	p3 = p3 + 0.5 * dt * (TanhPade32(p2 - p3) - TanhPade32(p3));
      }
      break;
      
    case DIODE_PREDICTOR_CORRECTOR_FULL_TANH:
      // predictor-corrector integration
      // with full tanh stages
      {
	float p0_prime, p1_prime, p2_prime, p3_prime, p3t_1;

	// predictor
	p0_prime = p0 + dt * (TanhPade32(ut_1 - fb * p3) - TanhPade32(p0 - p1));
	p1_prime = p1 + 0.5 * dt * (TanhPade32(p0 - p1) - TanhPade32(p1 - p2));
	p2_prime = p2 + 0.5 * dt * (TanhPade32(p1 - p2) - TanhPade32(p2 - p3));
	p3_prime = p3 + 0.5 * dt * (TanhPade32(p2 - p3) - TanhPade32(p3));

	// corrector
	p3t_1 = p3;
	p3 = p3 + 0.5 * dt * ((TanhPade32(p2 - p3) - TanhPade32(p3)) + (TanhPade32(p2_prime - p3_prime) - TanhPade32(p3_prime)));
	p2 = p2 + 0.5 * 0.5 * dt * ((TanhPade32(p1 - p2) - TanhPade32(p2 - p3)) + (TanhPade32(p1_prime - p2_prime) - TanhPade32(p2_prime - p3_prime)));
	p1 = p1 + 0.5 * 0.5 * dt * ((TanhPade32(p0 - p1) - TanhPade32(p1 - p2)) + (TanhPade32(p0_prime - p1_prime) - TanhPade32(p1_prime - p2_prime)));
	p0 = p0 + 0.5 * 0.5 * dt * ((TanhPade32(ut_1 - fb * p3t_1) - TanhPade32(p0 - p1)) + (TanhPade32(input - fb * p3) - TanhPade32(p0_prime - p1_prime)));
      }
      break;
      
    default:
      break;
    }

    // input at t-1
    ut_1 = input;

    //switch filter mode
    switch(filterMode){
    case DIODE_LOWPASS4_MODE:
      out = p3;
      break;
    case DIODE_LOWPASS2_MODE:
      out = 0.5f * p1;
      break;
    default:
      out = 0.0;
    }

    // downsampling filter
    if(oversamplingFactor > 1){
      out = iir->IIRfilter(out);
    }
  }
}
#else
void Diode::DiodeFilter(double input){
  // noise term
  double noise;

  // feedback amount
  double fb = 48.0 * Resonance;

  // update noise terms
  noise = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
  noise = 1.0e-6 * 2.0 * (noise - 0.5);

  input += noise;
  
  // integrate filter state
  // with oversampling
  for(int nn = 0; nn < oversamplingFactor; nn++){
    // switch integration method
    switch(integrationMethod){
    case DIODE_EULER_FULL_TANH:
      // semi-implicit euler integration
      // with full tanh stages
      {
	p0 = p0 + dt * (TanhPade32(input - fb * p3) - TanhPade32(p0 - p1));
	p1 = p1 + 0.5 * dt * (TanhPade32(p0 - p1) - TanhPade32(p1 - p2));
	p2 = p2 + 0.5 * dt * (TanhPade32(p1 - p2) - TanhPade32(p2 - p3));
	p3 = p3 + 0.5 * dt * (TanhPade32(p2 - p3) - TanhPade32(p3));
      }
      break;
      
    case DIODE_PREDICTOR_CORRECTOR_FULL_TANH:
      // predictor-corrector integration
      // with full tanh stages
      {
	double p0_prime, p1_prime, p2_prime, p3_prime, p3t_1;

	// predictor
	p0_prime = p0 + dt * (TanhPade32(ut_1 - fb * p3) - TanhPade32(p0 - p1));
	p1_prime = p1 + 0.5 * dt * (TanhPade32(p0 - p1) - TanhPade32(p1 - p2));
	p2_prime = p2 + 0.5 * dt * (TanhPade32(p1 - p2) - TanhPade32(p2 - p3));
	p3_prime = p3 + 0.5 * dt * (TanhPade32(p2 - p3) - TanhPade32(p3));

	// corrector
	p3t_1 = p3;
	p3 = p3 + 0.5 * dt * ((TanhPade32(p2 - p3) - TanhPade32(p3)) + (TanhPade32(p2_prime - p3_prime) - TanhPade32(p3_prime)));
	p2 = p2 + 0.5 * 0.5 * dt * ((TanhPade32(p1 - p2) - TanhPade32(p2 - p3)) + (TanhPade32(p1_prime - p2_prime) - TanhPade32(p2_prime - p3_prime)));
	p1 = p1 + 0.5 * 0.5 * dt * ((TanhPade32(p0 - p1) - TanhPade32(p1 - p2)) + (TanhPade32(p0_prime - p1_prime) - TanhPade32(p1_prime - p2_prime)));
	p0 = p0 + 0.5 * 0.5 * dt * ((TanhPade32(ut_1 - fb * p3t_1) - TanhPade32(p0 - p1)) + (TanhPade32(input - fb * p3) - TanhPade32(p0_prime - p1_prime)));
      }
      break;
      
    default:
      break;
    }

    // input at t-1
    ut_1 = input;

    //switch filter mode
    switch(filterMode){
    case DIODE_LOWPASS4_MODE:
      out = p3;
      break;
    case DIODE_LOWPASS2_MODE:
      out = 0.5 * p1;
      break;
    default:
      out = 0.0;
    }

    // downsampling filter
    if(oversamplingFactor > 1){
      out = iir->IIRfilter(out);
    }
  }
}
#endif

double Diode::GetFilterLowpass(){
  return p3;
}

double Diode::GetFilterBandpass(){
  return 0.0;
}

double Diode::GetFilterHighpass(){
  return 0.0;
}



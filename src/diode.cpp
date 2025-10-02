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
#define IIR_DOWNSAMPLING_BANDWIDTH 0.725

// maximum newton-raphson iteration steps
#define DIODE_MAX_NEWTON_STEPS 8

// check for newton-raphson breaking limit
#define DIODE_NEWTON_BREAKING_LIMIT 1

// thermal phase noise amplitude
#define DIODE_THERMAL_NOISE_AMPLITUDE 1.0e-2

// feedback DC decoupling integration rate
#define DIODE_FEEDBACK_DC_DECOUPLING_INTEGRATION_RATE 0.002

// output DC decoupling integration rate
#define DIODE_OUTPUT_DC_DECOUPLING_INTEGRATION_RATE 0.008

// maximum integration rate
#define DIODE_MAX_INTEGRATION_RATE 0.9

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

  hp0 = hp1 = hp2 = hp3 = hp4 = hp5 = hp6 = hp7 = 0.0;
  
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

  hp0 = hp1 = hp2 = hp3 = hp4 = hp5 = hp6 = hp7 = 0.0;
  
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
  p0 = p1 = p2 = p3 = out = ut_1 = 0.0;

  hp0 = hp1 = hp2 = hp3 = hp4 = hp5 = hp6 = hp7 = 0.0;
  
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
  dt = 44100.0 / (sampleRate * oversamplingFactor) * cutoffFrequency;

  // clamp integration rate
  if(dt < 0.0){
    dt = 0.0;
  }
  else if(dt > DIODE_MAX_INTEGRATION_RATE){
    dt = DIODE_MAX_INTEGRATION_RATE;
  }

  dt_hp = 44100.0 / (sampleRate * oversamplingFactor) * DIODE_FEEDBACK_DC_DECOUPLING_INTEGRATION_RATE;
  dt_hp2 = 44100.0 / (sampleRate * oversamplingFactor) * DIODE_OUTPUT_DC_DECOUPLING_INTEGRATION_RATE;
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
  float fb = 24.f * Resonance;

  // update noise terms
  noise = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
  noise = 1.0e-6f * 2.f * (noise - 0.5f);

  input += noise;
  
  // phase noise
  float theta_0 = 2.0f * (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f);
  float theta_1 = 2.0f * (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f);
  float theta_2 = 2.0f * (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f);
  float theta_3 = 2.0f * (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f);
  
  // inject thermal phase noise to filter stages
  float alpha_0 = 1.0f + DIODE_THERMAL_NOISE_AMPLITUDE * theta_0;
  float alpha_1 = 1.0f + DIODE_THERMAL_NOISE_AMPLITUDE * theta_1;
  float alpha_2 = 1.0f + DIODE_THERMAL_NOISE_AMPLITUDE * theta_2;
  float alpha_3 = 1.0f + DIODE_THERMAL_NOISE_AMPLITUDE * theta_3;
  
  // integrate filter state
  // with oversampling
  for(int nn = 0; nn < oversamplingFactor; nn++){
    // switch integration method
    switch(integrationMethod){
    case DIODE_EULER_FULL_TANH:
      // semi-implicit euler integration
      // with full tanh stages
      {
	p0 = p0 + alpha_0 * dt * (FloatTanhPade45(input - fb * hp3) - FloatTanhPade45(p0 - p1));
	p1 = p1 + alpha_1 * 0.5f * dt * (FloatTanhPade45(p0 - p1) - FloatTanhPade45(p1 - p2));
	p2 = p2 + alpha_2 * 0.5f * dt * (FloatTanhPade45(p1 - p2) - FloatTanhPade45(p2 - p3));
	p3 = p3 + alpha_3 * 0.5f * dt * (FloatTanhPade45(p2 - p3) - FloatTanhPade45(p3));

	hp0 = hp0 + dt_hp * (p3 - hp0);
	hp1 = p3 - hp0;
	
	hp2 = hp2 + dt_hp * (hp1 - hp2);
	hp3 = hp1 - hp2;

	hp6 = hp6 + dt_hp2 * (p1 - hp6);
	hp7 = p1 - hp6;	
      }
      break;
      
    case DIODE_PREDICTOR_CORRECTOR_FULL_TANH:
      // predictor-corrector integration
      // with full tanh stages
      {
	float p0_prime, p1_prime, p2_prime, p3_prime;
	float p0_new, p1_new, p2_new, p3_new;
	float hp0_prime, hp1_prime, hp2_prime, hp3_prime;
	float hp0_new, hp1_new, hp2_new, hp3_new;
	
	// euler step nonlinearities
	float tanh_ut1_fb_hp3 = FloatTanhPade45(ut_1 - fb * hp3);
	float tanh_p0_p1 = FloatTanhPade45(p0 - p1);
	float tanh_p1_p2 = FloatTanhPade45(p1 - p2);
	float tanh_p2_p3 = FloatTanhPade45(p2 - p3);
	float tanh_p3 = FloatTanhPade45(p3);
	
	// predictor
	p0_prime = p0 + alpha_0 * dt * (tanh_ut1_fb_hp3 - tanh_p0_p1);
	p1_prime = p1 + alpha_1 * 0.5f * dt * (tanh_p0_p1 - tanh_p1_p2);
	p2_prime = p2 + alpha_2 * 0.5f * dt * (tanh_p1_p2 - tanh_p2_p3);
	p3_prime = p3 + alpha_3 * 0.5f * dt * (tanh_p2_p3 - tanh_p3);

	hp0_prime = hp0 + dt_hp * (p3 - hp0);
	hp1_prime = p3_prime - hp0_prime;
	hp2_prime = hp2 + dt_hp * (hp1 - hp2);
	hp3_prime = hp1_prime - hp2_prime;

	// trapezoidal step nonlinearities
	float tanh_input_fb_hp3_prime = FloatTanhPade45(input - fb * hp3_prime);
	float tanh_p0_prime_p1_prime = FloatTanhPade45(p0_prime - p1_prime);
	float tanh_p1_prime_p2_prime = FloatTanhPade45(p1_prime - p2_prime);
	float tanh_p2_prime_p3_prime = FloatTanhPade45(p2_prime - p3_prime);
	float tanh_p3_prime = FloatTanhPade45(p3_prime);
	
	// corrector
	p0_new = p0 + alpha_0 * 0.5f * dt * ((tanh_ut1_fb_hp3 - tanh_p0_p1) + (tanh_input_fb_hp3_prime - tanh_p0_prime_p1_prime));
	p1_new = p1 + alpha_1 * 0.5f * 0.5f * dt * ((tanh_p0_p1 - tanh_p1_p2) + (tanh_p0_prime_p1_prime - tanh_p1_prime_p2_prime));
	p2_new = p2 + alpha_2 * 0.5f * 0.5f * dt * ((tanh_p1_p2 - tanh_p2_p3) + (tanh_p1_prime_p2_prime - tanh_p2_prime_p3_prime));
	p3_new = p3 + alpha_3 * 0.5f * 0.5f * dt * ((tanh_p2_p3 - tanh_p3) + (tanh_p2_prime_p3_prime - tanh_p3_prime));

	hp0_new = hp0 + 0.5f * dt_hp * (hp1_prime + (p3_prime - hp0_prime));
	hp1_new = p3_new - hp0_new;
	hp2_new = hp2 + 0.5f * dt_hp * (hp3_prime + (hp1_prime - hp2_prime));
	hp3_new = hp1_new - hp2_new; 

	hp0 = hp0_new;
	hp1 = hp1_new;
	hp2 = hp2_new;
	hp3 = hp3_new;
	
	p0 = p0_new;
	p1 = p1_new;
	p2 = p2_new;
	p3 = p3_new;

	hp6 = hp6 + dt_hp2 * (p1 - hp6);
	hp7 = p1 - hp6;
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
      out = hp1;
      break;
    case DIODE_LOWPASS2_MODE:
      out = 0.25f * hp7;
      break;
    default:
      out = 0.0f;
    }

    // downsampling filter
    if(oversamplingFactor > 1){
      out = iir->IIRfilter32(out);
    }
  }
}
#else
void Diode::DiodeFilter(double input){
  // noise term
  double noise;

  // feedback amount
  double fb = 24.0 * Resonance;

  // update noise terms
  noise = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
  noise = 1.0e-6 * 2.0 * (noise - 0.5);

  input += noise;

  // phase noise
  double theta_0 = 2.0 * (static_cast <double> (rand()) / static_cast <double> (RAND_MAX) - 0.5);
  double theta_1 = 2.0 * (static_cast <double> (rand()) / static_cast <double> (RAND_MAX) - 0.5);
  double theta_2 = 2.0 * (static_cast <double> (rand()) / static_cast <double> (RAND_MAX) - 0.5);
  double theta_3 = 2.0 * (static_cast <double> (rand()) / static_cast <double> (RAND_MAX) - 0.5);
  
  // inject thermal phase noise to filter stages
  double alpha_0 = 1.0 + DIODE_THERMAL_NOISE_AMPLITUDE * theta_0;
  double alpha_1 = 1.0 + DIODE_THERMAL_NOISE_AMPLITUDE * theta_1;
  double alpha_2 = 1.0 + DIODE_THERMAL_NOISE_AMPLITUDE * theta_2;
  double alpha_3 = 1.0 + DIODE_THERMAL_NOISE_AMPLITUDE * theta_3;
  
  // integrate filter state
  // with oversampling
  for(int nn = 0; nn < oversamplingFactor; nn++){
    // switch integration method
    switch(integrationMethod){
    case DIODE_EULER_FULL_TANH:
      // semi-implicit euler integration
      // with full tanh stages
      {
	p0 = p0 + alpha_0 * dt * (TanhPade45(input - fb * hp3) - TanhPade45(p0 - p1));
	p1 = p1 + alpha_1 * 0.5 * dt * (TanhPade45(p0 - p1) - TanhPade45(p1 - p2));
	p2 = p2 + alpha_2 * 0.5 * dt * (TanhPade45(p1 - p2) - TanhPade45(p2 - p3));
	p3 = p3 + alpha_3 * 0.5 * dt * (TanhPade45(p2 - p3) - TanhPade45(p3));

	hp0 = hp0 + dt_hp * (p3 - hp0);
	hp1 = p3 - hp0;
	
	hp2 = hp2 + dt_hp * (hp1 - hp2);
	hp3 = hp1 - hp2;

	hp6 = hp6 + dt_hp2 * (p1 - hp6);
	hp7 = p1 - hp6;	
      }
      break;
      
    case DIODE_PREDICTOR_CORRECTOR_FULL_TANH:
      // predictor-corrector integration
      // with full tanh stages
      {
	double p0_prime, p1_prime, p2_prime, p3_prime;
	double p0_new, p1_new, p2_new, p3_new;
	double hp0_prime, hp1_prime, hp2_prime, hp3_prime;
	double hp0_new, hp1_new, hp2_new, hp3_new;
	
	// euler step nonlinearities
	double tanh_ut1_fb_hp3 = TanhPade32(ut_1 - fb * hp3);
	double tanh_p0_p1 = TanhPade32(p0 - p1);
	double tanh_p1_p2 = TanhPade32(p1 - p2);
	double tanh_p2_p3 = TanhPade32(p2 - p3);
	double tanh_p3 = TanhPade32(p3);
	
	// predictor
	p0_prime = p0 + alpha_0 * dt * (tanh_ut1_fb_hp3 - tanh_p0_p1);
	p1_prime = p1 + alpha_1 * 0.5 * dt * (tanh_p0_p1 - tanh_p1_p2);
	p2_prime = p2 + alpha_2 * 0.5 * dt * (tanh_p1_p2 - tanh_p2_p3);
	p3_prime = p3 + alpha_3 * 0.5 * dt * (tanh_p2_p3 - tanh_p3);

	hp0_prime = hp0 + dt_hp * (p3 - hp0);
	hp1_prime = p3_prime - hp0_prime;
	hp2_prime = hp2 + dt_hp * (hp1 - hp2);
	hp3_prime = hp1_prime - hp2_prime;

	// trapezoidal step nonlinearities
	double tanh_input_fb_hp3_prime = TanhPade32(input - fb * hp3_prime);
	double tanh_p0_prime_p1_prime = TanhPade32(p0_prime - p1_prime);
	double tanh_p1_prime_p2_prime = TanhPade32(p1_prime - p2_prime);
	double tanh_p2_prime_p3_prime = TanhPade32(p2_prime - p3_prime);
	double tanh_p3_prime = TanhPade32(p3_prime);
	
	// corrector
	p0_new = p0 + alpha_0 * 0.5 * dt * ((tanh_ut1_fb_hp3 - tanh_p0_p1) + (tanh_input_fb_hp3_prime - tanh_p0_prime_p1_prime));
	p1_new = p1 + alpha_1 * 0.5 * 0.5 * dt * ((tanh_p0_p1 - tanh_p1_p2) + (tanh_p0_prime_p1_prime - tanh_p1_prime_p2_prime));
	p2_new = p2 + alpha_2 * 0.5 * 0.5 * dt * ((tanh_p1_p2 - tanh_p2_p3) + (tanh_p1_prime_p2_prime - tanh_p2_prime_p3_prime));
	p3_new = p3 + alpha_3 * 0.5 * 0.5 * dt * ((tanh_p2_p3 - tanh_p3) + (tanh_p2_prime_p3_prime - tanh_p3_prime));

	hp0_new = hp0 + 0.5 * dt_hp * (hp1_prime + (p3_prime - hp0_prime));
	hp1_new = p3_new - hp0_new;
	hp2_new = hp2 + 0.5 * dt_hp * (hp3_prime + (hp1_prime - hp2_prime));
	hp3_new = hp1_new - hp2_new; 

	hp0 = hp0_new;
	hp1 = hp1_new;
	hp2 = hp2_new;
	hp3 = hp3_new;
	
	p0 = p0_new;
	p1 = p1_new;
	p2 = p2_new;
	p3 = p3_new;

	hp6 = hp6 + dt_hp2 * (p1 - hp6);
	hp7 = p1 - hp6;
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
      out = hp1;
      break;
    case DIODE_LOWPASS2_MODE:
      out = 0.25 * hp7;
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
  return hp3;
}

double Diode::GetFilterBandpass(){
  return 0.0;
}

double Diode::GetFilterHighpass(){
  return 0.0;
}



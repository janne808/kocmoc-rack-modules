/*
 *  (C) 2019 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of Operator VCV Rack plugin.
 *
 *  Operator VCV Rack plugin is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Operator VCV Rack plugin is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Operator VCV Rack plugin.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cmath>
#include "phasor.h"

// constructor
Phasor::Phasor(double initialPhase, double initialFrequency, double initialSampleRate) {
  phase = initialPhase;
  frequency = initialFrequency;
  sampleRate = initialSampleRate;
  ComputePhaseIncrement();
}

// default constructor
Phasor::Phasor() {
  phase = 0.0;
  frequency = 440.0;
  sampleRate = 44100.0;
  ComputePhaseIncrement();  
}

// destructor
Phasor::~Phasor() {
}

// state tick
void Phasor::Tick() {
  // increment phase
  phase += phaseIncrement;

  // make sure phase stays in [-PI..PI]
  if(phase > M_PI) {
    phase -= 2.0*M_PI;
  }
}

void Phasor::SetPhase(double newPhase) {
  phase = newPhase;
}

void Phasor::SetFrequency(double newFrequency) {
  frequency = newFrequency;
  ComputePhaseIncrement();
}

void Phasor::SetPhaseModulation(double newPhaseModulation) {
  phaseModulation = newPhaseModulation;
}

void Phasor::SetSampleRate(double newSampleRate) {
  sampleRate = newSampleRate;
  ComputePhaseIncrement();
}

double Phasor::GetPhase() {
  return phase + phaseModulation;
}

double Phasor::GetPhaseIncrement() {
  return phaseIncrement;
}

double Phasor::GetFrequency() {
  return frequency;
}

double Phasor::GetSampleRate() {
  return sampleRate;
}

double Phasor::GetPhaseModulation() {
  return phaseModulation;
}

void Phasor::ComputePhaseIncrement(){
  phaseIncrement = 2.0*M_PI*frequency/sampleRate;
}

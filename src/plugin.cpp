/*
 *  (C) 2024 Janne Heikkarainen <janne808@radiofreerobotron.net>
 *
 *  All rights reserved.
 *
 *  This file is part of Kocmoc VCV Rack module.
 *
 *  Kocmoc VCV Rack module is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Kocmoc VCV Rack module is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Kocmoc VCV Rack module.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "plugin.hpp"

Plugin* pluginInstance;

void init(Plugin* p) {
	pluginInstance = p;

	// Add modules here
	// p->addModel(modelMyModule);
	p->addModel(modelSVF_1);
	p->addModel(modeluSVF);
	p->addModel(modelTRG);
	p->addModel(modelLADR);
	p->addModel(modelOP);
	p->addModel(modelPHASR);
	p->addModel(modelMUL);
	p->addModel(modelSKF);
	p->addModel(modelDDLY);
	
	// Any other plugin initialization may go here.
	// As an alternative, consider lazy-loading assets and lookup tables when your module is created to reduce startup times of Rack.
}

/*
 *  (C) 2026 Janne Heikkarainen <janne808@radiofreerobotron.net>
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

#ifndef __kocmocfastrandhh__
#define __kocmocfastrandhh__

#include <cstring>

static uint32_t s = 0x12345678; // seed, must be non-zero

static inline float frand() {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    uint32_t u = (s >> 9) | 0x3F800000u;
    float f;
    memcpy(&f, &u, sizeof f);   // type-pun safely; compiles to zero instructions
    return f - 1.0f;
}

#endif // __kocmocfastrandhh__

/***************************************************************************
 *   Copyright (C) 2021 PCSX-Redux authors                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#pragma once
#include "core/r3000a.h"
#if defined(DYNAREC_AA64)

#include <array>

#include "vixl/src/aarch64/macro-assembler-aarch64.h"
using namespace vixl::aarch64;

// Volatile = caller-saved
// Non-Volatile = callee-saved

const Register contextPointer = x19;
const Register runningPointer = x20;
const int ALLOCATEABLE_REG_COUNT = 19;
const int ALLOCATEABLE_NON_VOLATILE_COUNT = 8;
const std::array<Register, ALLOCATEABLE_REG_COUNT> allocateableRegisters = {w21, w22, w23, w24, w25, w26, w27, w28,
                                                                                 w8, w9, w10, w11, w12, w13, w14, w15,
                                                                                 w16, w17, w18};
const std::array<Register, 11> allocateableVolatiles = {w8, w9, w10, w11, w12, w13, w14, w15, w16, w17, w18};
const std::array<Register, 8> allocateableNonVolatiles = {w21, w22, w23, w24, w25, w26, w27, w28};

const Register arg1 = w0;
const Register arg2 = w1;
const Register arg3 = w2;
const Register arg4 = w3;
const Register scratch = w5;
#define IS_VOLATILE(x) ((x) >= ALLOCATEABLE_NON_VOLATILE_COUNT)
constexpr bool isWindows() { return false; }

#endif  // DYNAREC_AA64

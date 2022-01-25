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
#ifdef DYNAREC_AA64
#include "vixl/src/aarch64/macro-assembler-aarch64.h"
#include <sys/mman.h>

using namespace vixl::aarch64;

// Allocate 32MB for the code cache.
constexpr size_t codeCacheSize = 32 * 1024 * 1024;
constexpr size_t allocSize = codeCacheSize + 0x1000;

// Allocate a bit more memory to be safe.
// This has to be static so JIT code will be close enough to the executable to address stuff with pc-relative accesses
alignas(4096) static uint8_t s_codeCache[allocSize];

class Emitter : public MacroAssembler {
public:
    Emitter() : MacroAssembler(s_codeCache, allocSize) {}

    void L(Label& l) {
        bind(&l);
    }

    template <typename T = void*>
    T getCurr() {
        return GetCursorAddress<T>();
    }

    template <typename T = void*>
    T getCode() {
        return GetBuffer()->GetStartAddress<T>();
    }

    size_t getSize() {
        return GetCursorOffset();
    }
    // TODO: VIXL methods only allow for RW or RE; This will need to be handled manually for M1 Mac regardless
    bool setRWX() {
        // GetBuffer()->SetExecutable
        return mprotect(s_codeCache, allocSize, PROT_READ | PROT_WRITE | PROT_EXEC) != -1;
    }

    void align() {
        GetBuffer()->Align();
    }

    // TODO: Verify. VIXL states this must be called before code in buffer can be executed
    // Default option here is NoFallThrough, which means any code emiited after this is called
    // is ignore.
    void ready() { FinalizeCode(kFallThrough); }

    #define MAKE_CONDITIONAL_BRANCH(properName, alias) \
    void b##properName(Label& l) { b(&l, properName); } \
    void b##alias(Label& l) { b##properName(l); }

    MAKE_CONDITIONAL_BRANCH(ne, nz);
    MAKE_CONDITIONAL_BRANCH(eq, z);
    MAKE_CONDITIONAL_BRANCH(mi, s);
    MAKE_CONDITIONAL_BRANCH(pl, ns);
    MAKE_CONDITIONAL_BRANCH(cs, hs);
    MAKE_CONDITIONAL_BRANCH(cc, lo);
    void bvc(Label& l) { b(&l, vc); }
    void bvs(Label& l) { b(&l, vs); }
    void bhi(Label& l) { b(&l, hi); }
    void bls(Label& l) { b(&l, ls); }
    void bge(Label& l) { b(&l, ge); }
    void blt(Label& l) { b(&l, lt); }
    void bgt(Label& l) { b(&l, gt); }
    void ble(Label& l) { b(&l, le); }
    void bal(Label& l) { b(&l); }

    #undef MAKE_CONDITIONAL_BRANCH

    void dumpBuffer() {
        std::ofstream file("DynarecOutput.dump", std::ios::binary);  // Make a file for our dump
        file.write(getCode<const char*>(), getSize());       // Write the code buffer to the dump
    }

    // Returns a signed integer that shows how many bytes of free space are left in the code buffer
    int64_t getRemainingSize() { return (int64_t)codeCacheSize - (int64_t)getSize(); }

    // Adds "value" to "source" and stores the result in dest
    // Uses add if the value is non-zero, or mov otherwise
    // TODO: Possibly use WRegister here for safety
    void moveAndAdd(vixl::aarch64::Register dest, vixl::aarch64::Register source, uint32_t value) {
        if (value != 0) {
            Add(dest, source, value);
        } else {
            Mov(dest, source);
        }
    }

    // Returns whether dest is equal to value via the zero flag
    void cmpEqImm(vixl::aarch64::Register dest, uint32_t value) {
        if (value == 0) {
            Tst(dest, dest);
        } else {
            Cmp(dest, value);
        }
    }

    // Logical or dest by value (Skip the or if value == 0)
    void orImm(vixl::aarch64::Register dest, uint32_t value) {
        if (value != 0) {
            Orr(dest, dest, value);
        }
    }

    void orImm(vixl::aarch64::Register dest, vixl::aarch64::Register source, uint32_t value) {
        if (value != 0) {
            Orr(dest, source, value);
        }
    }
};

#endif // DYNAREC_AA64

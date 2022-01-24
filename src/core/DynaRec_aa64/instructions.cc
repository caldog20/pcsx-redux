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

#include "recompiler.h"
#if defined(DYNAREC_AA64)
#define BAILZERO(x) \
    if (!(x)) {     \
        return;     \
    }

void DynaRecCPU::recUnknown() {
    PCSX::g_system->message("Unknown instruction for dynarec - address %08x, instruction %08x\n", m_pc, m_psxRegs.code);
    recException(Exception::ReservedInstruction);
}
void DynaRecCPU::recSpecial() {
    const auto func = m_recSPC[m_psxRegs.code & 0x3F];  // Look up the opcode in our decoding LUT
    (*this.*func)();                                    // Jump into the handler to recompile it
}

// The Dynarec doesn't currently handle overflow exceptions, so we treat ADD the same as ADDU
void DynaRecCPU::recADD() { recADDU(); }


void DynaRecCPU::recADDIU() {
    BAILZERO(_Rt_);
    maybeCancelDelayedLoad(_Rt_);

    if (_Rs_ == _Rt_) {
        if (m_regs[_Rt_].isConst()) {
            m_regs[_Rt_].val += _Imm_;
        } else {
            allocateReg(_Rt_);
            m_regs[_Rt_].setWriteback(true);
            gen.Add(m_regs[_Rt_].allocatedReg, m_regs[_Rt_].allocatedReg, _Imm_);

        }
    } else {
        if (m_regs[_Rs_].isConst()) {
            markConst(_Rt_, m_regs[_Rs_].val + _Imm_);
        } else {
            alloc_rs_wb_rt();
            gen.moveAndAdd(m_regs[_Rt_].allocatedReg, m_regs[_Rs_].allocatedReg, _Imm_);
        }
    }
}


void DynaRecCPU::recADDU() {
    BAILZERO(_Rd_);
    maybeCancelDelayedLoad(_Rd_);

    if (m_regs[_Rs_].isConst() && m_regs[_Rt_].isConst()) {
        markConst(_Rd_, m_regs[_Rs_].val + m_regs[_Rt_].val);
    } else if (m_regs[_Rs_].isConst()) {
        alloc_rt_wb_rd();

        if (_Rt_ == _Rd_) {
            gen.Add(m_regs[_Rd_].allocatedReg, m_regs[_Rd_].allocatedReg, m_regs[_Rs_].val);
        } else {
            gen.moveAndAdd(m_regs[_Rd_].allocatedReg, m_regs[_Rt_].allocatedReg, m_regs[_Rs_].val);
        }
    } else if (m_regs[_Rt_].isConst()) {
        alloc_rs_wb_rd();

        if (_Rs_ == _Rd_) {
            gen.Add(m_regs[_Rd_].allocatedReg, m_regs[_Rd_].allocatedReg, m_regs[_Rt_].val);
        } else {
            gen.moveAndAdd(m_regs[_Rd_].allocatedReg, m_regs[_Rs_].allocatedReg, m_regs[_Rt_].val);
        }
    } else {
        alloc_rt_rs_wb_rd();

        if (_Rs_ == _Rd_) {  // Rd+= Rt
            gen.Add(m_regs[_Rd_].allocatedReg, m_regs[_Rd_].allocatedReg, m_regs[_Rt_].allocatedReg);
        } else if (_Rt_ == _Rd_) {  // Rd+= Rs
            gen.Add(m_regs[_Rd_].allocatedReg, m_regs[_Rd_].allocatedReg, m_regs[_Rs_].allocatedReg);
        } else {  // Rd = Rs + Rt
            gen.Add(m_regs[_Rd_].allocatedReg, m_regs[_Rs_].allocatedReg, m_regs[_Rt_].allocatedReg);
        }
    }
}


void DynaRecCPU::recAND() {
    BAILZERO(_Rd_);
    maybeCancelDelayedLoad(_Rd_);

    if (m_regs[_Rs_].isConst() && m_regs[_Rt_].isConst()) {
        markConst(_Rd_, m_regs[_Rs_].val & m_regs[_Rt_].val);
    } else if (m_regs[_Rs_].isConst()) {
        alloc_rt_wb_rd();

        gen.And(m_regs[_Rd_].allocatedReg, m_regs[_Rt_].allocatedReg, m_regs[_Rs_].val);
    } else if (m_regs[_Rt_].isConst()) {
        alloc_rs_wb_rd();

        gen.And(m_regs[_Rd_].allocatedReg, m_regs[_Rs_].allocatedReg, m_regs[_Rt_].val);
    } else {
        alloc_rt_rs_wb_rd();

        if (_Rd_ == _Rs_) {
            gen.And(m_regs[_Rd_].allocatedReg, m_regs[_Rd_].allocatedReg, m_regs[_Rt_].allocatedReg);
        } else if (_Rd_ == _Rt_) {
            gen.And(m_regs[_Rd_].allocatedReg, m_regs[_Rd_].allocatedReg, m_regs[_Rs_].allocatedReg);
        } else {
            gen.And(m_regs[_Rd_].allocatedReg, m_regs[_Rt_].allocatedReg, m_regs[_Rs_].allocatedReg);
        }
    }
}


void DynaRecCPU::recANDI() {
    BAILZERO(_Rt_);
    maybeCancelDelayedLoad(_Rt_);

    if (_Rs_ == _Rt_) {
        if (m_regs[_Rs_].isConst()) {
            m_regs[_Rt_].val &= _ImmU_;
        } else {
            allocateReg(_Rt_);
            m_regs[_Rt_].setWriteback(true);
            gen.And(m_regs[_Rt_].allocatedReg, m_regs[_Rt_].allocatedReg, _ImmU_);
        }
    } else {
        if (m_regs[_Rs_].isConst()) {
            markConst(_Rt_, m_regs[_Rs_].val & _ImmU_);
        } else {
            alloc_rs_wb_rt();
            gen.And(m_regs[_Rt_].allocatedReg, m_regs[_Rs_].allocatedReg, _ImmU_);
        }
    }
}


void DynaRecCPU::recBEQ() {
    const auto target = _Imm_ * 4 + m_pc;
    m_nextIsDelaySlot = true;

    if (target == m_pc + 4) {
        return;
    }

    if (m_regs[_Rs_].isConst() && m_regs[_Rt_].isConst()) {
        if (m_regs[_Rs_].val == m_regs[_Rt_].val) {
            m_pcWrittenBack = true;
            m_stopCompiling = true;
            gen.Mov(scratch, target);
            gen.Str(scratch, MemOperand(contextPointer, PC_OFFSET));

            m_linkedPC = target;
        }
        return;
    } else if (m_regs[_Rs_].isConst()) {
        allocateReg(_Rt_);
        gen.cmpEqImm(m_regs[_Rt_].allocatedReg, m_regs[_Rs_].val);
    } else if (m_regs[_Rt_].isConst()) {
        allocateReg(_Rs_);
        gen.cmpEqImm(m_regs[_Rs_].allocatedReg, m_regs[_Rt_].val);
    } else {
        alloc_rt_rs();
        gen.Cmp(m_regs[_Rt_].allocatedReg, m_regs[_Rs_].allocatedReg);
    }

    m_pcWrittenBack = true;
    m_stopCompiling = true;
    gen.Mov(w3, target); // addr if jump taken
    gen.Mov(scratch, m_pc + 4); // addr if jump not taken
    gen.Csel(w0, w3, scratch, vixl::aarch64::eq); // if equal, move the jump addr into w0
    gen.Str(w0, MemOperand(contextPointer, PC_OFFSET));
}


void DynaRecCPU::recBGTZ() {
    uint32_t target = _Imm_ * 4 + m_pc;

    m_nextIsDelaySlot = true;
    if (target == m_pc + 4) {
        return;
    }

    if (m_regs[_Rs_].isConst()) {
        if ((int32_t)m_regs[_Rs_].val > 0) {
            m_pcWrittenBack = true;
            m_stopCompiling = true;
            gen.Mov(scratch, target);
            gen.Str(scratch, MemOperand(contextPointer, PC_OFFSET));
            m_linkedPC = target;
        }
        return;
    }

    m_pcWrittenBack = true;
    m_stopCompiling = true;

    if (m_regs[_Rs_].isAllocated()) {  // Don't bother allocating Rs unless it's already allocated
        gen.Tst(m_regs[_Rs_].allocatedReg, m_regs[_Rs_].allocatedReg);
    } else {
        gen.Ldr(scratch, MemOperand(contextPointer, GPR_OFFSET(_Rs_)));
        gen.Cmp(scratch, 0);
    }

    gen.Mov(scratch, m_pc + 4); // scratch = addr if jump not taken
    gen.Mov(w3, target); // w3 = addr if jump is taken
    gen.Csel(w0, w3, scratch, gt); // if taken, move jump addr into w0
    gen.Str(w0, MemOperand(contextPointer, PC_OFFSET)); // store w0 jump addr to m_pc
}


void DynaRecCPU::recBLEZ() {
    uint32_t target = _Imm_ * 4 + m_pc;

    m_nextIsDelaySlot = true;
    if (target == m_pc + 4) {
        return;
    }

    if (m_regs[_Rs_].isConst()) {
        if ((int32_t)m_regs[_Rs_].val <= 0) {
            m_pcWrittenBack = true;
            m_stopCompiling = true;
            gen.Mov(scratch, target);
            gen.Str(scratch, MemOperand(contextPointer, PC_OFFSET));
            m_linkedPC = target;
        }
        return;
    }

    m_pcWrittenBack = true;
    m_stopCompiling = true;

    if (m_regs[_Rs_].isAllocated()) {  // Don't bother allocating Rs unless it's already allocated
        gen.Tst(m_regs[_Rs_].allocatedReg, m_regs[_Rs_].allocatedReg);
    } else {
        gen.Ldr(scratch, MemOperand(contextPointer, GPR_OFFSET(_Rs_)));
        gen.Cmp(scratch, 0);
    }


    gen.Mov(scratch, m_pc + 4); // scratch = addr if jump not taken
    gen.Mov(w3, target); // w3 = addr if jump is taken
    gen.Csel(w0, w3, scratch, le); // if taken, move jump addr into w0
    gen.Str(w0, MemOperand(contextPointer, PC_OFFSET)); // store w0 jump addr to m_pc

}


void DynaRecCPU::recBNE() {
    const auto target = _Imm_ * 4 + m_pc;
    m_nextIsDelaySlot = true;

    if (target == m_pc + 4) {
        return;
    }

    if (m_regs[_Rs_].isConst() && m_regs[_Rt_].isConst()) {
        if (m_regs[_Rs_].val != m_regs[_Rt_].val) {
            m_pcWrittenBack = true;
            m_stopCompiling = true;
            gen.Mov(scratch, target);
            gen.Str(scratch, MemOperand(contextPointer, PC_OFFSET));
            m_linkedPC = target;
        }
        return;
    } else if (m_regs[_Rs_].isConst()) {
        allocateReg(_Rt_);
        gen.cmpEqImm(m_regs[_Rt_].allocatedReg, m_regs[_Rs_].val);
    } else if (m_regs[_Rt_].isConst()) {
        allocateReg(_Rs_);
        gen.cmpEqImm(m_regs[_Rs_].allocatedReg, m_regs[_Rt_].val);
    } else {
        alloc_rt_rs();
        gen.Cmp(m_regs[_Rt_].allocatedReg, m_regs[_Rs_].allocatedReg);
    }

    m_pcWrittenBack = true;
    m_stopCompiling = true;

    gen.Mov(scratch, m_pc + 4); // scratch = addr if jump not taken
    gen.Mov(w3, target); // w3 = addr if jump is taken
    gen.Csel(w0, w3, scratch, ne); // if taken, move jump addr into w0
    gen.Str(w0, MemOperand(contextPointer, PC_OFFSET)); // store w0 jump addr to m_pc
}


void DynaRecCPU::recBREAK() {
    flushRegs();  // For PCDRV support, we need to flush all registers before handling the exception.
    recException(Exception::Break);
}


void DynaRecCPU::recCOP0() {
    switch (_Rs_) {  // figure out the type of COP0 opcode
        case 0:
            recMFC0();
            break;
        case 4:
            recMTC0();
            break;
        case 16:
            recRFE();
            break;
        default:
            fmt::print("Unimplemented cop0 op {}\n", _Rs_);
            recUnknown();
            break;
    }
}


void DynaRecCPU::recDIV() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] DIV instruction"); }
void DynaRecCPU::recDIVU() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] DIVU instruction"); }


void DynaRecCPU::recJ() {
    const uint32_t target = (m_pc & 0xf0000000) | (_Target_ << 2);
    m_nextIsDelaySlot = true;
    m_stopCompiling = true;
    m_pcWrittenBack = true;
    gen.Mov(scratch, target);
    gen.Str(scratch, MemOperand(contextPointer, PC_OFFSET)); // Write PC
    m_linkedPC = target;
    gen.dumpBuffer();
}


void DynaRecCPU::recJAL() {
    maybeCancelDelayedLoad(31);
    markConst(31, m_pc + 4);  // Set $ra to the return value, then treat instruction like a normal J
    recJ();
}


void DynaRecCPU::recJALR() {
    recJR();

    if (_Rd_) {
        maybeCancelDelayedLoad(_Rd_);
        markConst(_Rd_, m_pc + 4);  // Link
    }
}

void DynaRecCPU::recJR() {
    m_nextIsDelaySlot = true;
    m_stopCompiling = true;
    m_pcWrittenBack = true;

    if (m_regs[_Rs_].isConst()) {
        gen.Mov(scratch, m_regs[_Rs_].val & ~3);
        gen.Str(scratch, MemOperand(contextPointer, PC_OFFSET));
        m_linkedPC = m_regs[_Rs_].val;
    } else {
        allocateReg(_Rs_);
        // PC will get force aligned in the dispatcher since it discards the 2 lower bits
        gen.Str(m_regs[_Rs_].allocatedReg, MemOperand(contextPointer, PC_OFFSET));
    }
}


void DynaRecCPU::recLB() { recompileLoad<8, true>(); }
void DynaRecCPU::recLBU() { recompileLoad<8, false>(); }
void DynaRecCPU::recLH() { recompileLoad<16, true>(); }
void DynaRecCPU::recLHU() { recompileLoad<16, false>(); }
void DynaRecCPU::recLW() { recompileLoad<32, true>(); }

void DynaRecCPU::recLUI() {

    BAILZERO(_Rt_);

    maybeCancelDelayedLoad(_Rt_);
    markConst(_Rt_, m_psxRegs.code << 16);
}


template <int size, bool signExtend>
void DynaRecCPU::recompileLoad() {
    if (m_regs[_Rs_].isConst()) {  // Store the address in first argument register
        const uint32_t addr = m_regs[_Rs_].val + _Imm_;
        const auto pointer = PCSX::g_emulator->m_psxMem->psxMemPointerRead(addr);

        if (pointer != nullptr && (_Rt_) != 0) {
            allocateRegWithoutLoad(_Rt_);
            m_regs[_Rt_].setWriteback(true);
            load<size, signExtend>(m_regs[_Rt_].allocatedReg, pointer);
            return;
        }

        gen.Mov(arg1, addr);
    } else {
        allocateReg(_Rs_);
        gen.moveAndAdd(arg1, m_regs[_Rs_].allocatedReg, _Imm_);
    }

    switch (size) {
        case 8:
            call(psxMemRead8Wrapper);
            break;
        case 16:
            call(psxMemRead16Wrapper);
            break;
        case 32:
            call(psxMemRead32Wrapper);
            break;
        default:
            PCSX::g_system->message("Invalid size for memory load in dynarec. Instruction %08x\n", m_psxRegs.code);
            break;
    }

    if (_Rt_) {
        allocateRegWithoutLoad(_Rt_);  // Allocate $rt after calling the read function, otherwise call() might flush it.
        m_regs[_Rt_].setWriteback(true);

        switch (size) {
            case 8:
                signExtend ? gen.Sxtb(m_regs[_Rt_].allocatedReg, w0) : gen.Uxtb(m_regs[_Rt_].allocatedReg, w0);
                break;
            case 16:
                signExtend ? gen.Sxth(m_regs[_Rt_].allocatedReg, w0) : gen.Uxth(m_regs[_Rt_].allocatedReg, w0);
                break;
            case 32:
                gen.Mov(m_regs[_Rt_].allocatedReg, w0);
                break;
        }
    }
}


//void DynaRecCPU::recLWC2() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] LWC2 instruction"); }
void DynaRecCPU::recLWL() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] LWL instruction"); }
void DynaRecCPU::recLWR() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] LWR instruction"); }


void DynaRecCPU::recMFC0() {
    PCSX::g_system->message("recMFC0\n");
    BAILZERO(_Rt_);
    maybeCancelDelayedLoad(_Rt_);
    allocateRegWithoutLoad(_Rt_);
    m_regs[_Rt_].setWriteback(true);
    gen.Ldr(m_regs[_Rt_].allocatedReg, MemOperand(contextPointer, COP0_OFFSET(_Rd_)));
}


//void DynaRecCPU::recMFC2() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] MFC2 instruction"); }
void DynaRecCPU::recMFHI() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] MFHI instruction"); }
void DynaRecCPU::recMFLO() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] MFLO instruction"); }

// TODO: Handle all COP0 register writes properly. Don't treat read-only field as writeable!
void DynaRecCPU::recMTC0() {
    PCSX::g_system->message("recMTC0\n");
    if (m_regs[_Rt_].isConst()) {
        if (_Rd_ == 13) {
                gen.Mov(scratch,  m_regs[_Rt_].val & ~0xFC00);
                gen.Str(scratch, MemOperand(contextPointer, COP0_OFFSET(_Rd_)));
            } else if (_Rd_ != 6 && _Rd_ != 14 && _Rd_ != 15) {  // Don't write to JUMPDEST, EPC or PRID
                gen.Mov(scratch, m_regs[_Rt_].val);
                gen.Str(scratch, MemOperand(contextPointer, COP0_OFFSET(_Rd_)));
            }
    }

    else {
        allocateReg(_Rt_);
        if (_Rd_ == 13) {
                gen.And(m_regs[_Rt_].allocatedReg, m_regs[_Rt_].allocatedReg, ~0xFC00);
            } else if (_Rd_ != 6 && _Rd_ != 14 && _Rd_ != 15) {  // Don't write to JUMPDEST, EPC or PRID
                gen.Str(m_regs[_Rt_].allocatedReg, MemOperand(contextPointer, COP0_OFFSET(_Rd_))); // Write rt to the cop0 reg
            }
    }

    // Writing to SR/Cause can sometimes forcefully fire an interrupt. So we need to emit extra code to check.
    if (_Rd_ == 12 || _Rd_ == 13) {
        testSoftwareInterrupt<true>();
    }
}

//void DynaRecCPU::recMTC2() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] MTC2 instruction"); }
void DynaRecCPU::recMTHI() { gen.dumpBuffer(); gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] MTHI instruction"); }
void DynaRecCPU::recMTLO() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] MTLP instruction"); }
void DynaRecCPU::recMULT() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] MULT instruction"); }
void DynaRecCPU::recMULTU() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] MULTU instruction"); }
void DynaRecCPU::recNOR() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] NOR instruction"); }


void DynaRecCPU::recOR() {
    BAILZERO(_Rd_);
    maybeCancelDelayedLoad(_Rd_);

    if (m_regs[_Rs_].isConst() && m_regs[_Rt_].isConst()) {
        markConst(_Rd_, m_regs[_Rs_].val | m_regs[_Rt_].val);
    } else if (m_regs[_Rs_].isConst()) {
        alloc_rt_wb_rd();

        gen.Orr(m_regs[_Rd_].allocatedReg, m_regs[_Rt_].allocatedReg, m_regs[_Rs_].val);
    } else if (m_regs[_Rt_].isConst()) {
        alloc_rs_wb_rd();

        gen.Orr(m_regs[_Rd_].allocatedReg, m_regs[_Rs_].allocatedReg, m_regs[_Rt_].val);
    } else {
        alloc_rt_rs_wb_rd();
        gen.Orr(m_regs[_Rd_].allocatedReg, m_regs[_Rs_].allocatedReg, m_regs[_Rt_].allocatedReg);
    }
}


void DynaRecCPU::recORI() {
    BAILZERO(_Rt_);
    maybeCancelDelayedLoad(_Rt_);

    if (_Rs_ == _Rt_) {
        if (m_regs[_Rs_].isConst()) {
            m_regs[_Rt_].val |= _ImmU_;
        } else {
            allocateReg(_Rt_);
            m_regs[_Rt_].setWriteback(true);
            gen.Orr(m_regs[_Rt_].allocatedReg, m_regs[_Rt_].allocatedReg, _ImmU_);
        }
    } else {
        if (m_regs[_Rs_].isConst()) {
            markConst(_Rt_, m_regs[_Rs_].val | _ImmU_);
        } else {
            alloc_rs_wb_rt();
            gen.Mov(m_regs[_Rt_].allocatedReg, m_regs[_Rs_].allocatedReg);
            gen.Orr(m_regs[_Rt_].allocatedReg, m_regs[_Rt_].allocatedReg, _ImmU_);
        }
    }
}


void DynaRecCPU::recREGIMM() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] REGIMM instruction"); }
void DynaRecCPU::recRFE() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] RFE instruction"); }
void DynaRecCPU::recSB() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] SB instruction"); }
void DynaRecCPU::recSH() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] SH instruction"); }


void DynaRecCPU::recSLL() {
    BAILZERO(_Rd_);
    maybeCancelDelayedLoad(_Rd_);

    if (m_regs[_Rt_].isConst()) {
        markConst(_Rd_, m_regs[_Rt_].val << _Sa_);
    } else {
        alloc_rt_wb_rd();
        gen.Lsl(m_regs[_Rd_].allocatedReg, m_regs[_Rt_].allocatedReg,  _Sa_);
    }
}


void DynaRecCPU::recSLLV() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] SLLV instruction"); }
void DynaRecCPU::recSLT() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] SLT instruction"); }
void DynaRecCPU::recSLTI() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] SLTI instruction"); }
void DynaRecCPU::recSLTIU() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] SLTIU instruction"); }
void DynaRecCPU::recSLTU() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] SLTU instruction"); }
void DynaRecCPU::recSRA() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] SRA instruction"); }
void DynaRecCPU::recSRAV() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] SRAV instruction"); }
void DynaRecCPU::recSRL() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] SRL instruction"); }
void DynaRecCPU::recSRLV() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] SRLV instruction"); }


void DynaRecCPU::recSUB() { recSUBU(); }


void DynaRecCPU::recSUBU() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] SUBU instruction"); }


void DynaRecCPU::recSW() {
    if (m_regs[_Rs_].isConst()) {
        const uint32_t addr = m_regs[_Rs_].val + _Imm_;
        const auto pointer = PCSX::g_emulator->m_psxMem->psxMemPointerWrite(addr);
        if (pointer != nullptr) {
            if (m_regs[_Rt_].isConst()) {
                gen.Mov(scratch, m_regs[_Rt_].val);
                store<32>(scratch, pointer);
            } else {
                allocateReg(_Rt_);
                store<32>(m_regs[_Rt_].allocatedReg, pointer);
            }

            return;
        }

        if (m_regs[_Rt_].isConst()) {  // Value to write in arg2
            gen.Mov(arg2, m_regs[_Rt_].val);
        } else {
            allocateReg(_Rt_);
            gen.Mov(arg2, m_regs[_Rt_].allocatedReg);
        }

        gen.Mov(arg1, addr);  // Address to write to in arg1   TODO: Optimize
        call(psxMemWrite32Wrapper);
    }

    else {
        if (m_regs[_Rt_].isConst()) {  // Value to write in arg2
            gen.Mov(arg2, m_regs[_Rt_].val);
        } else {
            allocateReg(_Rt_);
            gen.Mov(arg2, m_regs[_Rt_].allocatedReg);
        }

        allocateReg(_Rs_);
        gen.Add(arg1, m_regs[_Rs_].allocatedReg, _Imm_);  // Address to write to in arg1   TODO: Optimize

        call(psxMemWrite32Wrapper);
    }
}


//void DynaRecCPU::recSWC2() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] SWC2 instruction"); }
void DynaRecCPU::recSWL() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] SWL instruction"); }
void DynaRecCPU::recSWR() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] SWR instruction"); }


void DynaRecCPU::recSYSCALL() { recException(Exception::Syscall); }


void DynaRecCPU::recXOR() {
    BAILZERO(_Rd_);
    maybeCancelDelayedLoad(_Rd_);

    if (m_regs[_Rs_].isConst() && m_regs[_Rt_].isConst()) {
        markConst(_Rd_, m_regs[_Rs_].val ^ m_regs[_Rt_].val);
    } else if (m_regs[_Rs_].isConst()) {
        alloc_rt_wb_rd();
        gen.Eor(m_regs[_Rd_].allocatedReg, m_regs[_Rt_].allocatedReg, m_regs[_Rs_].val);
    } else if (m_regs[_Rt_].isConst()) {
        alloc_rs_wb_rd();
        gen.Eor(m_regs[_Rd_].allocatedReg, m_regs[_Rs_].allocatedReg, m_regs[_Rt_].val);
    } else {
        alloc_rt_rs_wb_rd();
        gen.Eor(m_regs[_Rd_].allocatedReg, m_regs[_Rt_].allocatedReg, m_regs[_Rs_].allocatedReg);
    }
}


void DynaRecCPU::recXORI() {
    BAILZERO(_Rt_);
    maybeCancelDelayedLoad(_Rt_);

    if (_Rs_ == _Rt_) {
        if (m_regs[_Rs_].isConst()) {
            m_regs[_Rt_].val ^= _ImmU_;
        } else {
            allocateReg(_Rt_);
            m_regs[_Rt_].setWriteback(true);
            gen.Eor(m_regs[_Rt_].allocatedReg, m_regs[_Rt_].allocatedReg, _ImmU_);
        }
    } else {
        if (m_regs[_Rs_].isConst()) {
            markConst(_Rt_, m_regs[_Rs_].val ^ _ImmU_);
        } else {
            alloc_rs_wb_rt();
            gen.Mov(m_regs[_Rt_].allocatedReg, m_regs[_Rs_].allocatedReg);
            if (_ImmU_) {
                gen.Eor(m_regs[_Rt_].allocatedReg, m_regs[_Rt_].allocatedReg, _ImmU_);
            }
        }
    }
}


void DynaRecCPU::recException(Exception e) {
    m_pcWrittenBack = true;
    m_stopCompiling = true;

    loadThisPointer(arg1.X());                                                  // Pointer to this object in arg1
    gen.Mov(arg2, static_cast<std::underlying_type<Exception>::type>(e) << 2); // Exception type in arg2
    gen.Mov(arg3, (int32_t)m_inDelaySlot);             // Store whether we're in a delay slot in arg3
    gen.Mov(scratch, m_pc - 4);
    gen.Str(scratch, MemOperand(contextPointer, PC_OFFSET)); // PC for exception handler to use

    call(psxExceptionWrapper);  // Call the exception wrapper
}

// Checks if a write to SR/CAUSE forcibly triggered an interrupt
// loadSR: Shows if SR is already in eax or if it should be loaded from memory
template <bool loadSR>
void DynaRecCPU::testSoftwareInterrupt() {
    PCSX::g_system->message("TestSoftwareInterrupt\n");
    vixl::aarch64::Label label;
    if (!m_pcWrittenBack) {
        gen.Mov(w4, m_pc);
        gen.Str(w4, MemOperand(contextPointer, PC_OFFSET));
        m_pcWrittenBack = true;
    }

    m_stopCompiling = true;

    if constexpr (loadSR) {
        gen.Ldr(w0, MemOperand(contextPointer, COP0_OFFSET(12))); // w0 = SR
    }
    gen.Cbz(w0, &label); // Check if interrupts are enabled // If not, skip to the end
    gen.Ldr(arg2, MemOperand(contextPointer, COP0_OFFSET(13))); // arg2 = CAUSE
    gen.And(w0, w0, arg2);
    gen.Tst(w0, 0x300);                             // Check if an interrupt was force-fired // Skip to the end if not
    gen.B(eq, &label);

    // Fire the interrupt if it was triggered
    // This object in arg1. Exception code is already in arg2 from before (will be masked by exception handler)
    loadThisPointer(arg1.X());
    gen.Mov(arg3, (int32_t)m_inDelaySlot);             // Store whether we're in a delay slot in arg3
    gen.Mov(scratch, m_pc - 4);  // PC for exception handler to use
    gen.Str(scratch, MemOperand(contextPointer, PC_OFFSET)); // Store m_pc back to host
    call(psxExceptionWrapper);                             // Call the exception wrapper function

    gen.L(label);
    gen.dumpBuffer();
}

#undef BAILZERO
#endif  // DYNAREC_AA64

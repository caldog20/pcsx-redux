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

void DynaRecCPU::recADD() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] ADD instruction"); }


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


void DynaRecCPU::recADDU() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] ADDU instruction"); }
void DynaRecCPU::recAND() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] AND instruction"); }
void DynaRecCPU::recANDI() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] ANDI instruction"); }
void DynaRecCPU::recBEQ() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] BEQ instruction"); }
void DynaRecCPU::recBGTZ() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] BGTZ instruction"); }
void DynaRecCPU::recBLEZ() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] BLEZ instruction"); }
void DynaRecCPU::recBNE() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] BNE instruction"); }
void DynaRecCPU::recBREAK() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] BREAK instruction"); }
//void DynaRecCPU::recCFC2() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] CFC2 instruction"); }
void DynaRecCPU::recCOP0() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] COP0 instruction"); }
//void DynaRecCPU::recCOP2() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] COP2 instruction"); }
//void DynaRecCPU::recCTC2() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] CTC2 instruction"); }
void DynaRecCPU::recDIV() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] DIV instruction"); }
void DynaRecCPU::recDIVU() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] DIVU instruction"); }
void DynaRecCPU::recJ() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] J instruction"); }
void DynaRecCPU::recJAL() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] JAL instruction"); }
void DynaRecCPU::recJALR() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] JALR instruction"); }
void DynaRecCPU::recJR() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] JR instruction"); }
void DynaRecCPU::recLB() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] LB instruction"); }
void DynaRecCPU::recLBU() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] LBU instruction"); }
void DynaRecCPU::recLH() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] LH instruction"); }
void DynaRecCPU::recLHU() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] LHU instruction"); }


void DynaRecCPU::recLUI() {

    BAILZERO(_Rt_);

    maybeCancelDelayedLoad(_Rt_);
    markConst(_Rt_, m_psxRegs.code << 16);
}


void DynaRecCPU::recLW() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] LW instruction"); }
//void DynaRecCPU::recLWC2() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] LWC2 instruction"); }
void DynaRecCPU::recLWL() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] LWL instruction"); }
void DynaRecCPU::recLWR() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] LWR instruction"); }
void DynaRecCPU::recMFC0() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] MFC0 instruction"); }
//void DynaRecCPU::recMFC2() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] MFC2 instruction"); }
void DynaRecCPU::recMFHI() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] MFHI instruction"); }
void DynaRecCPU::recMFLO() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] MFLO instruction"); }
void DynaRecCPU::recMTC0() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] MTC0 instruction"); }
//void DynaRecCPU::recMTC2() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] MTC2 instruction"); }
void DynaRecCPU::recMTHI() { gen.dumpBuffer(); gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] MTHI instruction"); }
void DynaRecCPU::recMTLO() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] MTLP instruction"); }
void DynaRecCPU::recMULT() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] MULT instruction"); }
void DynaRecCPU::recMULTU() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] MULTU instruction"); }
void DynaRecCPU::recNOR() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] NOR instruction"); }
void DynaRecCPU::recOR() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] OR instruction"); }


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
void DynaRecCPU::recSUB() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] SUB instruction"); }
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
void DynaRecCPU::recSYSCALL() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] SYSCALL instruction"); }
void DynaRecCPU::recXOR() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] XOR instruction"); }
void DynaRecCPU::recXORI() { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] XORI instruction"); }
void DynaRecCPU::recException(Exception e) { gen.dumpBuffer(); throw std::runtime_error("[Unimplemented] Recompile exception"); }

#undef BAILZERO
#endif  // DYNAREC_AA64

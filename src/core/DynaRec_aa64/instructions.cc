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
void DynaRecCPU::recSpecial() { throw std::runtime_error("[Unimplemented] Special instruction"); }

void DynaRecCPU::recADD() { throw std::runtime_error("[Unimplemented] ADD instruction"); }
void DynaRecCPU::recADDIU() { throw std::runtime_error("[Unimplemented] ADDIU instruction"); }
void DynaRecCPU::recADDU() { throw std::runtime_error("[Unimplemented] ADDU instruction"); }
void DynaRecCPU::recAND() { throw std::runtime_error("[Unimplemented] AND instruction"); }
void DynaRecCPU::recANDI() { throw std::runtime_error("[Unimplemented] ANDI instruction"); }
void DynaRecCPU::recBEQ() { throw std::runtime_error("[Unimplemented] BEQ instruction"); }
void DynaRecCPU::recBGTZ() { throw std::runtime_error("[Unimplemented] BGTZ instruction"); }
void DynaRecCPU::recBLEZ() { throw std::runtime_error("[Unimplemented] BLEZ instruction"); }
void DynaRecCPU::recBNE() { throw std::runtime_error("[Unimplemented] BNE instruction"); }
void DynaRecCPU::recBREAK() { throw std::runtime_error("[Unimplemented] BREAK instruction"); }
void DynaRecCPU::recCFC2() { throw std::runtime_error("[Unimplemented] CFC2 instruction"); }
void DynaRecCPU::recCOP0() { throw std::runtime_error("[Unimplemented] COP0 instruction"); }
void DynaRecCPU::recCOP2() { throw std::runtime_error("[Unimplemented] COP2 instruction"); }
void DynaRecCPU::recCTC2() { throw std::runtime_error("[Unimplemented] CTC2 instruction"); }
void DynaRecCPU::recDIV() { throw std::runtime_error("[Unimplemented] DIV instruction"); }
void DynaRecCPU::recDIVU() { throw std::runtime_error("[Unimplemented] DIVU instruction"); }
void DynaRecCPU::recJ() { throw std::runtime_error("[Unimplemented] J instruction"); }
void DynaRecCPU::recJAL() { throw std::runtime_error("[Unimplemented] JAL instruction"); }
void DynaRecCPU::recJALR() { throw std::runtime_error("[Unimplemented] JALR instruction"); }
void DynaRecCPU::recJR() { throw std::runtime_error("[Unimplemented] JR instruction"); }
void DynaRecCPU::recLB() { throw std::runtime_error("[Unimplemented] LB instruction"); }
void DynaRecCPU::recLBU() { throw std::runtime_error("[Unimplemented] LBU instruction"); }
void DynaRecCPU::recLH() { throw std::runtime_error("[Unimplemented] LH instruction"); }
void DynaRecCPU::recLHU() { throw std::runtime_error("[Unimplemented] LHU instruction"); }
void DynaRecCPU::recLUI() { throw std::runtime_error("[Unimplemented] LUI instruction"); }
void DynaRecCPU::recLW() { throw std::runtime_error("[Unimplemented] LW instruction"); }
void DynaRecCPU::recLWC2() { throw std::runtime_error("[Unimplemented] LWC2 instruction"); }
void DynaRecCPU::recLWL() { throw std::runtime_error("[Unimplemented] LWL instruction"); }
void DynaRecCPU::recLWR() { throw std::runtime_error("[Unimplemented] LWR instruction"); }
void DynaRecCPU::recMFC0() { throw std::runtime_error("[Unimplemented] MFC0 instruction"); }
void DynaRecCPU::recMFC2() { throw std::runtime_error("[Unimplemented] MFC2 instruction"); }
void DynaRecCPU::recMFHI() { throw std::runtime_error("[Unimplemented] MFHI instruction"); }
void DynaRecCPU::recMFLO() { throw std::runtime_error("[Unimplemented] MFLO instruction"); }
void DynaRecCPU::recMTC0() { throw std::runtime_error("[Unimplemented] MTC0 instruction"); }
void DynaRecCPU::recMTC2() { throw std::runtime_error("[Unimplemented] MTC2 instruction"); }
void DynaRecCPU::recMTHI() { throw std::runtime_error("[Unimplemented] MTHI instruction"); }
void DynaRecCPU::recMTLO() { throw std::runtime_error("[Unimplemented] MTLP instruction"); }
void DynaRecCPU::recMULT() { throw std::runtime_error("[Unimplemented] MULT instruction"); }
void DynaRecCPU::recMULTU() { throw std::runtime_error("[Unimplemented] MULTU instruction"); }
void DynaRecCPU::recNOR() { throw std::runtime_error("[Unimplemented] NOR instruction"); }
void DynaRecCPU::recOR() { throw std::runtime_error("[Unimplemented] OR instruction"); }
void DynaRecCPU::recORI() { throw std::runtime_error("[Unimplemented] ORI instruction"); }
void DynaRecCPU::recREGIMM() { throw std::runtime_error("[Unimplemented] REGIMM instruction"); }
void DynaRecCPU::recRFE() { throw std::runtime_error("[Unimplemented] RFE instruction"); }
void DynaRecCPU::recSB() { throw std::runtime_error("[Unimplemented] SB instruction"); }
void DynaRecCPU::recSH() { throw std::runtime_error("[Unimplemented] SH instruction"); }
void DynaRecCPU::recSLL() { throw std::runtime_error("[Unimplemented] SLL instruction"); }
void DynaRecCPU::recSLLV() { throw std::runtime_error("[Unimplemented] SLLV instruction"); }
void DynaRecCPU::recSLT() { throw std::runtime_error("[Unimplemented] SLT instruction"); }
void DynaRecCPU::recSLTI() { throw std::runtime_error("[Unimplemented] SLTI instruction"); }
void DynaRecCPU::recSLTIU() { throw std::runtime_error("[Unimplemented] SLTIU instruction"); }
void DynaRecCPU::recSLTU() { throw std::runtime_error("[Unimplemented] SLTU instruction"); }
void DynaRecCPU::recSRA() { throw std::runtime_error("[Unimplemented] SRA instruction"); }
void DynaRecCPU::recSRAV() { throw std::runtime_error("[Unimplemented] SRAV instruction"); }
void DynaRecCPU::recSRL() { throw std::runtime_error("[Unimplemented] SRL instruction"); }
void DynaRecCPU::recSRLV() { throw std::runtime_error("[Unimplemented] SRLV instruction"); }
void DynaRecCPU::recSUB() { throw std::runtime_error("[Unimplemented] SUB instruction"); }
void DynaRecCPU::recSUBU() { throw std::runtime_error("[Unimplemented] SUBU instruction"); }
void DynaRecCPU::recSW() { throw std::runtime_error("[Unimplemented] SW instruction"); }
void DynaRecCPU::recSWC2() { throw std::runtime_error("[Unimplemented] SWC2 instruction"); }
void DynaRecCPU::recSWL() { throw std::runtime_error("[Unimplemented] SWL instruction"); }
void DynaRecCPU::recSWR() { throw std::runtime_error("[Unimplemented] SWR instruction"); }
void DynaRecCPU::recSYSCALL() { throw std::runtime_error("[Unimplemented] SYSCALL instruction"); }
void DynaRecCPU::recXOR() { throw std::runtime_error("[Unimplemented] XOR instruction"); }
void DynaRecCPU::recXORI() { throw std::runtime_error("[Unimplemented] XORI instruction"); }
void DynaRecCPU::recException(Exception e) { throw std::runtime_error("[Unimplemented] Recompile exception"); }

#undef BAILZERO
#endif  // DYNAREC_AA64
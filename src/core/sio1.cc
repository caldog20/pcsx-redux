/***************************************************************************
 *   Copyright (C) 2022 PCSX-Redux authors                                 *
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

#include "core/sio1.h"
#include "fmt/format.h"
#include <iostream>
#include <string>
#include <fstream>

PCSX::SIOPayload PCSX::SIO1::makePayload(std::string data) {
    bool dtr = (m_regs.control & CR_DTR) == 0x0002;
    bool rts = (m_regs.control & CR_RTSOUTLVL) == 0x0020;
    DataTransferData dt;
    if (data.size() == 0) {
        dt = {};
    } else {
        dt = {data};
    }
    return SIOPayload {
        DataTransfer {
            dt,
                },
    FlowControl { dtr, rts },
    };
}

bool PCSX::SIO1::tryDecodeMessage() {
//    printf("tryDecodeMessage\n");
//    printf("RXfifo size: %d\n", m_pbfifo.size());
    read_length:
//        printf("read_length\n");
        if (checkSize) {
            messageSize = m_pbfifo.byte();
            checkSize = false;
//            printf("message size: %d\n", messageSize);
        }
        if (m_pbfifo.size() < messageSize) {
//            printf("not enough data\n");
            return false;
        }
    process:
//        printf("process\n");
        auto data = m_pbfifo.readString( messageSize);
        if (data.size() != messageSize) {
//            printf("Error reading message\n");
            m_pbfifo.reset();
            checkSize = true;
            return false;
        }
//        printf("Decoding Message\n");
        SIOPayload payload;
        Protobuf::InSlice inslice(reinterpret_cast<const uint8_t *>(data.data()), data.size());
        payload.deserialize(&inslice, 0);
        auto newdxr = payload.get<FlowControlField>().get<FlowControlDXR>().value;
        auto newxts = payload.get<FlowControlField>().get<FlowControlXTS>().value;
        if (m_pbfifo.size() > 0) {
            checkSize = true;
        }
    final:
//        printf("final\n");
        if (payload.get<DataTransferField>().get<DataTransferData>().hasData()) {
//            printf("Has data\n");
            std::string byte = payload.get<DataTransferField>().get<DataTransferData>().value;
            std::cout << byte << std::endl;
            PCSX::Slice pushByte;
            pushByte.acquire(std::move(byte));
            m_fifo.pushSlice(std::move(pushByte));
            receiveCallback();
        }
        if (newdxr) {
            m_regs.status |= SR_DSR;
        } else {
            m_regs.status &= ~SR_DSR;
        }
        if (newxts) {
            m_regs.status |= SR_CTS;
        } else {
            m_regs.status &= ~SR_CTS;
        }
    checkSize = true;
    return true;
}

void PCSX::SIO1::encodeDataMessage(bool withData) {
//    printf("Encoding data message\n");
    SIOPayload payload;
//    printf("txfifo size: %d\n", m_txfifo.size());
    std::string byte;
    if (m_txfifo.size() > 0 && withData) {
        byte = m_txfifo.byte();
    }
    payload = makePayload(byte);
    Protobuf::OutSlice outslice;
    payload.serialize(&outslice);
    std::string data = outslice.finalize();
    uint8_t size = data.size();
    g_emulator->m_sio1Server->write(size);
    g_emulator->m_sio1Server->write(data);
}

void PCSX::SIO1::interrupt() {
    SIO1_LOG("SIO1 Interrupt (CP0.Status = %x)\n", PCSX::g_emulator->m_cpu->m_regs.CP0.n.Status);
    m_regs.status |= SR_IRQ;
    psxHu32ref(0x1070) |= SWAP_LEu32(IRQ8_SIO);

    if (m_regs.control & CR_RXIRQEN) {
        if (!(m_regs.status & SR_IRQ)) {
            switch ((m_regs.control & 0x300) >> 8) {
                case 0:
                    if (!(m_fifo.size() >= 1)) return;
                    break;

                case 1:
                    if (!(m_fifo.size() >= 2)) return;
                    break;

                case 2:
                    if (!(m_fifo.size() >= 4)) return;
                    break;

                case 3:
                    if (!(m_fifo.size() >= 8)) return;
                    break;
            }

            scheduleInterrupt(SIO1_CYCLES);
            m_regs.status |= SR_IRQ;
        }
    }
}

uint8_t PCSX::SIO1::readData8() {
    updateStat();
    if (m_regs.status & SR_RXRDY) {
//        printf("SIO1: Read data\n");
        m_regs.data = m_fifo.byte();
        psxHu8(0x1050) = m_regs.data;
    }
    updateStat();

    return m_regs.data;
}

uint8_t PCSX::SIO1::readStat8() {
    updateStat();
    return m_regs.status;
}

uint16_t PCSX::SIO1::readStat16() {
    updateStat();
    return m_regs.status;
}

uint32_t PCSX::SIO1::readStat32() {
    updateStat();
    return m_regs.status;
}

void PCSX::SIO1::receiveCallback() {
    printf("Received callback\n");
    updateStat();

    if (m_regs.control & CR_RXIRQEN) {
//        printf("RXIRQEN: %d\n", m_regs.control & CR_RXIRQEN);
        if (!(m_regs.status & SR_IRQ)) {
//            printf("SR_IRQ %d\n", m_regs.status & SR_IRQ);
//            printf("switch: %d\n", (m_regs.control & 0x300) >> 8);
            switch ((m_regs.control & 0x300) >> 8) {
                case 0:
                    if (!(m_fifo.size() >= 1)) return;
                    break;

                case 1:
                    if (!(m_fifo.size() >= 2)) return;
                    break;

                case 2:
                    if (!(m_fifo.size() >= 4)) return;
                    break;

                case 3:
                    if (!(m_fifo.size() >= 8)) return;
                    break;
            }
            printf("SIO1: Receive IRQ\n");
            scheduleInterrupt(SIO1_CYCLES);
            m_regs.status |= SR_IRQ;
        }
    }
}

void PCSX::SIO1::queueTransmit() {
    Slice sl;
    sl.copy(static_cast<void*>(&m_regs.data), 1);
    m_txfifo.pushSlice(std::move(sl));
    encodeDataMessage();
}

void PCSX::SIO1::transmitData() {
    queueTransmit();

    if (m_regs.control & CR_TXIRQEN) {
        if (m_regs.status & SR_TXRDY || m_regs.status & SR_TXRDY2) {
            if (!(m_regs.status & SR_IRQ)) {
                scheduleInterrupt(SIO1_CYCLES);
                m_regs.status |= SWAP_LEu32(SR_IRQ);
            }
        }
    }
}

bool PCSX::SIO1::isTransmitReady() {
    return (m_regs.control & CR_TXEN) && (m_regs.status & SR_CTS) && (m_regs.status & SR_TXRDY2);
}

void PCSX::SIO1::updateStat() {
    if (m_fifo.size() > 0) {
        m_regs.status |= SR_RXRDY;
    } else {
        m_regs.status &= ~SR_RXRDY;
    }
    psxHu32ref(0x1054) = SWAP_LEu32(m_regs.status);
//    encodeDataMessage(false);
}
void PCSX::SIO1::writeBaud16(uint16_t v) {
    m_regs.baud = v;
    psxHu8ref(0x105E) = m_regs.baud;
//    encodeDataMessage(false);
}

void PCSX::SIO1::writeCtrl16(uint16_t v) {
    uint16_t old_ctrl = m_regs.control;
    m_regs.control = v;

    if (!(old_ctrl & CR_TXEN) && (m_regs.control & CR_TXEN)) {
        if (isTransmitReady()) {
            transmitData();
        }
    }

    if (m_regs.control & CR_ACK) {
        m_regs.control &= ~CR_ACK;
        m_regs.status &= ~(SR_PARITYERR | SR_RXOVERRUN | SR_FRAMINGERR | SR_IRQ);
    }

    if (m_regs.control & CR_RESET) {
        m_regs.status &= ~SR_IRQ;
        m_regs.status |= (SR_TXRDY | SR_TXRDY2);
        m_regs.mode = 0;
        m_regs.control = 0;
        m_regs.baud = 0;

        PCSX::g_emulator->m_cpu->m_regs.interrupt &= ~(1 << PCSX::PSXINT_SIO1);
    }
    psxHu16ref(0x105A) = SWAP_LE16(m_regs.control);
    encodeDataMessage(false);
}

void PCSX::SIO1::writeData8(uint8_t v) {
    m_regs.data = v;

    if (isTransmitReady()) {
        transmitData();
    }

    psxHu8ref(0x1050) = m_regs.data;
}

void PCSX::SIO1::writeMode16(uint16_t v) {
    m_regs.mode = v;
//    encodeDataMessage(false);
}

void PCSX::SIO1::writeStat32(uint32_t v) {
    m_regs.status = v;
    if (isTransmitReady()) {
        transmitData();
    }
    psxHu32ref(0x1054) = SWAP_LE32(m_regs.status);
}

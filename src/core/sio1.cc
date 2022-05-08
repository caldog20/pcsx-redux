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

PCSX::SIOPayload PCSX::SIO1::makeFCPayload() {
    bool dxr = (m_regs.control & CR_DTR);
    bool xts = (m_regs.control & CR_RTSOUTLVL);
    sentDxr = dxr;
    sentXts = xts;

    return SIOPayload {
        DataTransfer {},
        FlowControl { dxr, xts },
    };
}

PCSX::SIOPayload PCSX::SIO1::makeDataPayload(std::string data) {
    bool dxr = (m_regs.control & CR_DTR);
    bool xts = (m_regs.control & CR_RTSOUTLVL);

    return SIOPayload {
        DataTransfer {
            DataTransferData { data },
        },
        FlowControl {dxr, xts},
    };
}

void PCSX::SIO1::encodeDataMessage() {
    if (!m_fifo) return;
    SIOPayload payload;
    std::string datastr(1, m_regs.data);
    payload = makeDataPayload(datastr);

    Protobuf::OutSlice outslice;
    payload.serialize(&outslice);
    std::string data = outslice.finalize();
    uint8_t size = data.size();
    m_fifo->write<uint8_t>(size);
    Slice sliceData;
    sliceData.acquire(std::move(data));
    m_fifo->write(std::move(sliceData));
}

void PCSX::SIO1::encodeFCMessage() {
    if (!m_fifo) return;
    if (!initialMessage) {
        bool dxr = (m_regs.control & CR_DTR);
        bool xts = (m_regs.control & CR_RTSOUTLVL);
        if (sentDxr == dxr && sentXts == xts) {
            return;
        }
    }

    SIOPayload payload;
    payload = makeFCPayload();
    Protobuf::OutSlice outslice;
    payload.serialize(&outslice);
    std::string data = outslice.finalize();
    uint8_t size = data.size();
    m_fifo->write<uint8_t>(size);
    Slice sliceFc;
    sliceFc.acquire(std::move(data));
    m_fifo->write(std::move(sliceFc));
    initialMessage = false;
}

bool PCSX::SIO1::decodeMessage() {
    if (!m_fifo) return false;
    std::string message = m_fifo->readString(messageSize);
    if (message.size() != messageSize) {
        m_fifo.reset();
        m_sio1fifo.reset();
        return false;
    } else {
        SIOPayload payload;
        Protobuf::InSlice inslice(reinterpret_cast<const uint8_t*>(message.data()), message.size());
        try {
            payload.deserialize(&inslice, 0);
        } catch(...) {
            // Insert some log here
            return false;
        }

        if (payload.get<FlowControlField>().hasData()) {
            if (payload.get<FlowControlField>().get<FlowControlDXR>().hasData()) {
                if (payload.get<FlowControlField>().get<FlowControlDXR>().value) {
                    m_regs.status |= SR_DSR;
                } else {
                    m_regs.status &= ~SR_DSR;
                }
            } else {
                m_regs.status &= ~SR_DSR;
            }
            if (payload.get<FlowControlField>().get<FlowControlXTS>().hasData()) {
                if (payload.get<FlowControlField>().get<FlowControlXTS>().value) {
                    m_regs.status |= SR_CTS;
                } else {
                    m_regs.status &= ~SR_DSR;
                }
            } else {
                m_regs.status &= ~SR_CTS;
            }
        } else {
            m_regs.status &= ~SR_DSR;
            m_regs.status &= ~SR_CTS;
        }
        if (payload.get<DataTransferField>().get<DataTransferData>().hasData()) {
            std::string byte = payload.get<DataTransferField>().get<DataTransferData>().value;
            PCSX::Slice pushByte;
            pushByte.acquire(std::move(byte));
            m_sio1fifo.pushSlice(std::move(pushByte));
            receiveCallback();
        }
    }
    return true;
}

bool PCSX::SIO1::sio1StateMachine() {
    if (!m_fifo) return false;
    if (m_sio1Mode == SIO1Mode::Raw) {
        if (m_fifo->size() > 0) {
            auto byte = m_fifo->byte();
            Slice byteslice;
            byteslice.copy(static_cast<uint8_t*>(&byte), 1);
            m_sio1fifo.pushSlice(std::move(byteslice));
            receiveCallback();
            return true;
        }
    }
    if (m_fifo->size() <= 0) {
        return false;
    }

read_length:
    if (checkSize) {
        messageSize = m_fifo->byte();
        checkSize = false;
    }
    if (m_fifo->size() < messageSize) {
        return false;
    }

    if (!decodeMessage()) {
        checkSize = true;
        return false;
    } else {
        checkSize = true;
        if (m_fifo->size() > 0) {
            goto read_length;
        }
        return true;
    }
}

void PCSX::SIO1::interrupt() {
    SIO1_LOG("SIO1 Interrupt (CP0.Status = %x)\n", PCSX::g_emulator->m_cpu->m_regs.CP0.n.Status);
    m_regs.status |= SR_IRQ;
    psxHu32ref(0x1070) |= SWAP_LEu32(IRQ8_SIO);
        if (m_regs.control & CR_RXIRQEN) {
            if (!(m_regs.status & SR_IRQ)) {
                switch ((m_regs.control & 0x300) >> 8) {
                    case 0:
                        if (!(m_sio1fifo.size() >= 1)) return;
                        break;

                    case 1:
                        if (!(m_sio1fifo.size() >= 2)) return;
                        break;

                    case 2:
                        if (!(m_sio1fifo.size() >= 4)) return;
                        break;

                    case 3:
                        if (!(m_sio1fifo.size() >= 8)) return;
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
        m_regs.data = m_sio1fifo.byte();
        psxHu8(0x1050) = m_regs.data;
    }
    updateStat();
    return m_regs.data;
}

uint16_t PCSX::SIO1::readData16() {
    updateStat();
    if (m_regs.status & SR_RXRDY) {
        m_sio1fifo.read(&m_regs.data, 2);
        psxHu16(0x1050) = m_regs.data;
    }
    updateStat();
    return m_regs.data;
}

uint32_t PCSX::SIO1::readData32() {
    updateStat();
    if (m_regs.status & SR_RXRDY) {
        m_sio1fifo.read(&m_regs.data, 4);
        psxHu32(0x1050) = m_regs.data;
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
    updateStat();

    if (m_regs.control & CR_RXIRQEN) {
        if (!(m_regs.status & SR_IRQ)) {
            switch ((m_regs.control & 0x300) >> 8) {
                case 0:
                    if (!(m_sio1fifo.size() >= 1)) return;
                    break;

                case 1:
                    if (!(m_sio1fifo.size() >= 2)) return;
                    break;

                case 2:
                    if (!(m_sio1fifo.size() >= 4)) return;
                    break;

                case 3:
                    if (!(m_sio1fifo.size() >= 8)) return;
                    break;
            }

            scheduleInterrupt(SIO1_CYCLES);
            m_regs.status |= SR_IRQ;
        }
    }
}

void PCSX::SIO1::transmitData() {
    if (m_sio1Mode == SIO1Mode::Protobuf) {
        encodeDataMessage();
    } else if (m_sio1Mode == SIO1Mode::Raw) {
        if (m_fifo) m_fifo->write<uint8_t>(m_regs.data);
    }
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
    if (m_sio1fifo.size() > 0) {
        m_regs.status |= SR_RXRDY;
    } else {
        m_regs.status &= ~SR_RXRDY;
    }
    psxHu32ref(0x1054) = SWAP_LEu32(m_regs.status);
}

void PCSX::SIO1::writeBaud16(uint16_t v) {
    m_regs.baud = v;
    if (m_sio1Mode == SIO1Mode::Protobuf)
        encodeFCMessage();
    psxHu8ref(0x105E) = m_regs.baud;
}

void PCSX::SIO1::writeCtrl16(uint16_t v) {
    uint16_t old_ctrl = m_regs.control;
    m_regs.control = v;

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
        resetFifo();

        PCSX::g_emulator->m_cpu->m_regs.interrupt &= ~(1 << PCSX::PSXINT_SIO1);
    }
    if (m_sio1Mode == SIO1Mode::Protobuf)
        encodeFCMessage();
    psxHu16ref(0x105A) = SWAP_LE16(m_regs.control);
}

void PCSX::SIO1::writeData8(uint8_t v) {
    m_regs.data = v;
    if (isTransmitReady()) {
        transmitData();
    }
    psxHu8ref(0x1050) = m_regs.data;
}

void PCSX::SIO1::writeMode16(uint16_t v) {
    if (m_sio1Mode == SIO1Mode::Protobuf)
        encodeFCMessage();
    m_regs.mode = v;
}

void PCSX::SIO1::writeStat32(uint32_t v) {
    m_regs.status = v;
    if (isTransmitReady()) {
        transmitData();
    }
    psxHu32ref(0x1054) = SWAP_LE32(m_regs.status);
}

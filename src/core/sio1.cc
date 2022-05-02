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

PCSX::SIOPayload PCSX::SIO1::makePayloadFC(bool dxr, bool xts) {
    return SIOPayload {
        DataTransfer {},
    FlowControl { dxr, xts },
    };
}

PCSX::SIOPayload PCSX::SIO1::makePayloadData(std::string data) {
    bool xts = (m_regs.control & 32);
    bool dxr = (m_regs.control & 2);
    DataTransferData dt;
    dt = {data};
    return SIOPayload {
        DataTransfer {
            dt,
        },
        FlowControl {},
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
            printf("Error reading message\n");
            m_pbfifo.reset();
            checkSize = true;
            return false;
        }
//        printf("Decoding Message\n");
        SIOPayload payload;
        Protobuf::InSlice inslice(reinterpret_cast<const uint8_t *>(data.data()), data.size());
        payload.deserialize(&inslice, 0);
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
            if (m_pbfifo.size() > 0) {
                checkSize = true;
                goto read_length;
            }
        } else if (payload.get<FlowControlField>().hasData()) {
            printf("DXR HAS DATA %d\n", payload.get<FlowControlField>().get<FlowControlDXR>().hasData());
            printf("XTS HAS DATA %d\n", payload.get<FlowControlField>().get<FlowControlXTS>().hasData());

            if (payload.get<FlowControlField>().get<FlowControlDXR>().hasData()) {
                m_regs.status |= SR_DSR;
            } else {
                m_regs.status &= ~SR_DSR;
            }
            if (payload.get<FlowControlField>().get<FlowControlXTS>().hasData()) {
                m_regs.status |= SR_CTS;
            } else {
                m_regs.status &= ~SR_CTS;
            }
            updateStat();
        } else {
            m_regs.status &= ~SR_DSR;
            m_regs.status &= ~SR_CTS;
            updateStat();
        }
        printf("setting FC to dxr: %x, xts: %x\n", m_regs.status & SR_DSR, m_regs.status & SR_CTS);
        if (m_pbfifo.size() > 0) {
            checkSize = true;
            goto read_length;
        }
    checkSize = true;
    return true;
}

void PCSX::SIO1::encodeDataMessage() {
//    printf("Encoding data message\n");
    SIOPayload payload;
    std::string datastr(1, m_regs.data);
    payload = makePayloadData(datastr);

    Protobuf::OutSlice outslice;
    payload.serialize(&outslice);
    std::string data = outslice.finalize();
    uint8_t size = data.size();
    g_emulator->m_sio1Server->write(size);
    g_emulator->m_sio1Server->write(data);
}

void PCSX::SIO1::encodeFCMessage(bool dxr, bool xts) {
    printf("Encoding fc message dxr: %d, xts: %d\n", dxr, xts);
    SIOPayload payload;
    payload = makePayloadFC(dxr, xts);
    Protobuf::OutSlice outslice;
    payload.serialize(&outslice);
    std::string data = outslice.finalize();
    uint8_t size = data.size();
    g_emulator->m_sio1Server->write(size);
    g_emulator->m_sio1Server->write(data);
}

void PCSX::SIO1::sendfc() {
    exc_data exc_data_send;
    memset(&exc_data_send, 0x00, sizeof(exc_data_send));
    exc_data_send.reg = m_regs.control;
    exc_data_send.len = 0;
    g_emulator->m_sio1Server->write((uint8_t*)&exc_data_send, sizeof(exc_data_send));
}

void PCSX::SIO1::senddata(uint32_t data) {
    exc_data exc_data_send;
    exc_data_send.reg = m_regs.control;
    exc_data_send.data = (uint8_t)data;
    exc_data_send.len = 1;
    g_emulator->m_sio1Server->write((uint8_t*)&exc_data_send, sizeof(exc_data_send));
}

void PCSX::SIO1::receive() {
    exc_data exc_data_recv;
    read:
        memset(&exc_data_recv, 0x00, sizeof(exc_data_recv));
        if (m_pbfifo.size() >= sizeof(exc_data_recv)) {
            m_pbfifo.read(&exc_data_recv, sizeof(exc_data_recv));
                if (exc_data_recv.reg & CR_DTR) {
                    m_regs.status |= SR_DSR;
                } else {
                    m_regs.status &= ~SR_DSR;
                }
                if (exc_data_recv.reg & CR_RTSOUTLVL) {
                    m_regs.status |= SR_CTS;
                } else {
                    m_regs.status &= ~SR_CTS;
                }
            if (exc_data_recv.len == 1) {
                Slice slice;
                slice.copy(&exc_data_recv.data, 1);
                m_fifo.pushSlice(std::move(slice));
                receiveCallback();
            }
        } else {
            return;
        }
    if (m_pbfifo.size() >= sizeof(exc_data_recv)) {
        goto read;
    }
}

void PCSX::SIO1::exchange(int32_t data) {
    exc_data exc_data_send, exc_data_recv;
    memset(&exc_data_send, 0x00, sizeof(exc_data_send));
    memset(&exc_data_recv, 0x00, sizeof(exc_data_recv));
    if (data == -2) {
        if (m_pbfifo.size() >= sizeof(exc_data_recv)) {
            m_pbfifo.read(&exc_data_recv, sizeof(exc_data_recv));
            if (exc_data_recv.reg & CR_DTR) {
                m_regs.status |= SR_DSR;
            } else {
                m_regs.status &= ~SR_DSR;
            }
            if (exc_data_recv.reg & CR_RTSOUTLVL) {
                m_regs.status |= SR_CTS;
            } else {
                m_regs.status &= ~SR_CTS;
            }
            updateStat();
            if (exc_data_recv.len > 0) {
                Slice slice;
                slice.copy(&exc_data_recv.data, 1);
                m_fifo.pushSlice(std::move(slice));
                receiveCallback();
            }
        }
        return;
    }
    exc_data_send.reg = m_regs.control;
    exc_data_send.len = 0;
    if (data >= 0) {
        exc_data_send.data = (uint8_t)data;
        exc_data_send.len = 1;
    }
    g_emulator->m_sio1Server->write((uint8_t*)&exc_data_send, sizeof(exc_data_send));

//    if (m_pbfifo.size() >= sizeof(exc_data_recv)) {
//        m_pbfifo.read(&exc_data_recv, sizeof(exc_data_recv));
//        if (exc_data_recv.reg & CR_DTR) {
//            m_regs.status |= SR_DSR;
//        } else {
//            m_regs.status &= ~SR_DSR;
//        }
//        if (exc_data_recv.reg & CR_RTSOUTLVL) {
//            m_regs.status |= SR_CTS;
//        } else {
//            m_regs.status &= ~SR_CTS;
//        }
//    }
//
//    if (exc_data_recv.len > 0) {
//        Slice slice;
//        slice.copy(&exc_data_recv.data, 1);
//        m_fifo.pushSlice(std::move(slice));
//        receiveCallback();
//    }
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
//    printf("Read data8\n");
    updateStat();
    if (m_regs.status & SR_RXRDY) {
        m_regs.data = m_fifo.byte();
        psxHu8(0x1050) = m_regs.data;
    }
    updateStat();
    return m_regs.data;
}

uint16_t PCSX::SIO1::readData16() {
//    printf("Read data16\n");
    updateStat();
    if (m_regs.status & SR_RXRDY) {
        m_fifo.read(&m_regs.data, 2);
        psxHu16(0x1050) = m_regs.data;
    }
    updateStat();
    return m_regs.data;
}

uint32_t PCSX::SIO1::readData32() {
//    printf("Read data32\n");
    updateStat();
    if (m_regs.status & SR_RXRDY) {
        m_fifo.read(&m_regs.data, 4);
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

void PCSX::SIO1::queueTransmit() {
    Slice sl;
    sl.copy(static_cast<void*>(&m_regs.data), 1);
    m_txfifo.pushSlice(std::move(sl));
    encodeDataMessage();
}

void PCSX::SIO1::transmitData() {
//    printf("Transmit data\n");
    senddata(m_regs.data);
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
}
void PCSX::SIO1::writeBaud16(uint16_t v) {
    m_regs.baud = v;
    sendfc();
    psxHu8ref(0x105E) = m_regs.baud;

}

void PCSX::SIO1::writeCtrl16(uint16_t v) {
    uint16_t old_ctrl = m_regs.control;
    m_regs.control = v;
//    if (!(old_ctrl & CR_TXEN) && (m_regs.control & CR_TXEN)) {
//        if (isTransmitReady()) {
//            transmitData();
//        }
//    }

//    if (m_regs.control & CR_TXEN) {
//        if (!(m_regs.status & SR_IRQ)){
//            scheduleInterrupt(SIO1_CYCLES);
//            m_regs.status |= SWAP_LEu32(SR_IRQ);
//        }
//    }

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
    sendfc();
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
//    printf("writeMode16\n");
    sendfc();
    m_regs.mode = v;
}

void PCSX::SIO1::writeStat32(uint32_t v) {
    m_regs.status = v;
    if (isTransmitReady()) {
        transmitData();
    }

    psxHu32ref(0x1054) = SWAP_LE32(m_regs.status);

}

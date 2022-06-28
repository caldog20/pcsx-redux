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
#include <string>
#include <vector>

#include "core/r3000a.h"

#if defined(DYNAREC_X86_32)
#define CS_ARCH CS_ARCH_X86
#define CS_MODE CS_MODE_32
#elif defined(DYNAREC_X86_64)
#define CS_ARCH CS_ARCH_X86
#define CS_MODE CS_MODE_64
#elif defined(DYNAREC_AA64)
#define CS_ARCH CS_ARCH_ARM64
#define CS_MODE CS_MODE_ARM
#else  // Default to x86_32 if unknown architecture
#define CS_ARCH CS_ARCH_X86
#define CS_MODE CS_MODE_32
#endif

namespace PCSX {
class GUI;

namespace Widgets {

class Disassembly {
  public:
    Disassembly(bool& show) : m_show(show) {}
    void draw(GUI*, const char*);
    bool& m_show;

  private:
    std::vector<std::string> m_items;
    std::vector<std::string> m_history;
    int m_historyPos = -1;  // -1: new line, 0..History.Size-1 browsing history.
    bool m_autoScroll = true;
    bool m_scrollToBottom = false;
    bool m_mono = true;
    bool m_showError = false;
    bool m_outputFile = false;
    size_t m_codeSize = 0;

    // Class Methods
    size_t disassembleBuffer();
    void addInstruction(const std::string& str) {
        if (m_items.size() >= 320000) m_items.clear();
        m_items.push_back(str);
    }
    void writeFile();
};

}  // namespace Widgets
}  // namespace PCSX

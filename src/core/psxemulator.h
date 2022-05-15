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

/*
 * This file contains common definitions and includes for all parts of the
 * emulator core.
 */

#pragma once

// System includes
#include <assert.h>
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <zlib.h>

#include <filesystem>
#include <memory>
#include <string>

#include "support/settings.h"

#ifndef MAXPATHLEN
#ifdef _WIN32
#include "support/windowswrapper.h"
#endif
#ifdef MAX_PATH
#define MAXPATHLEN MAX_PATH
#endif
#ifdef PATH_MAX
#define MAXPATHLEN PATH_MAX
#endif
#endif

#ifndef PACKAGE_VERSION
#define PACKAGE_VERSION "0"
#endif

// Local includes from core - anything else from core is forbidden
#include "core/logger.h"
#include "core/system.h"
#include "support/strings-helpers.h"

#if defined(__linux__) || defined(__MACOSX__)
#define strnicmp strncasecmp
#endif

#ifdef _WIN32
typedef intptr_t ssize_t;
#endif

namespace PCSX {

class CallStacks;
class CDRom;
class Cheats;
class Counters;
class Debug;
class GdbServer;
class GPU;
class GTE;
class HW;
class Lua;
class MDEC;
class Memory;
class Pads;
class R3000Acpu;
class SIO;
class SPUInterface;
class System;
class WebServer;
class SIO1;
class SIO1Server;
class SIO1Client;

class Emulator;
extern Emulator* g_emulator;

class Emulator {
  public:
    Emulator();
    ~Emulator();
    Emulator(Emulator&&) = delete;
    Emulator(const Emulator&) = delete;
    Emulator& operator=(const Emulator&) = delete;
    enum VideoType { PSX_TYPE_NTSC = 0, PSX_TYPE_PAL };    // PSX Types
    enum CDDAType { CDDA_DISABLED = 0, CDDA_ENABLED_LE };  // CDDA Types
    struct OverlaySetting {
        typedef SettingPath<TYPESTRING("Filename")> Filename;
        typedef Setting<uint32_t, TYPESTRING("FileOffset")> FileOffset;
        typedef Setting<uint32_t, TYPESTRING("LoadOffset")> LoadOffset;
        typedef Setting<uint32_t, TYPESTRING("LoadSize")> LoadSize;
        typedef Setting<bool, TYPESTRING("Enabled")> Enabled;
        typedef Settings<Filename, FileOffset, LoadOffset, LoadSize, Enabled> type;
    };
    struct DebugSettings {
        typedef Setting<bool, TYPESTRING("Debug")> Debug;
        typedef Setting<bool, TYPESTRING("Trace")> Trace;
        typedef Setting<bool, TYPESTRING("KernelLog")> KernelLog;
        typedef Setting<uint32_t, TYPESTRING("FirstChanceException"), 0x00001cf0> FirstChanceException;
        typedef Setting<bool, TYPESTRING("SkipISR")> SkipISR;
        typedef Setting<bool, TYPESTRING("LoggingCDROM"), false> LoggingCDROM;
        typedef Setting<bool, TYPESTRING("GdbServer"), false> GdbServer;
        typedef Setting<bool, TYPESTRING("GdbManifest"), true> GdbManifest;
        enum class GdbLog {
            None,
            TTY,
            All,
        };
        typedef Setting<GdbLog, TYPESTRING("GdbLog"), GdbLog::TTY> GdbLogSetting;
        typedef Setting<int, TYPESTRING("GdbServerPort"), 3333> GdbServerPort;
        typedef Setting<bool, TYPESTRING("GdbServerTrace"), false> GdbServerTrace;
        typedef Setting<bool, TYPESTRING("WebServer"), false> WebServer;
        typedef Setting<int, TYPESTRING("WebServerPort"), 8080> WebServerPort;
        typedef Setting<uint32_t, TYPESTRING("KernelCallA0_00_1f"), 0xffffffff> KernelCallA0_00_1f;
        typedef Setting<uint32_t, TYPESTRING("KernelCallA0_20_3f"), 0xffffffff> KernelCallA0_20_3f;
        typedef Setting<uint32_t, TYPESTRING("KernelCallA0_40_5f"), 0xffffffff> KernelCallA0_40_5f;
        typedef Setting<uint32_t, TYPESTRING("KernelCallA0_60_7f"), 0xffffffff> KernelCallA0_60_7f;
        typedef Setting<uint32_t, TYPESTRING("KernelCallA0_80_9f"), 0xffffffff> KernelCallA0_80_9f;
        typedef Setting<uint32_t, TYPESTRING("KernelCallA0_a0_bf"), 0xffffffff> KernelCallA0_a0_bf;
        typedef Setting<uint32_t, TYPESTRING("KernelCallB0_00_1f"), 0xffffffff> KernelCallB0_00_1f;
        typedef Setting<uint32_t, TYPESTRING("KernelCallB0_20_3f"), 0xffffffff> KernelCallB0_20_3f;
        typedef Setting<uint32_t, TYPESTRING("KernelCallB0_40_5f"), 0xffffffff> KernelCallB0_40_5f;
        typedef Setting<uint32_t, TYPESTRING("KernelCallC0_00_1f"), 0xffffffff> KernelCallC0_00_1f;
        typedef Setting<bool, TYPESTRING("PCdrv"), false> PCdrv;
        typedef SettingPath<TYPESTRING("PCdrvBase")> PCdrvBase;
        typedef Setting<bool, TYPESTRING("SIO1Server"), false> SIO1Server;
        typedef Setting<int, TYPESTRING("SIO1ServerPort"), 6699> SIO1ServerPort;
        typedef Setting<bool, TYPESTRING("SIO1Client"), false> SIO1Client;
        typedef SettingString<TYPESTRING("SIO1Clienthost")> SIO1ClientHost;
        typedef Setting<int, TYPESTRING("SIO1ClientPort"), 6699> SIO1ClientPort;
        enum class SIO1Mode {
            Protobuf,
            Raw,
        };
        typedef Setting<SIO1Mode, TYPESTRING("SIO1Mode"), SIO1Mode::Protobuf> SIO1ModeSetting;
        typedef Settings<Debug, Trace, KernelLog, FirstChanceException, SkipISR, LoggingCDROM, GdbServer, GdbManifest,
                         GdbLogSetting, GdbServerPort, GdbServerTrace, WebServer, WebServerPort, KernelCallA0_00_1f,
                         KernelCallA0_20_3f, KernelCallA0_40_5f, KernelCallA0_60_7f, KernelCallA0_80_9f,
                         KernelCallA0_a0_bf, KernelCallB0_00_1f, KernelCallB0_20_3f, KernelCallB0_40_5f,
                         KernelCallC0_00_1f, PCdrv, PCdrvBase, SIO1Server, SIO1ServerPort, SIO1Client, SIO1ClientHost, SIO1ClientPort, SIO1ModeSetting>
            type;
    };
    typedef SettingNested<TYPESTRING("Debug"), DebugSettings::type> SettingDebugSettings;
    typedef SettingArray<TYPESTRING("Overlay"), OverlaySetting::type> SettingBiosOverlay;
    typedef Setting<bool, TYPESTRING("Stdout")> SettingStdout;
    typedef SettingPath<TYPESTRING("Logfile")> SettingLogfile;
    typedef SettingPath<TYPESTRING("Mcd1")> SettingMcd1;
    typedef SettingPath<TYPESTRING("Mcd2")> SettingMcd2;
    typedef SettingPath<TYPESTRING("Bios")> SettingBios;
    typedef SettingPath<TYPESTRING("PpfDir")> SettingPpfDir;
    typedef SettingPath<TYPESTRING("PsxExe")> SettingPsxExe;
    typedef Setting<bool, TYPESTRING("Xa"), true> SettingXa;
    typedef Setting<bool, TYPESTRING("SpuIrq")> SettingSpuIrq;
    typedef Setting<bool, TYPESTRING("BnWMdec")> SettingBnWMdec;
    typedef Setting<int, TYPESTRING("Scaler"), 100> SettingScaler;
    typedef Setting<bool, TYPESTRING("AutoVideo"), true> SettingAutoVideo;
    typedef Setting<VideoType, TYPESTRING("Video"), PSX_TYPE_NTSC> SettingVideo;
    typedef Setting<bool, TYPESTRING("FastBoot"), false> SettingFastBoot;
    typedef Setting<bool, TYPESTRING("RCntFix")> SettingRCntFix;
    typedef SettingPath<TYPESTRING("IsoPath")> SettingIsoPath;
    typedef SettingString<TYPESTRING("Locale")> SettingLocale;
    typedef Setting<bool, TYPESTRING("Mcd1Inserted"), true> SettingMcd1Inserted;
    typedef Setting<bool, TYPESTRING("Mcd2Inserted"), true> SettingMcd2Inserted;
    typedef Setting<bool, TYPESTRING("Dynarec"), true> SettingDynarec;
    typedef Setting<bool, TYPESTRING("8Megs"), false> Setting8MB;
    typedef Setting<int, TYPESTRING("GUITheme"), 0> SettingGUITheme;
    typedef Setting<int, TYPESTRING("Dither"), 2> SettingDither;
    typedef Setting<bool, TYPESTRING("ReportGLErrors"), false> SettingGLErrorReporting;
    typedef Setting<bool, TYPESTRING("FullCaching"), false> SettingFullCaching;
    typedef Setting<bool, TYPESTRING("ShownAutoUpdateConfig"), false> SettingShownAutoUpdateConfig;
    typedef Setting<bool, TYPESTRING("AutoUpdate"), false> SettingAutoUpdate;

    Settings<SettingStdout, SettingLogfile, SettingMcd1, SettingMcd2, SettingBios, SettingPpfDir, SettingPsxExe,
             SettingXa, SettingSpuIrq, SettingBnWMdec, SettingScaler, SettingAutoVideo, SettingVideo, SettingFastBoot,
             SettingDebugSettings, SettingRCntFix, SettingIsoPath, SettingLocale, SettingMcd1Inserted,
             SettingMcd2Inserted, SettingBiosOverlay, SettingDynarec, Setting8MB, SettingGUITheme, SettingDither,
             SettingGLErrorReporting, SettingFullCaching, SettingShownAutoUpdateConfig, SettingAutoUpdate>
        settings;
    class PcsxConfig {
      public:
        bool VSyncWA = false;
        bool PerGameMcd = false;
        bool Widescreen = false;
        bool HideCursor = false;
        bool SaveWindowPos = false;
        int32_t WindowPos[2] = {0, 0};
        uint32_t RewindCount = 0;
        uint32_t RewindInterval = 0;
        uint32_t AltSpeed1 = 0;  // Percent relative to natural speed.
        uint32_t AltSpeed2 = 0;
        uint8_t HackFix = 0;
        uint8_t MemHack = 0;
        bool OverClock = false;  // enable overclocking
        float PsxClock = 0.0f;
        // PGXP variables
        bool PGXP_GTE = false;
        bool PGXP_Cache = false;
        bool PGXP_Texture = false;
        uint32_t PGXP_Mode = 0;
    };

    // It is safe if these overflow
    uint32_t m_rewind_counter = 0;

    // Used for overclocking
    // Make the timing events trigger faster as we are currently assuming everything
    // takes one cycle, which is not the case on real hardware.
    // FIXME: Count the proper cycle and get rid of this
    uint32_t m_psxClockSpeed = 33868800 /* 33.8688 MHz */;
    enum { BIAS = 2 };

    int init();
    void reset();
    void shutdown();
    void vsync();
    void setPGXPMode(uint32_t pgxpMode);

    void setLua();

    PcsxConfig& config() { return m_config; }

    std::unique_ptr<CallStacks> m_callStacks;
    std::unique_ptr<CDRom> m_cdrom;
    std::unique_ptr<Cheats> m_cheats;
    std::unique_ptr<Counters> m_counters;
    std::unique_ptr<Debug> m_debug;
    std::unique_ptr<GdbServer> m_gdbServer;
    std::unique_ptr<GPU> m_gpu;
    std::unique_ptr<GTE> m_gte;
    std::unique_ptr<HW> m_hw;
    std::unique_ptr<Lua> m_lua;
    std::unique_ptr<MDEC> m_mdec;
    std::unique_ptr<Memory> m_mem;
    std::unique_ptr<Pads> m_pads;
    std::unique_ptr<R3000Acpu> m_cpu;
    std::unique_ptr<SIO> m_sio;
    std::unique_ptr<SIO1> m_sio1;
    std::unique_ptr<SIO1Server> m_sio1Server;
    std::unique_ptr<SIO1Client> m_sio1Client;
    std::unique_ptr<SPUInterface> m_spu;
    std::unique_ptr<WebServer> m_webServer;

  private:
    PcsxConfig m_config;
};

}  // namespace PCSX

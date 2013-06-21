/* ten90, a Mode S/A/C message decoding library.
 *
 * Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
 * Copyright 2013 John Wiseman <jjwiseman@gmail.com>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __TEN90_H
#define __TEN90_H

#include <stdint.h>

/* ============================= #defines =============================== */

// When changing, change also fixBitErrors() and modesInitErrorTable() !!
#define MODES_MAX_BITERRORS      2                          // Global max for fixable bit erros

#define MODEAC_MSG_SAMPLES       (25 * 2)                     // include up to the SPI bit
#define MODEAC_MSG_BYTES         2
#define MODEAC_MSG_SQUELCH_LEVEL 0x07FF                      // Average signal strength limit
#define MODEAC_MSG_FLAG          (1<<0)
#define MODEAC_MSG_MODES_HIT     (1<<1)
#define MODEAC_MSG_MODEA_HIT     (1<<2)
#define MODEAC_MSG_MODEC_HIT     (1<<3)
#define MODEAC_MSG_MODEA_ONLY    (1<<4)
#define MODEAC_MSG_MODEC_OLD     (1<<5)

#define MODES_PREAMBLE_US       8              // microseconds = bits
#define MODES_PREAMBLE_SAMPLES  (MODES_PREAMBLE_US       * 2)
#define MODES_PREAMBLE_SIZE     (MODES_PREAMBLE_SAMPLES  * sizeof(uint16_t))
#define MODES_LONG_MSG_BYTES    14
#define MODES_SHORT_MSG_BYTES   7
#define MODES_LONG_MSG_BITS     (MODES_LONG_MSG_BYTES    * 8)
#define MODES_SHORT_MSG_BITS    (MODES_SHORT_MSG_BYTES   * 8)
#define MODES_LONG_MSG_SAMPLES  (MODES_LONG_MSG_BITS     * 2)
#define MODES_SHORT_MSG_SAMPLES (MODES_SHORT_MSG_BITS    * 2)
#define MODES_LONG_MSG_SIZE     (MODES_LONG_MSG_SAMPLES  * sizeof(uint16_t))
#define MODES_SHORT_MSG_SIZE    (MODES_SHORT_MSG_SAMPLES * sizeof(uint16_t))

#define MODES_UNIT_FEET 0
#define MODES_UNIT_METERS 1

#define MODES_ACFLAGS_LATLON_VALID   (1<<0)  // Aircraft Lat/Lon is decoded
#define MODES_ACFLAGS_ALTITUDE_VALID (1<<1)  // Aircraft altitude is known
#define MODES_ACFLAGS_HEADING_VALID  (1<<2)  // Aircraft heading is known
#define MODES_ACFLAGS_SPEED_VALID    (1<<3)  // Aircraft speed is known
#define MODES_ACFLAGS_VERTRATE_VALID (1<<4)  // Aircraft vertical rate is known
#define MODES_ACFLAGS_SQUAWK_VALID   (1<<5)  // Aircraft Mode A Squawk is known
#define MODES_ACFLAGS_CALLSIGN_VALID (1<<6)  // Aircraft Callsign Identity
#define MODES_ACFLAGS_EWSPEED_VALID  (1<<7)  // Aircraft East West Speed is known
#define MODES_ACFLAGS_NSSPEED_VALID  (1<<8)  // Aircraft North South Speed is known
#define MODES_ACFLAGS_AOG            (1<<9)  // Aircraft is On the Ground
#define MODES_ACFLAGS_LLEVEN_VALID   (1<<10) // Aircraft Even Lot/Lon is known
#define MODES_ACFLAGS_LLODD_VALID    (1<<11) // Aircraft Odd Lot/Lon is known
#define MODES_ACFLAGS_AOG_VALID      (1<<12) // MODES_ACFLAGS_AOG is valid
#define MODES_ACFLAGS_FS_VALID       (1<<13) // Aircraft Flight Status is known
#define MODES_ACFLAGS_NSEWSPD_VALID  (1<<14) // Aircraft EW and NS Speed is known
#define MODES_ACFLAGS_LATLON_REL_OK  (1<<15) // Indicates it's OK to do a relative CPR

#define MODES_ACFLAGS_LLEITHER_VALID (MODES_ACFLAGS_LLEVEN_VALID | MODES_ACFLAGS_LLODD_VALID)
#define MODES_ACFLAGS_LLBOTH_VALID   (MODES_ACFLAGS_LLEVEN_VALID | MODES_ACFLAGS_LLODD_VALID)
#define MODES_ACFLAGS_AOG_GROUND     (MODES_ACFLAGS_AOG_VALID    | MODES_ACFLAGS_AOG)

/* ======================== structure declarations ========================= */

typedef struct {
  uint32_t *icao_cache;      // Recently seen ICAO addresses cache
  int nfix_crc;                  // Number of crc bit error(s) to correct
} Ten90Context;

// The struct we use to store information about a decoded message.
typedef struct {
    // Generic fields
    unsigned char msg[MODES_LONG_MSG_BYTES];      // Binary message.
    int           msgbits;                        // Number of bits in message
    int           msgtype;                        // Downlink format #
    int           crcok;                          // True if CRC was valid
    uint32_t      crc;                            // Message CRC
    int           correctedbits;                  // No. of bits corrected
    char          corrected[MODES_MAX_BITERRORS]; // corrected bit positions
    uint32_t      addr;                           // ICAO Address from bytes 1 2 and 3
    int           phase_corrected;                // True if phase correction was applied
    uint64_t      timestampMsg;                   // Timestamp of the message
    int           remote;                         // If set this message is from a remote station
    unsigned char signalLevel;                    // Signal Amplitude

    // DF 11
    int  ca;                    // Responder capabilities
    int  iid;

    // DF 17, DF 18
    int    metype;              // Extended squitter message type.
    int    mesub;               // Extended squitter message subtype.
    int    heading;             // Reported by aircraft, or computed from from EW and NS velocity
    int    raw_latitude;        // Non decoded latitude.
    int    raw_longitude;       // Non decoded longitude.
    double fLat;                // Coordinates obtained from CPR encoded data if/when decoded
    double fLon;                // Coordinates obtained from CPR encoded data if/when decoded
    char   flight[16];          // 8 chars flight number.
    int    ew_velocity;         // E/W velocity.
    int    ns_velocity;         // N/S velocity.
    int    vert_rate;           // Vertical rate.
    int    velocity;            // Reported by aircraft, or computed from from EW and NS velocity

    // DF4, DF5, DF20, DF21
    int  fs;                    // Flight status for DF4,5,20,21
    int  modeA;                 // 13 bits identity (Squawk).

    // Fields used by multiple message types.
    int  altitude;
    int  unit;
    int  bFlags;                // Flags related to fields in this structure
} Ten90Message;

/* ======================== function declarations ========================= */

#ifdef __cplusplus
extern "C" {
#endif

const char* Ten90GetVersion();
int Ten90ContextInit(Ten90Context *context);
void Ten90ContextDestroy(Ten90Context *context);
void Ten90DecodeModeAMessage(Ten90Message *mm, int ModeA);
int Ten90DecodeFrame(unsigned char *bytes, Ten90Context *context,
                     Ten90Message *mm);
uint32_t Ten90ModeSChecksum(unsigned char *msg, int bits);
void Ten90AddRecentlySeenIcaoAddr(uint32_t addr, Ten90Context*);
int Ten90IcaoAddressWasRecentlySeen(uint32_t addr, Ten90Context*);
int Ten90DecodeId13Field(int ID13Field);
int Ten90DecodeAc13Field(int AC13Field, int *unit);
int Ten90DecodeAc12Field(int AC12Field, int *unit);
int Ten90DecodeMovementField(int movement);
int Ten90ModeAToModeC(unsigned int ModeA);
int Ten90ModeSMessageLenByType(int type);
int Ten90FixBitErrors(unsigned char *msg, int bits, int maxfix, char *fixedbits);
int Ten90FixSingleBitErrors(unsigned char *msg, int bits);
int Ten90ModeAToModec(unsigned int ModeA);
void Ten90DisplayModeSMessage(Ten90Message *mm);

#ifdef __cplusplus
}
#endif

#endif // __TEN90_H

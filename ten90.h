// ten90, a Mode S/A/C message decoding library.
//
// Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
// Copyright 2013 John Wiseman <jjwiseman@gmail.com>
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef TEN90_H_
#define TEN90_H_

#include <stdint.h>


// When changing, change also fixBitErrors() and modesInitErrorTable() !!
#define MODES_MAX_BITERRORS      2          // Global max for fixable bit errors

#define MODES_LONG_MSG_BYTES    14
#define MODES_SHORT_MSG_BYTES   7
#define MODES_LONG_MSG_BITS     (MODES_LONG_MSG_BYTES    * 8)
#define MODES_SHORT_MSG_BITS    (MODES_SHORT_MSG_BYTES   * 8)
#define MODES_LONG_MSG_SAMPLES  (MODES_LONG_MSG_BITS     * 2)
#define MODES_SHORT_MSG_SAMPLES (MODES_SHORT_MSG_BITS    * 2)
#define MODES_LONG_MSG_SIZE     (MODES_LONG_MSG_SAMPLES  * sizeof(uint16_t))
#define MODES_SHORT_MSG_SIZE    (MODES_SHORT_MSG_SAMPLES * sizeof(uint16_t))

extern int kTen90FlagsLatLonValid;
extern int kTen90FlagsAltitudeValid;
extern int kTen90FlagsHeadingValid;
extern int kTen90FlagsSpeedValid;
extern int kTen90FlagsVerticalRateValid;
extern int kTen90FlagsSquawkValid;
extern int kTen90FlagsCallsignValid;
extern int kTen90FlagsEastWestSpeedValid;
extern int kTen90FlagsNorthSouthSpeedValid;
extern int kTen90FlagsAircraftOnGround;
extern int kTen90FlagsCprEvenValid;
extern int kTen90FlagsCprOddValid;
extern int kTen90FlagsAircraftOnGroundValid;
extern int kTen90FlagsFlightStatusValid;
extern int kTen90FlagsNorthSouthEastWestSpeedValid;

extern int kTen90DefaultIcaoCacheSize;
extern int kTen90DefaultIcaoCacheTtl;

extern int kTen90UnitFeet;
extern int kTen90UnitMeters;


typedef struct {
  uint32_t *icao_cache;         // Recently seen ICAO addresses cache
  int icao_cache_size;
  int icao_cache_ttl;
  int max_crc_bit_corrections;  // Number of crc bit error(s) to correct
} Ten90Context;


// The struct we use to store information about a decoded message.

typedef struct {
  // Generic fields
  unsigned char msg[MODES_LONG_MSG_BYTES];  // Binary message.
  int           msg_number_bits;      // Number of bits in message
  int           msg_type;             // Downlink format #
  int           crcok;                // True if CRC was valid
  uint32_t      crc;                  // Message CRC
  int           number_corrected_bits;  // No. of bits corrected
  char          corrected[MODES_MAX_BITERRORS];  // corrected bit positions
  uint32_t      addr;                 // ICAO Address from bytes 1 2 and 3
  int           phase_corrected;      // True if phase correction was applied
  uint64_t      msg_timestamp;        // Timestamp of the message
  int           remote;        // If set this message is from a remote station
  unsigned char signal_level;         // Signal Amplitude

  // DF 11
  int  ca;                    // Responder capabilities
  int  iid;

  // DF 17, DF 18
  int    es_type;             // Extended squitter message type.
  int    es_subtype;          // Extended squitter message subtype.
  // Reported by aircraft, or computed from from EW and NS velocity
  int    heading;
  int    raw_latitude;        // Non decoded latitude.
  int    raw_longitude;       // Non decoded longitude.
  // Coordinates obtained from CPR encoded data if/when decoded
  double decoded_lat;
  // Coordinates obtained from CPR encoded data if/when decoded
  double decoded_lon;
  char   flight[16];          // 8 chars flight number.
  int    ew_velocity;         // E/W velocity.
  int    ns_velocity;         // N/S velocity.
  int    vert_rate;           // Vertical rate.
  // Reported by aircraft, or computed from from EW and NS velocity
  int    velocity;

  // DF4, DF5, DF20, DF21
  int  fs;                    // Flight status for DF4,5,20,21
  int  mode_a;                // 13 bits identity (Squawk).

  // Fields used by multiple message types.
  int  altitude;
  int  unit;
  int  flags;                 // Flags related to fields in this structure
} Ten90Frame;


#ifdef __cplusplus
extern "C" {
#endif

  const char* Ten90GetVersion();
  int Ten90ContextInit(Ten90Context *context, int icao_cache_size,
                       int icao_cache_ttl);
  void Ten90ContextDestroy(Ten90Context *context);
  void Ten90DecodeModeAFrame(Ten90Frame *frame, int ModeA);
  int Ten90DecodeFrame(unsigned char *bytes, Ten90Context *context,
                       Ten90Frame *frame);
  uint32_t Ten90ModeSChecksum(unsigned char *msg, int bits);
  void Ten90AddRecentlySeenIcaoAddr(Ten90Context *context, uint32_t addr);
  int Ten90IcaoAddressWasRecentlySeen(Ten90Context *context, uint32_t addr);
  int Ten90DecodeId13Field(int ID13Field);
  int Ten90DecodeAc13Field(int AC13Field, int *unit);
  int Ten90DecodeAc12Field(int AC12Field, int *unit);
  int Ten90DecodeMovementField(int movement);
  int Ten90ModeAToModeC(unsigned int ModeA);
  int Ten90ModeSMessageLenByType(int type);
  int Ten90FixBitErrors(unsigned char *msg, int bits, int maxfix,
                        char *fixedbits);
  int Ten90FixSingleBitErrors(unsigned char *msg, int bits);
  int Ten90ModeAToModeC(unsigned int ModeA);
  void Ten90DisplayFrame(Ten90Frame *frame);
  int Ten90DecodeCPRRelative(double reference_lat, double reference_lon,
                             int odd, int surface, int cpr_lat, int cpr_lon,
                             double *decoded_lat, double *decoded_lon);
  int Ten90DecodeCpr(int even_cpr_lat, int even_cpr_lon,
                     int odd_cpr_lat, int odd_cpr_lon,
                     double receiver_lat, double receiver_lon,
                     int odd, int surface,
                     double *result_lat, double *result_lon);

#ifdef __cplusplus
}
#endif

#endif  // TEN90_H_

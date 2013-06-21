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
  // Recently seen ICAO addresses cache
  uint32_t *icao_cache;
  int icao_cache_size;
  int icao_cache_ttl;
  // Number of crc bit error(s) to correct
  int max_crc_bit_corrections;
} Ten90Context;


// The struct we use to store information about a decoded message.

typedef struct {
  // ---------- Generic fields
  // Binary message.
  unsigned char msg[MODES_LONG_MSG_BYTES];
  // Number of bits in message.
  int msg_number_bits;
  // Downlink format #.
  int msg_type;
  // True if CRC was valid.
  int crc_ok;
  // Message CRC.
  uint32_t crc;
  // No. of bits corrected.
  int number_corrected_bits;
  // corrected bit positions.
  char corrected[MODES_MAX_BITERRORS];
  // ICAO Address from bytes 1 2 and 3.
  uint32_t addr;
  // True if phase correction was applied.
  int phase_corrected;
  // Timestamp of the message.
  uint64_t msg_timestamp;
  // If set this message is from a remote station.
  int remote;
  // Signal Amplitude.
  unsigned char signal_level;

  // ---------- DF 11
  // Responder capabilities.
  int ca;
  int iid;

  // ---------- DF 17, DF 18
  // Extended squitter message type.
  int es_type;
  // Extended squitter message subtype.
  int es_subtype;
  // Reported by aircraft, or computed from from EW and NS velocity.
  int heading;
  // Non decoded latitude.
  int raw_latitude;
  // Non decoded longitude.
  int raw_longitude;
  // Coordinates obtained from CPR encoded data if/when decoded.
  double decoded_lat;
  // Coordinates obtained from CPR encoded data if/when decoded.
  double decoded_lon;
  // 8 chars flight number.
  char flight[16];
  // East/west velocity.
  int ew_velocity;
  // North/south velocity.
  int ns_velocity;
  // Vertical rate.
  int vert_rate;
  // Reported by aircraft, or computed from from EW and NS velocity.
  int velocity;

  // ---------- DF 4, DF 5, DF 20, DF 21
  // Flight status.
  int fs;
  // 13 bits identity (squawk).
  int mode_a;

  // Fields used by multiple message types.
  int altitude;
  int unit;
  // Flags related to fields in this structure.
  int flags;
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

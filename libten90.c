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

#include "ten90.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>


int kTen90DefaultIcaoCacheSize = 1024;
int kTen90DefaultIcaoCacheTtl = 60;

int kTen90UnitFeet = 0;
int kTen90UnitMeters = 1;


// Returns the library version.  See http://semver.org/

const char *Ten90GetVersion(void) {
  return "2.0.0";
}


// Decodes a raw Mode S message demodulated as a stream of bytes and
// split it into fields populating a Ten90Frame structure.

int Ten90DecodeFrame(unsigned char *bytes,
                     Ten90Context *context,
                     Ten90Frame *frame) {
  char *ais_charset = "?ABCDEFGHIJKLMNOPQRSTUVWXYZ????? "
                      "???????????????0123456789??????";

  // Work on our local copy
  memcpy(frame->msg, bytes, MODES_LONG_MSG_BYTES);
  bytes = frame->msg;

  // Get the message type ASAP as other operations depend on this
  frame->msg_type = bytes[0] >> 3;  // Downlink Format
  frame->msg_number_bits = Ten90ModeSMessageLenByType(frame->msg_type);
  frame->crc = Ten90ModeSChecksum(bytes, frame->msg_number_bits);

  if ((frame->crc) && (context->max_crc_fixes) &&
      ((frame->msg_type == 17) || (frame->msg_type == 18))) {
    //  if ((frame->crc) && (Modes.nfix_crc) && ((frame->msg_type == 11) ||
    //  (frame->msg_type == 17))) {
    //
    // Fixing single bit errors in DF-11 is a bit dodgy because we
    // have no way to know for sure if the crc is supposed to be 0 or
    // not - it could be any value less than 80. Therefore, attempting
    // to fix DF-11 errors can result in a multitude of possible crc
    // solutions, only one of which is correct.
    //
    // We should probably perform some sanity checks on corrected
    // DF-11's before using the results. Perhaps check the ICAO
    // against known aircraft, and check IID against known good
    // IID's. That's a TODO.
    frame->number_corrected_bits = Ten90FixBitErrors(
        bytes, frame->msg_number_bits, context->max_crc_fixes,
        frame->corrected);

    // If we correct, validate ICAO addr to help filter birthday
    // paradox solutions.
    if (frame->number_corrected_bits) {
      uint32_t addr = (bytes[1] << 16) | (bytes[2] << 8) | (bytes[3]);
      if (!Ten90IcaoAddressWasRecentlySeen(context, addr))
        frame->number_corrected_bits = 0;
    }
  }
  // Note that most of the other computation happens *after* we fix
  // the single/two bit errors, otherwise we would need to recompute
  // the fields again.
  if (frame->msg_type == 11) {
    // DF 11
    frame->crcok = (frame->crc < 80);
    frame->iid = frame->crc;
    frame->addr = (bytes[1] << 16) | (bytes[2] << 8) | (bytes[3]);
    frame->ca = (bytes[0] & 0x07);  // Responder capabilities

    if (0 == frame->crc) {
      // DF 11 : if crc == 0 try to populate our ICAO addresses
      // whitelist.
      Ten90AddRecentlySeenIcaoAddr(context, frame->addr);
    }
  } else if (frame->msg_type == 17) {
    // DF 17
    frame->crcok = (frame->crc == 0);
    frame->addr = (bytes[1] << 16) | (bytes[2] << 8) | (bytes[3]);
    frame->ca = (bytes[0] & 0x07);  // Responder capabilities

    if (0 == frame->crc) {
      // DF 17 : if crc == 0 try to populate our ICAO addresses
      // whitelist.
      Ten90AddRecentlySeenIcaoAddr(context, frame->addr);
    }

  } else if (frame->msg_type == 18) {
    // DF 18
    frame->crcok = (frame->crc == 0);
    frame->addr = (bytes[1] << 16) | (bytes[2] << 8) | (bytes[3]);
    frame->ca = (bytes[0] & 0x07);  // Control Field

    if (0 == frame->crc) {
      // DF 18 : if crc == 0 try to populate our ICAO addresses
      // whitelist.
      Ten90AddRecentlySeenIcaoAddr(context, frame->addr);
    }

  } else {
    // All other DF's.  Compare the checksum with the whitelist of
    // recently seen ICAO addresses. If it matches one, then declare
    // the message as valid
    frame->addr  = frame->crc;
    frame->crcok = Ten90IcaoAddressWasRecentlySeen(context, frame->crc);
  }

  // Fields for DF0, DF16
  if (frame->msg_type == 0  || frame->msg_type == 16) {
    if (bytes[0] & 0x04) {                       // VS Bit
      frame->flags |= MODES_ACFLAGS_AOG_VALID | MODES_ACFLAGS_AOG;
    } else {
      frame->flags |= MODES_ACFLAGS_AOG_VALID;
    }
  }

  // Fields for DF11, DF17
  if (frame->msg_type == 11 || frame->msg_type == 17) {
    if (frame->ca == 4) {
      frame->flags |= MODES_ACFLAGS_AOG_VALID | MODES_ACFLAGS_AOG;
    } else if (frame->ca == 5) {
      frame->flags |= MODES_ACFLAGS_AOG_VALID;
    }
  }

  // Fields for DF5, DF21 = Gillham encoded Squawk
  if (frame->msg_type == 5  || frame->msg_type == 21) {
    int ID13Field = ((bytes[2] << 8) | bytes[3]) & 0x1FFF;
    if (ID13Field) {
      frame->flags |= MODES_ACFLAGS_SQUAWK_VALID;
      frame->mode_a = Ten90DecodeId13Field(ID13Field);
    }
  }

  // Fields for DF0, DF4, DF16, DF20 13 bit altitude
  if (frame->msg_type == 0  || frame->msg_type == 4 ||
      frame->msg_type == 16 || frame->msg_type == 20) {
    int AC13Field = ((bytes[2] << 8) | bytes[3]) & 0x1FFF;
    if (AC13Field) {
      // Only attempt to decode if a valid (non zero) altitude is
      // present
      frame->flags |= MODES_ACFLAGS_ALTITUDE_VALID;
      frame->altitude = Ten90DecodeAc13Field(AC13Field, &frame->unit);
    }
  }

  // Fields for DF4, DF5, DF20, DF21
  if ((frame->msg_type == 4) || (frame->msg_type == 20) ||
      (frame->msg_type == 5) || (frame->msg_type == 21)) {
    frame->flags |= MODES_ACFLAGS_FS_VALID;
    frame->fs = bytes[0] & 7;               // Flight status for DF4,5,20,21
    if (frame->fs <= 3) {
      frame->flags |= MODES_ACFLAGS_AOG_VALID;
      if (frame->fs & 1)
      {frame->flags |= MODES_ACFLAGS_AOG;}
    }
  }

  // Fields for DF17, DF18_CF0, DF18_CF1, DF18_CF6 squitters
  if ((frame->msg_type == 17) ||
      ((frame->msg_type == 18) &&
       ((frame->ca == 0) || (frame->ca == 1) || (frame->ca == 6)))) {
    // Extended squitter message type
    int es_type = frame->es_type = bytes[4] >> 3;
    // Extended squitter message subtype
    int es_subtype = frame->es_subtype = bytes[4]  & 7;

    // Decode the extended squitter message
    if (es_type >= 1 && es_type <= 4) {
      // Aircraft Identification and Category
      uint32_t chars;
      frame->flags |= MODES_ACFLAGS_CALLSIGN_VALID;

      chars = (bytes[5] << 16) | (bytes[6] << 8) | (bytes[7]);
      frame->flight[3] = ais_charset[chars & 0x3F]; chars = chars >> 6;
      frame->flight[2] = ais_charset[chars & 0x3F]; chars = chars >> 6;
      frame->flight[1] = ais_charset[chars & 0x3F]; chars = chars >> 6;
      frame->flight[0] = ais_charset[chars & 0x3F];

      chars = (bytes[8] << 16) | (bytes[9] << 8) | (bytes[10]);
      frame->flight[7] = ais_charset[chars & 0x3F]; chars = chars >> 6;
      frame->flight[6] = ais_charset[chars & 0x3F]; chars = chars >> 6;
      frame->flight[5] = ais_charset[chars & 0x3F]; chars = chars >> 6;
      frame->flight[4] = ais_charset[chars & 0x3F];

      frame->flight[8] = '\0';

    } else if (es_type >= 5 && es_type <= 18) {
      // Position Message
      frame->raw_latitude  = (((bytes[6] & 3) << 15) | (bytes[7] << 7) |
                              (bytes[8] >> 1));
      frame->raw_longitude = (((bytes[8] & 1) << 16) | (bytes[9] << 8) |
                              (bytes[10]));
      frame->flags |= (frame->msg[6] & 0x04) ? MODES_ACFLAGS_LLODD_VALID
                      : MODES_ACFLAGS_LLEVEN_VALID;
      if (es_type >= 9) {
        // Airborne
        int AC12Field = ((bytes[5] << 4) | (bytes[6] >> 4)) & 0x0FFF;
        frame->flags |= MODES_ACFLAGS_AOG_VALID;
        if (AC12Field) {
          // Only attempt to decode if a valid (non zero) altitude is present
          frame->flags |= MODES_ACFLAGS_ALTITUDE_VALID;
          frame->altitude = Ten90DecodeAc12Field(AC12Field, &frame->unit);
        }
      } else {
        // Ground
        int movement = ((bytes[4] << 4) | (bytes[5] >> 4)) & 0x007F;
        frame->flags |= MODES_ACFLAGS_AOG_VALID | MODES_ACFLAGS_AOG;
        if ((movement) && (movement < 125)) {
          frame->flags |= MODES_ACFLAGS_SPEED_VALID;
          frame->velocity = Ten90DecodeMovementField(movement);
        }

        if (bytes[5] & 0x08) {
          frame->flags |= MODES_ACFLAGS_HEADING_VALID;
          frame->heading = (((((bytes[5] << 4) |
                               (bytes[6] >> 4)) & 0x007F) * 45) >> 4);
        }
      }

    } else if (es_type == 19) {
      // Airborne Velocity Message.  Presumably airborne if we get an
      // Airborne Velocity Message
      frame->flags |= MODES_ACFLAGS_AOG_VALID;

      if ( (es_subtype >= 1) && (es_subtype <= 4) ) {
        int vert_rate = ((bytes[8] & 0x07) << 6) | (bytes[9] >> 2);
        if (vert_rate) {
          --vert_rate;
          if (bytes[8] & 0x08) {
            vert_rate = 0 - vert_rate;
          }
          frame->vert_rate =  vert_rate * 64;
          frame->flags |= MODES_ACFLAGS_VERTRATE_VALID;
        }
      }

      if ((es_subtype == 1) || (es_subtype == 2)) {
        int ew_raw = ((bytes[5] & 0x03) << 8) |  bytes[6];
        int ew_vel = ew_raw - 1;
        int ns_raw = ((bytes[7] & 0x7F) << 3) | (bytes[8] >> 5);
        int ns_vel = ns_raw - 1;

        if (es_subtype == 2) {
          // If (supersonic) unit is 4 kts
          ns_vel = ns_vel << 2;
          ew_vel = ew_vel << 2;
        }

        if (ew_raw) {
          // Do East/West
          frame->flags |= MODES_ACFLAGS_EWSPEED_VALID;
          if (bytes[5] & 0x04)
          {ew_vel = 0 - ew_vel;}
          frame->ew_velocity = ew_vel;
        }

        if (ns_raw) {
          // Do North/South
          frame->flags |= MODES_ACFLAGS_NSSPEED_VALID;
          if (bytes[7] & 0x80)
          {ns_vel = 0 - ns_vel;}
          frame->ns_velocity = ns_vel;
        }

        if (ew_raw && ns_raw) {
          // Compute velocity and angle from the two speed components
          frame->flags |= (MODES_ACFLAGS_SPEED_VALID |
                           MODES_ACFLAGS_HEADING_VALID |
                           MODES_ACFLAGS_NSEWSPD_VALID);
          frame->velocity = (int) sqrt((ns_vel * ns_vel) + (ew_vel * ew_vel));
          if (frame->velocity) {
            frame->heading = (int) (atan2(ew_vel, ns_vel) * 180.0 / M_PI);
            // We don't want negative values but a 0-360 scale
            if (frame->heading < 0) frame->heading += 360;
          }
        }
      } else if (es_subtype == 3 || es_subtype == 4) {
        int airspeed = ((bytes[7] & 0x7f) << 3) | (bytes[8] >> 5);
        if (airspeed) {
          frame->flags |= MODES_ACFLAGS_SPEED_VALID;
          --airspeed;
          if (es_subtype == 4) {
            // If (supersonic) unit is 4 kts
            airspeed = airspeed << 2;
          }
          frame->velocity =  airspeed;
        }

        if (bytes[5] & 0x04) {
          frame->flags |= MODES_ACFLAGS_HEADING_VALID;
          frame->heading = ((((bytes[5] & 0x03) << 8) | bytes[6]) * 45) >> 7;
        }
      }
    }
  }

  // Fields for DF20, DF21 Comm-B
  if ((frame->msg_type == 20) || (frame->msg_type == 21)) {
    if (bytes[4] == 0x20) {
      // Aircraft Identification
      uint32_t chars;
      frame->flags |= MODES_ACFLAGS_CALLSIGN_VALID;
      chars = (bytes[5] << 16) | (bytes[6] << 8) | (bytes[7]);
      frame->flight[3] = ais_charset[chars & 0x3F]; chars = chars >> 6;
      frame->flight[2] = ais_charset[chars & 0x3F]; chars = chars >> 6;
      frame->flight[1] = ais_charset[chars & 0x3F]; chars = chars >> 6;
      frame->flight[0] = ais_charset[chars & 0x3F];

      chars = (bytes[8] << 16) | (bytes[9] << 8) | (bytes[10]);
      frame->flight[7] = ais_charset[chars & 0x3F]; chars = chars >> 6;
      frame->flight[6] = ais_charset[chars & 0x3F]; chars = chars >> 6;
      frame->flight[5] = ais_charset[chars & 0x3F]; chars = chars >> 6;
      frame->flight[4] = ais_charset[chars & 0x3F];

      frame->flight[8] = '\0';
    } else {
    }
  }
}


void Ten90DecodeModeAFrame(Ten90Frame *frame, int mode_a) {
  // Valid Mode S DF's are DF-00 to DF-31, so use 32 to indicate Mode
  // A/C.
  frame->msg_type = 32;

  // Fudge up a Mode S style data stream.
  frame->msg_number_bits = 16;
  frame->msg[0] = (mode_a >> 8);
  frame->msg[1] = (mode_a);

  // Fudge an ICAO address based on Mode A (remove the Ident bit) Use
  // an upper address byte of FF, since this is ICAO unallocated.
  frame->addr = 0x00FF0000 | (mode_a & 0x0000FF7F);

  // Set the Identity field to Mode A.
  frame->mode_a   = mode_a & 0x7777;
  frame->flags |= MODES_ACFLAGS_SQUAWK_VALID;

  // Flag ident in flight status.
  frame->fs = mode_a & 0x0080;

  // Not much else we can tell from a Mode A/C reply.  Just fudge up a
  // few bits to keep other code happy.
  frame->crcok = 1;
  frame->number_corrected_bits = 0;
}


// ========== Mode S detection and decoding

// Parity table for Mode S Messages.
//
// The table contains 112 elements, every element corresponds to a bit
// set in the message, starting from the first bit of actual data
// after the preamble.
//
// For messages of 112 bit, the whole table is used.  For messages of
// 56 bits only the last 56 elements are used.
//
// The algorithm is as simple as xoring all the elements in this table
// for which the corresponding bit on the message is set to 1.
//
// The latest 24 elements in this table are set to 0 as the checksum
// at the end of the message should not affect the computation.
//
// Note: this function can be used with DF11 and DF17, other modes
// have the CRC xored with the sender address as they are reply to
// interrogations, but a casual listener can't split the address from
// the checksum.

uint32_t modes_checksum_table[112] = {
  0x3935ea, 0x1c9af5, 0xf1b77e, 0x78dbbf, 0xc397db, 0x9e31e9, 0xb0e2f0,
  0x587178, 0x2c38bc, 0x161c5e, 0x0b0e2f, 0xfa7d13, 0x82c48d, 0xbe9842,
  0x5f4c21, 0xd05c14, 0x682e0a, 0x341705, 0xe5f186, 0x72f8c3, 0xc68665,
  0x9cb936, 0x4e5c9b, 0xd8d449, 0x939020, 0x49c810, 0x24e408, 0x127204,
  0x093902, 0x049c81, 0xfdb444, 0x7eda22, 0x3f6d11, 0xe04c8c, 0x702646,
  0x381323, 0xe3f395, 0x8e03ce, 0x4701e7, 0xdc7af7, 0x91c77f, 0xb719bb,
  0xa476d9, 0xadc168, 0x56e0b4, 0x2b705a, 0x15b82d, 0xf52612, 0x7a9309,
  0xc2b380, 0x6159c0, 0x30ace0, 0x185670, 0x0c2b38, 0x06159c, 0x030ace,
  0x018567, 0xff38b7, 0x80665f, 0xbfc92b, 0xa01e91, 0xaff54c, 0x57faa6,
  0x2bfd53, 0xea04ad, 0x8af852, 0x457c29, 0xdd4410, 0x6ea208, 0x375104,
  0x1ba882, 0x0dd441, 0xf91024, 0x7c8812, 0x3e4409, 0xe0d800, 0x706c00,
  0x383600, 0x1c1b00, 0x0e0d80, 0x0706c0, 0x038360, 0x01c1b0, 0x00e0d8,
  0x00706c, 0x003836, 0x001c1b, 0xfff409, 0x000000, 0x000000, 0x000000,
  0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
  0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
  0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000
};


uint32_t Ten90ModeSChecksum(unsigned char *msg, int bits) {
  uint32_t crc = 0;
  uint32_t rem = 0;
  int offset = (bits == 112) ? 0 : (112-56);
  uint8_t theByte = *msg;
  uint32_t *pCRCTable = &modes_checksum_table[offset];
  int j;

  // We don't really need to include the checksum itself.
  bits -= 24;
  for (j = 0; j < bits; j++) {
    if ((j & 7) == 0)
      theByte = *msg++;

    // If bit is set, xor with corresponding table entry.
    if (theByte & 0x80) {crc ^= *pCRCTable;}
    pCRCTable++;
    theByte = theByte << 1;
  }

  rem = (msg[0] << 16) | (msg[1] << 8) | msg[2];  // message checksum
  // 24 bit checksum syndrome.
  return ((crc ^ rem) & 0x00FFFFFF);
}


// Hash the ICAO address to index our cache (the size of which must be
// a power of 2).

uint32_t Ten90IcaoCacheHashAddress(Ten90Context *context, uint32_t a) {
  // The following three rounds wil make sure that every bit affects
  // every output bit with ~ 50% of probability.
  a = ((a >> 16) ^ a) * 0x45d9f3b;
  a = ((a >> 16) ^ a) * 0x45d9f3b;
  a = ((a >> 16) ^ a);
  return a & (context->icao_cache_size - 1);
}


// Add the specified entry to the cache of recently seen ICAO
// addresses.  Note that we also add a timestamp so that we can make
// sure that the entry is only valid for until the TTL expires.

void Ten90AddRecentlySeenIcaoAddr(Ten90Context *context, uint32_t addr) {
  uint32_t h = Ten90IcaoCacheHashAddress(context, addr);
  context->icao_cache[h*2] = addr;
  context->icao_cache[h*2+1] = (uint32_t) time(NULL);
}


// Returns 1 if the specified ICAO address was seen in a DF format
// with proper checksum (not xored with address) recently.  Otherwise
// returns 0.

int Ten90IcaoAddressWasRecentlySeen(Ten90Context *context, uint32_t addr) {
  uint32_t h = Ten90IcaoCacheHashAddress(context, addr);
  uint32_t a = context->icao_cache[h*2];
  uint32_t t = context->icao_cache[h*2+1];
  return a && a == addr && time(NULL) - t <= context->icao_cache_ttl;
}


// In the squawk (identity) field bits are interleaved as follows in
// (message bit 20 to bit 32):
//
// C1-A1-C2-A2-C4-A4-ZERO-B1-D1-B2-D2-B4-D4
//
// So every group of three bits A, B, C, D represent an integer from 0
// to 7.
//
// The actual meaning is just 4 octal numbers, but we convert it into
// a hex number tha happens to represent the four octal numbers.
//
// For more info: http://en.wikipedia.org/wiki/Gillham_code

int Ten90DecodeId13Field(int ID13Field) {
  int hexGillham = 0;
  if (ID13Field & 0x1000) {hexGillham |= 0x0010;}  // Bit 12 = C1
  if (ID13Field & 0x0800) {hexGillham |= 0x1000;}  // Bit 11 = A1
  if (ID13Field & 0x0400) {hexGillham |= 0x0020;}  // Bit 10 = C2
  if (ID13Field & 0x0200) {hexGillham |= 0x2000;}  // Bit  9 = A2
  if (ID13Field & 0x0100) {hexGillham |= 0x0040;}  // Bit  8 = C4
  if (ID13Field & 0x0080) {hexGillham |= 0x4000;}  // Bit  7 = A4
  // if (ID13Field & 0x0040) {hexGillham |= 0x0800;}  // Bit  6 = X  or M
  if (ID13Field & 0x0020) {hexGillham |= 0x0100;}  // Bit  5 = B1
  if (ID13Field & 0x0010) {hexGillham |= 0x0001;}  // Bit  4 = D1 or Q
  if (ID13Field & 0x0008) {hexGillham |= 0x0200;}  // Bit  3 = B2
  if (ID13Field & 0x0004) {hexGillham |= 0x0002;}  // Bit  2 = D2
  if (ID13Field & 0x0002) {hexGillham |= 0x0400;}  // Bit  1 = B4
  if (ID13Field & 0x0001) {hexGillham |= 0x0004;}  // Bit  0 = D4
  return (hexGillham);
}


// Decode the 13 bit AC altitude field (in DF 20 and others).  Returns
// the altitude, and set 'unit' to either kTen90UnitMeters or
// kTen90UnitFeet.

int Ten90DecodeAc13Field(int AC13Field, int *unit) {
  // set = meters, clear = feet.
  int m_bit = AC13Field & 0x0040;
  // set = 25 ft encoding, clear = Gillham Mode C encoding.
  int q_bit = AC13Field & 0x0010;

  if (!m_bit) {
    *unit = kTen90UnitFeet;
    if (q_bit) {
      // N is the 11 bit integer resulting from the removal of bit Q and M.
      int n = ((AC13Field & 0x1F80) >> 2) |
              ((AC13Field & 0x0020) >> 1) |
              (AC13Field & 0x000F);
      // The final altitude is resulting number multiplied by 25, minus 1000.
      return ((n * 25) - 1000);
    } else {
      // N is an 11 bit Gillham coded altitude.
      int n = Ten90ModeAToModeC(Ten90DecodeId13Field(AC13Field));
      if (n < -12) {n = 0;}
      return (100 * n);
    }
  } else {
    *unit = kTen90UnitMeters;
    // TODO(wiseman): Implement altitude when meter unit is selected.
  }
  return 0;
}


// Decode the 12 bit AC altitude field (in DF 17 and others).

int Ten90DecodeAc12Field(int AC12Field, int *unit) {
  int q_bit  = AC12Field & 0x10;  // Bit 48 = Q

  *unit = kTen90UnitFeet;
  if (q_bit) {
    // N is the 11 bit integer resulting from the removal of bit Q at bit 4.
    int n = ((AC12Field & 0x0FE0) >> 1) |
            (AC12Field & 0x000F);
    // The final altitude is the resulting number multiplied by 25, minus 1000.
    return ((n * 25) - 1000);
  } else {
    // Make N a 13 bit Gillham coded altitude by inserting M=0 at bit 6.
    int n = ((AC12Field & 0x0FC0) << 1) |
            (AC12Field & 0x003F);
    n = Ten90ModeAToModeC(Ten90DecodeId13Field(n));
    if (n < -12) {
      n = 0;
    }
    return 100 * n;
  }
}


// Decode the 7 bit ground movement field PWL exponential style scale.

int Ten90DecodeMovementField(int movement) {
  int gspeed;

  // Movement codes 0,125,126,127 are all invalid.
  if (0 == movement || 125 == movement || 126 == movement ||
      127 == movement) {
    return -1;
  }
  if      (movement  > 123) gspeed = 199;  // > 175kt
  else if (movement  > 108) gspeed = ((movement - 108)  * 5) + 100;
  else if (movement  >  93) gspeed = ((movement -  93)  * 2) +  70;
  else if (movement  >  38) gspeed = ((movement -  38)     ) +  15;
  else if (movement  >  12) gspeed = ((movement -  11) >> 1) +   2;
  else if (movement  >   8) gspeed = ((movement -   6) >> 2) +   1;
  else                      gspeed = 0;
  return (gspeed);
}


// Input format is : 00:A4:A2:A1:00:B4:B2:B1:00:C4:C2:C1:00:D4:D2:D1

int Ten90ModeAToModeC(unsigned int ModeA) {
  unsigned int FiveHundreds = 0;
  unsigned int OneHundreds  = 0;

  if ((ModeA & 0xFFFF888B) || ((ModeA & 0x000000F0) == 0)) {
    // D1 set is illegal. D2 set is > 62700ft which is unlikely.
    // C1,,C4 cannot be zero.
    return -9999;
  }

  if (ModeA & 0x0010) {OneHundreds ^= 0x007;}  // C1
  if (ModeA & 0x0020) {OneHundreds ^= 0x003;}  // C2
  if (ModeA & 0x0040) {OneHundreds ^= 0x001;}  // C4

  // Remove 7s from OneHundreds (Make 7->5, snd 5->7).
  if ((OneHundreds & 5) == 5) {OneHundreds ^= 2;}

  // Check for invalid codes, only 1 to 5 are valid
  if (OneHundreds > 5) {
    return -9999;
  }

  // if (ModeA & 0x0001) {FiveHundreds ^= 0x1FF;} // D1 never used for altitude
  if (ModeA & 0x0002) {FiveHundreds ^= 0x0FF;}  // D2
  if (ModeA & 0x0004) {FiveHundreds ^= 0x07F;}  // D4

  if (ModeA & 0x1000) {FiveHundreds ^= 0x03F;}  // A1
  if (ModeA & 0x2000) {FiveHundreds ^= 0x01F;}  // A2
  if (ModeA & 0x4000) {FiveHundreds ^= 0x00F;}  // A4

  if (ModeA & 0x0100) {FiveHundreds ^= 0x007;}  // B1
  if (ModeA & 0x0200) {FiveHundreds ^= 0x003;}  // B2
  if (ModeA & 0x0400) {FiveHundreds ^= 0x001;}  // B4

  // Correct order of OneHundreds.
  if (FiveHundreds & 1) {OneHundreds = 6 - OneHundreds;}

  return ((FiveHundreds * 5) + OneHundreds - 13);
}


// Given the Downlink Format (DF) of the message, return the message
// length in bits.
//
// All known DF's 16 or greater are long. All known DF's 15 or less
// are short.  There are lots of unused codes in both category, so we
// can assume ICAO will stick to these rules, meaning that the most
// significant bit of the DF indicates the length.

int Ten90ModeSMessageLenByType(int type) {
  return (type & 0x10) ? MODES_LONG_MSG_BITS : MODES_SHORT_MSG_BITS;
}


/*
 * Code for introducing a less CPU-intensive method of correcting
 * single bit errors.
 *
 * Makes use of the fact that the crc checksum is linear with respect
 * to the bitwise xor operation, i.e.
 *
 *      crc(m^e) = (crc(m)^crc(e)
 *
 * where m and e are the message resp. error bit vectors.
 *
 * Call crc(e) the syndrome.
 *
 * The code below works by precomputing a table of (crc(e), e) for all
 * possible error vectors e (here only single bit and double bit
 * errors), search for the syndrome in the table, and correct the then
 * known error.  The error vector e is represented by one or two bit
 * positions that are changed. If a second bit position is not used,
 * it is -1.
 *
 * Run-time is binary search in a sorted table, plus some constant
 * overhead, instead of running through all possible bit positions
 * (respective pairs of bit positions).
 */

struct errorinfo {
  uint32_t syndrome;             // CRC syndrome
  int bits;                      // Number of bit positions to fix
  int pos[MODES_MAX_BITERRORS];  // Bit positions corrected by this syndrome
};

#define NERRORINFO                                                      \
  (MODES_LONG_MSG_BITS+MODES_LONG_MSG_BITS*(MODES_LONG_MSG_BITS-1)/2)
static struct errorinfo bitErrorTable[NERRORINFO];


// Compare function as needed for stdlib's qsort and bsearch
// functions.

static int CmpErrorInfo(const void *p0, const void *p1) {
  struct errorinfo *e0 = (struct errorinfo*)p0;
  struct errorinfo *e1 = (struct errorinfo*)p1;
  if (e0->syndrome == e1->syndrome) {
    return 0;
  } else if (e0->syndrome < e1->syndrome) {
    return -1;
  } else {
    return 1;
  }
}


// Search for syndrome in table and if an entry is found, flip the
// necessary bits. Make sure the indices fit into the array Additional
// parameter: fix only less than maxcorrected bits, and record fixed
// bit positions in corrected[]. This array can be NULL, otherwise
// must be of length at least maxcorrected.
//
// Return number of fixed bits.

int Ten90FixBitErrors(unsigned char *msg, int msg_number_bits, int maxfix,
                      char *fixedbits) {
  struct errorinfo *pei;
  struct errorinfo ei;
  int bitpos, offset, res, i;

  memset(&ei, 0, sizeof(struct errorinfo));
  ei.syndrome = Ten90ModeSChecksum(msg, msg_number_bits);
  pei = bsearch(&ei, bitErrorTable, NERRORINFO,
                sizeof(struct errorinfo), CmpErrorInfo);
  if (pei == NULL) {
    return 0;  // No syndrome found.
  }

  // Check if the syndrome fixes more bits than we allow.
  if (maxfix < pei->bits) {
    return 0;
  }

  // Check that all bit positions lie inside the message length.
  offset = MODES_LONG_MSG_BITS - msg_number_bits;
  for (i = 0;  i < pei->bits;  i++) {
    bitpos = pei->pos[i] - offset;
    if ((bitpos < 0) || (bitpos >= msg_number_bits)) {
      return 0;
    }
  }

  // Fix the bits.
  for (i = res = 0; i < pei->bits; i++) {
    bitpos = pei->pos[i] - offset;
    msg[bitpos >> 3] ^= (1 << (7 - (bitpos & 7)));
    if (fixedbits) {
      fixedbits[res++] = bitpos;
    }
  }
  return res;
}


// Try to fix single bit errors using the checksum. On success
// modifies the original buffer with the fixed version, and returns
// the position of the error bit. Otherwise if fixing failed -1 is
// returned.
//
// Can only be used for DF17.

int Ten90FixSingleBitErrors(unsigned char *msg, int msg_number_bits) {
  int j;
  unsigned char aux[MODES_LONG_MSG_BYTES];

  memcpy(aux, msg, msg_number_bits / 8);

  // Do not attempt to error correct Bits 0-4. These contain the DF,
  // and must be correct because we can only error correct DF17
  for (j = 5; j < msg_number_bits; j++) {
    int byte_idx = j/8;
    int bitmask = 1 << (7 - (j & 7));
    aux[byte_idx] ^= bitmask;  // Flip j-th bit
    if (0 == Ten90ModeSChecksum(aux, msg_number_bits)) {
      // The error is fixed. Overwrite the original buffer with the
      // corrected sequence, and returns the error bit position.
      msg[byte_idx] = aux[byte_idx];
      return j;
    }
    aux[byte_idx] ^= bitmask;  // Flip j-th bit back again
  }
  return (-1);
}


// Compute the table of all syndromes for 1-bit and 2-bit error
// vectors.

static void InitErrorInfo() {
  unsigned char msg[MODES_LONG_MSG_BYTES];
  int i, j, n;
  uint32_t crc;
  n = 0;
  memset(bitErrorTable, 0, sizeof(bitErrorTable));
  memset(msg, 0, MODES_LONG_MSG_BYTES);
  // Add all possible single and double bit errors don't include
  // errors in first 5 bits (DF type)
  for (i = 5;  i < MODES_LONG_MSG_BITS;  i++) {
    int bytepos0 = (i >> 3);
    int mask0 = 1 << (7 - (i & 7));
    msg[bytepos0] ^= mask0;  // create error0
    crc = Ten90ModeSChecksum(msg, MODES_LONG_MSG_BITS);
    bitErrorTable[n].syndrome = crc;  // single bit error case
    bitErrorTable[n].bits = 1;
    bitErrorTable[n].pos[0] = i;
    bitErrorTable[n].pos[1] = -1;
    n += 1;

    for (j = i+1;  j < MODES_LONG_MSG_BITS;  j++) {
      int bytepos1 = (j >> 3);
      int mask1 = 1 << (7 - (j & 7));
      msg[bytepos1] ^= mask1;  // create error1
      crc = Ten90ModeSChecksum(msg, MODES_LONG_MSG_BITS);
      if (n >= NERRORINFO) {
        // fprintf(stderr,
        //         "Internal error, too many entries, fix NERRORINFO\n");
        break;
      }
      bitErrorTable[n].syndrome = crc;  // two bit error case
      bitErrorTable[n].bits = 2;
      bitErrorTable[n].pos[0] = i;
      bitErrorTable[n].pos[1] = j;
      n += 1;
      msg[bytepos1] ^= mask1;  // revert error1
    }
    msg[bytepos0] ^= mask0;  // revert error0
  }
  qsort(bitErrorTable, NERRORINFO, sizeof(struct errorinfo), CmpErrorInfo);

  // Test code: report if any syndrome appears at least twice. In this
  // case the correction cannot be done without ambiguity.
  // Tried it, does not happen for 1- and 2-bit errors.
  /*
    for (i = 1;  i < NERRORINFO;  i++) {
    if (bitErrorTable[i-1].syndrome == bitErrorTable[i].syndrome) {
    fprintf(stderr, "modesInitErrorInfo: Collision for syndrome %06x\n",
    (int)bitErrorTable[i].syndrome);
    }
    }

    for (i = 0;  i < NERRORINFO;  i++) {
    printf("syndrome %06x    bit0 %3d    bit1 %3d\n",
    bitErrorTable[i].syndrome,
    bitErrorTable[i].pos0, bitErrorTable[i].pos1);
    }
  */
}


/* Capability table. */
static char *ca_str[8] = {
  /* 0 */ "Level 1 (Survillance Only)",
  /* 1 */ "Level 2 (DF0,4,5,11)",
  /* 2 */ "Level 3 (DF0,4,5,11,20,21)",
  /* 3 */ "Level 4 (DF0,4,5,11,20,21,24)",
  /* 4 */ "Level 2+3+4 (DF0,4,5,11,20,21,24,code7 - is on ground)",
  /* 5 */ "Level 2+3+4 (DF0,4,5,11,20,21,24,code7 - is on airborne)",
  /* 6 */ "Level 2+3+4 (DF0,4,5,11,20,21,24,code7)",
  /* 7 */ "Level 7 ???"
};

// DF 18 Control field table.
static char *cf_str[8] = {
  /* 0 */ "ADS-B ES/NT device with ICAO 24-bit address",
  /* 1 */ "ADS-B ES/NT device with other address",
  /* 2 */ "Fine format TIS-B",
  /* 3 */ "Coarse format TIS-B",
  /* 4 */ "TIS-B managment message",
  /* 5 */ "TIS-B relay of ADS-B message with other address",
  /* 6 */ "ADS-B rebroadcast using DF-17 message format",
  /* 7 */ "Reserved"
};

/* Flight status table. */
static char *fs_str[8] = {
  /* 0 */ "Normal, Airborne",
  /* 1 */ "Normal, On the ground",
  /* 2 */ "ALERT,  Airborne",
  /* 3 */ "ALERT,  On the ground",
  /* 4 */ "ALERT & Special Position Identification. Airborne or Ground",
  /* 5 */ "Special Position Identification. Airborne or Ground",
  /* 6 */ "Value 6 is not assigned",
  /* 7 */ "Value 7 is not assigned"
};

static char *getMEDescription(int es_type, int mesub) {
  char *mename = "Unknown";

  if (es_type >= 1 && es_type <= 4)
    mename = "Aircraft Identification and Category";
  else if (es_type >= 5 && es_type <= 8)
    mename = "Surface Position";
  else if (es_type >= 9 && es_type <= 18)
    mename = "Airborne Position (Baro Altitude)";
  else if (es_type == 19 && mesub >=1 && mesub <= 4)
    mename = "Airborne Velocity";
  else if (es_type >= 20 && es_type <= 22)
    mename = "Airborne Position (GNSS Height)";
  else if (es_type == 23 && mesub == 0)
    mename = "Test Message";
  else if (es_type == 24 && mesub == 1)
    mename = "Surface System Status";
  else if (es_type == 28 && mesub == 1)
    mename = "Extended Squitter Aircraft Status (Emergency)";
  else if (es_type == 28 && mesub == 2)
    mename = "Extended Squitter Aircraft Status (1090ES TCAS RA)";
  else if (es_type == 29 && (mesub == 0 || mesub == 1))
    mename = "Target State and Status Message";
  else if (es_type == 31 && (mesub == 0 || mesub == 1))
    mename = "Aircraft Operational Status Message";
  return mename;
}


// This function gets a decoded Mode S Message and prints it on the
// screen in a human readable format.

void Ten90DisplayFrame(Ten90Frame *frame) {
  int j;
  unsigned char * pTimeStamp;

  // Handle only addresses mode first.
  /* if (Modes.onlyaddr) { */
  /*     printf("%06x\n", frame->addr); */
  /*     return;         // Enough for --onlyaddr mode */
  /* } */

  // Show the raw message.
  /* if (Modes.mlat && frame->timestampMsg) { */
  /*     printf("@"); */
  /*     pTimeStamp = (unsigned char *) &frame->timestampMsg; */
  /*     for (j=5; j>=0;j--) { */
  /*         printf("%02X",pTimeStamp[j]); */
  /*     } */
  /* } else */
  /*     printf("*"); */

  printf("*");
  for (j = 0; j < frame->msg_number_bits/8; j++) printf("%02x", frame->msg[j]);
  printf(";\n");

  /* if (Modes.raw) { */
  /*     fflush(stdout); // Provide data to the reader ASAP */
  /*     return;         // Enough for --raw mode */
  /* } */

  if (frame->msg_type < 32)
    printf("CRC: %06x (%s)\n", (int)frame->crc, frame->crcok ? "ok" : "wrong");

  if (frame->number_corrected_bits != 0)
    printf("No. of bit errors fixed: %d\n", frame->number_corrected_bits);

  if (frame->msg_type == 0) {
    // DF 0
    printf("DF 0: Short Air-Air Surveillance.\n");
    printf("  Altitude       : %d %s\n", frame->altitude,
           (frame->unit == kTen90UnitMeters) ? "meters" : "feet");
    printf("  ICAO Address   : %06x\n", frame->addr);

  } else if (frame->msg_type == 4 || frame->msg_type == 20) {
    printf("DF %d: %s, Altitude Reply.\n", frame->msg_type,
           (frame->msg_type == 4) ? "Surveillance" : "Comm-B");
    printf("  Flight Status  : %s\n", fs_str[frame->fs]);
    printf("  DR             : %d\n", ((frame->msg[1] >> 3) & 0x1F));
    printf("  UM             : %d\n",
           (((frame->msg[1]  & 7) << 3) | (frame->msg[2] >> 5)));
    printf("  Altitude       : %d %s\n", frame->altitude,
           (frame->unit == kTen90UnitMeters) ? "meters" : "feet");
    printf("  ICAO Address   : %06x\n", frame->addr);

    if (frame->msg_type == 20) {
      printf("  Comm-B BDS     : %x\n", frame->msg[4]);
      // Decode the extended squitter message
      if (frame->msg[4] == 0x20) {
        // BDS 2,0 Aircraft identification
        printf("    BDS 2,0 Aircraft Identification : %s\n", frame->flight);
      }
    }
  } else if (frame->msg_type == 5 || frame->msg_type == 21) {
    printf("DF %d: %s, Identity Reply.\n", frame->msg_type,
           (frame->msg_type == 5) ? "Surveillance" : "Comm-B");
    printf("  Flight Status  : %s\n", fs_str[frame->fs]);
    printf("  DR             : %d\n", ((frame->msg[1] >> 3) & 0x1F));
    printf("  UM             : %d\n",
           (((frame->msg[1]  & 7) << 3) | (frame->msg[2] >> 5)));
    printf("  Squawk         : %x\n", frame->mode_a);
    printf("  ICAO Address   : %06x\n", frame->addr);

    if (frame->msg_type == 21) {
      printf("  Comm-B BDS     : %x\n", frame->msg[4]);

      // Decode the extended squitter message
      if (frame->msg[4] == 0x20) {
        // BDS 2,0 Aircraft identification
        printf("    BDS 2,0 Aircraft Identification : %s\n", frame->flight);
      }
    }
  } else if (frame->msg_type == 11) {
    // DF 11
    printf("DF 11: All Call Reply.\n");
    printf("  Capability  : %d (%s)\n", frame->ca, ca_str[frame->ca]);
    printf("  ICAO Address: %06x\n", frame->addr);
    if (frame->iid > 16)
    {printf("  IID         : SI-%02d\n", frame->iid-16);}
    else
    {printf("  IID         : II-%02d\n", frame->iid);}

  } else if (frame->msg_type == 16) {
    // DF 16
    printf("DF 16: Long Air to Air ACAS\n");

  } else if (frame->msg_type == 17) {
    // DF 17
    printf("DF 17: ADS-B message.\n");
    printf("  Capability     : %d (%s)\n", frame->ca, ca_str[frame->ca]);
    printf("  ICAO Address   : %06x\n", frame->addr);
    printf("  Extended Squitter  Type: %d\n", frame->es_type);
    printf("  Extended Squitter  Sub : %d\n", frame->es_subtype);
    printf("  Extended Squitter  Name: %s\n",
           getMEDescription(frame->es_type, frame->es_subtype));
    // Decode the extended squitter message
    if (frame->es_type >= 1 && frame->es_type <= 4) {
      // Aircraft identification
      printf("    Aircraft Type  : %c%d\n",
             ('A' + 4 - frame->es_type), frame->es_subtype);
      printf("    Identification : %s\n", frame->flight);

      // } else if (frame->es_type >= 5 && frame->es_type <= 8) {
      //   // Surface position

    } else if (frame->es_type >= 9 && frame->es_type <= 18) {
      // Airborne position Baro
      printf("    F flag   : %s\n", (frame->msg[6] & 0x04) ? "odd" : "even");
      printf("    T flag   : %s\n", (frame->msg[6] & 0x08) ? "UTC" : "non-UTC");
      printf("    Altitude : %d feet\n", frame->altitude);
      if (frame->flags & MODES_ACFLAGS_LATLON_VALID) {
        printf("    Latitude : %f\n", frame->decoded_lat);
        printf("    Longitude: %f\n", frame->decoded_lon);
      } else {
        printf("    Latitude : %d (not decoded)\n", frame->raw_latitude);
        printf("    Longitude: %d (not decoded)\n", frame->raw_longitude);
      }

    } else if (frame->es_type == 19) {
      // Airborne Velocity
      if (frame->es_subtype == 1 || frame->es_subtype == 2) {
        printf("    EW status         : %s\n",
               (frame->flags & MODES_ACFLAGS_EWSPEED_VALID) ?
               "Valid" : "Unavailable");
        printf("    EW velocity       : %d\n", frame->ew_velocity);
        printf("    NS status         : %s\n",
               (frame->flags & MODES_ACFLAGS_NSSPEED_VALID) ?
               "Valid" : "Unavailable");
        printf("    NS velocity       : %d\n", frame->ns_velocity);
        printf("    Vertical status   : %s\n",
               (frame->flags & MODES_ACFLAGS_VERTRATE_VALID) ?
               "Valid" : "Unavailable");
        printf("    Vertical rate src : %d\n", ((frame->msg[8] >> 4) & 1));
        printf("    Vertical rate     : %d\n", frame->vert_rate);

      } else if (frame->es_subtype == 3 || frame->es_subtype == 4) {
        printf("    Heading status    : %s\n",
               (frame->flags & MODES_ACFLAGS_HEADING_VALID) ?
               "Valid" : "Unavailable");
        printf("    Heading           : %d\n", frame->heading);
        printf("    Airspeed status   : %s\n",
               (frame->flags & MODES_ACFLAGS_SPEED_VALID) ?
               "Valid" : "Unavailable");
        printf("    Airspeed          : %d\n", frame->velocity);
        printf("    Vertical status   : %s\n",
               (frame->flags & MODES_ACFLAGS_VERTRATE_VALID) ?
               "Valid" : "Unavailable");
        printf("    Vertical rate src : %d\n", ((frame->msg[8] >> 4) & 1));
        printf("    Vertical rate     : %d\n", frame->vert_rate);

      } else {
        printf("    Unrecognized ME subtype: %d subtype: %d\n",
               frame->es_type, frame->es_subtype);
      }

      // } else if (frame->es_type >= 20 && frame->es_type <= 22) {
      //   // Airborne position GNSS
    } else {
      printf("    Unrecognized ME type: %d subtype: %d\n",
             frame->es_type, frame->es_subtype);
    }

  } else if (frame->msg_type == 18) {
    // DF 18
    printf("DF 18: Extended Squitter.\n");
    printf("  Control Field : %d (%s)\n", frame->ca, cf_str[frame->ca]);
    if ((frame->ca == 0) || (frame->ca == 1) || (frame->ca == 6)) {
      if (frame->ca == 1) {
        printf("  Other Address : %06x\n", frame->addr);
      } else {
        printf("  ICAO Address  : %06x\n", frame->addr);
      }
      printf("  Extended Squitter  Type: %d\n", frame->es_type);
      printf("  Extended Squitter  Sub : %d\n", frame->es_subtype);
      printf("  Extended Squitter  Name: %s\n",
             getMEDescription(frame->es_type, frame->es_subtype));

      // Decode the extended squitter message
      if (frame->es_type >= 1 && frame->es_type <= 4) {
        // Aircraft identification
        printf("    Aircraft Type  : %c%d\n",
               ('A' + 4 - frame->es_type), frame->es_subtype);
        printf("    Identification : %s\n", frame->flight);

        // } else if (frame->es_type >= 5 && frame->es_type <= 8) {
        //   // Surface position

      } else if (frame->es_type >= 9 && frame->es_type <= 18) {
        // Airborne position Baro
        printf("    F flag   : %s\n", (frame->msg[6] & 0x04) ? "odd" : "even");
        printf("    T flag   : %s\n",
               (frame->msg[6] & 0x08) ? "UTC" : "non-UTC");
        printf("    Altitude : %d feet\n", frame->altitude);
        if (frame->flags & MODES_ACFLAGS_LATLON_VALID) {
          printf("    Latitude : %f\n", frame->decoded_lat);
          printf("    Longitude: %f\n", frame->decoded_lon);
        } else {
          printf("    Latitude : %d (not decoded)\n", frame->raw_latitude);
          printf("    Longitude: %d (not decoded)\n", frame->raw_longitude);
        }

      } else if (frame->es_type == 19) {
        // Airborne Velocity
        if (frame->es_subtype == 1 || frame->es_subtype == 2) {
          printf("    EW status         : %s\n",
                 (frame->flags & MODES_ACFLAGS_EWSPEED_VALID) ?
                 "Valid" : "Unavailable");
          printf("    EW velocity       : %d\n", frame->ew_velocity);
          printf("    NS status         : %s\n",
                 (frame->flags & MODES_ACFLAGS_NSSPEED_VALID) ?
                 "Valid" : "Unavailable");
          printf("    NS velocity       : %d\n", frame->ns_velocity);
          printf("    Vertical status   : %s\n",
                 (frame->flags & MODES_ACFLAGS_VERTRATE_VALID) ?
                 "Valid" : "Unavailable");
          printf("    Vertical rate src : %d\n", ((frame->msg[8] >> 4) & 1));
          printf("    Vertical rate     : %d\n", frame->vert_rate);

        } else if (frame->es_subtype == 3 || frame->es_subtype == 4) {
          printf("    Heading status    : %s\n",
                 (frame->flags & MODES_ACFLAGS_HEADING_VALID) ?
                 "Valid" : "Unavailable");
          printf("    Heading           : %d\n", frame->heading);
          printf("    Airspeed status   : %s\n",
                 (frame->flags & MODES_ACFLAGS_SPEED_VALID) ?
                 "Valid" : "Unavailable");
          printf("    Airspeed          : %d\n", frame->velocity);
          printf("    Vertical status   : %s\n",
                 (frame->flags & MODES_ACFLAGS_VERTRATE_VALID) ?
                 "Valid" : "Unavailable");
          printf("    Vertical rate src : %d\n", ((frame->msg[8] >> 4) & 1));
          printf("    Vertical rate     : %d\n", frame->vert_rate);

        } else {
          printf("    Unrecognized ME subtype: %d subtype: %d\n",
                 frame->es_type, frame->es_subtype);
        }

        //  } else if (frame->es_type >= 20 && frame->es_type <= 22) {
        //    // Airborne position GNSS

      } else {
        printf("    Unrecognized ME type: %d subtype: %d\n",
               frame->es_type, frame->es_subtype);
      }
    }

  } else if (frame->msg_type == 19) {
    // DF 19
    printf("DF 19: Military Extended Squitter.\n");

  } else if (frame->msg_type == 22) {
    // DF 22
    printf("DF 22: Military Use.\n");

  } else if (frame->msg_type == 24) {
    // DF 24
    printf("DF 24: Comm D Extended Length Message.\n");

  } else if (frame->msg_type == 32) {
    // DF 32 is special code we use for Mode A/C
    printf("SSR : Mode A/C Reply.\n");
    if (frame->fs & 0x0080) {
      printf("  Mode A : %04x IDENT\n", frame->mode_a);
    } else {
      printf("  Mode A : %04x\n", frame->mode_a);
      if (frame->flags & MODES_ACFLAGS_ALTITUDE_VALID)
      {printf("  Mode C : %d feet\n", frame->altitude);}
    }

  } else {
    printf("DF %d: Unknown DF Format.\n", frame->msg_type);
  }

  printf("\n");
}


// See http://graphics.stanford.edu/~seander/bithacks.html#RoundUpPowerOf2
static int NearestPowerOf2(int n) {
  n--;
  n |= n >> 1;
  n |= n >> 2;
  n |= n >> 4;
  n |= n >> 8;
  n |= n >> 16;
  n++;
  return n;
}


int Ten90ContextInit(Ten90Context *context, int icao_cache_size,
                     int icao_cache_ttl) {
  memset(context, 0, sizeof(Ten90Context));
  icao_cache_size = NearestPowerOf2(icao_cache_size);
  context->icao_cache_size = icao_cache_size;
  context->icao_cache_ttl = icao_cache_ttl;
  if ((context->icao_cache = (uint32_t *)calloc(
          icao_cache_size * 2,
          sizeof(uint32_t))) == NULL) {
    return -1;
  }
  InitErrorInfo(context);
  return 0;
}

void Ten90ContextDestroy(Ten90Context *context) {
  free(context->icao_cache);
}

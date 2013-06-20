#include "ten90.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

//
// Turn an hex digit into its 4 bit decimal value.
// Returns -1 if the digit is not in the 0-F range.
static int hexDigitVal(int c) {
    c = tolower(c);
    if (c >= '0' && c <= '9') return c-'0';
    else if (c >= 'a' && c <= 'f') return c-'a'+10;
    else return -1;
}
//
// This function decodes a string representing message in raw hex format
// like: *8D4B969699155600E87406F5B69F; The string is null-terminated.
//
// The message is passed to the higher level layers, so it feeds
// the selected screen output, the network output and so forth.
//
// If the message looks invalid it is silently discarded.
//
// The function always returns 0 (success) to the caller as there is no
// case where we want broken messages here to close the client connection.
int ten90_decode_hex_message(struct modesMessage *mm, char *hex, ten90_context *context) {
    int l = strlen(hex), j;
    unsigned char msg[MODES_LONG_MSG_BYTES];
    memset(mm, 0, sizeof(mm));

    // Mark messages received over the internet as remote so that we don't try to
    // pass them off as being received by this instance when forwarding them
    mm->remote      =    1;
    mm->signalLevel = 0xFF;

    // Remove spaces on the left and on the right
    while(l && isspace(hex[l-1])) {
        hex[l-1] = '\0'; l--;
    }
    while(isspace(*hex)) {
        hex++; l--;
    }

    // Turn the message into binary.
    // Accept *-AVR raw @-AVR/BEAST timeS+raw %-AVR timeS+raw (CRC good) <-BEAST timeS+sigL+raw
    // and some AVR records that we can understand
    if (hex[l-1] != ';') {return (0);} // not complete - abort

    switch(hex[0]) {
        case '<': {
            mm->signalLevel = (hexDigitVal(hex[13])<<4) | hexDigitVal(hex[14]);
            hex += 15; l -= 16; // Skip <, timestamp and siglevel, and ;
            break;}

        case '@':     // No CRC check
        case '%': {   // CRC is OK
            hex += 13; l -= 14; // Skip @,%, and timestamp, and ;
            break;}

        case '*':
        case ':': {
            hex++; l-=2; // Skip * and ;
            break;}

        default: {
            return (0); // We don't know what this is, so abort
            break;}
    }

    if ( (l != (MODEAC_MSG_BYTES      * 2))
      && (l != (MODES_SHORT_MSG_BYTES * 2))
      && (l != (MODES_LONG_MSG_BYTES  * 2)) )
        {return (0);} // Too short or long message... broken

    if ( (0 == context->mode_ac)
      && (l == (MODEAC_MSG_BYTES * 2)) )
        {return (0);} // Right length for ModeA/C, but not enabled

    for (j = 0; j < l; j += 2) {
        int high = hexDigitVal(hex[j]);
        int low  = hexDigitVal(hex[j+1]);

        if (high == -1 || low == -1) return 0;
        msg[j/2] = (high << 4) | low;
    }

    if (l == (MODEAC_MSG_BYTES * 2)) {  // ModeA or ModeC
      ten90_decode_mode_a_message(mm, ((msg[0] << 8) | msg[1]));
    } else {       // Assume ModeS
      ten90_decode_mode_s_message(mm, msg, context);
    }
    return (0);
}


//
// Decode a raw Mode S message demodulated as a stream of bytes by detectModeS(),
// and split it into fields populating a modesMessage structure.
//
void ten90_decode_mode_s_message(struct modesMessage *mm, unsigned char *msg,
                                 ten90_context *context) {
    char *ais_charset = "?ABCDEFGHIJKLMNOPQRSTUVWXYZ????? ???????????????0123456789??????";

    // Work on our local copy
    memcpy(mm->msg, msg, MODES_LONG_MSG_BYTES);
    msg = mm->msg;

    // Get the message type ASAP as other operations depend on this
    mm->msgtype         = msg[0] >> 3; // Downlink Format
    mm->msgbits         = ten90_mode_s_message_len_by_type(mm->msgtype);
    mm->crc             = ten90_mode_s_checksum(msg, mm->msgbits);

    if ((mm->crc) && (context->nfix_crc) && ((mm->msgtype == 17) || (mm->msgtype == 18))) {
//  if ((mm->crc) && (Modes.nfix_crc) && ((mm->msgtype == 11) || (mm->msgtype == 17))) {
        //
        // Fixing single bit errors in DF-11 is a bit dodgy because we have no way to
        // know for sure if the crc is supposed to be 0 or not - it could be any value
        // less than 80. Therefore, attempting to fix DF-11 errors can result in a
        // multitude of possible crc solutions, only one of which is correct.
        //
        // We should probably perform some sanity checks on corrected DF-11's before
        // using the results. Perhaps check the ICAO against known aircraft, and check
        // IID against known good IID's. That's a TODO.
        //
        mm->correctedbits = ten90_fix_bit_errors(msg, mm->msgbits, context->nfix_crc, mm->corrected);

        // If we correct, validate ICAO addr to help filter birthday paradox solutions.
        if (mm->correctedbits) {
            uint32_t addr = (msg[1] << 16) | (msg[2] << 8) | (msg[3]);
            if (!ten90_icao_address_was_recently_seen(addr, context))
                mm->correctedbits = 0;
        }
    }
    //
    // Note that most of the other computation happens *after* we fix the
    // single/two bit errors, otherwise we would need to recompute the fields again.
    //
    if (mm->msgtype == 11) { // DF 11
        mm->crcok = (mm->crc < 80);
        mm->iid   =  mm->crc;
        mm->addr  = (msg[1] << 16) | (msg[2] << 8) | (msg[3]);
        mm->ca    = (msg[0] & 0x07); // Responder capabilities

        if (0 == mm->crc) {
            // DF 11 : if crc == 0 try to populate our ICAO addresses whitelist.
          ten90_add_recently_seen_icao_addr(mm->addr, context);
        }

    } else if (mm->msgtype == 17) { // DF 17
        mm->crcok = (mm->crc == 0);
        mm->addr  = (msg[1] << 16) | (msg[2] << 8) | (msg[3]);
        mm->ca    = (msg[0] & 0x07); // Responder capabilities

        if (0 == mm->crc) {
            // DF 17 : if crc == 0 try to populate our ICAO addresses whitelist.
          ten90_add_recently_seen_icao_addr(mm->addr, context);
        }

    } else if (mm->msgtype == 18) { // DF 18
        mm->crcok = (mm->crc == 0);
        mm->addr  = (msg[1] << 16) | (msg[2] << 8) | (msg[3]);
        mm->ca    = (msg[0] & 0x07); // Control Field

        if (0 == mm->crc) {
            // DF 18 : if crc == 0 try to populate our ICAO addresses whitelist.
          ten90_add_recently_seen_icao_addr(mm->addr, context);
        }

    } else { // All other DF's
        // Compare the checksum with the whitelist of recently seen ICAO
        // addresses. If it matches one, then declare the message as valid
        mm->addr  = mm->crc;
        mm->crcok = ten90_icao_address_was_recently_seen(mm->crc, context);
    }

    // Fields for DF0, DF16
    if (mm->msgtype == 0  || mm->msgtype == 16) {
        if (msg[0] & 0x04) {                       // VS Bit
            mm->bFlags |= MODES_ACFLAGS_AOG_VALID | MODES_ACFLAGS_AOG;
        } else {
            mm->bFlags |= MODES_ACFLAGS_AOG_VALID;
        }
    }

    // Fields for DF11, DF17
    if (mm->msgtype == 11 || mm->msgtype == 17) {
        if (mm->ca == 4) {
            mm->bFlags |= MODES_ACFLAGS_AOG_VALID | MODES_ACFLAGS_AOG;
        } else if (mm->ca == 5) {
            mm->bFlags |= MODES_ACFLAGS_AOG_VALID;
        }
    }

    // Fields for DF5, DF21 = Gillham encoded Squawk
    if (mm->msgtype == 5  || mm->msgtype == 21) {
        int ID13Field = ((msg[2] << 8) | msg[3]) & 0x1FFF;
        if (ID13Field) {
            mm->bFlags |= MODES_ACFLAGS_SQUAWK_VALID;
            mm->modeA   = ten90_decode_id13_field(ID13Field);
        }
    }

    // Fields for DF0, DF4, DF16, DF20 13 bit altitude
    if (mm->msgtype == 0  || mm->msgtype == 4 ||
        mm->msgtype == 16 || mm->msgtype == 20) {
        int AC13Field = ((msg[2] << 8) | msg[3]) & 0x1FFF;
        if (AC13Field) { // Only attempt to decode if a valid (non zero) altitude is present
            mm->bFlags  |= MODES_ACFLAGS_ALTITUDE_VALID;
            mm->altitude = ten90_decode_ac13_field(AC13Field, &mm->unit);
        }
    }

    // Fields for DF4, DF5, DF20, DF21
    if ((mm->msgtype == 4) || (mm->msgtype == 20) ||
        (mm->msgtype == 5) || (mm->msgtype == 21)) {
        mm->bFlags  |= MODES_ACFLAGS_FS_VALID;
        mm->fs       = msg[0]  & 7;               // Flight status for DF4,5,20,21
        if (mm->fs <= 3) {
            mm->bFlags |= MODES_ACFLAGS_AOG_VALID;
            if (mm->fs & 1)
                {mm->bFlags |= MODES_ACFLAGS_AOG;}
        }
    }

    // Fields for DF17, DF18_CF0, DF18_CF1, DF18_CF6 squitters
    if (  (mm->msgtype == 17)
      || ((mm->msgtype == 18) && ((mm->ca == 0) || (mm->ca == 1) || (mm->ca == 6)) )) {
         int metype = mm->metype = msg[4] >> 3;   // Extended squitter message type
         int mesub  = mm->mesub  = msg[4]  & 7;   // Extended squitter message subtype

        // Decode the extended squitter message

        if (metype >= 1 && metype <= 4) { // Aircraft Identification and Category
            uint32_t chars;
            mm->bFlags |= MODES_ACFLAGS_CALLSIGN_VALID;

            chars = (msg[5] << 16) | (msg[6] << 8) | (msg[7]);
            mm->flight[3] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[2] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[1] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[0] = ais_charset[chars & 0x3F];

            chars = (msg[8] << 16) | (msg[9] << 8) | (msg[10]);
            mm->flight[7] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[6] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[5] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[4] = ais_charset[chars & 0x3F];

            mm->flight[8] = '\0';

        } else if (metype >= 5 && metype <= 18) { // Position Message
            mm->raw_latitude  = ((msg[6] & 3) << 15) | (msg[7] << 7) | (msg[8] >> 1);
            mm->raw_longitude = ((msg[8] & 1) << 16) | (msg[9] << 8) | (msg[10]);
            mm->bFlags       |= (mm->msg[6] & 0x04) ? MODES_ACFLAGS_LLODD_VALID
                                                    : MODES_ACFLAGS_LLEVEN_VALID;
            if (metype >= 9) {        // Airborne
                int AC12Field = ((msg[5] << 4) | (msg[6] >> 4)) & 0x0FFF;
                mm->bFlags |= MODES_ACFLAGS_AOG_VALID;
                if (AC12Field) {// Only attempt to decode if a valid (non zero) altitude is present
                    mm->bFlags |= MODES_ACFLAGS_ALTITUDE_VALID;
                    mm->altitude = ten90_decode_ac12_field(AC12Field, &mm->unit);
                }
            } else {                      // Ground
                int movement = ((msg[4] << 4) | (msg[5] >> 4)) & 0x007F;
                mm->bFlags |= MODES_ACFLAGS_AOG_VALID | MODES_ACFLAGS_AOG;
                if ((movement) && (movement < 125)) {
                    mm->bFlags |= MODES_ACFLAGS_SPEED_VALID;
                    mm->velocity = ten90_decode_movement_field(movement);
                }

                if (msg[5] & 0x08) {
                    mm->bFlags |= MODES_ACFLAGS_HEADING_VALID;
                    mm->heading = ((((msg[5] << 4) | (msg[6] >> 4)) & 0x007F) * 45) >> 4;
                }
            }

        } else if (metype == 19) { // Airborne Velocity Message

           // Presumably airborne if we get an Airborne Velocity Message
            mm->bFlags |= MODES_ACFLAGS_AOG_VALID;

            if ( (mesub >= 1) && (mesub <= 4) ) {
                int vert_rate = ((msg[8] & 0x07) << 6) | (msg[9] >> 2);
                if (vert_rate) {
                    --vert_rate;
                    if (msg[8] & 0x08)
                      {vert_rate = 0 - vert_rate;}
                    mm->vert_rate =  vert_rate * 64;
                    mm->bFlags   |= MODES_ACFLAGS_VERTRATE_VALID;
                }
            }

            if ((mesub == 1) || (mesub == 2)) {
                int ew_raw = ((msg[5] & 0x03) << 8) |  msg[6];
                int ew_vel = ew_raw - 1;
                int ns_raw = ((msg[7] & 0x7F) << 3) | (msg[8] >> 5);
                int ns_vel = ns_raw - 1;

                if (mesub == 2) { // If (supersonic) unit is 4 kts
                   ns_vel = ns_vel << 2;
                   ew_vel = ew_vel << 2;
                }

                if (ew_raw) { // Do East/West
                    mm->bFlags |= MODES_ACFLAGS_EWSPEED_VALID;
                    if (msg[5] & 0x04)
                        {ew_vel = 0 - ew_vel;}
                    mm->ew_velocity = ew_vel;
                }

                if (ns_raw) { // Do North/South
                    mm->bFlags |= MODES_ACFLAGS_NSSPEED_VALID;
                    if (msg[7] & 0x80)
                        {ns_vel = 0 - ns_vel;}
                    mm->ns_velocity = ns_vel;
                }

                if (ew_raw && ns_raw) {
                    // Compute velocity and angle from the two speed components
                    mm->bFlags |= (MODES_ACFLAGS_SPEED_VALID | MODES_ACFLAGS_HEADING_VALID | MODES_ACFLAGS_NSEWSPD_VALID);
                    mm->velocity = (int) sqrt((ns_vel * ns_vel) + (ew_vel * ew_vel));

                    if (mm->velocity) {
                        mm->heading = (int) (atan2(ew_vel, ns_vel) * 180.0 / M_PI);
                        // We don't want negative values but a 0-360 scale
                        if (mm->heading < 0) mm->heading += 360;
                    }
                }

            } else if (mesub == 3 || mesub == 4) {
                int airspeed = ((msg[7] & 0x7f) << 3) | (msg[8] >> 5);
                if (airspeed) {
                    mm->bFlags |= MODES_ACFLAGS_SPEED_VALID;
                    --airspeed;
                    if (mesub == 4)  // If (supersonic) unit is 4 kts
                        {airspeed = airspeed << 2;}
                    mm->velocity =  airspeed;
                }

                if (msg[5] & 0x04) {
                    mm->bFlags |= MODES_ACFLAGS_HEADING_VALID;
                    mm->heading = ((((msg[5] & 0x03) << 8) | msg[6]) * 45) >> 7;
                }
            }
        }
    }

    // Fields for DF20, DF21 Comm-B
    if ((mm->msgtype == 20) || (mm->msgtype == 21)){

        if (msg[4] == 0x20) { // Aircraft Identification
            uint32_t chars;
            mm->bFlags |= MODES_ACFLAGS_CALLSIGN_VALID;

            chars = (msg[5] << 16) | (msg[6] << 8) | (msg[7]);
            mm->flight[3] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[2] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[1] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[0] = ais_charset[chars & 0x3F];

            chars = (msg[8] << 16) | (msg[9] << 8) | (msg[10]);
            mm->flight[7] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[6] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[5] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[4] = ais_charset[chars & 0x3F];

            mm->flight[8] = '\0';
        } else {
        }
    }
}


void ten90_decode_mode_a_message(struct modesMessage *mm, int ModeA)
{
  mm->msgtype = 32; // Valid Mode S DF's are DF-00 to DF-31.
                    // so use 32 to indicate Mode A/C

  mm->msgbits = 16; // Fudge up a Mode S style data stream
  mm->msg[0] = (ModeA >> 8);
  mm->msg[1] = (ModeA);

  // Fudge an ICAO address based on Mode A (remove the Ident bit)
  // Use an upper address byte of FF, since this is ICAO unallocated
  mm->addr = 0x00FF0000 | (ModeA & 0x0000FF7F);

  // Set the Identity field to ModeA
  mm->modeA   = ModeA & 0x7777;
  mm->bFlags |= MODES_ACFLAGS_SQUAWK_VALID;

  // Flag ident in flight status
  mm->fs = ModeA & 0x0080;

  // Not much else we can tell from a Mode A/C reply.
  // Just fudge up a few bits to keep other code happy
  mm->crcok = 1;
  mm->correctedbits = 0;
}

/* ===================== Mode S detection and decoding  ===================== */

/* Parity table for MODE S Messages.
 * The table contains 112 elements, every element corresponds to a bit set
 * in the message, starting from the first bit of actual data after the
 * preamble.
 *
 * For messages of 112 bit, the whole table is used.
 * For messages of 56 bits only the last 56 elements are used.
 *
 * The algorithm is as simple as xoring all the elements in this table
 * for which the corresponding bit on the message is set to 1.
 *
 * The latest 24 elements in this table are set to 0 as the checksum at the
 * end of the message should not affect the computation.
 *
 * Note: this function can be used with DF11 and DF17, other modes have
 * the CRC xored with the sender address as they are reply to interrogations,
 * but a casual listener can't split the address from the checksum.
 */
uint32_t modes_checksum_table[112] = {
0x3935ea, 0x1c9af5, 0xf1b77e, 0x78dbbf, 0xc397db, 0x9e31e9, 0xb0e2f0, 0x587178,
0x2c38bc, 0x161c5e, 0x0b0e2f, 0xfa7d13, 0x82c48d, 0xbe9842, 0x5f4c21, 0xd05c14,
0x682e0a, 0x341705, 0xe5f186, 0x72f8c3, 0xc68665, 0x9cb936, 0x4e5c9b, 0xd8d449,
0x939020, 0x49c810, 0x24e408, 0x127204, 0x093902, 0x049c81, 0xfdb444, 0x7eda22,
0x3f6d11, 0xe04c8c, 0x702646, 0x381323, 0xe3f395, 0x8e03ce, 0x4701e7, 0xdc7af7,
0x91c77f, 0xb719bb, 0xa476d9, 0xadc168, 0x56e0b4, 0x2b705a, 0x15b82d, 0xf52612,
0x7a9309, 0xc2b380, 0x6159c0, 0x30ace0, 0x185670, 0x0c2b38, 0x06159c, 0x030ace,
0x018567, 0xff38b7, 0x80665f, 0xbfc92b, 0xa01e91, 0xaff54c, 0x57faa6, 0x2bfd53,
0xea04ad, 0x8af852, 0x457c29, 0xdd4410, 0x6ea208, 0x375104, 0x1ba882, 0x0dd441,
0xf91024, 0x7c8812, 0x3e4409, 0xe0d800, 0x706c00, 0x383600, 0x1c1b00, 0x0e0d80,
0x0706c0, 0x038360, 0x01c1b0, 0x00e0d8, 0x00706c, 0x003836, 0x001c1b, 0xfff409,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000
};

uint32_t ten90_mode_s_checksum(unsigned char *msg, int bits) {
    uint32_t   crc = 0;
    uint32_t   rem = 0;
    int        offset = (bits == 112) ? 0 : (112-56);
    uint8_t    theByte = *msg;
    uint32_t * pCRCTable = &modes_checksum_table[offset];
    int j;

    // We don't really need to include the checksum itself
    bits -= 24;
    for(j = 0; j < bits; j++) {
        if ((j & 7) == 0)
            theByte = *msg++;

        // If bit is set, xor with corresponding table entry.
        if (theByte & 0x80) {crc ^= *pCRCTable;}
        pCRCTable++;
        theByte = theByte << 1;
    }

    rem = (msg[0] << 16) | (msg[1] << 8) | msg[2]; // message checksum
    return ((crc ^ rem) & 0x00FFFFFF); // 24 bit checksum syndrome.
}


/* Hash the ICAO address to index our cache of MODES_ICAO_CACHE_LEN
 * elements, that is assumed to be a power of two. */
uint32_t ten90_icao_cache_hash_address(uint32_t a) {
    /* The following three rounds wil make sure that every bit affects
     * every output bit with ~ 50% of probability. */
    a = ((a >> 16) ^ a) * 0x45d9f3b;
    a = ((a >> 16) ^ a) * 0x45d9f3b;
    a = ((a >> 16) ^ a);
    return a & (MODES_ICAO_CACHE_LEN-1);
}

/* Add the specified entry to the cache of recently seen ICAO addresses.
 * Note that we also add a timestamp so that we can make sure that the
 * entry is only valid for MODES_ICAO_CACHE_TTL seconds. */
void ten90_add_recently_seen_icao_addr(uint32_t addr, ten90_context *context) {
    uint32_t h = ten90_icao_cache_hash_address(addr);
    context->icao_cache[h*2] = addr;
    context->icao_cache[h*2+1] = (uint32_t) time(NULL);
}

/* Returns 1 if the specified ICAO address was seen in a DF format with
 * proper checksum (not xored with address) no more than * MODES_ICAO_CACHE_TTL
 * seconds ago. Otherwise returns 0. */
int ten90_icao_address_was_recently_seen(uint32_t addr, ten90_context *context) {
    uint32_t h = ten90_icao_cache_hash_address(addr);
    uint32_t a = context->icao_cache[h*2];
    uint32_t t = context->icao_cache[h*2+1];

    return a && a == addr && time(NULL)-t <= MODES_ICAO_CACHE_TTL;
}

//
// In the squawk (identity) field bits are interleaved as follows in
// (message bit 20 to bit 32):
//
// C1-A1-C2-A2-C4-A4-ZERO-B1-D1-B2-D2-B4-D4
//
// So every group of three bits A, B, C, D represent an integer from 0 to 7.
//
// The actual meaning is just 4 octal numbers, but we convert it into a hex
// number tha happens to represent the four octal numbers.
//
// For more info: http://en.wikipedia.org/wiki/Gillham_code
//
int ten90_decode_id13_field(int ID13Field) {
    int hexGillham = 0;

    if (ID13Field & 0x1000) {hexGillham |= 0x0010;} // Bit 12 = C1
    if (ID13Field & 0x0800) {hexGillham |= 0x1000;} // Bit 11 = A1
    if (ID13Field & 0x0400) {hexGillham |= 0x0020;} // Bit 10 = C2
    if (ID13Field & 0x0200) {hexGillham |= 0x2000;} // Bit  9 = A2
    if (ID13Field & 0x0100) {hexGillham |= 0x0040;} // Bit  8 = C4
    if (ID13Field & 0x0080) {hexGillham |= 0x4000;} // Bit  7 = A4
  //if (ID13Field & 0x0040) {hexGillham |= 0x0800;} // Bit  6 = X  or M
    if (ID13Field & 0x0020) {hexGillham |= 0x0100;} // Bit  5 = B1
    if (ID13Field & 0x0010) {hexGillham |= 0x0001;} // Bit  4 = D1 or Q
    if (ID13Field & 0x0008) {hexGillham |= 0x0200;} // Bit  3 = B2
    if (ID13Field & 0x0004) {hexGillham |= 0x0002;} // Bit  2 = D2
    if (ID13Field & 0x0002) {hexGillham |= 0x0400;} // Bit  1 = B4
    if (ID13Field & 0x0001) {hexGillham |= 0x0004;} // Bit  0 = D4

    return (hexGillham);
    }
//
// Decode the 13 bit AC altitude field (in DF 20 and others).
// Returns the altitude, and set 'unit' to either MODES_UNIT_METERS or MDOES_UNIT_FEETS.
//
int ten90_decode_ac13_field(int AC13Field, int *unit) {
    int m_bit  = AC13Field & 0x0040; // set = meters, clear = feet
    int q_bit  = AC13Field & 0x0010; // set = 25 ft encoding, clear = Gillham Mode C encoding

    if (!m_bit) {
        *unit = MODES_UNIT_FEET;
        if (q_bit) {
            // N is the 11 bit integer resulting from the removal of bit Q and M
            int n = ((AC13Field & 0x1F80) >> 2) |
                    ((AC13Field & 0x0020) >> 1) |
                     (AC13Field & 0x000F);
            // The final altitude is resulting number multiplied by 25, minus 1000.
            return ((n * 25) - 1000);
        } else {
            // N is an 11 bit Gillham coded altitude
            int n = ten90_mode_a_to_mode_c(ten90_decode_id13_field(AC13Field));
            if (n < -12) {n = 0;}

            return (100 * n);
        }
    } else {
        *unit = MODES_UNIT_METERS;
        // TODO: Implement altitude when meter unit is selected
    }
    return 0;
}
//
// Decode the 12 bit AC altitude field (in DF 17 and others).
//
int ten90_decode_ac12_field(int AC12Field, int *unit) {
    int q_bit  = AC12Field & 0x10; // Bit 48 = Q

    *unit = MODES_UNIT_FEET;
    if (q_bit) {
        /// N is the 11 bit integer resulting from the removal of bit Q at bit 4
        int n = ((AC12Field & 0x0FE0) >> 1) |
                 (AC12Field & 0x000F);
        // The final altitude is the resulting number multiplied by 25, minus 1000.
        return ((n * 25) - 1000);
    } else {
        // Make N a 13 bit Gillham coded altitude by inserting M=0 at bit 6
        int n = ((AC12Field & 0x0FC0) << 1) |
                 (AC12Field & 0x003F);
        n = ten90_mode_a_to_mode_c(ten90_decode_id13_field(n));
        if (n < -12) {n = 0;}

        return (100 * n);
    }
}
//
// Decode the 7 bit ground movement field PWL exponential style scale
//
int ten90_decode_movement_field(int movement) {
    int gspeed;

    // Note : movement codes 0,125,126,127 are all invalid, but they are
    //        trapped for before this function is called.

    if      (movement  > 123) gspeed = 199; // > 175kt
    else if (movement  > 108) gspeed = ((movement - 108)  * 5) + 100;
    else if (movement  >  93) gspeed = ((movement -  93)  * 2) +  70;
    else if (movement  >  38) gspeed = ((movement -  38)     ) +  15;
    else if (movement  >  12) gspeed = ((movement -  11) >> 1) +   2;
    else if (movement  >   8) gspeed = ((movement -   6) >> 2) +   1;
    else                      gspeed = 0;

    return (gspeed);
}

// Input format is : 00:A4:A2:A1:00:B4:B2:B1:00:C4:C2:C1:00:D4:D2:D1
int ten90_mode_a_to_mode_c(unsigned int ModeA)
{
  unsigned int FiveHundreds = 0;
  unsigned int OneHundreds  = 0;

  if (  (ModeA & 0xFFFF888B)         // D1 set is illegal. D2 set is > 62700ft which is unlikely
    || ((ModeA & 0x000000F0) == 0) ) // C1,,C4 cannot be Zero
    {return -9999;}

  if (ModeA & 0x0010) {OneHundreds ^= 0x007;} // C1
  if (ModeA & 0x0020) {OneHundreds ^= 0x003;} // C2
  if (ModeA & 0x0040) {OneHundreds ^= 0x001;} // C4

  // Remove 7s from OneHundreds (Make 7->5, snd 5->7).
  if ((OneHundreds & 5) == 5) {OneHundreds ^= 2;}

  // Check for invalid codes, only 1 to 5 are valid
  if (OneHundreds > 5)
    {return -9999;}

//if (ModeA & 0x0001) {FiveHundreds ^= 0x1FF;} // D1 never used for altitude
  if (ModeA & 0x0002) {FiveHundreds ^= 0x0FF;} // D2
  if (ModeA & 0x0004) {FiveHundreds ^= 0x07F;} // D4

  if (ModeA & 0x1000) {FiveHundreds ^= 0x03F;} // A1
  if (ModeA & 0x2000) {FiveHundreds ^= 0x01F;} // A2
  if (ModeA & 0x4000) {FiveHundreds ^= 0x00F;} // A4

  if (ModeA & 0x0100) {FiveHundreds ^= 0x007;} // B1
  if (ModeA & 0x0200) {FiveHundreds ^= 0x003;} // B2
  if (ModeA & 0x0400) {FiveHundreds ^= 0x001;} // B4

  // Correct order of OneHundreds.
  if (FiveHundreds & 1) {OneHundreds = 6 - OneHundreds;}

  return ((FiveHundreds * 5) + OneHundreds - 13);
}

//
// Given the Downlink Format (DF) of the message, return the message length in bits.
//
// All known DF's 16 or greater are long. All known DF's 15 or less are short.
// There are lots of unused codes in both category, so we can assume ICAO will stick to
// these rules, meaning that the most significant bit of the DF indicates the length.
//
int ten90_mode_s_message_len_by_type(int type) {
    return (type & 0x10) ? MODES_LONG_MSG_BITS : MODES_SHORT_MSG_BITS ;
}


#define NERRORINFO \
        (MODES_LONG_MSG_BITS+MODES_LONG_MSG_BITS*(MODES_LONG_MSG_BITS-1)/2)
struct errorinfo bitErrorTable[NERRORINFO];

/* Compare function as needed for stdlib's qsort and bsearch functions */
int cmpErrorInfo(const void *p0, const void *p1) {
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

//
// Search for syndrome in table and if an entry is found, flip the necessary
// bits. Make sure the indices fit into the array
// Additional parameter: fix only less than maxcorrected bits, and record
// fixed bit positions in corrected[]. This array can be NULL, otherwise
// must be of length at least maxcorrected.
// Return number of fixed bits.
//
int ten90_fix_bit_errors(unsigned char *msg, int bits, int maxfix, char *fixedbits) {
    struct errorinfo *pei;
    struct errorinfo ei;
    int bitpos, offset, res, i;
    memset(&ei, 0, sizeof(struct errorinfo));
    ei.syndrome = ten90_mode_s_checksum(msg, bits);
    pei = bsearch(&ei, bitErrorTable, NERRORINFO,
                  sizeof(struct errorinfo), cmpErrorInfo);
    if (pei == NULL) {
        return 0; // No syndrome found
    }

    // Check if the syndrome fixes more bits than we allow
    if (maxfix < pei->bits) {
        return 0;
    }

    // Check that all bit positions lie inside the message length
    offset = MODES_LONG_MSG_BITS-bits;
    for (i = 0;  i < pei->bits;  i++) {
            bitpos = pei->pos[i] - offset;
            if ((bitpos < 0) || (bitpos >= bits)) {
                    return 0;
            }
    }

    // Fix the bits
    for (i = res = 0;  i < pei->bits;  i++) {
            bitpos = pei->pos[i] - offset;
            msg[bitpos >> 3] ^= (1 << (7 - (bitpos & 7)));
            if (fixedbits) {
                    fixedbits[res++] = bitpos;
            }
    }
    return res;
}


/* ===================== Mode A/C detection and decoding  =================== */

//
// This table is used to build the Mode A/C variable called ModeABits.Each
// bit period is inspected, and if it's value exceeds the threshold limit,
// then the value in this table is or-ed into ModeABits.
//
// At the end of message processing, ModeABits will be the decoded ModeA value.
//
// We can also flag noise in bits that should be zeros - the xx bits. Noise in
// these bits cause bits (31-16) in ModeABits to be set. Then at the end of message
// processing we can test for errors by looking at these bits.
//
static uint32_t ModeABitTable[24] = {
0x00000000, // F1 = 1
0x00000010, // C1
0x00001000, // A1
0x00000020, // C2
0x00002000, // A2
0x00000040, // C4
0x00004000, // A4
0x40000000, // xx = 0  Set bit 30 if we see this high
0x00000100, // B1
0x00000001, // D1
0x00000200, // B2
0x00000002, // D2
0x00000400, // B4
0x00000004, // D4
0x00000000, // F2 = 1
0x08000000, // xx = 0  Set bit 27 if we see this high
0x04000000, // xx = 0  Set bit 26 if we see this high
0x00000080, // SPI
0x02000000, // xx = 0  Set bit 25 if we see this high
0x01000000, // xx = 0  Set bit 24 if we see this high
0x00800000, // xx = 0  Set bit 23 if we see this high
0x00400000, // xx = 0  Set bit 22 if we see this high
0x00200000, // xx = 0  Set bit 21 if we see this high
0x00100000, // xx = 0  Set bit 20 if we see this high
};
//
// This table is used to produce an error variable called ModeAErrs.Each
// inter-bit period is inspected, and if it's value falls outside of the
// expected range, then the value in this table is or-ed into ModeAErrs.
//
// At the end of message processing, ModeAErrs will indicate if we saw
// any inter-bit anomolies, and the bits that are set will show which
// bits had them.
//
static uint32_t ModeAMidTable[24] = {
0x80000000, // F1 = 1  Set bit 31 if we see F1_C1  error
0x00000010, // C1      Set bit  4 if we see C1_A1  error
0x00001000, // A1      Set bit 12 if we see A1_C2  error
0x00000020, // C2      Set bit  5 if we see C2_A2  error
0x00002000, // A2      Set bit 13 if we see A2_C4  error
0x00000040, // C4      Set bit  6 if we see C3_A4  error
0x00004000, // A4      Set bit 14 if we see A4_xx  error
0x40000000, // xx = 0  Set bit 30 if we see xx_B1  error
0x00000100, // B1      Set bit  8 if we see B1_D1  error
0x00000001, // D1      Set bit  0 if we see D1_B2  error
0x00000200, // B2      Set bit  9 if we see B2_D2  error
0x00000002, // D2      Set bit  1 if we see D2_B4  error
0x00000400, // B4      Set bit 10 if we see B4_D4  error
0x00000004, // D4      Set bit  2 if we see D4_F2  error
0x20000000, // F2 = 1  Set bit 29 if we see F2_xx  error
0x08000000, // xx = 0  Set bit 27 if we see xx_xx  error
0x04000000, // xx = 0  Set bit 26 if we see xx_SPI error
0x00000080, // SPI     Set bit 15 if we see SPI_xx error
0x02000000, // xx = 0  Set bit 25 if we see xx_xx  error
0x01000000, // xx = 0  Set bit 24 if we see xx_xx  error
0x00800000, // xx = 0  Set bit 23 if we see xx_xx  error
0x00400000, // xx = 0  Set bit 22 if we see xx_xx  error
0x00200000, // xx = 0  Set bit 21 if we see xx_xx  error
0x00100000, // xx = 0  Set bit 20 if we see xx_xx  error
};
//
// The "off air" format is,,
// _F1_C1_A1_C2_A2_C4_A4_xx_B1_D1_B2_D2_B4_D4_F2_xx_xx_SPI_
//
// Bit spacing is 1.45uS, with 0.45uS high, and 1.00us low. This is a problem
// because we ase sampling at 2Mhz (500nS) so we are below Nyquist.
//
// The bit spacings are..
// F1 :  0.00,
//       1.45,  2.90,  4.35,  5.80,  7.25,  8.70,
// X  : 10.15,
//    : 11.60, 13.05, 14.50, 15.95, 17.40, 18.85,
// F2 : 20.30,
// X  : 21.75, 23.20, 24.65
//
// This equates to the following sample point centers at 2Mhz.
// [ 0.0],
// [ 2.9], [ 5.8], [ 8.7], [11.6], [14.5], [17.4],
// [20.3],
// [23.2], [26.1], [29.0], [31.9], [34.8], [37.7]
// [40.6]
// [43.5], [46.4], [49.3]
//
// We know that this is a supposed to be a binary stream, so the signal
// should either be a 1 or a 0. Therefore, any energy above the noise level
// in two adjacent samples must be from the same pulse, so we can simply
// add the values together..
//
int ten90_detect_mode_a(uint16_t *m, struct modesMessage *mm)
{
  int j, lastBitWasOne;
  int ModeABits = 0;
  int ModeAErrs = 0;
  int byte, bit;
  int thisSample, lastBit, lastSpace = 0;
  int m0, m1, m2, m3, mPhase;
  int n0, n1, n2 ,n3;
  int F1_sig, F1_noise;
  int F2_sig, F2_noise;
  int fSig, fNoise, fLevel, fLoLo;

  // m[0] contains the energy from    0 ->  499 nS
  // m[1] contains the energy from  500 ->  999 nS
  // m[2] contains the energy from 1000 -> 1499 nS
  // m[3] contains the energy from 1500 -> 1999 nS
  //
  // We are looking for a Frame bit (F1) whose width is 450nS, followed by
  // 1000nS of quiet.
  //
  // The width of the frame bit is 450nS, which is 90% of our sample rate.
  // Therefore, in an ideal world, all the energy for the frame bit will be
  // in a single sample, preceeded by (at least) one zero, and followed by
  // two zeros, Best case we can look for ...
  //
  // 0 - 1 - 0 - 0
  //
  // However, our samples are not phase aligned, so some of the energy from
  // each bit could be spread over two consecutive samples. Worst case is
  // that we sample half in one bit, and half in the next. In that case,
  // we're looking for
  //
  // 0 - 0.5 - 0.5 - 0.

  m0 = m[0]; m1 = m[1];

  if (m0 >= m1)   // m1 *must* be bigger than m0 for this to be F1
    {return (0);}

  m2 = m[2]; m3 = m[3];

  //
  // if (m2 <= m0), then assume the sample bob on (Phase == 0), so don't look at m3
  if ((m2 <= m0) || (m2 < m3))
    {m3 = m2; m2 = m0;}

  if (  (m3 >= m1)   // m1 must be bigger than m3
     || (m0 >  m2)   // m2 can be equal to m0 if ( 0,1,0,0 )
     || (m3 >  m2) ) // m2 can be equal to m3 if ( 0,1,0,0 )
    {return (0);}

  // m0 = noise
  // m1 = noise + (signal *    X))
  // m2 = noise + (signal * (1-X))
  // m3 = noise
  //
  // Hence, assuming all 4 samples have similar amounts of noise in them
  //      signal = (m1 + m2) - ((m0 + m3) * 2)
  //      noise  = (m0 + m3) / 2
  //
  F1_sig   = (m1 + m2) - ((m0 + m3) << 1);
  F1_noise = (m0 + m3) >> 1;

  if ( (F1_sig < MODEAC_MSG_SQUELCH_LEVEL) // minimum required  F1 signal amplitude
    || (F1_sig < (F1_noise << 2)) )        // minimum allowable Sig/Noise ratio 4:1
    {return (0);}

  // If we get here then we have a potential F1, so look for an equally valid F2 20.3uS later
  //
  // Our F1 is centered somewhere between samples m[1] and m[2]. We can guestimate where F2 is
  // by comparing the ratio of m1 and m2, and adding on 20.3 uS (40.6 samples)
  //
  mPhase = ((m2 * 20) / (m1 + m2));
  byte   = (mPhase + 812) / 20;
  n0     = m[byte++]; n1 = m[byte++];

  if (n0 >= n1)   // n1 *must* be bigger than n0 for this to be F2
    {return (0);}

  n2 = m[byte++];
  //
  // if the sample bob on (Phase == 0), don't look at n3
  //
  if ((mPhase + 812) % 20)
    {n3 = m[byte++];}
  else
    {n3 = n2; n2 = n0;}

  if (  (n3 >= n1)   // n1 must be bigger than n3
     || (n0 >  n2)   // n2 can be equal to n0 ( 0,1,0,0 )
     || (n3 >  n2) ) // n2 can be equal to n3 ( 0,1,0,0 )
    {return (0);}

  F2_sig   = (n1 + n2) - ((n0 + n3) << 1);
  F2_noise = (n0 + n3) >> 1;

  if ( (F2_sig < MODEAC_MSG_SQUELCH_LEVEL) // minimum required  F2 signal amplitude
    || (F2_sig < (F2_noise << 2)) )       // maximum allowable Sig/Noise ratio 4:1
    {return (0);}

  fSig          = (F1_sig   + F2_sig)   >> 1;
  fNoise        = (F1_noise + F2_noise) >> 1;
  fLoLo         = fNoise    + (fSig >> 2);       // 1/2
  fLevel        = fNoise    + (fSig >> 1);
  lastBitWasOne = 1;
  lastBit       = F1_sig;
  //
  // Now step by a half ModeA bit, 0.725nS, which is 1.45 samples, which is 29/20
  // No need to do bit 0 because we've already selected it as a valid F1
  // Do several bits past the SPI to increase error rejection
  //
  for (j = 1, mPhase += 29; j < 48; mPhase += 29, j ++)
    {
    byte  = 1 + (mPhase / 20);

    thisSample = m[byte] - fNoise;
    if (mPhase % 20)                     // If the bit is split over two samples...
      {thisSample += (m[byte+1] - fNoise);}  //    add in the second sample's energy

     // If we're calculating a space value
    if (j & 1)
      {lastSpace = thisSample;}

    else
      {// We're calculating a new bit value
      bit = j >> 1;
      if (thisSample >= fLevel)
        {// We're calculating a new bit value, and its a one
        ModeABits |= ModeABitTable[bit--];  // or in the correct bit

        if (lastBitWasOne)
          { // This bit is one, last bit was one, so check the last space is somewhere less than one
          if ( (lastSpace >= (thisSample>>1)) || (lastSpace >= lastBit) )
            {ModeAErrs |= ModeAMidTable[bit];}
          }

        else
          {// This bit,is one, last bit was zero, so check the last space is somewhere less than one
          if (lastSpace >= (thisSample >> 1))
            {ModeAErrs |= ModeAMidTable[bit];}
          }

        lastBitWasOne = 1;
        }


      else
        {// We're calculating a new bit value, and its a zero
        if (lastBitWasOne)
          { // This bit is zero, last bit was one, so check the last space is somewhere in between
          if (lastSpace >= lastBit)
            {ModeAErrs |= ModeAMidTable[bit];}
          }

        else
          {// This bit,is zero, last bit was zero, so check the last space is zero too
          if (lastSpace >= fLoLo)
            {ModeAErrs |= ModeAMidTable[bit];}
          }

        lastBitWasOne = 0;
        }

      lastBit = (thisSample >> 1);
      }
    }

  //
  // Output format is : 00:A4:A2:A1:00:B4:B2:B1:00:C4:C2:C1:00:D4:D2:D1
  //
  if ((ModeABits < 3) || (ModeABits & 0xFFFF8808) || (ModeAErrs) )
    {return (ModeABits = 0);}

  fSig            = (fSig + 0x7F) >> 8;
  mm->signalLevel = ((fSig < 255) ? fSig : 255);

  return ModeABits;
  }

//
// Try to fix single bit errors using the checksum. On success modifies
// the original buffer with the fixed version, and returns the position
// of the error bit. Otherwise if fixing failed -1 is returned.
//
int ten90_fix_single_bit_errors(unsigned char *msg, int bits) {
    int j;
    unsigned char aux[MODES_LONG_MSG_BYTES];

    memcpy(aux, msg, bits/8);

    // Do not attempt to error correct Bits 0-4. These contain the DF, and must
    // be correct because we can only error correct DF17
    for (j = 5; j < bits; j++) {
        int byte    = j/8;
        int bitmask = 1 << (7 - (j & 7));

        aux[byte] ^= bitmask; // Flip j-th bit

        if (0 == ten90_mode_s_checksum(aux, bits)) {
            // The error is fixed. Overwrite the original buffer with the
            // corrected sequence, and returns the error bit position
            msg[byte] = aux[byte];
            return (j);
        }

        aux[byte] ^= bitmask; // Flip j-th bit back again
    }
    return (-1);
}


/* Compute the table of all syndromes for 1-bit and 2-bit error vectors */
static void init_error_info(ten90_context *context) {
    unsigned char msg[MODES_LONG_MSG_BYTES];
    int i, j, n;
    uint32_t crc;
    n = 0;
    memset(bitErrorTable, 0, sizeof(bitErrorTable));
    memset(msg, 0, MODES_LONG_MSG_BYTES);
    // Add all possible single and double bit errors
    // don't include errors in first 5 bits (DF type)
    for (i = 5;  i < MODES_LONG_MSG_BITS;  i++) {
        int bytepos0 = (i >> 3);
        int mask0 = 1 << (7 - (i & 7));
        msg[bytepos0] ^= mask0;          // create error0
        crc = ten90_mode_s_checksum(msg, MODES_LONG_MSG_BITS);
        bitErrorTable[n].syndrome = crc;      // single bit error case
        bitErrorTable[n].bits = 1;
        bitErrorTable[n].pos[0] = i;
        bitErrorTable[n].pos[1] = -1;
        n += 1;

        if (context->nfix_crc > 1) {
            for (j = i+1;  j < MODES_LONG_MSG_BITS;  j++) {
                int bytepos1 = (j >> 3);
                int mask1 = 1 << (7 - (j & 7));
                msg[bytepos1] ^= mask1;  // create error1
                crc = ten90_mode_s_checksum(msg, MODES_LONG_MSG_BITS);
                if (n >= NERRORINFO) {
                    //fprintf(stderr, "Internal error, too many entries, fix NERRORINFO\n");
                    break;
                }
                bitErrorTable[n].syndrome = crc; // two bit error case
                bitErrorTable[n].bits = 2;
                bitErrorTable[n].pos[0] = i;
                bitErrorTable[n].pos[1] = j;
                n += 1;
                msg[bytepos1] ^= mask1;  // revert error1
            }
        }
        msg[bytepos0] ^= mask0;          // revert error0
    }
    qsort(bitErrorTable, NERRORINFO, sizeof(struct errorinfo), cmpErrorInfo);

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


int ten90_context_init(ten90_context *context)
{
  memset(context, 0, sizeof(ten90_context));
  if ((context->icao_cache = (uint32_t *)calloc(MODES_ICAO_CACHE_LEN * 2, sizeof(uint32_t))) == NULL) {
    return -1;
  }
  init_error_info(context);
  return 0;
}

void ten90_context_destroy(ten90_context *context)
{
  free(context->icao_cache);
}


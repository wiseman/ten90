/* dump1090, a Mode S messages decoder for RTLSDR devices.
 *
 * Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *  *  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  *  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* ============================= Include files ========================== */

#ifndef _WIN32
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <math.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/timeb.h>
#include "rtl-sdr.h"
#include "ten90.h"
#include "anet.h"
#else
#include "winstubs.h" //Put everything Windows specific in here
#include "rtl-sdr.h"
#include "ten90.h"
#endif


#define DUMP1090_VERSION "2.0.0"

#ifdef USER_LATITUDE
#define MODES_USER_LATITUDE_DFLT   (USER_LATITUDE)
#define MODES_USER_LONGITUDE_DFLT  (USER_LONGITUDE)
#else
#define MODES_USER_LATITUDE_DFLT   (0.0)
#define MODES_USER_LONGITUDE_DFLT  (0.0)
#endif

#define MODES_DEFAULT_RATE         2000000
#define MODES_DEFAULT_FREQ         1090000000
#define MODES_DEFAULT_WIDTH        1000
#define MODES_DEFAULT_HEIGHT       700
#define MODES_ASYNC_BUF_NUMBER     12
#define MODES_ASYNC_BUF_SIZE       (16*16384)                 // 256k
#define MODES_ASYNC_BUF_SAMPLES    (MODES_ASYNC_BUF_SIZE / 2) // Each sample is 2 bytes
#define MODES_AUTO_GAIN            -100                       // Use automatic gain
#define MODES_MAX_GAIN             999999                     // Use max available gain
#define MODES_MSG_SQUELCH_LEVEL    0x02FF                     // Average signal strength limit
#define MODES_MSG_ENCODER_ERRS     3                          // Maximum number of encoding errors

#define MODES_RAWOUT_BUF_SIZE   (1500)
#define MODES_RAWOUT_BUF_FLUSH  (MODES_RAWOUT_BUF_SIZE - 200)
#define MODES_RAWOUT_BUF_RATE   (1000)            // 1000 * 64mS = 1 Min approx

#define MODES_USER_LATLON_VALID (1<<0)

// When debug is set to MODES_DEBUG_NOPREAMBLE, the first sample must be
// at least greater than a given level for us to dump the signal.
#define MODES_DEBUG_NOPREAMBLE_LEVEL 25

#define MODES_INTERACTIVE_REFRESH_TIME 250      // Milliseconds
#define MODES_INTERACTIVE_ROWS 15               // Rows on screen
#define MODES_INTERACTIVE_TTL 60                // TTL before being removed

#define MODES_NET_MAX_FD 1024
#define MODES_NET_INPUT_RAW_PORT    30001
#define MODES_NET_OUTPUT_RAW_PORT   30002
#define MODES_NET_OUTPUT_SBS_PORT   30003
#define MODES_NET_INPUT_BEAST_PORT  30004
#define MODES_NET_OUTPUT_BEAST_PORT 30005
#define MODES_NET_HTTP_PORT          8080
#define MODES_CLIENT_BUF_SIZE  1024
#define MODES_NET_SNDBUF_SIZE (1024*64)

#define MODES_DEBUG_DEMOD (1<<0)
#define MODES_DEBUG_DEMODERR (1<<1)
#define MODES_DEBUG_BADCRC (1<<2)
#define MODES_DEBUG_GOODCRC (1<<3)
#define MODES_DEBUG_NOPREAMBLE (1<<4)
#define MODES_DEBUG_NET (1<<5)
#define MODES_DEBUG_JS (1<<6)


#ifndef HTMLPATH
#define HTMLPATH   "./public_html"      // default path for gmap.html etc
#endif

#define MODES_NOTUSED(V) ((void) V)


// Structure used to describe a networking client
struct client {
  int  fd;                           // File descriptor
  int  service;                      // TCP port the client is connected to
  char buf[MODES_CLIENT_BUF_SIZE+1]; // Read buffer
  int  buflen;                       // Amount of data on buffer
};

// Structure used to describe an aircraft in iteractive mode
struct aircraft {
  uint32_t      addr;           // ICAO address
  char          flight[16];     // Flight number
  unsigned char signalLevel[8]; // Last 8 Signal Amplitudes
  int           altitude;       // Altitude
  int           speed;          // Velocity
  int           track;          // Angle of flight
  int           vert_rate;      // Vertical rate.
  time_t        seen;           // Time at which the last packet was received
  time_t        seenLatLon;     // Time at which the last lat long was calculated
  uint64_t      timestamp;      // Timestamp at which the last packet was received
  uint64_t      timestampLatLon;// Timestamp at which the last lat long was calculated
  long          messages;       // Number of Mode S messages received
  int           modeA;          // Squawk
  int           modeC;          // Altitude
  long          modeAcount;     // Mode A Squawk hit Count
  long          modeCcount;     // Mode C Altitude hit Count
  int           modeACflags;    // Flags for mode A/C recognition

  // Encoded latitude and longitude as extracted by odd and even CPR encoded messages
  int           odd_cprlat;
  int           odd_cprlon;
  int           even_cprlat;
  int           even_cprlon;
  uint64_t      odd_cprtime;
  uint64_t      even_cprtime;
  double        lat, lon;       // Coordinated obtained from CPR encoded data
  int           bFlags;         // Flags related to valid fields in this structure
  struct aircraft *next;        // Next aircraft in our linked list
};

// Program global state
struct {                             // Internal state
  Ten90Context    ctx;
  pthread_t       reader_thread;
  pthread_mutex_t data_mutex;      // Mutex to synchronize buffer access
  pthread_cond_t  data_cond;       // Conditional variable associated
  uint16_t       *data;            // Raw IQ samples buffer
  uint16_t       *magnitude;       // Magnitude vector
  struct timeb    stSystemTimeRTL; // System time when RTL passed us the Latest block
  uint64_t        timestampBlk;    // Timestamp of the start of the current block
  struct timeb    stSystemTimeBlk; // System time when RTL passed us currently processing this block
  int             fd;              // --ifile option file descriptor
  int             data_ready;      // Data ready to be processed
  uint32_t       *icao_cache;      // Recently seen ICAO addresses cache
  uint16_t       *maglut;          // I/Q -> Magnitude lookup table
  int             exit;            // Exit from the main loop when true

  // RTLSDR
  int           dev_index;
  int           gain;
  int           enable_agc;
  rtlsdr_dev_t *dev;
  int           freq;
  int           ppm_error;

  // Networking
  char           aneterr[ANET_ERR_LEN];
  struct client *clients[MODES_NET_MAX_FD]; // Our clients
  int            maxfd;                     // Greatest fd currently active
  int            sbsos;                     // SBS output listening socket
  int            ros;                       // Raw output listening socket
  int            ris;                       // Raw input listening socket
  int            bos;                       // Beast output listening socket
  int            bis;                       // Beast input listening socket
  int            https;                     // HTTP listening socket
  char          *rawOut;                    // Buffer for building raw output data
  int            rawOutUsed;                // How much of the buffer is currently used
  char          *beastOut;                  // Buffer for building beast output data
  int            beastOutUsed;              // How much if the buffer is currently used

  // Configuration
  char *filename;                  // Input form file, --ifile option
  int   phase_enhance;             // Enable phase enhancement if true
  int   nfix_crc;                  // Number of crc bit error(s) to correct
  int   check_crc;                 // Only display messages with good CRC
  int   raw;                       // Raw output format
  int   beast;                     // Beast binary format output
  int   mode_ac;                   // Enable decoding of SSR Modes A & C
  int   debug;                     // Debugging mode
  int   net;                       // Enable networking
  int   net_only;                  // Enable just networking
  int   net_output_sbs_port;       // SBS output TCP port
  int   net_output_raw_size;       // Minimum Size of the output raw data
  int   net_output_raw_rate;       // Rate (in 64mS increments) of output raw data
  int   net_output_raw_rate_count; // Rate (in 64mS increments) of output raw data
  int   net_output_raw_port;       // Raw output TCP port
  int   net_input_raw_port;        // Raw input TCP port
  int   net_output_beast_port;     // Beast output TCP port
  int   net_input_beast_port;      // Beast input TCP port
  int   net_http_port;             // HTTP port
  int   quiet;                     // Suppress stdout
  int   interactive;               // Interactive mode
  int   interactive_rows;          // Interactive mode: max number of rows
  int   interactive_ttl;           // Interactive mode: TTL before deletion
  int   stats;                     // Print stats at exit in --ifile mode
  int   onlyaddr;                  // Print only ICAO addresses
  int   metric;                    // Use metric units
  int   mlat;                      // Use Beast ascii format for raw data output, i.e. @...; iso *...;
  int   interactive_rtl1090;       // flight table in interactive mode is formatted like RTL1090

  // User details
  double fUserLat;                // Users receiver/antenna lat/lon needed for initial surface location
  double fUserLon;                // Users receiver/antenna lat/lon needed for initial surface location
  int    bUserFlags;              // Flags relating to the user details

  // Interactive mode
  struct aircraft *aircrafts;
  uint64_t         interactive_last_update; // Last screen update in milliseconds

  // Statistics
  unsigned int stat_valid_preamble;
  unsigned int stat_demodulated0;
  unsigned int stat_demodulated1;
  unsigned int stat_demodulated2;
  unsigned int stat_demodulated3;
  unsigned int stat_goodcrc;
  unsigned int stat_badcrc;
  unsigned int stat_fixed;

  // Histogram of fixed bit errors: index 0 for single bit erros,
  // index 1 for double bit errors etc.
  unsigned int stat_bit_fix[MODES_MAX_BITERRORS];

  unsigned int stat_http_requests;
  unsigned int stat_sbs_connections;
  unsigned int stat_raw_connections;
  unsigned int stat_beast_connections;
  unsigned int stat_out_of_phase;
  unsigned int stat_ph_demodulated0;
  unsigned int stat_ph_demodulated1;
  unsigned int stat_ph_demodulated2;
  unsigned int stat_ph_demodulated3;
  unsigned int stat_ph_goodcrc;
  unsigned int stat_ph_badcrc;
  unsigned int stat_ph_fixed;
  // Histogram of fixed bit errors: index 0 for single bit erros,
  // index 1 for double bit errors etc.
  unsigned int stat_ph_bit_fix[MODES_MAX_BITERRORS];

  unsigned int stat_DF_Len_Corrected;
  unsigned int stat_DF_Type_Corrected;
  unsigned int stat_ModeAC;
} Modes;


void interactiveShowData(void);
struct aircraft* interactiveReceiveData(Ten90Message *mm);
void modesSendAllClients  (int service, void *msg, int len);
void modesSendRawOutput   (Ten90Message *mm);
void modesSendBeastOutput (Ten90Message *mm);
void modesSendSBSOutput   (Ten90Message *mm);
void useModesMessage      (Ten90Message *mm);

/* ============================= Utility functions ========================== */

static uint64_t mstime(void) {
  struct timeval tv;
  uint64_t mst;

  gettimeofday(&tv, NULL);
  mst = ((uint64_t)tv.tv_sec)*1000;
  mst += tv.tv_usec/1000;
  return mst;
}

void sigintHandler(int dummy) {
  MODES_NOTUSED(dummy);
  signal(SIGINT, SIG_DFL);  // reset signal handler - bit extra safety
  Modes.exit = 1;           // Signal to threads that we are done
}

/* =============================== Initialization =========================== */

void modesInitConfig(void) {
  // Default everything to zero/NULL
  memset(&Modes, 0, sizeof(Modes));

  // Now initialise things that should not be 0/NULL to their defaults
  Modes.gain                  = MODES_MAX_GAIN;
  Modes.freq                  = MODES_DEFAULT_FREQ;
  Modes.check_crc             = 1;
  Modes.net_output_sbs_port   = MODES_NET_OUTPUT_SBS_PORT;
  Modes.net_output_raw_port   = MODES_NET_OUTPUT_RAW_PORT;
  Modes.net_input_raw_port    = MODES_NET_INPUT_RAW_PORT;
  Modes.net_output_beast_port = MODES_NET_OUTPUT_BEAST_PORT;
  Modes.net_input_beast_port  = MODES_NET_INPUT_BEAST_PORT;
  Modes.net_http_port         = MODES_NET_HTTP_PORT;
  Modes.interactive_rows      = MODES_INTERACTIVE_ROWS;
  Modes.interactive_ttl       = MODES_INTERACTIVE_TTL;
  Modes.fUserLat              = MODES_USER_LATITUDE_DFLT;
  Modes.fUserLon              = MODES_USER_LONGITUDE_DFLT;
}

void modesInit(void) {
  int i, q;

  if (Ten90ContextInit(&Modes.ctx)) {
    fprintf(stderr, "Unable to initialize ten90.\n");
    exit(1);
  }

  pthread_mutex_init(&Modes.data_mutex,NULL);
  pthread_cond_init(&Modes.data_cond,NULL);

  // Allocate the various buffers used by Modes
  if (((Modes.data       = (uint16_t *) malloc(MODES_ASYNC_BUF_SIZE)                                         ) == NULL) ||
      ((Modes.magnitude  = (uint16_t *) malloc(MODES_ASYNC_BUF_SIZE+MODES_PREAMBLE_SIZE+MODES_LONG_MSG_SIZE) ) == NULL) ||
      ((Modes.maglut     = (uint16_t *) malloc(sizeof(uint16_t) * 256 * 256)                                 ) == NULL) ||
      ((Modes.beastOut   = (char     *) malloc(MODES_RAWOUT_BUF_SIZE)                                        ) == NULL) ||
      ((Modes.rawOut     = (char     *) malloc(MODES_RAWOUT_BUF_SIZE)                                        ) == NULL) )
    {
      fprintf(stderr, "Out of memory allocating data buffer.\n");
      exit(1);
    }

  // Clear the buffers that have just been allocated, just in-case
  memset(Modes.data,       127, MODES_ASYNC_BUF_SIZE);
  memset(Modes.magnitude,  0,   MODES_ASYNC_BUF_SIZE+MODES_PREAMBLE_SIZE+MODES_LONG_MSG_SIZE);

  // Validate the users Lat/Lon home location inputs
  if ( (Modes.fUserLat >   90.0)  // Latitude must be -90 to +90
       || (Modes.fUserLat <  -90.0)  // and
       || (Modes.fUserLon >  360.0)  // Longitude must be -180 to +360
       || (Modes.fUserLon < -180.0) ) {
    Modes.fUserLat = Modes.fUserLon = 0.0;
  } else if (Modes.fUserLon > 180.0) { // If Longitude is +180 to +360, make it -180 to 0
    Modes.fUserLon -= 360.0;
  }
  // If both Lat and Lon are 0.0 then the users location is either invalid/not-set, or (s)he's in the
  // Atlantic ocean off the west coast of Africa. This is unlikely to be correct.
  // Set the user LatLon valid flag only if either Lat or Lon are non zero. Note the Greenwich meridian
  // is at 0.0 Lon,so we must check for either fLat or fLon being non zero not both.
  // Testing the flag at runtime will be much quicker than ((fLon != 0.0) || (fLat != 0.0))
  Modes.bUserFlags &= ~MODES_USER_LATLON_VALID;
  if ((Modes.fUserLat != 0.0) || (Modes.fUserLon != 0.0)) {
    Modes.bUserFlags |= MODES_USER_LATLON_VALID;
  }

  // Limit the maximum requested raw output size to less than one Ethernet Block
  if (Modes.net_output_raw_size > (MODES_RAWOUT_BUF_FLUSH))
    {Modes.net_output_raw_size = MODES_RAWOUT_BUF_FLUSH;}
  if (Modes.net_output_raw_rate > (MODES_RAWOUT_BUF_RATE))
    {Modes.net_output_raw_rate = MODES_RAWOUT_BUF_RATE;}

  // Initialise the Block Timers to something half sensible
  ftime(&Modes.stSystemTimeRTL);
  Modes.stSystemTimeBlk         = Modes.stSystemTimeRTL;

  // Each I and Q value varies from 0 to 255, which represents a range from -1 to +1. To get from the
  // unsigned (0-255) range you therefore subtract 127 (or 128 or 127.5) from each I and Q, giving you
  // a range from -127 to +128 (or -128 to +127, or -127.5 to +127.5)..
  //
  // To decode the AM signal, you need the magnitude of the waveform, which is given by sqrt((I^2)+(Q^2))
  // The most this could be is if I&Q are both 128 (or 127 or 127.5), so you could end up with a magnitude
  // of 181.019 (or 179.605, or 180.312)
  //
  // However, in reality the magnitude of the signal should never exceed the range -1 to +1, because the
  // values are I = rCos(w) and Q = rSin(w). Therefore the integer computed magnitude should (can?) never
  // exceed 128 (or 127, or 127.5 or whatever)
  //
  // If we scale up the results so that they range from 0 to 65535 (16 bits) then we need to multiply
  // by 511.99, (or 516.02 or 514). antirez's original code multiplies by 360, presumably because he's
  // assuming the maximim calculated amplitude is 181.019, and (181.019 * 360) = 65166.
  //
  // So lets see if we can improve things by subtracting 127.5, Well in integer arithmatic we can't
  // subtract half, so, we'll double everything up and subtract one, and then compensate for the doubling
  // in the multiplier at the end.
  //
  // If we do this we can never have I or Q equal to 0 - they can only be as small as +/- 1.
  // This gives us a minimum magnitude of root 2 (0.707), so the dynamic range becomes (1.414-255). This
  // also affects our scaling value, which is now 65535/(255 - 1.414), or 258.433254
  //
  // The sums then become mag = 258.433254 * (sqrt((I*2-255)^2 + (Q*2-255)^2) - 1.414)
  //                   or mag = (258.433254 * sqrt((I*2-255)^2 + (Q*2-255)^2)) - 365.4798
  //
  // We also need to clip mag just incaes any rogue I/Q values somehow do have a magnitude greater than 255.
  //

  for (i = 0; i <= 255; i++) {
    for (q = 0; q <= 255; q++) {
      int mag, mag_i, mag_q;

      mag_i = (i * 2) - 255;
      mag_q = (q * 2) - 255;

      mag = (int) round((sqrt((mag_i*mag_i)+(mag_q*mag_q)) * 258.433254) - 365.4798);

      Modes.maglut[(i*256)+q] = (uint16_t) ((mag < 65535) ? mag : 65535);
    }
  }
}

/* =============================== RTLSDR handling ========================== */

void modesInitRTLSDR(void) {
  int j;
  int device_count;
  char vendor[256], product[256], serial[256];

  device_count = rtlsdr_get_device_count();
  if (!device_count) {
    fprintf(stderr, "No supported RTLSDR devices found.\n");
    exit(1);
  }

  fprintf(stderr, "Found %d device(s):\n", device_count);
  for (j = 0; j < device_count; j++) {
    rtlsdr_get_device_usb_strings(j, vendor, product, serial);
    fprintf(stderr, "%d: %s, %s, SN: %s %s\n", j, vendor, product, serial,
            (j == Modes.dev_index) ? "(currently selected)" : "");
  }

  if (rtlsdr_open(&Modes.dev, Modes.dev_index) < 0) {
    fprintf(stderr, "Error opening the RTLSDR device: %s\n",
            strerror(errno));
    exit(1);
  }

  /* Set gain, frequency, sample rate, and reset the device. */
  rtlsdr_set_tuner_gain_mode(Modes.dev,
                             (Modes.gain == MODES_AUTO_GAIN) ? 0 : 1);
  if (Modes.gain != MODES_AUTO_GAIN) {
    if (Modes.gain == MODES_MAX_GAIN) {
      /* Find the maximum gain available. */
      int numgains;
      int gains[100];

      numgains = rtlsdr_get_tuner_gains(Modes.dev, gains);
      Modes.gain = gains[numgains-1];
      fprintf(stderr, "Max available gain is: %.2f\n", Modes.gain/10.0);
    }
    rtlsdr_set_tuner_gain(Modes.dev, Modes.gain);
    fprintf(stderr, "Setting gain to: %.2f\n", Modes.gain/10.0);
  } else {
    fprintf(stderr, "Using automatic gain control.\n");
  }
  rtlsdr_set_freq_correction(Modes.dev, Modes.ppm_error);
  if (Modes.enable_agc) rtlsdr_set_agc_mode(Modes.dev, 1);
  rtlsdr_set_center_freq(Modes.dev, Modes.freq);
  rtlsdr_set_sample_rate(Modes.dev, MODES_DEFAULT_RATE);
  rtlsdr_reset_buffer(Modes.dev);
  fprintf(stderr, "Gain reported by device: %.2f\n",
          rtlsdr_get_tuner_gain(Modes.dev)/10.0);
}

/* We use a thread reading data in background, while the main thread
 * handles decoding and visualization of data to the user.
 *
 * The reading thread calls the RTLSDR API to read data asynchronously, and
 * uses a callback to populate the data buffer.
 * A Mutex is used to avoid races with the decoding thread. */
void rtlsdrCallback(unsigned char *buf, uint32_t len, void *ctx) {
  MODES_NOTUSED(ctx);

  pthread_mutex_lock(&Modes.data_mutex);
  ftime(&Modes.stSystemTimeRTL);
  if (len > MODES_ASYNC_BUF_SIZE) len = MODES_ASYNC_BUF_SIZE;
  /* Read the new data. */
  memcpy(Modes.data, buf, len);
  Modes.data_ready = 1;
  /* Signal to the other thread that new data is ready */
  pthread_cond_signal(&Modes.data_cond);
  pthread_mutex_unlock(&Modes.data_mutex);
}

/* This is used when --ifile is specified in order to read data from file
 * instead of using an RTLSDR device. */
void readDataFromFile(void) {
  pthread_mutex_lock(&Modes.data_mutex);
  while(1) {
    ssize_t nread, toread;
    unsigned char *p;

    if (Modes.exit == 1) break;
    if (Modes.data_ready) {
      pthread_cond_wait(&Modes.data_cond,&Modes.data_mutex);
      continue;
    }

    if (Modes.interactive) {
      /* When --ifile and --interactive are used together, slow down
       * playing at the natural rate of the RTLSDR received. */
      pthread_mutex_unlock(&Modes.data_mutex);
      usleep(64000);
      pthread_mutex_lock(&Modes.data_mutex);
    }

    toread = MODES_ASYNC_BUF_SIZE;
    p = (unsigned char *) Modes.data;
    while(toread) {
      nread = read(Modes.fd, p, toread);
      if (nread <= 0) {
        Modes.exit = 1; /* Signal the other thread to exit. */
        break;
      }
      p += nread;
      toread -= nread;
    }
    if (toread) {
      /* Not enough data on file to fill the buffer? Pad with
       * no signal. */
      memset(p,127,toread);
    }
    Modes.data_ready = 1;
    /* Signal to the other thread that new data is ready */
    pthread_cond_signal(&Modes.data_cond);
  }
}

/* We read data using a thread, so the main thread only handles decoding
 * without caring about data acquisition. */
void *readerThreadEntryPoint(void *arg) {
  MODES_NOTUSED(arg);

  if (Modes.filename == NULL) {
    rtlsdr_read_async(Modes.dev, rtlsdrCallback, NULL,
                      MODES_ASYNC_BUF_NUMBER,
                      MODES_ASYNC_BUF_SIZE);
  } else {
    readDataFromFile();
  }
  /* Signal to the other thread that new data is ready - dummy really so threads don't mutually lock */
  Modes.data_ready = 1;
  pthread_cond_signal(&Modes.data_cond);
  pthread_mutex_unlock(&Modes.data_mutex);
  pthread_exit(NULL);
}

/* ============================== Debugging ================================= */

/* Helper function for dumpMagnitudeVector().
 * It prints a single bar used to display raw signals.
 *
 * Since every magnitude sample is between 0-255, the function uses
 * up to 63 characters for every bar. Every character represents
 * a length of 4, 3, 2, 1, specifically:
 *
 * "O" is 4
 * "o" is 3
 * "-" is 2
 * "." is 1
 */
void dumpMagnitudeBar(int index, int magnitude) {
  char *set = " .-o";
  char buf[256];
  int div = magnitude / 256 / 4;
  int rem = magnitude / 256 % 4;

  memset(buf,'O',div);
  buf[div] = set[rem];
  buf[div+1] = '\0';

  if (index >= 0)
    printf("[%.3d] |%-66s %d\n", index, buf, magnitude);
  else
    printf("[%.2d] |%-66s %d\n", index, buf, magnitude);
}

/* Display an ASCII-art alike graphical representation of the undecoded
 * message as a magnitude signal.
 *
 * The message starts at the specified offset in the "m" buffer.
 * The function will display enough data to cover a short 56 bit message.
 *
 * If possible a few samples before the start of the messsage are included
 * for context. */

void dumpMagnitudeVector(uint16_t *m, uint32_t offset) {
  uint32_t padding = 5; /* Show a few samples before the actual start. */
  uint32_t start = (offset < padding) ? 0 : offset-padding;
  uint32_t end = offset + (MODES_PREAMBLE_SAMPLES)+(MODES_SHORT_MSG_SAMPLES) - 1;
  uint32_t j;

  for (j = start; j <= end; j++) {
    dumpMagnitudeBar(j-offset, m[j]);
  }
}

/* Produce a raw representation of the message as a Javascript file
 * loadable by debug.html. */
void dumpRawMessageJS(char *descr, unsigned char *msg,
                      uint16_t *m, uint32_t offset, int fixable, char *bitpos)
{
  int padding = 5; /* Show a few samples before the actual start. */
  int start = offset - padding;
  int end = offset + (MODES_PREAMBLE_SAMPLES)+(MODES_LONG_MSG_SAMPLES) - 1;
  FILE *fp;
  int j;

  MODES_NOTUSED(fixable);
  if ((fp = fopen("frames.js","a")) == NULL) {
    fprintf(stderr, "Error opening frames.js: %s\n", strerror(errno));
    exit(1);
  }

  fprintf(fp,"frames.push({\"descr\": \"%s\", \"mag\": [", descr);
  for (j = start; j <= end; j++) {
    fprintf(fp,"%d", j < 0 ? 0 : m[j]);
    if (j != end) fprintf(fp,",");
  }
  fprintf(fp,"], \"fix1\": %d, \"fix2\": %d, \"bits\": %d, \"hex\": \"",
          bitpos[0], bitpos[1] , Ten90ModeSMessageLenByType(msg[0]>>3));
  for (j = 0; j < MODES_LONG_MSG_BYTES; j++)
    fprintf(fp,"\\x%02x",msg[j]);
  fprintf(fp,"\"});\n");
  fclose(fp);
}

/* This is a wrapper for dumpMagnitudeVector() that also show the message
 * in hex format with an additional description.
 *
 * descr  is the additional message to show to describe the dump.
 * msg    points to the decoded message
 * m      is the original magnitude vector
 * offset is the offset where the message starts
 *
 * The function also produces the Javascript file used by debug.html to
 * display packets in a graphical format if the Javascript output was
 * enabled.
 */
void dumpRawMessage(char *descr, unsigned char *msg,
                    uint16_t *m, uint32_t offset)
{
  int  j;
  int  msgtype = msg[0] >> 3;
  int  fixable = 0;
  char bitpos[MODES_MAX_BITERRORS];

  for (j = 0;  j < MODES_MAX_BITERRORS;  j++) {
    bitpos[j] = -1;
  }
  if (msgtype == 17) {
    fixable = Ten90FixBitErrors(msg, MODES_LONG_MSG_BITS, MODES_MAX_BITERRORS,
                                bitpos);
  }

  if (Modes.debug & MODES_DEBUG_JS) {
    dumpRawMessageJS(descr, msg, m, offset, fixable, bitpos);
    return;
  }

  printf("\n--- %s\n    ", descr);
  for (j = 0; j < MODES_LONG_MSG_BYTES; j++) {
    printf("%02x",msg[j]);
    if (j == MODES_SHORT_MSG_BYTES-1) printf(" ... ");
  }
  printf(" (DF %d, Fixable: %d)\n", msgtype, fixable);
  dumpMagnitudeVector(m,offset);
  printf("---\n\n");
}

//
// Similar to fixSingleBitErrors() but try every possible two bit combination.
// This is very slow and should be tried only against DF17 messages that
// don't pass the checksum, and only in Aggressive Mode.
/*
  int fixTwoBitsErrors(unsigned char *msg, int bits) {
  int j, i;
  unsigned char aux[MODES_LONG_MSG_BYTES];

  memcpy(aux, msg, bits/8);

  // Do not attempt to error correct Bits 0-4. These contain the DF, and must
  // be correct because we can only error correct DF17
  for (j = 5; j < bits; j++) {
  int byte1    = j/8;
  int bitmask1 = 1 << (7 - (j & 7));
  aux[byte1] ^= bitmask1; // Flip j-th bit

  // Don't check the same pairs multiple times, so i starts from j+1
  for (i = j+1; i < bits; i++) {
  int byte2    = i/8;
  int bitmask2 = 1 << (7 - (i & 7));

  aux[byte2] ^= bitmask2; // Flip i-th bit

  if (0 == modesChecksum(aux, bits)) {
  // The error is fixed. Overwrite the original buffer with
  // the corrected sequence, and returns the error bit position
  msg[byte1] = aux[byte1];
  msg[byte2] = aux[byte2];

  // We return the two bits as a 16 bit integer by shifting
  // 'i' on the left. This is possible since 'i' will always
  // be non-zero because i starts from j+1
  return (j | (i << 8));

  aux[byte2] ^= bitmask2; // Flip i-th bit back
  }

  aux[byte1] ^= bitmask1; // Flip j-th bit back
  }
  }
  return (-1);
  }
*/

/* Code for testing the timing: run all possible 1- and 2-bit error
 * the test message by all 1-bit errors. Run the old code against
 * all of them, and new the code.
 *
 * Example measurements:
 * Timing old vs. new crc correction code:
 *    Old code: 1-bit errors on 112 msgs: 3934 usecs
 *    New code: 1-bit errors on 112 msgs: 104 usecs
 *    Old code: 2-bit errors on 6216 msgs: 407743 usecs
 *    New code: 2-bit errors on 6216 msgs: 5176 usecs
 * indicating a 37-fold resp. 78-fold improvement in speed for 1-bit resp.
 * 2-bit error.
 */
unsigned char tmsg0[MODES_LONG_MSG_BYTES] = {
  /* Test data: first ADS-B message from testfiles/modes1.bin */
  0x8f, 0x4d, 0x20, 0x23, 0x58, 0x7f, 0x34, 0x5e,
  0x35, 0x83, 0x7e, 0x22, 0x18, 0xb2
};
#define NTWOBITS (MODES_LONG_MSG_BITS*(MODES_LONG_MSG_BITS-1)/2)
unsigned char tmsg1[MODES_LONG_MSG_BITS][MODES_LONG_MSG_BYTES];
unsigned char tmsg2[NTWOBITS][MODES_LONG_MSG_BYTES];
/* Init an array of cloned messages with all possible 1-bit errors present,
 * applied to each message at the respective position
 */
void inittmsg1() {
  int i, bytepos, mask;
  for (i = 0;  i < MODES_LONG_MSG_BITS;  i++) {
    bytepos = i >> 3;
    mask = 1 << (7 - (i & 7));
    memcpy(&tmsg1[i][0], tmsg0, MODES_LONG_MSG_BYTES);
    tmsg1[i][bytepos] ^= mask;
  }
}

/* Run sanity check on all but first 5 messages / bits, as those bits
 * are not corrected.
 */
void checktmsg1(FILE *out) {
  int i, k;
  uint32_t crc;
  for (i = 5;  i < MODES_LONG_MSG_BITS;  i++) {
    crc = Ten90ModeSChecksum(&tmsg1[i][0], MODES_LONG_MSG_BITS);
    if (crc != 0) {
      fprintf(out, "CRC not fixed for "
              "positon %d\n", i);
      fprintf(out, "  MSG ");
      for (k = 0;  k < MODES_LONG_MSG_BYTES;  k++) {
        fprintf(out, "%02x", tmsg1[i][k]);
      }
      fprintf(out, "\n");
    }
  }
}

void inittmsg2() {
  int i, j, n, bytepos0, bytepos1, mask0, mask1;
  n = 0;
  for (i = 0;  i < MODES_LONG_MSG_BITS;  i++) {
    bytepos0 = i >> 3;
    mask0 = 1 << (7 - (i & 7));
    for (j = i+1;  j < MODES_LONG_MSG_BITS;  j++) {
      bytepos1 = j >> 3;
      mask1 = 1 << (7 - (j & 7));
      memcpy(&tmsg2[n][0], tmsg0, MODES_LONG_MSG_BYTES);
      tmsg2[n][bytepos0] ^= mask0;
      tmsg2[n][bytepos1] ^= mask1;
      n += 1;
    }
  }
}

long difftvusec(struct timeval *t0, struct timeval *t1) {
  long res = 0;
  res = t1->tv_usec-t0->tv_usec;
  res += (t1->tv_sec-t0->tv_sec)*1000000L;
  return res;
}

/* the actual test code */
void testAndTimeBitCorrection() {
  struct timeval starttv, endtv;
  int i;
  /* Run timing on 1-bit errors */
  printf("Timing old vs. new crc correction code:\n");
  inittmsg1();
  gettimeofday(&starttv, NULL);
  for (i = 0;  i < MODES_LONG_MSG_BITS;  i++) {
    Ten90FixSingleBitErrors(&tmsg1[i][0], MODES_LONG_MSG_BITS);
  }
  gettimeofday(&endtv, NULL);
  printf("   Old code: 1-bit errors on %d msgs: %ld usecs\n",
         MODES_LONG_MSG_BITS, difftvusec(&starttv, &endtv));
  checktmsg1(stdout);
  /* Re-init */
  inittmsg1();
  gettimeofday(&starttv, NULL);
  for (i = 0;  i < MODES_LONG_MSG_BITS;  i++) {
    Ten90FixBitErrors(&tmsg1[i][0], MODES_LONG_MSG_BITS, MODES_MAX_BITERRORS, NULL);
  }
  gettimeofday(&endtv, NULL);
  printf("   New code: 1-bit errors on %d msgs: %ld usecs\n",
         MODES_LONG_MSG_BITS, difftvusec(&starttv, &endtv));
  checktmsg1(stdout);
  /* Run timing on 2-bit errors */
  inittmsg2();
  gettimeofday(&starttv, NULL);
  for (i = 0;  i < NTWOBITS;  i++) {
    Ten90FixSingleBitErrors(&tmsg2[i][0], MODES_LONG_MSG_BITS);
  }
  gettimeofday(&endtv, NULL);
  printf("   Old code: 2-bit errors on %d msgs: %ld usecs\n",
         NTWOBITS, difftvusec(&starttv, &endtv));
  /* Re-init */
  inittmsg2();
  gettimeofday(&starttv, NULL);
  for (i = 0;  i < NTWOBITS;  i++) {
    Ten90FixBitErrors(&tmsg2[i][0], MODES_LONG_MSG_BITS, MODES_MAX_BITERRORS, NULL);
  }
  gettimeofday(&endtv, NULL);
  printf("   New code: 2-bit errors on %d msgs: %ld usecs\n",
         NTWOBITS, difftvusec(&starttv, &endtv));
}


/* Turn I/Q samples pointed by Modes.data into the magnitude vector
 * pointed by Modes.magnitude. */
void computeMagnitudeVector(void) {
  uint16_t *m = &Modes.magnitude[MODES_PREAMBLE_SAMPLES+MODES_LONG_MSG_SAMPLES];
  uint16_t *p = Modes.data;
  uint32_t j;

  memcpy(Modes.magnitude,&Modes.magnitude[MODES_ASYNC_BUF_SAMPLES], MODES_PREAMBLE_SIZE+MODES_LONG_MSG_SIZE);

  /* Compute the magnitudo vector. It's just SQRT(I^2 + Q^2), but
   * we rescale to the 0-255 range to exploit the full resolution. */
  for (j = 0; j < MODES_ASYNC_BUF_SAMPLES; j ++) {
    *m++ = Modes.maglut[*p++];
  }
}

/* Return -1 if the message is out of fase left-side
 * Return  1 if the message is out of fase right-size
 * Return  0 if the message is not particularly out of phase.
 *
 * Note: this function will access pPreamble[-1], so the caller should make sure to
 * call it only if we are not at the start of the current buffer. */
int detectOutOfPhase(uint16_t *pPreamble) {
  if (pPreamble[ 3] > pPreamble[2]/3) return  1;
  if (pPreamble[10] > pPreamble[9]/3) return  1;
  if (pPreamble[ 6] > pPreamble[7]/3) return -1;
  if (pPreamble[-1] > pPreamble[1]/3) return -1;
  return 0;
}

/* This function does not really correct the phase of the message, it just
 * applies a transformation to the first sample representing a given bit:
 *
 * If the previous bit was one, we amplify it a bit.
 * If the previous bit was zero, we decrease it a bit.
 *
 * This simple transformation makes the message a bit more likely to be
 * correctly decoded for out of phase messages:
 *
 * When messages are out of phase there is more uncertainty in
 * sequences of the same bit multiple times, since 11111 will be
 * transmitted as continuously altering magnitude (high, low, high, low...)
 *
 * However because the message is out of phase some part of the high
 * is mixed in the low part, so that it is hard to distinguish if it is
 * a zero or a one.
 *
 * However when the message is out of phase passing from 0 to 1 or from
 * 1 to 0 happens in a very recognizable way, for instance in the 0 -> 1
 * transition, magnitude goes low, high, high, low, and one of of the
 * two middle samples the high will be *very* high as part of the previous
 * or next high signal will be mixed there.
 *
 * Applying our simple transformation we make more likely if the current
 * bit is a zero, to detect another zero. Symmetrically if it is a one
 * it will be more likely to detect a one because of the transformation.
 * In this way similar levels will be interpreted more likely in the
 * correct way. */
void applyPhaseCorrection(uint16_t *pPayload) {
  int j;

  for (j = 0; j < MODES_LONG_MSG_SAMPLES; j += 2, pPayload += 2) {
    if (pPayload[0] > pPayload[1]) { /* One */
      pPayload[2] = (pPayload[2] * 5) / 4;
    } else {                           /* Zero */
      pPayload[2] = (pPayload[2] * 4) / 5;
    }
  }
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
int DetectModeA(uint16_t *m, Ten90Message *mm)
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


/* Detect a Mode S messages inside the magnitude buffer pointed by 'm' and of
 * size 'mlen' bytes. Every detected Mode S message is convert it into a
 * stream of bits and passed to the function to display it. */
void detectModeS(uint16_t *m, uint32_t mlen) {
  Ten90Message mm;
  unsigned char msg[MODES_LONG_MSG_BYTES], *pMsg;
  uint16_t aux[MODES_LONG_MSG_SAMPLES];
  uint32_t j;
  int use_correction = 0;

  memset(&mm, 0, sizeof(mm));

  /* The Mode S preamble is made of impulses of 0.5 microseconds at
   * the following time offsets:
   *
   * 0   - 0.5 usec: first impulse.
   * 1.0 - 1.5 usec: second impulse.
   * 3.5 - 4   usec: third impulse.
   * 4.5 - 5   usec: last impulse.
   *
   * Since we are sampling at 2 Mhz every sample in our magnitude vector
   * is 0.5 usec, so the preamble will look like this, assuming there is
   * an impulse at offset 0 in the array:
   *
   * 0   -----------------
   * 1   -
   * 2   ------------------
   * 3   --
   * 4   -
   * 5   --
   * 6   -
   * 7   ------------------
   * 8   --
   * 9   -------------------
   */
  for (j = 0; j < mlen; j++) {
    int high, i, errors, errors56, errorsTy;
    uint16_t *pPreamble, *pPayload, *pPtr;
    uint8_t  theByte, theErrs;
    int msglen, scanlen, sigStrength;

    pPreamble = &m[j];
    pPayload  = &m[j+MODES_PREAMBLE_SAMPLES];

    // Rather than clear the whole mm structure, just clear the parts which are required. The clear
    // is required for every bit of the input stream, and we don't want to be memset-ing the whole
    // ten90_mode_s_message structure two million times per second if we don't have to..
    mm.bFlags          =
      mm.crcok           =
      mm.correctedbits   = 0;

    if (!use_correction)  // This is not a re-try with phase correction
      {                 // so try to find a new preamble

        if (Modes.mode_ac)
          {
            int ModeA = DetectModeA(pPreamble, &mm);

            if (ModeA) // We have found a valid ModeA/C in the data
              {
                mm.timestampMsg = Modes.timestampBlk + ((j+1) * 6);

                // Decode the received message
                Ten90DecodeModeAMessage(&mm, ModeA);

                // Pass data to the next layer
                useModesMessage(&mm);

                j += MODEAC_MSG_SAMPLES;
                Modes.stat_ModeAC++;
                continue;
              }
          }

        /* First check of relations between the first 10 samples
         * representing a valid preamble. We don't even investigate further
         * if this simple test is not passed. */
        if (!(pPreamble[0] > pPreamble[1] &&
              pPreamble[1] < pPreamble[2] &&
              pPreamble[2] > pPreamble[3] &&
              pPreamble[3] < pPreamble[0] &&
              pPreamble[4] < pPreamble[0] &&
              pPreamble[5] < pPreamble[0] &&
              pPreamble[6] < pPreamble[0] &&
              pPreamble[7] > pPreamble[8] &&
              pPreamble[8] < pPreamble[9] &&
              pPreamble[9] > pPreamble[6]))
          {
            if (Modes.debug & MODES_DEBUG_NOPREAMBLE &&
                *pPreamble  > MODES_DEBUG_NOPREAMBLE_LEVEL)
              dumpRawMessage("Unexpected ratio among first 10 samples", msg, m, j);
            continue;
          }

        /* The samples between the two spikes must be < than the average
         * of the high spikes level. We don't test bits too near to
         * the high levels as signals can be out of phase so part of the
         * energy can be in the near samples. */
        high = (pPreamble[0] + pPreamble[2] + pPreamble[7] + pPreamble[9]) / 6;
        if (pPreamble[4] >= high ||
            pPreamble[5] >= high)
          {
            if (Modes.debug & MODES_DEBUG_NOPREAMBLE &&
                *pPreamble  > MODES_DEBUG_NOPREAMBLE_LEVEL)
              dumpRawMessage("Too high level in samples between 3 and 6", msg, m, j);
            continue;
          }

        /* Similarly samples in the range 11-14 must be low, as it is the
         * space between the preamble and real data. Again we don't test
         * bits too near to high levels, see above. */
        if (pPreamble[11] >= high ||
            pPreamble[12] >= high ||
            pPreamble[13] >= high ||
            pPreamble[14] >= high)
          {
            if (Modes.debug & MODES_DEBUG_NOPREAMBLE &&
                *pPreamble  > MODES_DEBUG_NOPREAMBLE_LEVEL)
              dumpRawMessage("Too high level in samples between 10 and 15", msg, m, j);
            continue;
          }
        Modes.stat_valid_preamble++;
      }

    else {
      /* If the previous attempt with this message failed, retry using
       * magnitude correction. */
      // Make a copy of the Payload, and phase correct the copy
      memcpy(aux, pPayload, sizeof(aux));
      applyPhaseCorrection(aux);
      Modes.stat_out_of_phase++;
      pPayload = aux;
      /* TODO ... apply other kind of corrections. */
    }

    /* Decode all the next 112 bits, regardless of the actual message
     * size. We'll check the actual message type later. */
    pMsg    = &msg[0];
    pPtr    = pPayload;
    theByte = 0;
    theErrs = 0; errorsTy = 0;
    errors  = 0; errors56 = 0;

    // We should have 4 'bits' of 0/1 and 1/0 samples in the preamble,
    // so include these in the signal strength
    sigStrength = (pPreamble[0]-pPreamble[1])
      + (pPreamble[2]-pPreamble[3])
      + (pPreamble[7]-pPreamble[6])
      + (pPreamble[9]-pPreamble[8]);

    msglen = scanlen = MODES_LONG_MSG_BITS;
    for (i = 0; i < scanlen; i++) {
      uint32_t a = *pPtr++;
      uint32_t b = *pPtr++;

      if      (a > b)
        {theByte |= 1; if (i < 56) {sigStrength += (a-b);}}
      else if (a < b)
        {/*theByte |= 0;*/ if (i < 56) {sigStrength += (b-a);}}
      else if (i >= MODES_SHORT_MSG_BITS) //(a == b), and we're in the long part of a frame
        {errors++;  /*theByte |= 0;*/}
      else if (i >= 5)                    //(a == b), and we're in the short part of a frame
        {scanlen = MODES_LONG_MSG_BITS; errors56 = ++errors;/*theByte |= 0;*/}
      else if (i)                         //(a == b), and we're in the message type part of a frame
        {errorsTy = errors56 = ++errors; theErrs |= 1; /*theByte |= 0;*/}
      else                                //(a == b), and we're in the first bit of the message type part of a frame
        {errorsTy = errors56 = ++errors; theErrs |= 1; theByte |= 1;}

      if ((i & 7) == 7)
        {*pMsg++ = theByte;}
      else if (i == 4) {
        msglen  = Ten90ModeSMessageLenByType(theByte);
        if (errors == 0)
          {scanlen = msglen;}
      }

      theByte = theByte << 1;
      if (i < 7)
        {theErrs = theErrs << 1;}

      // If we've exceeded the permissible number of encoding errors, abandon ship now
      if (errors > MODES_MSG_ENCODER_ERRS) {

        if        (i < MODES_SHORT_MSG_BITS) {
          msglen = 0;

        } else if ((errorsTy == 1) && (theErrs == 0x80)) {
          // If we only saw one error in the first bit of the byte of the frame, then it's possible
          // we guessed wrongly about the value of the bit. We may be able to correct it by guessing
          // the other way.
          //
          // We guessed a '1' at bit 7, which is the DF length bit == 112 Bits.
          // Inverting bit 7 will change the message type from a long to a short.
          // Invert the bit, cross your fingers and carry on.
          msglen  = MODES_SHORT_MSG_BITS;
          msg[0] ^= theErrs; errorsTy = 0;
          errors  = errors56; // revert to the number of errors prior to bit 56
          Modes.stat_DF_Len_Corrected++;

        } else if (i < MODES_LONG_MSG_BITS) {
          msglen = MODES_SHORT_MSG_BITS;
          errors = errors56;

        } else {
          msglen = MODES_LONG_MSG_BITS;
        }

        break;
      }
    }

    // Ensure msglen is consistent with the DF type
    i = Ten90ModeSMessageLenByType(msg[0] >> 3);
    if      (msglen > i) {msglen = i;}
    else if (msglen < i) {msglen = 0;}

    //
    // If we guessed at any of the bits in the DF type field, then look to see if our guess was sensible.
    // Do this by looking to see if the original guess results in the DF type being one of the ICAO defined
    // message types. If it isn't then toggle the guessed bit and see if this new value is ICAO defined.
    // if the new value is ICAO defined, then update it in our message.
    if ((msglen) && (errorsTy == 1) && (theErrs & 0x78)) {
      // We guessed at one (and only one) of the message type bits. See if our guess is "likely"
      // to be correct by comparing the DF against a list of known good DF's
      int      thisDF      = ((theByte = msg[0]) >> 3) & 0x1f;
      uint32_t validDFbits = 0x017F0831;   // One bit per 32 possible DF's. Set bits 0,4,5,11,16.17.18.19,20,21,22,24
      uint32_t thisDFbit   = (1 << thisDF);
      if (0 == (validDFbits & thisDFbit)) {
        // The current DF is not ICAO defined, so is probably an errors.
        // Toggle the bit we guessed at and see if the resultant DF is more likely
        theByte  ^= theErrs;
        thisDF    = (theByte >> 3) & 0x1f;
        thisDFbit = (1 << thisDF);
        // if this DF any more likely?
        if (validDFbits & thisDFbit) {
          // Yep, more likely, so update the main message
          msg[0] = theByte;
          Modes.stat_DF_Type_Corrected++;
          errors--; // decrease the error count so we attempt to use the modified DF.
        }
      }
    }

    // We measured signal strength over the first 56 bits. Don't forget to add 4
    // for the preamble samples, so round up and divide by 60.
    sigStrength = (sigStrength + 29) / 60;

    // When we reach this point, if error is small, and the signal strength is large enough
    // we may have a Mode S message on our hands. It may still be broken and the CRC may not
    // be correct, but this can be handled by the next layer.
    if ( (msglen)
         && (sigStrength >  MODES_MSG_SQUELCH_LEVEL)
         && (errors      <= MODES_MSG_ENCODER_ERRS) ) {

      // Set initial mm structure details
      mm.timestampMsg = Modes.timestampBlk + (j*6);
      sigStrength    = (sigStrength + 0x7F) >> 8;
      mm.signalLevel = ((sigStrength < 255) ? sigStrength : 255);
      mm.phase_corrected = use_correction;

      // Decode the received message
      Ten90DecodeFrame(msg, &Modes.ctx, &mm);

      // Update statistics
      if (Modes.stats) {
        if (mm.crcok || use_correction || mm.correctedbits) {

          if (use_correction) {
            switch (errors) {
            case 0: {Modes.stat_ph_demodulated0++; break;}
            case 1: {Modes.stat_ph_demodulated1++; break;}
            case 2: {Modes.stat_ph_demodulated2++; break;}
            default:{Modes.stat_ph_demodulated3++; break;}
            }
          } else {
            switch (errors) {
            case 0: {Modes.stat_demodulated0++; break;}
            case 1: {Modes.stat_demodulated1++; break;}
            case 2: {Modes.stat_demodulated2++; break;}
            default:{Modes.stat_demodulated3++; break;}
            }
          }

          if (mm.correctedbits == 0) {
            if (use_correction) {
              if (mm.crcok) {Modes.stat_ph_goodcrc++;}
              else          {Modes.stat_ph_badcrc++;}
            } else {
              if (mm.crcok) {Modes.stat_goodcrc++;}
              else          {Modes.stat_badcrc++;}
            }

          } else if (use_correction) {
            Modes.stat_ph_badcrc++;
            Modes.stat_ph_fixed++;
            if ( (mm.correctedbits)
                 && (mm.correctedbits <= MODES_MAX_BITERRORS) ) {
              Modes.stat_ph_bit_fix[mm.correctedbits-1] += 1;
            }

          } else {
            Modes.stat_badcrc++;
            Modes.stat_fixed++;
            if ( (mm.correctedbits)
                 && (mm.correctedbits <= MODES_MAX_BITERRORS) ) {
              Modes.stat_bit_fix[mm.correctedbits-1] += 1;
            }
          }
        }
      }

      // Output debug mode info if needed
      if (use_correction) {
        if (Modes.debug & MODES_DEBUG_DEMOD)
          dumpRawMessage("Demodulated with 0 errors", msg, m, j);
        else if (Modes.debug & MODES_DEBUG_BADCRC &&
                 mm.msgtype == 17 &&
                 (!mm.crcok || mm.correctedbits != 0))
          dumpRawMessage("Decoded with bad CRC", msg, m, j);
        else if (Modes.debug & MODES_DEBUG_GOODCRC && mm.crcok &&
                 mm.correctedbits == 0)
          dumpRawMessage("Decoded with good CRC", msg, m, j);
      }

      // Skip this message if we are sure it's fine
      if (mm.crcok) {
        j += (MODES_PREAMBLE_US+msglen)*2;
      }

      // Pass data to the next layer
      useModesMessage(&mm);

    } else {
      if (Modes.debug & MODES_DEBUG_DEMODERR && use_correction) {
        printf("The following message has %d demod errors\n", errors);
        dumpRawMessage("Demodulated with errors", msg, m, j);
      }
    }

    // Retry with phase correction if enabled, necessary and possible.
    if (Modes.phase_enhance && !mm.crcok && !mm.correctedbits && !use_correction && j && detectOutOfPhase(pPreamble)) {
      use_correction = 1; j--;
    } else {
      use_correction = 0;
    }
  }

  //Send any remaining partial raw buffers now
  if (Modes.rawOutUsed || Modes.beastOutUsed)
    {
      Modes.net_output_raw_rate_count++;
      if (Modes.net_output_raw_rate_count > Modes.net_output_raw_rate)
        {
          if (Modes.rawOutUsed) {
            modesSendAllClients(Modes.ros, Modes.rawOut, Modes.rawOutUsed);
            Modes.rawOutUsed = 0;
          }
          if (Modes.beastOutUsed) {
            modesSendAllClients(Modes.bos, Modes.beastOut, Modes.beastOutUsed);
            Modes.beastOutUsed = 0;
          }
          Modes.net_output_raw_rate_count = 0;
        }
    }
}

//
// When a new message is available, because it was decoded from the RTL device,
// file, or received in the TCP input port, or any other way we can receive a
// decoded message, we call this function in order to use the message.
//
// Basically this function passes a raw message to the upper layers for further
// processing and visualization
//
void useModesMessage(Ten90Message *mm) {
  if ((Modes.check_crc == 0) || (mm->crcok) || (mm->correctedbits)) { // not checking, ok or fixed

    // Track aircrafts if...
    if ( (Modes.interactive)          //       in interactive mode
         || (Modes.stat_http_requests)   // or if the HTTP interface is enabled
         || (Modes.stat_sbs_connections) // or if sbs connections are established
         || (Modes.mode_ac) ) {          // or if mode A/C decoding is enabled
      interactiveReceiveData(mm);
    }

    // In non-interactive non-quiet mode, display messages on standard output
    if (!Modes.interactive && !Modes.quiet) {
      Ten90DisplayModeSMessage(mm);
    }

    // Feed output clients
    if (Modes.stat_sbs_connections)   {modesSendSBSOutput(mm);}
    if (Modes.stat_beast_connections) {modesSendBeastOutput(mm);}
    if (Modes.stat_raw_connections)   {modesSendRawOutput(mm);}
  }
}

/* ========================= Interactive mode =============================== */
//
// Return a new aircraft structure for the interactive mode linked list
// of aircraft
//
struct aircraft *interactiveCreateAircraft(Ten90Message *mm) {
  struct aircraft *a = (struct aircraft *) malloc(sizeof(*a));

  // Default everything to zero/NULL
  memset(a, 0, sizeof(*a));

  // Now initialise things that should not be 0/NULL to their defaults
  a->addr = mm->addr;
  a->lat  = a->lon = 0.0;
  memset(a->signalLevel, mm->signalLevel, 8); // First time, initialise everything
  // to the first signal strength

  // mm->msgtype 32 is used to represent Mode A/C. These values can never change, so
  // set them once here during initialisation, and don't bother to set them every
  // time this ModeA/C is received again in the future
  if (mm->msgtype == 32) {
    int modeC      = Ten90ModeAToModeC(mm->modeA | mm->fs);
    a->modeACflags = MODEAC_MSG_FLAG;
    if (modeC < -12) {
      a->modeACflags |= MODEAC_MSG_MODEA_ONLY;
    } else {
      mm->altitude = modeC * 100;
      mm->bFlags  |= MODES_ACFLAGS_ALTITUDE_VALID;
    }
  }
  return (a);
}
//
// Return the aircraft with the specified address, or NULL if no aircraft
// exists with this address.
//
struct aircraft *interactiveFindAircraft(uint32_t addr) {
  struct aircraft *a = Modes.aircrafts;

  while(a) {
    if (a->addr == addr) return (a);
    a = a->next;
  }
  return (NULL);
}
//
// We have received a Mode A or C response.
//
// Search through the list of known Mode-S aircraft and tag them if this Mode A/C
// matches their known Mode S Squawks or Altitudes(+/- 50feet).
//
// A Mode S equipped aircraft may also respond to Mode A and Mode C SSR interrogations.
// We can't tell if this is a Mode A or C, so scan through the entire aircraft list
// looking for matches on Mode A (squawk) and Mode C (altitude). Flag in the Mode S
// records that we have had a potential Mode A or Mode C response from this aircraft.
//
// If an aircraft responds to Mode A then it's highly likely to be responding to mode C
// too, and vice verca. Therefore, once the mode S record is tagged with both a Mode A
// and a Mode C flag, we can be fairly confident that this Mode A/C frame relates to that
// Mode S aircraft.
//
// Mode C's are more likely to clash than Mode A's; There could be several aircraft
// cruising at FL370, but it's less likely (though not impossible) that there are two
// aircraft on the same squawk. Therefore, give precidence to Mode A record matches
//
// Note : It's theoretically possible for an aircraft to have the same value for Mode A
// and Mode C. Therefore we have to check BOTH A AND C for EVERY S.
//
void interactiveUpdateAircraftModeA(struct aircraft *a) {
  struct aircraft *b = Modes.aircrafts;

  while(b) {
    if ((b->modeACflags & MODEAC_MSG_FLAG) == 0) {// skip any fudged ICAO records

      // If both (a) and (b) have valid squawks...
      if ((a->bFlags & b->bFlags) & MODES_ACFLAGS_SQUAWK_VALID) {
        // ...check for Mode-A == Mode-S Squawk matches
        if (a->modeA == b->modeA) { // If a 'real' Mode-S ICAO exists using this Mode-A Squawk
          b->modeAcount   = a->messages;
          b->modeACflags |= MODEAC_MSG_MODEA_HIT;
          a->modeACflags |= MODEAC_MSG_MODEA_HIT;
          if ( (b->modeAcount > 0) &&
               ( (b->modeCcount > 1)
                 || (a->modeACflags & MODEAC_MSG_MODEA_ONLY)) ) // Allow Mode-A only matches if this Mode-A is invalid Mode-C
            {
              a->modeACflags |= MODEAC_MSG_MODES_HIT;
              fprintf(stderr, "Squawk hit\n");
            }    // flag this ModeA/C probably belongs to a known Mode S
        }
      }

      // If both (a) and (b) have valid altitudes...
      if ((a->bFlags & b->bFlags) & MODES_ACFLAGS_ALTITUDE_VALID) {
        // ... check for Mode-C == Mode-S Altitude matches
        if (  (a->modeC     == b->modeC    )     // If a 'real' Mode-S ICAO exists at this Mode-C Altitude
              || (a->modeC     == b->modeC + 1)     //          or this Mode-C - 100 ft
              || (a->modeC + 1 == b->modeC    ) ) { //          or this Mode-C + 100 ft
          b->modeCcount   = a->messages;
          b->modeACflags |= MODEAC_MSG_MODEC_HIT;
          a->modeACflags |= MODEAC_MSG_MODEC_HIT;
          if ( (b->modeAcount > 0) &&
               (b->modeCcount > 1) ) {
            {a->modeACflags |= (MODEAC_MSG_MODES_HIT | MODEAC_MSG_MODEC_OLD);} // flag this ModeA/C probably belongs to a known Mode S
            fprintf(stderr, "Altitude hit\n");
          }
        }
      }
    }
    b = b->next;
  }
}

void interactiveUpdateAircraftModeS() {
  struct aircraft *a = Modes.aircrafts;

  while(a) {
    int flags = a->modeACflags;
    if (flags & MODEAC_MSG_FLAG) { // find any fudged ICAO records

      // clear the current A,C and S hit bits ready for this attempt
      a->modeACflags = flags & ~(MODEAC_MSG_MODEA_HIT | MODEAC_MSG_MODEC_HIT | MODEAC_MSG_MODES_HIT);

      interactiveUpdateAircraftModeA(a);  // and attempt to match them with Mode-S
    }
    a = a->next;
  }
}

/* Always positive MOD operation, used for CPR decoding. */
int cprModFunction(int a, int b) {
  int res = a % b;
  if (res < 0) res += b;
  return res;
}

/* The NL function uses the precomputed table from 1090-WP-9-14 */
int cprNLFunction(double lat) {
  if (lat < 0) lat = -lat; /* Table is simmetric about the equator. */
  if (lat < 10.47047130) return 59;
  if (lat < 14.82817437) return 58;
  if (lat < 18.18626357) return 57;
  if (lat < 21.02939493) return 56;
  if (lat < 23.54504487) return 55;
  if (lat < 25.82924707) return 54;
  if (lat < 27.93898710) return 53;
  if (lat < 29.91135686) return 52;
  if (lat < 31.77209708) return 51;
  if (lat < 33.53993436) return 50;
  if (lat < 35.22899598) return 49;
  if (lat < 36.85025108) return 48;
  if (lat < 38.41241892) return 47;
  if (lat < 39.92256684) return 46;
  if (lat < 41.38651832) return 45;
  if (lat < 42.80914012) return 44;
  if (lat < 44.19454951) return 43;
  if (lat < 45.54626723) return 42;
  if (lat < 46.86733252) return 41;
  if (lat < 48.16039128) return 40;
  if (lat < 49.42776439) return 39;
  if (lat < 50.67150166) return 38;
  if (lat < 51.89342469) return 37;
  if (lat < 53.09516153) return 36;
  if (lat < 54.27817472) return 35;
  if (lat < 55.44378444) return 34;
  if (lat < 56.59318756) return 33;
  if (lat < 57.72747354) return 32;
  if (lat < 58.84763776) return 31;
  if (lat < 59.95459277) return 30;
  if (lat < 61.04917774) return 29;
  if (lat < 62.13216659) return 28;
  if (lat < 63.20427479) return 27;
  if (lat < 64.26616523) return 26;
  if (lat < 65.31845310) return 25;
  if (lat < 66.36171008) return 24;
  if (lat < 67.39646774) return 23;
  if (lat < 68.42322022) return 22;
  if (lat < 69.44242631) return 21;
  if (lat < 70.45451075) return 20;
  if (lat < 71.45986473) return 19;
  if (lat < 72.45884545) return 18;
  if (lat < 73.45177442) return 17;
  if (lat < 74.43893416) return 16;
  if (lat < 75.42056257) return 15;
  if (lat < 76.39684391) return 14;
  if (lat < 77.36789461) return 13;
  if (lat < 78.33374083) return 12;
  if (lat < 79.29428225) return 11;
  if (lat < 80.24923213) return 10;
  if (lat < 81.19801349) return 9;
  if (lat < 82.13956981) return 8;
  if (lat < 83.07199445) return 7;
  if (lat < 83.99173563) return 6;
  if (lat < 84.89166191) return 5;
  if (lat < 85.75541621) return 4;
  if (lat < 86.53536998) return 3;
  if (lat < 87.00000000) return 2;
  else return 1;
}

int cprNFunction(double lat, int fflag) {
  int nl = cprNLFunction(lat) - (fflag ? 1 : 0);
  if (nl < 1) nl = 1;
  return nl;
}

double cprDlonFunction(double lat, int fflag, int surface) {
  return (surface ? 90.0 : 360.0) / cprNFunction(lat, fflag);
}

/* This algorithm comes from:
 * http://www.lll.lu/~edward/edward/adsb/DecodingADSBposition.html.
 *
 * A few remarks:
 * 1) 131072 is 2^17 since CPR latitude and longitude are encoded in 17 bits.
 * 2) We assume that we always received the odd packet as last packet for
 *    simplicity. This may provide a position that is less fresh of a few
 *    seconds.
 */
void decodeCPR(struct aircraft *a, int fflag, int surface) {
  double AirDlat0 = (surface ? 90.0 : 360.0) / 60.0;
  double AirDlat1 = (surface ? 90.0 : 360.0) / 59.0;
  double lat0 = a->even_cprlat;
  double lat1 = a->odd_cprlat;
  double lon0 = a->even_cprlon;
  double lon1 = a->odd_cprlon;

  // Compute the Latitude Index "j"
  int    j     = (int) floor(((59*lat0 - 60*lat1) / 131072) + 0.5);
  double rlat0 = AirDlat0 * (cprModFunction(j,60) + lat0 / 131072);
  double rlat1 = AirDlat1 * (cprModFunction(j,59) + lat1 / 131072);

  if (surface) {
    // If we're on the ground, make sure we have our receiver base station Lat/Lon
    if (0 == (Modes.bUserFlags & MODES_USER_LATLON_VALID))
      {return;}
    rlat0 += floor(Modes.fUserLat / 90.0) * 90.0;  // Move from 1st quadrant to our quadrant
    rlat1 += floor(Modes.fUserLat / 90.0) * 90.0;
  } else {
    if (rlat0 >= 270) rlat0 -= 360;
    if (rlat1 >= 270) rlat1 -= 360;
  }

  // Check that both are in the same latitude zone, or abort.
  if (cprNLFunction(rlat0) != cprNLFunction(rlat1)) return;

  // Compute ni and the Longitude Index "m"
  if (fflag) { // Use odd packet.
    int ni = cprNFunction(rlat1,1);
    int m = (int) floor((((lon0 * (cprNLFunction(rlat1)-1)) -
                          (lon1 * cprNLFunction(rlat1))) / 131072.0) + 0.5);
    a->lon = cprDlonFunction(rlat1, 1, surface) * (cprModFunction(m, ni)+lon1/131072);
    a->lat = rlat1;
  } else {     // Use even packet.
    int ni = cprNFunction(rlat0,0);
    int m = (int) floor((((lon0 * (cprNLFunction(rlat0)-1)) -
                          (lon1 * cprNLFunction(rlat0))) / 131072) + 0.5);
    a->lon = cprDlonFunction(rlat0, 0, surface) * (cprModFunction(m, ni)+lon0/131072);
    a->lat = rlat0;
  }

  if (surface) {
    a->lon += floor(Modes.fUserLon / 90.0) * 90.0;  // Move from 1st quadrant to our quadrant
  } else if (a->lon > 180) {
    a->lon -= 360;
  }

  a->seenLatLon      = a->seen;
  a->timestampLatLon = a->timestamp;
  a->bFlags         |= (MODES_ACFLAGS_LATLON_VALID | MODES_ACFLAGS_LATLON_REL_OK);
}

/* This algorithm comes from:
 * 1090-WP29-07-Draft_CPR101 (which also defines decodeCPR() )
 *
 * There is an error in this document related to CPR relative decode.
 * Should use trunc() rather than the floor() function in Eq 38 and related for deltaZI.
 * floor() returns integer less than argument
 * trunc() returns integer closer to zero than argument.
 * Note:   text of document describes trunc() functionality for deltaZI calculation
 *         but the formulae use floor().
 */
int decodeCPRrelative(struct aircraft *a, int fflag, int surface) {
  double AirDlat;
  double AirDlon;
  double lat;
  double lon;
  double lonr, latr;
  double rlon, rlat;
  int j,m;

  if (a->bFlags & MODES_ACFLAGS_LATLON_REL_OK) { // Ok to try aircraft relative first
    latr = a->lat;
    lonr = a->lon;
  } else if (Modes.bUserFlags & MODES_USER_LATLON_VALID) { // Try ground station relative next
    latr = Modes.fUserLat;
    lonr = Modes.fUserLon;
  } else {
    return (-1); // Exit with error - can't do relative if we don't have ref.
  }

  if (fflag) { // odd
    AirDlat = (surface ? 90.0 : 360.0) / 59.0;
    lat = a->odd_cprlat;
    lon = a->odd_cprlon;
  } else {    // even
    AirDlat = (surface ? 90.0 : 360.0) / 60.0;
    lat = a->even_cprlat;
    lon = a->even_cprlon;
  }

  // Compute the Latitude Index "j"
  j = (int) (floor(latr/AirDlat) +
             trunc(0.5 + cprModFunction((int)latr, (int)AirDlat)/AirDlat - lat/131072));
  rlat = AirDlat * (j + lat/131072);
  if (rlat >= 270) rlat -= 360;

  // Check to see that answer is reasonable - ie no more than 1/2 cell away
  if (fabs(rlat - a->lat) > (AirDlat/2)) {
    a->bFlags &= ~MODES_ACFLAGS_LATLON_REL_OK; // This will cause a quick exit next time if no global has been done
    return (-1);                               // Time to give up - Latitude error
  }

  // Compute the Longitude Index "m"
  AirDlon = cprDlonFunction(rlat, fflag, surface);
  m = (int) (floor(lonr/AirDlon) +
             trunc(0.5 + cprModFunction((int)lonr, (int)AirDlon)/AirDlon - lon/131072));
  rlon = AirDlon * (m + lon/131072);
  if (rlon > 180) rlon -= 360;

  // Check to see that answer is reasonable - ie no more than 1/2 cell away
  if (fabs(rlon - a->lon) > (AirDlon/2)) {
    a->bFlags &= ~MODES_ACFLAGS_LATLON_REL_OK; // This will cause a quick exit next time if no global has been done
    return (-1);                               // Time to give up - Longitude error
  }

  a->lat = rlat;
  a->lon = rlon;

  a->seenLatLon      = a->seen;
  a->timestampLatLon = a->timestamp;
  a->bFlags         |= (MODES_ACFLAGS_LATLON_VALID | MODES_ACFLAGS_LATLON_REL_OK);
  return (0);
}

/* Receive new messages and populate the interactive mode with more info. */
struct aircraft *interactiveReceiveData(Ten90Message *mm) {
  struct aircraft *a, *aux;

  // Return if (checking crc) AND (not crcok) AND (not fixed)
  if (Modes.check_crc && (mm->crcok == 0) && (mm->correctedbits == 0))
    return NULL;

  // Loookup our aircraft or create a new one
  a = interactiveFindAircraft(mm->addr);
  if (!a) {                              // If it's a currently unknown aircraft....
    a = interactiveCreateAircraft(mm); // ., create a new record for it,
    a->next = Modes.aircrafts;         // .. and put it at the head of the list
    Modes.aircrafts = a;
  } else {
    /* If it is an already known aircraft, move it on head
     * so we keep aircrafts ordered by received message time.
     *
     * However move it on head only if at least one second elapsed
     * since the aircraft that is currently on head sent a message,
     * othewise with multiple aircrafts at the same time we have an
     * useless shuffle of positions on the screen. */
    if (0 && Modes.aircrafts != a && (time(NULL) - a->seen) >= 1) {
      aux = Modes.aircrafts;
      while(aux->next != a) aux = aux->next;
      /* Now we are a node before the aircraft to remove. */
      aux->next = aux->next->next; /* removed. */
      /* Add on head */
      a->next = Modes.aircrafts;
      Modes.aircrafts = a;
    }
  }

  a->signalLevel[a->messages & 7] = mm->signalLevel;// replace the 8th oldest signal strength
  a->seen      = time(NULL);
  a->timestamp = mm->timestampMsg;
  a->messages++;

  // If a (new) CALLSIGN has been received, copy it to the aircraft structure
  if (mm->bFlags & MODES_ACFLAGS_CALLSIGN_VALID) {
    memcpy(a->flight, mm->flight, sizeof(a->flight));
  }

  // If a (new) ALTITUDE has been received, copy it to the aircraft structure
  if (mm->bFlags & MODES_ACFLAGS_ALTITUDE_VALID) {
    if ( (a->modeCcount)                   // if we've a modeCcount already
         && (a->altitude  != mm->altitude ) ) // and Altitude has changed
      //        && (a->modeC     != mm->modeC + 1)   // and Altitude not changed by +100 feet
      //        && (a->modeC + 1 != mm->modeC    ) ) // and Altitude not changes by -100 feet
      {
        a->modeCcount   = 0;               //....zero the hit count
        a->modeACflags &= ~MODEAC_MSG_MODEC_HIT;
      }
    a->altitude = mm->altitude;
    a->modeC    = (mm->altitude + 49) / 100;
  }

  // If a (new) SQUAWK has been received, copy it to the aircraft structure
  if (mm->bFlags & MODES_ACFLAGS_SQUAWK_VALID) {
    if (a->modeA != mm->modeA) {
      a->modeAcount   = 0; // Squawk has changed, so zero the hit count
      a->modeACflags &= ~MODEAC_MSG_MODEA_HIT;
    }
    a->modeA = mm->modeA;
  }

  // If a (new) HEADING has been received, copy it to the aircraft structure
  if (mm->bFlags & MODES_ACFLAGS_HEADING_VALID) {
    a->track = mm->heading;
  }

  // If a (new) SPEED has been received, copy it to the aircraft structure
  if (mm->bFlags & MODES_ACFLAGS_SPEED_VALID) {
    a->speed = mm->velocity;
  }

  // If a (new) Vertical Descent rate has been received, copy it to the aircraft structure
  if (mm->bFlags & MODES_ACFLAGS_VERTRATE_VALID) {
    a->vert_rate = mm->vert_rate;
  }

  // if the Aircraft has landed or taken off since the last message, clear the even/odd CPR flags
  if ((mm->bFlags & MODES_ACFLAGS_AOG_VALID) && ((a->bFlags ^ mm->bFlags) & MODES_ACFLAGS_AOG)) {
    a->bFlags &= ~(MODES_ACFLAGS_LLBOTH_VALID | MODES_ACFLAGS_AOG);

  } else  if (   (mm->bFlags & MODES_ACFLAGS_LLEITHER_VALID)
                 && (((mm->bFlags | a->bFlags) & MODES_ACFLAGS_LLEITHER_VALID) == MODES_ACFLAGS_LLBOTH_VALID) ) {
    // If it's a new even/odd raw lat/lon, and we now have both even and odd,decode the CPR
    int fflag;

    if (mm->bFlags & MODES_ACFLAGS_LLODD_VALID) {
      fflag = 1;
      a->odd_cprlat  = mm->raw_latitude;
      a->odd_cprlon  = mm->raw_longitude;
      a->odd_cprtime = mstime();
    } else {
      fflag = 0;
      a->even_cprlat  = mm->raw_latitude;
      a->even_cprlon  = mm->raw_longitude;
      a->even_cprtime = mstime();
    }
    // Try relative CPR first
    if (decodeCPRrelative(a, fflag, (mm->bFlags & MODES_ACFLAGS_AOG))) {
      // If it fails then try global if the two data are less than 10 seconds apart
      if (abs((int)(a->even_cprtime - a->odd_cprtime)) <= 10000) {
        decodeCPR(a, fflag, (mm->bFlags & MODES_ACFLAGS_AOG));
      }
    }

    //If we sucessfully decoded, back copy the results to mm so that we can print them in list output
    if (a->bFlags & MODES_ACFLAGS_LATLON_VALID) {
      mm->bFlags |= MODES_ACFLAGS_LATLON_VALID;
      mm->fLat    = a->lat;
      mm->fLon    = a->lon;
    }
  }

  // Update the aircrafts a->bFlags to reflect the newly received mm->bFlags;
  a->bFlags |= mm->bFlags;

  if (mm->msgtype == 32) {
    int flags = a->modeACflags;
    if ((flags & (MODEAC_MSG_MODEC_HIT | MODEAC_MSG_MODEC_OLD)) == MODEAC_MSG_MODEC_OLD) {
      //
      // This Mode-C doesn't currently hit any known Mode-S, but it used to because MODEAC_MSG_MODEC_OLD is
      // set  So the aircraft it used to match has either changed altitude, or gone out of our receiver range
      //
      // We've now received this Mode-A/C again, so it must be a new aircraft. It could be another aircraft
      // at the same Mode-C altitude, or it could be a new airctraft with a new Mods-A squawk.
      //
      // To avoid masking this aircraft from the interactive display, clear the MODEAC_MSG_MODES_OLD flag
      // and set messages to 1;
      //
      a->modeACflags = flags & ~MODEAC_MSG_MODEC_OLD;
      a->messages    = 1;
    }
  }

  return (a);
}

/* Show the currently captured interactive data on screen. */
void interactiveShowData(void) {
  struct aircraft *a = Modes.aircrafts;
  time_t now = time(NULL);
  int count = 0;
  char progress;
  char spinner[4] = "|/-\\";

  progress = spinner[time(NULL)%4];

  printf("\x1b[H\x1b[2J");    /* Clear the screen */

  if (Modes.interactive_rtl1090 == 0) {
    printf (
            "Hex     Mode  Sqwk  Flight   Alt    Spd  Hdg    Lat      Long   Sig  Msgs   Ti%c\n", progress);
  } else {
    printf (
            "Hex    Flight   Alt      V/S GS  TT  SSR  G*456^ Msgs    Seen %c\n", progress);
  }
  printf(
         "-------------------------------------------------------------------------------\n");

  while(a && count < Modes.interactive_rows) {
    int msgs  = a->messages;
    int flags = a->modeACflags;

    if ( (((flags & (MODEAC_MSG_FLAG                             )) == 0                    )                 )
         || (((flags & (MODEAC_MSG_MODES_HIT | MODEAC_MSG_MODEA_ONLY)) == MODEAC_MSG_MODEA_ONLY) && (msgs > 4  ) )
         || (((flags & (MODEAC_MSG_MODES_HIT | MODEAC_MSG_MODEC_OLD )) == 0                    ) && (msgs > 127) )
         ) {
      int altitude = a->altitude, speed = a->speed;
      char strSquawk[5] = " ";
      char strFl[6]     = " ";
      char strTt[5]     = " ";
      char strGs[5]     = " ";

      // Convert units to metric if --metric was specified
      if (Modes.metric) {
        altitude = (int) (altitude / 3.2828);
        speed    = (int) (speed    * 1.852);
      }

      if (a->bFlags & MODES_ACFLAGS_SQUAWK_VALID) {
        snprintf(strSquawk,5,"%04x", a->modeA);}

      if (a->bFlags & MODES_ACFLAGS_SPEED_VALID) {
        snprintf (strGs, 5,"%3d", speed);}

      if (a->bFlags & MODES_ACFLAGS_HEADING_VALID) {
        snprintf (strTt, 5,"%03d", a->track);}

      if (msgs > 99999) {
        msgs = 99999;}

      if (Modes.interactive_rtl1090) { // RTL1090 display mode

        if (a->bFlags & MODES_ACFLAGS_ALTITUDE_VALID) {
          snprintf(strFl,6,"F%03d",(altitude/100));
        }
        printf("%06x %-8s %-4s         %-3s %-3s %4s        %-6d  %-2d\n",
               a->addr, a->flight, strFl, strGs, strTt, strSquawk, msgs, (int)(now - a->seen));

      } else {                         // Dump1090 display mode
        char strMode[5]               = "    ";
        char strLat[8]                = " ";
        char strLon[9]                = " ";
        unsigned char * pSig       = a->signalLevel;
        unsigned int signalAverage = (pSig[0] + pSig[1] + pSig[2] + pSig[3] +
                                      pSig[4] + pSig[5] + pSig[6] + pSig[7] + 3) >> 3;

        if ((flags & MODEAC_MSG_FLAG) == 0) {
          strMode[0] = 'S';
        } else if (flags & MODEAC_MSG_MODEA_ONLY) {
          strMode[0] = 'A';
        }
        if (flags & MODEAC_MSG_MODEA_HIT) {strMode[2] = 'a';}
        if (flags & MODEAC_MSG_MODEC_HIT) {strMode[3] = 'c';}

        if (a->bFlags & MODES_ACFLAGS_LATLON_VALID) {
          snprintf(strLat, 8,"%7.03f", a->lat);
          snprintf(strLon, 9,"%8.03f", a->lon);
        }

        if (a->bFlags & MODES_ACFLAGS_AOG) {
          snprintf(strFl, 6," grnd");
        } else if (a->bFlags & MODES_ACFLAGS_ALTITUDE_VALID) {
          snprintf(strFl, 6, "%5d", altitude);
        }

        //              printf("%06x  %-4s  %-4s  %-8s %5d  %3d  %3d  %7.03f %8.03f  %3d %5d   %2d\n",
        //              a->addr, strMode, strSquawk, a->flight, altitude, speed, a->track,
        //              a->lat, a->lon, signalAverage, msgs, (int)(now - a->seen));

        printf("%06x  %-4s  %-4s  %-8s %5s  %3s  %3s  %7s %8s  %3d %5d   %2d\n",
               a->addr, strMode, strSquawk, a->flight, strFl, strGs, strTt,
               strLat, strLon, signalAverage, msgs, (int)(now - a->seen));
      }
      count++;
    }
    a = a->next;
  }
}

/* When in interactive mode If we don't receive new nessages within
 * MODES_INTERACTIVE_TTL seconds we remove the aircraft from the list. */
void interactiveRemoveStaleAircrafts(void) {
  struct aircraft *a = Modes.aircrafts;
  struct aircraft *prev = NULL;
  time_t now = time(NULL);

  while(a) {
    if ((now - a->seen) > Modes.interactive_ttl) {
      struct aircraft *next = a->next;
      /* Remove the element from the linked list, with care
       * if we are removing the first element. */
      free(a);
      if (!prev)
        Modes.aircrafts = next;
      else
        prev->next = next;
      a = next;
    } else {
      prev = a;
      a = a->next;
    }
  }
}

/* ============================== Snip mode ================================= */

/* Get raw IQ samples and filter everything is < than the specified level
 * for more than 256 samples in order to reduce example file size. */
void snipMode(int level) {
  int i, q;
  uint64_t c = 0;

  while ((i = getchar()) != EOF && (q = getchar()) != EOF) {
    if (abs(i-127) < level && abs(q-127) < level) {
      c++;
      if (c > MODES_PREAMBLE_SIZE) continue;
    } else {
      c = 0;
    }
    putchar(i);
    putchar(q);
  }
}

/* ============================= Networking =================================
 * Note: here we disregard any kind of good coding practice in favor of
 * extreme simplicity, that is:
 *
 * 1) We only rely on the kernel buffers for our I/O without any kind of
 *    user space buffering.
 * 2) We don't register any kind of event handler, from time to time a
 *    function gets called and we accept new connections. All the rest is
 *    handled via non-blocking I/O and manually polling clients to see if
 *    they have something new to share with us when reading is needed.
 */

/* Networking "stack" initialization. */
void modesInitNet(void) {
  struct {
    char *descr;
    int *socket;
    int port;
  } services[6] = {
    {"Raw TCP output", &Modes.ros, Modes.net_output_raw_port},
    {"Raw TCP input", &Modes.ris, Modes.net_input_raw_port},
    {"Beast TCP output", &Modes.bos, Modes.net_output_beast_port},
    {"Beast TCP input", &Modes.bis, Modes.net_input_beast_port},
    {"HTTP server", &Modes.https, Modes.net_http_port},
    {"Basestation TCP output", &Modes.sbsos, Modes.net_output_sbs_port}
  };
  int j;

  memset(Modes.clients,0,sizeof(Modes.clients));
  Modes.maxfd = -1;

  for (j = 0; j < 6; j++) {
    int s = anetTcpServer(Modes.aneterr, services[j].port, NULL);
    if (s == -1) {
      fprintf(stderr, "Error opening the listening port %d (%s): %s\n",
              services[j].port, services[j].descr, strerror(errno));
      exit(1);
    }
    anetNonBlock(Modes.aneterr, s);
    *services[j].socket = s;
  }

  signal(SIGPIPE, SIG_IGN);
}

/* This function gets called from time to time when the decoding thread is
 * awakened by new data arriving. This usually happens a few times every
 * second. */
void modesAcceptClients(void) {
  int fd, port;
  unsigned int j;
  struct client *c;
  int services[6];

  services[0] = Modes.ros;
  services[1] = Modes.ris;
  services[2] = Modes.bos;
  services[3] = Modes.bis;
  services[4] = Modes.https;
  services[5] = Modes.sbsos;

  for (j = 0; j < sizeof(services)/sizeof(int); j++) {
    fd = anetTcpAccept(Modes.aneterr, services[j], NULL, &port);
    if (fd == -1) continue;

    if (fd >= MODES_NET_MAX_FD) {
      close(fd);
      return; /* Max number of clients reached. */
    }

    anetNonBlock(Modes.aneterr, fd);
    c = (struct client *) malloc(sizeof(*c));
    c->service = services[j];
    c->fd = fd;
    c->buflen = 0;
    Modes.clients[fd] = c;
    anetSetSendBuffer(Modes.aneterr,fd,MODES_NET_SNDBUF_SIZE);

    if (Modes.maxfd < fd) Modes.maxfd = fd;
    if (services[j] == Modes.sbsos) Modes.stat_sbs_connections++;
    if (services[j] == Modes.ros)   Modes.stat_raw_connections++;
    if (services[j] == Modes.bos)   Modes.stat_beast_connections++;

    j--; /* Try again with the same listening port. */

    if (Modes.debug & MODES_DEBUG_NET)
      printf("Created new client %d\n", fd);
  }
}

/* On error free the client, collect the structure, adjust maxfd if needed. */
void modesFreeClient(int fd) {
  close(fd);
  if (Modes.clients[fd]->service == Modes.sbsos) {
    if (Modes.stat_sbs_connections) Modes.stat_sbs_connections--;
  }
  else if (Modes.clients[fd]->service == Modes.ros) {
    if (Modes.stat_raw_connections) Modes.stat_raw_connections--;
  }
  else if (Modes.clients[fd]->service == Modes.bos) {
    if (Modes.stat_beast_connections) Modes.stat_beast_connections--;
  }
  free(Modes.clients[fd]);
  Modes.clients[fd] = NULL;

  if (Modes.debug & MODES_DEBUG_NET)
    printf("Closing client %d\n", fd);

  /* If this was our maxfd, rescan the full clients array to check what's
   * the new max. */
  if (Modes.maxfd == fd) {
    int j;

    Modes.maxfd = -1;
    for (j = 0; j < MODES_NET_MAX_FD; j++) {
      if (Modes.clients[j]) Modes.maxfd = j;
    }
  }
}

/* Send the specified message to all clients listening for a given service. */
void modesSendAllClients(int service, void *msg, int len) {
  int j;
  struct client *c;

  for (j = 0; j <= Modes.maxfd; j++) {
    c = Modes.clients[j];
    if (c && c->service == service) {
      int nwritten = write(j, msg, len);
      if (nwritten != len) {
        modesFreeClient(j);
      }
    }
  }
}

/* Write raw output in Beast Binary format with Timestamp to TCP clients */
void modesSendBeastOutput(Ten90Message *mm) {
  char *p = &Modes.beastOut[Modes.beastOutUsed];
  int  msgLen = mm->msgbits / 8;
  char * pTimeStamp;
  int  j;

  *p++ = 0x1a;
  if      (msgLen == MODES_SHORT_MSG_BYTES)
    {*p++ = '2';}
  else if (msgLen == MODES_LONG_MSG_BYTES)
    {*p++ = '3';}
  else if (msgLen == MODEAC_MSG_BYTES)
    {*p++ = '1';}
  else
    {return;}

  pTimeStamp = (char *) &mm->timestampMsg;
  for (j = 5; j >= 0; j--) {
    *p++ = pTimeStamp[j];
  }

  *p++ = mm->signalLevel;

  memcpy(p, mm->msg, msgLen);

  Modes.beastOutUsed += (msgLen + 9);
  if (Modes.beastOutUsed >= Modes.net_output_raw_size)
    {
      modesSendAllClients(Modes.bos, Modes.beastOut, Modes.beastOutUsed);
      Modes.beastOutUsed = 0;
      Modes.net_output_raw_rate_count = 0;
    }
}

/* Write raw output to TCP clients. */
void modesSendRawOutput(Ten90Message *mm) {
  char *p = &Modes.rawOut[Modes.rawOutUsed];
  int  msgLen = mm->msgbits / 8;
  int j;
  unsigned char * pTimeStamp;

  if (Modes.mlat && mm->timestampMsg) {
    *p++ = '@';
    pTimeStamp = (unsigned char *) &mm->timestampMsg;
    for (j = 5; j >= 0; j--) {
      sprintf(p, "%02X", pTimeStamp[j]);
      p += 2;
    }
    Modes.rawOutUsed += 12; // additional 12 characters for timestamp
  } else
    *p++ = '*';

  for (j = 0; j < msgLen; j++) {
    sprintf(p, "%02X", mm->msg[j]);
    p += 2;
  }

  *p++ = ';';
  *p++ = '\n';

  Modes.rawOutUsed += ((msgLen*2) + 3);
  if (Modes.rawOutUsed >= Modes.net_output_raw_size)
    {
      modesSendAllClients(Modes.ros, Modes.rawOut, Modes.rawOutUsed);
      Modes.rawOutUsed = 0;
      Modes.net_output_raw_rate_count = 0;
    }
}
//
// Write SBS output to TCP clients
// The message structure mm->bFlags tells us what has been updated by this message
//
void modesSendSBSOutput(Ten90Message *mm) {
  char msg[256], *p = msg;
  uint32_t     offset;
  struct timeb epocTime;
  struct tm    stTime;
  int          msgType;

  //
  // SBS BS style output checked against the following reference
  // http://www.homepages.mcb.net/bones/SBS/Article/Barebones42_Socket_Data.htm - seems comprehensive
  //

  // Decide on the basic SBS Message Type
  if        ((mm->msgtype ==  4) || (mm->msgtype == 20)) {
    msgType = 5;
  } else if ((mm->msgtype ==  5) || (mm->msgtype == 21)) {
    msgType = 6;
  } else if ((mm->msgtype ==  0) || (mm->msgtype == 16)) {
    msgType = 7;
  } else if  (mm->msgtype == 11) {
    msgType = 8;
  } else if ((mm->msgtype != 17) && (mm->msgtype != 18)) {
    return;
  } else if ((mm->metype >= 1) && (mm->metype <=  4)) {
    msgType = 1;
  } else if ((mm->metype >= 5) && (mm->metype <=  8)) {
    if (mm->bFlags & MODES_ACFLAGS_LATLON_VALID)
      {msgType = 2;}
    else
      {msgType = 7;}
  } else if ((mm->metype >= 9) && (mm->metype <= 18)) {
    if (mm->bFlags & MODES_ACFLAGS_LATLON_VALID)
      {msgType = 3;}
    else
      {msgType = 7;}
  } else if (mm->metype !=  19) {
    return;
  } else if ((mm->mesub == 1) || (mm->mesub == 2)) {
    msgType = 4;
  } else {
    return;
  }

  // Fields 1 to 6 : SBS message type and ICAO address of the aircraft and some other stuff
  p += sprintf(p, "MSG,%d,111,11111,%06X,111111,", msgType, mm->addr);

  // Fields 7 & 8 are the current time and date
  if (mm->timestampMsg) {                                       // Make sure the records' timestamp is valid before outputing it
    epocTime = Modes.stSystemTimeBlk;                         // This is the time of the start of the Block we're processing
    offset   = (int) (mm->timestampMsg - Modes.timestampBlk); // This is the time (in 12Mhz ticks) into the Block
    offset   = offset / 12000;                                // convert to milliseconds
    epocTime.millitm += offset;                               // add on the offset time to the Block start time
    if (epocTime.millitm > 999)                               // if we've caused an overflow into the next second...
      {epocTime.millitm -= 1000; epocTime.time ++;}         //    ..correct the overflow
    stTime   = *localtime(&epocTime.time);                    // convert the time to year, month  day, hours, min, sec
    p += sprintf(p, "%04d/%02d/%02d,", (stTime.tm_year+1900),(stTime.tm_mon+1), stTime.tm_mday);
    p += sprintf(p, "%02d:%02d:%02d.%03d,", stTime.tm_hour, stTime.tm_min, stTime.tm_sec, epocTime.millitm);
  } else {
    p += sprintf(p, ",,");
  }

  // Fields 9 & 10 are the current time and date
  ftime(&epocTime);                                         // get the current system time & date
  stTime = *localtime(&epocTime.time);                      // convert the time to year, month  day, hours, min, sec
  p += sprintf(p, "%04d/%02d/%02d,", (stTime.tm_year+1900),(stTime.tm_mon+1), stTime.tm_mday);
  p += sprintf(p, "%02d:%02d:%02d.%03d", stTime.tm_hour, stTime.tm_min, stTime.tm_sec, epocTime.millitm);

  // Field 11 is the callsign (if we have it)
  if (mm->bFlags & MODES_ACFLAGS_CALLSIGN_VALID) {p += sprintf(p, ",%s", mm->flight);}
  else                                           {p += sprintf(p, ",");}

  // Field 12 is the altitude (if we have it) - force to zero if we're on the ground
  if ((mm->bFlags & MODES_ACFLAGS_AOG_GROUND) == MODES_ACFLAGS_AOG_GROUND) {
    p += sprintf(p, ",0");
  } else if (mm->bFlags & MODES_ACFLAGS_ALTITUDE_VALID) {
    p += sprintf(p, ",%d", mm->altitude);
  } else {
    p += sprintf(p, ",");
  }

  // Field 13 and 14 are the ground Speed and Heading (if we have them)
  if (mm->bFlags & MODES_ACFLAGS_NSEWSPD_VALID) {p += sprintf(p, ",%d,%d", mm->velocity, mm->heading);}
  else                                          {p += sprintf(p, ",,");}

  // Fields 15 and 16 are the Lat/Lon (if we have it)
  if (mm->bFlags & MODES_ACFLAGS_LATLON_VALID) {p += sprintf(p, ",%1.5f,%1.5f", mm->fLat, mm->fLon);}
  else                                         {p += sprintf(p, ",,");}

  // Field 17 is the VerticalRate (if we have it)
  if (mm->bFlags & MODES_ACFLAGS_VERTRATE_VALID) {p += sprintf(p, ",%d", mm->vert_rate);}
  else                                           {p += sprintf(p, ",");}

  // Field 18 is  the Squawk (if we have it)
  if (mm->bFlags & MODES_ACFLAGS_SQUAWK_VALID) {p += sprintf(p, ",%x", mm->modeA);}
  else                                         {p += sprintf(p, ",");}

  // Field 19 is the Squawk Changing Alert flag (if we have it)
  if (mm->bFlags & MODES_ACFLAGS_FS_VALID) {
    if ((mm->fs >= 2) && (mm->fs <= 4)) {
      p += sprintf(p, ",-1");
    } else {
      p += sprintf(p, ",0");
    }
  } else {
    p += sprintf(p, ",");
  }

  // Field 20 is the Squawk Emergency flag (if we have it)
  if (mm->bFlags & MODES_ACFLAGS_SQUAWK_VALID) {
    if ((mm->modeA == 0x7500) || (mm->modeA == 0x7600) || (mm->modeA == 0x7700)) {
      p += sprintf(p, ",-1");
    } else {
      p += sprintf(p, ",0");
    }
  } else {
    p += sprintf(p, ",");
  }

  // Field 21 is the Squawk Ident flag (if we have it)
  if (mm->bFlags & MODES_ACFLAGS_FS_VALID) {
    if ((mm->fs >= 4) && (mm->fs <= 5)) {
      p += sprintf(p, ",-1");
    } else {
      p += sprintf(p, ",0");
    }
  } else {
    p += sprintf(p, ",");
  }

  // Field 22 is the OnTheGround flag (if we have it)
  if (mm->bFlags & MODES_ACFLAGS_AOG_VALID) {
    if (mm->bFlags & MODES_ACFLAGS_AOG) {
      p += sprintf(p, ",-1");
    } else {
      p += sprintf(p, ",0");
    }
  } else {
    p += sprintf(p, ",");
  }

  p += sprintf(p, "\r\n");
  modesSendAllClients(Modes.sbsos, msg, p-msg);
}


// Turn an hex digit into its 4 bit decimal value.  Returns -1 if the
// digit is not in the 0-F range.
static int hexDigitVal(int c) {
  c = tolower(c);
  if (c >= '0' && c <= '9') return c-'0';
  else if (c >= 'a' && c <= 'f') return c-'a'+10;
  else return -1;
}


// This function decodes a string representing message in raw hex
// format like: *8D4B969699155600E87406F5B69F; The string is
// null-terminated.
//
// The message is passed to the higher level layers, so it feeds the
// selected screen output, the network output and so forth.
//
// If the message looks invalid it is silently discarded.
//
// The function always returns 0 (success) to the caller as there is
// no case where we want broken messages here to close the client
// connection.

int decodeHexMessage(struct client *c, char *hex) {
  Ten90Message mm;
  int l = strlen(hex), j;
  unsigned char msg[MODES_LONG_MSG_BYTES];

  memset(&mm, 0, sizeof(mm));

  // Mark messages received over the internet as remote so that we
  // don't try to pass them off as being received by this instance
  // when forwarding them
  mm.remote = 1;
  mm.signalLevel = 0xFF;

  // Remove spaces on the left and on the right
  while (l && isspace(hex[l-1])) {
    hex[l-1] = '\0'; l--;
  }
  while (isspace(*hex)) {
    hex++; l--;
  }

  // Turn the message into binary.
  // Accept *-AVR raw @-AVR/BEAST timeS+raw %-AVR timeS+raw (CRC good) <-BEAST timeS+sigL+raw
  // and some AVR records that we can understand
  if (hex[l - 1] != ';') {
    // not complete - abort
    return 0;
  }

  switch(hex[0]) {
  case '<': {
    mm.signalLevel = (hexDigitVal(hex[13]) << 4) | hexDigitVal(hex[14]);
    // Skip <, timestamp and siglevel, and ;
    hex += 15; l -= 16;
    break;
  }

  case '@':     // No CRC check
  case '%': {   // CRC is OK
    // Skip @,%, and timestamp, and ;
    hex += 13; l -= 14;
    break;
  }
  case '*':
  case ':': {
    // Skip * and ;
    hex++; l -= 2;
    break;
  }

  default: {
    // We don't know what this is, so abort
    return 0;
  }
  }

  if ((l != (MODEAC_MSG_BYTES * 2)) &&
      (l != (MODES_SHORT_MSG_BYTES * 2)) &&
      (l != (MODES_LONG_MSG_BYTES * 2))) {
    // Too short or long message... broken
    return 0;
  }

  if ((0 == Modes.mode_ac) &&
      (l == (MODEAC_MSG_BYTES * 2))) {
    // Right length for ModeA/C, but not enabled
    return 0;
  }

  for (j = 0; j < l; j += 2) {
    int high = hexDigitVal(hex[j]);
    int low  = hexDigitVal(hex[j + 1]);
    if (high == -1 || low == -1) {
      return 0;
    }
    msg[j/2] = (high << 4) | low;
  }

  if (l == (MODEAC_MSG_BYTES * 2)) {
    // ModeA or ModeC
    Ten90DecodeModeAMessage(&mm, ((msg[0] << 8) | msg[1]));
  } else {
    // Assume ModeS
    Ten90DecodeFrame(msg, &Modes.ctx, &mm);
  }

  useModesMessage(&mm);

  return 0;
}


// This function decodes a Beast binary format message
//
// The message is passed to the higher level layers, so it feeds the
// selected screen output, the network output and so forth.
//
// If the message looks invalid it is silently discarded.
//
// The function always returns 0 (success) to the caller as there is
// no case where we want broken messages here to close the client
// connection.

int decodeBinMessage(struct client *c, char *p) {
  Ten90Message mm;
  int msgLen = 0;
  unsigned char msg[MODES_LONG_MSG_BYTES];

  memset(&mm, 0, sizeof(mm));
  // Mark messages received over the internet as remote so that we
  // don't try to pass them off as being received by this instance
  // when forwarding them
  mm.remote = 1;

  if ((*p == '1') && (Modes.mode_ac)) { // skip ModeA/C unless user enables --modes-ac
    msgLen = MODEAC_MSG_BYTES;
  } else if (*p == '2') {
    msgLen = MODES_SHORT_MSG_BYTES;
  } else if (*p == '3') {
    msgLen = MODES_LONG_MSG_BYTES;
  }

  if (msgLen) {
    p += 7;                 // Skip the timestamp
    mm.signalLevel = *p++;  // Grab the signal level
    memcpy(msg, p, msgLen); // and the data

    if (msgLen == MODEAC_MSG_BYTES) { // ModeA or ModeC
      Ten90DecodeModeAMessage(&mm, ((msg[0] << 8) | msg[1]));
    } else {
      Ten90DecodeFrame(msg, &Modes.ctx, &mm);
    }

    useModesMessage(&mm);
  }

  return 0;
}


/* Return a description of planes in json. */
char *aircraftsToJson(int *len) {
  time_t now = time(NULL);
  struct aircraft *a = Modes.aircrafts;
  int buflen = 1024; /* The initial buffer is incremented as needed. */
  char *buf = (char *) malloc(buflen), *p = buf;
  int l;

  l = snprintf(p,buflen,"[\n");
  p += l; buflen -= l;
  while(a) {
    int altitude = a->altitude, speed = a->speed;
    int position = 0;
    int track = 0;

    if (a->modeACflags & MODEAC_MSG_FLAG) { // skip any fudged ICAO records Mode A/C
      a = a->next;
      continue;
    }

    /* Convert units to metric if --metric was specified. */
    if (Modes.metric) {
      altitude = (int) (altitude / 3.2828);
      speed    = (int) (speed * 1.852);
    }

    if (a->bFlags & MODES_ACFLAGS_LATLON_VALID) {
      position = 1;
    }

    if (a->bFlags & MODES_ACFLAGS_HEADING_VALID) {
      track = 1;
    }

    l = snprintf(p,buflen,
                 "{\"hex\":\"%06x\", \"squawk\":\"%04x\", \"flight\":\"%s\", \"lat\":%f, "
                 "\"lon\":%f, \"validposition\":%d, \"altitude\":%d, \"track\":%d, \"validtrack\":%d,"
                 "\"speed\":%d, \"messages\":%ld, \"seen\":%d},\n",
                 a->addr, a->modeA, a->flight, a->lat, a->lon, position, a->altitude, a->track, track,
                 a->speed, a->messages, (int)(now - a->seen));
    p += l; buflen -= l;

    /* Resize if needed. */
    if (buflen < 256) {
      int used = p-buf;
      buflen += 1024; // Our increment.
      buf = (char *) realloc(buf,used+buflen);
      p = buf+used;
    }

    a = a->next;
  }
  /* Remove the final comma if any, and closes the json array. */
  if (*(p-2) == ',') {
    *(p-2) = '\n';
    p--;
    buflen++;
  }
  l = snprintf(p,buflen,"]\n");
  p += l; buflen -= l;

  *len = p-buf;
  return buf;
}

#define MODES_CONTENT_TYPE_HTML "text/html;charset=utf-8"
#define MODES_CONTENT_TYPE_CSS  "text/css;charset=utf-8"
#define MODES_CONTENT_TYPE_JSON "application/json;charset=utf-8"
#define MODES_CONTENT_TYPE_JS   "application/javascript;charset=utf-8"

/* Get an HTTP request header and write the response to the client.
 * Again here we assume that the socket buffer is enough without doing
 * any kind of userspace buffering.
 *
 * Returns 1 on error to signal the caller the client connection should
 * be closed. */
int handleHTTPRequest(struct client *c, char *p) {
  char hdr[512];
  int clen, hdrlen;
  int httpver, keepalive;
  char *url, *content;
  char ctype[48];
  char getFile[1024];
  char *ext;

  if (Modes.debug & MODES_DEBUG_NET)
    printf("\nHTTP request: %s\n", c->buf);

  // Minimally parse the request.
  httpver = (strstr(p, "HTTP/1.1") != NULL) ? 11 : 10;
  if (httpver == 10) {
    // HTTP 1.0 defaults to close, unless otherwise specified.
    keepalive = strstr(p, "Connection: keep-alive") != NULL;
  } else if (httpver == 11) {
    // HTTP 1.1 defaults to keep-alive, unless close is specified.
    keepalive = strstr(p, "Connection: close") == NULL;
  }

  // Identify he URL.
  p = strchr(p,' ');
  if (!p) return 1; /* There should be the method and a space... */
  url = ++p; /* Now this should point to the requested URL. */
  p = strchr(p, ' ');
  if (!p) return 1; /* There should be a space before HTTP/... */
  *p = '\0';

  if (Modes.debug & MODES_DEBUG_NET) {
    printf("\nHTTP keep alive: %d\n", keepalive);
    printf("HTTP requested URL: %s\n\n", url);
  }

  if (strlen(url) < 2) {
    snprintf(getFile, sizeof getFile, "%s/gmap.html", HTMLPATH); // Default file
  } else {
    snprintf(getFile, sizeof getFile, "%s/%s", HTMLPATH, url);
  }

  /* Select the content to send, we have just two so far:
   * "/" -> Our google map application.
   * "/data.json" -> Our ajax request to update planes. */
  if (strstr(url, "/data.json")) {
    content = aircraftsToJson(&clen);
    //snprintf(ctype, sizeof ctype, MODES_CONTENT_TYPE_JSON);
  } else {
    struct stat sbuf;
    int fd = -1;

    if (stat(getFile, &sbuf) != -1 && (fd = open(getFile, O_RDONLY)) != -1) {
      content = (char *) malloc(sbuf.st_size);
      if (read(fd, content, sbuf.st_size) == -1) {
        snprintf(content, sbuf.st_size, "Error reading from file: %s", strerror(errno));
      }
      clen = sbuf.st_size;
    } else {
      char buf[128];
      clen = snprintf(buf,sizeof(buf),"Error opening HTML file: %s", strerror(errno));
      content = strdup(buf);
    }

    if (fd != -1) {
      close(fd);
    }
  }

  // Get file extension and content type
  snprintf(ctype, sizeof ctype, MODES_CONTENT_TYPE_HTML); // Default content type
  ext = strrchr(getFile, '.');

  if (strlen(ext) > 0) {
    if (strstr(ext, ".json")) {
      snprintf(ctype, sizeof ctype, MODES_CONTENT_TYPE_JSON);
    } else if (strstr(ext, ".css")) {
      snprintf(ctype, sizeof ctype, MODES_CONTENT_TYPE_CSS);
    } else if (strstr(ext, ".js")) {
      snprintf(ctype, sizeof ctype, MODES_CONTENT_TYPE_JS);
    }
  }

  /* Create the header and send the reply. */
  hdrlen = snprintf(hdr, sizeof(hdr),
                    "HTTP/1.1 200 OK\r\n"
                    "Server: Dump1090\r\n"
                    "Content-Type: %s\r\n"
                    "Connection: %s\r\n"
                    "Content-Length: %d\r\n"
                    "\r\n",
                    ctype,
                    keepalive ? "keep-alive" : "close",
                    clen);

  if (Modes.debug & MODES_DEBUG_NET) {
    printf("HTTP Reply header:\n%s", hdr);
  }

  // Send header and content.
  if (write(c->fd, hdr, hdrlen) == -1 || write(c->fd, content, clen) == -1) {
    free(content);
    return 1;
  }
  free(content);
  Modes.stat_http_requests++;
  return !keepalive;
}
//
// This function polls the clients using read() in order to receive new
// messages from the net.
//
// The message is supposed to be separated from the next message by the
// separator 'sep', which is a null-terminated C string.
//
// Every full message received is decoded and passed to the higher layers
// calling the function's 'handler'.
//
// The handler returns 0 on success, or 1 to signal this function we should
// close the connection with the client in case of non-recoverable errors.
void modesReadFromClient(struct client *c, char *sep,
                         int(*handler)(struct client *, char *)) {
  int left;
  int nread;
  int fullmsg;
  char *s, *e;

  while(1) {

    fullmsg = 0;
    left = MODES_CLIENT_BUF_SIZE - c->buflen;
    // If our buffer is full discard it, this is some badly formatted shit
    if (left == 0) {
      c->buflen = 0;
      left = MODES_CLIENT_BUF_SIZE;
      // If there is garbage, read more to discard it ASAP
    }
    nread = read(c->fd, c->buf+c->buflen, left);

    if (nread <= 0) {
      if (nread == 0 || errno != EAGAIN) { // Error, or end of file
        modesFreeClient(c->fd);
      }
      break; // Serve next client
    }
    c->buflen += nread;

    // Always null-term so we are free to use strstr() (it won't affect binary case)
    c->buf[c->buflen] = '\0';

    e = s = c->buf;                                // Start with the start of buffer, first message

    if (c->service == Modes.bis) {
      // This is the Bease Binary scanning case.
      // If there is a complete message still in the buffer, there must be the separator 'sep'
      // in the buffer, note that we full-scan the buffer at every read for simplicity.

      left = c->buflen;                                  // Length of valid search for memchr()
      while (left && ((s = memchr(e, (char) 0x1a, left)) != NULL)) {    // In reality the first byte of buffer 'should' be 0x1a
        s++;                                           // skip the 0x1a
        if        (*s == '1') {
          e = s + MODEAC_MSG_BYTES      + 8;         // point past remainder of message
        } else if (*s == '2') {
          e = s + MODES_SHORT_MSG_BYTES + 8;
        } else if (*s == '3') {
          e = s + MODES_LONG_MSG_BYTES  + 8;
        } else {
          e = s;                                     // Not a valid beast message, skip
          left = &(c->buf[c->buflen]) - e;
          continue;
        }
        left = &(c->buf[c->buflen]) - e;
        if (left < 0) {                                // Incomplete message in buffer
          e = s - 1;                                 // point back at last found 0x1a.
          break;
        }
        // Have a 0x1a followed by 1, 2 or 3 - pass message less 0x1a to handler.
        if (handler(c, s)) {
          modesFreeClient(c->fd);
          return;
        }
        fullmsg = 1;
      }
      s = e;     // For the buffer remainder below

    } else {
      // This is the ASCII scanning case, AVR RAW or HTTP at present
      // If there is a complete message still in the buffer, there must be the separator 'sep'
      // in the buffer, note that we full-scan the buffer at every read for simplicity.

      while ((e = strstr(s, sep)) != NULL) { // end of first message if found
        *e = '\0';                         // The handler expects null terminated strings
        if (handler(c, s)) {               // Pass message to handler.
          modesFreeClient(c->fd);        // Handler returns 1 on error to signal we .
          return;                        // should close the client connection
        }
        s = e + strlen(sep);               // Move to start of next message
        fullmsg = 1;
      }
    }

    if (fullmsg) { // We processed something - so
      c->buflen = &(c->buf[c->buflen]) - s;  // The unprocessed buffer length
      memmove(c->buf, s, c->buflen);         // move what's remaining to the start of the buffer
    } else { // If no message was decoded process the next client
      break;
    }
  }
}
//
// Read data from clients. This function actually delegates a lower-level
// function that depends on the kind of service (raw, http, ...).
void modesReadFromClients(void) {
  int j;
  struct client *c;

  for (j = 0; j <= Modes.maxfd; j++) {
    if ((c = Modes.clients[j]) == NULL) continue;
    if (c->service == Modes.ris)
      modesReadFromClient(c,"\n", decodeHexMessage);
    else if (c->service == Modes.bis)
      modesReadFromClient(c,"", decodeBinMessage);
    else if (c->service == Modes.https)
      modesReadFromClient(c,"\r\n\r\n", handleHTTPRequest);
  }
}

/* ================================ Main ==================================== */

void showHelp(void) {
  printf(
         "-----------------------------------------------------------------------------\n"
         "|                        dump1090 ModeS Receiver         Ver : " DUMP1090_VERSION " |\n"
         "-----------------------------------------------------------------------------\n"
         "--device-index <index>   Select RTL device (default: 0)\n"
         "--gain <db>              Set gain (default: max gain. Use -100 for auto-gain)\n"
         "--enable-agc             Enable the Automatic Gain Control (default: off)\n"
         "--freq <hz>              Set frequency (default: 1090 Mhz)\n"
         "--ifile <filename>       Read data from file (use '-' for stdin)\n"
         "--interactive            Interactive mode refreshing data on screen\n"
         "--interactive-rows <num> Max number of rows in interactive mode (default: 15)\n"
         "--interactive-ttl <sec>  Remove from list if idle for <sec> (default: 60)\n"
         "--interactive-rtl1090    Display flight table in RTL1090 format\n"
         "--raw                    Show only messages hex values\n"
         "--net                    Enable networking\n"
         "--modeac                 Enable decoding of SSR Modes 3/A & 3/C\n"
         "--net-beast              TCP raw output in Beast binary format\n"
         "--net-only               Enable just networking, no RTL device or file used\n"
         "--net-http-port <port>   HTTP server port (default: 8080)\n"
         "--net-ri-port <port>     TCP raw input listen port  (default: 30001)\n"
         "--net-ro-port <port>     TCP raw output listen port (default: 30002)\n"
         "--net-sbs-port <port>    TCP BaseStation output listen port (default: 30003)\n"
         "--net-bi-port <port>     TCP Beast input listen port  (default: 30004)\n"
         "--net-bo-port <port>     TCP Beast output listen port (default: 30005)\n"
         "--net-ro-size <size>     TCP raw output minimum size (default: 0)\n"
         "--net-ro-rate <rate>     TCP raw output memory flush rate (default: 0)\n"
         "--lat <latitude>         Reference/receiver latitide for surface posn (opt)\n"
         "--lon <longitude>        Reference/receiver longitude for surface posn (opt)\n"
         "--fix                    Enable single-bits error correction using CRC\n"
         "--no-fix                 Disable single-bits error correction using CRC\n"
         "--no-crc-check           Disable messages with broken CRC (discouraged)\n"
         "--phase-enhance          Enable phase enhancement\n"
         "--aggressive             More CPU for more messages (two bits fixes, ...)\n"
         "--mlat                   display raw messages in Beast ascii mode\n"
         "--stats                  With --ifile print stats at exit. No other output\n"
         "--onlyaddr               Show only ICAO addresses (testing purposes)\n"
         "--metric                 Use metric units (meters, km/h, ...)\n"
         "--snip <level>           Strip IQ file removing samples < level\n"
         "--debug <flags>          Debug mode (verbose), see README for details\n"
         "--quiet                  Disable output to stdout. Use for daemon applications\n"
         "--ppm <error>            Set receiver error in parts per million (default 0)\n"
         "--help                   Show this help\n"
         "\n"
         "Debug mode flags: d = Log frames decoded with errors\n"
         "                  D = Log frames decoded with zero errors\n"
         "                  c = Log frames with bad CRC\n"
         "                  C = Log frames with good CRC\n"
         "                  p = Log frames with bad preamble\n"
         "                  n = Log network debugging info\n"
         "                  j = Log frames to frames.js, loadable by debug.html\n"
         );
}

/* This function is called a few times every second by main in order to
 * perform tasks we need to do continuously, like accepting new clients
 * from the net, refreshing the screen in interactive mode, and so forth. */
void backgroundTasks(void) {
  if (Modes.net) {
    modesAcceptClients();
    modesReadFromClients();
  }

  // If Modes.aircrafts is not NULL, remove any stale aircraft
  if (Modes.aircrafts)
    {interactiveRemoveStaleAircrafts();}

  // Refresh screen when in interactive mode
  if ((Modes.interactive) &&
      ((mstime() - Modes.interactive_last_update) > MODES_INTERACTIVE_REFRESH_TIME) ) {

    // Attempt to reconsile any ModeA/C with known Mode-S
    // We can't condition on Modes.modeac because ModeA/C could be comming
    // in from a raw input port which we can't turn off.
    interactiveUpdateAircraftModeS();

    // Now display Mode-S and any non-reconsiled Modes-A/C
    interactiveShowData();

    Modes.interactive_last_update = mstime();
  }
}

int main(int argc, char **argv) {
  int j;

  // Set sane defaults
  modesInitConfig();
  signal(SIGINT, sigintHandler); // Define Ctrl/C handler (exit program)

  /* Parse the command line options */
  for (j = 1; j < argc; j++) {
    int more = j+1 < argc; /* There are more arguments. */

    if (!strcmp(argv[j],"--device-index") && more) {
      Modes.dev_index = atoi(argv[++j]);
    } else if (!strcmp(argv[j],"--gain") && more) {
      Modes.gain = (int) atof(argv[++j])*10; /* Gain is in tens of DBs */
    } else if (!strcmp(argv[j],"--enable-agc")) {
      Modes.enable_agc++;
    } else if (!strcmp(argv[j],"--freq") && more) {
      Modes.freq = (int) strtoll(argv[++j],NULL,10);
    } else if (!strcmp(argv[j],"--ifile") && more) {
      Modes.filename = strdup(argv[++j]);
    } else if (!strcmp(argv[j],"--fix")) {
      Modes.nfix_crc = 1;
    } else if (!strcmp(argv[j],"--no-fix")) {
      Modes.nfix_crc = 0;
    } else if (!strcmp(argv[j],"--no-crc-check")) {
      Modes.check_crc = 0;
    } else if (!strcmp(argv[j],"--phase-enhance")) {
      Modes.phase_enhance = 1;
    } else if (!strcmp(argv[j],"--raw")) {
      Modes.raw = 1;
    } else if (!strcmp(argv[j],"--net")) {
      Modes.net = 1;
    } else if (!strcmp(argv[j],"--modeac")) {
      Modes.mode_ac = 1;
    } else if (!strcmp(argv[j],"--net-beast")) {
      Modes.beast = 1;
    } else if (!strcmp(argv[j],"--net-only")) {
      Modes.net = 1;
      Modes.net_only = 1;
    } else if (!strcmp(argv[j],"--net-ro-size") && more) {
      Modes.net_output_raw_size = atoi(argv[++j]);
    } else if (!strcmp(argv[j],"--net-ro-rate") && more) {
      Modes.net_output_raw_rate = atoi(argv[++j]);
    } else if (!strcmp(argv[j],"--net-ro-port") && more) {
      if (Modes.beast) // Required for legacy backward compatibility
        {Modes.net_output_beast_port = atoi(argv[++j]);;}
      else
        {Modes.net_output_raw_port = atoi(argv[++j]);}
    } else if (!strcmp(argv[j],"--net-ri-port") && more) {
      Modes.net_input_raw_port = atoi(argv[++j]);
    } else if (!strcmp(argv[j],"--net-bo-port") && more) {
      Modes.net_output_beast_port = atoi(argv[++j]);
    } else if (!strcmp(argv[j],"--net-bi-port") && more) {
      Modes.net_input_beast_port = atoi(argv[++j]);
    } else if (!strcmp(argv[j],"--net-http-port") && more) {
      Modes.net_http_port = atoi(argv[++j]);
    } else if (!strcmp(argv[j],"--net-sbs-port") && more) {
      Modes.net_output_sbs_port = atoi(argv[++j]);
    } else if (!strcmp(argv[j],"--onlyaddr")) {
      Modes.onlyaddr = 1;
    } else if (!strcmp(argv[j],"--metric")) {
      Modes.metric = 1;
    } else if (!strcmp(argv[j],"--aggressive")) {
      Modes.nfix_crc = MODES_MAX_BITERRORS;
    } else if (!strcmp(argv[j],"--interactive")) {
      Modes.interactive = 1;
    } else if (!strcmp(argv[j],"--interactive-rows") && more) {
      Modes.interactive_rows = atoi(argv[++j]);
    } else if (!strcmp(argv[j],"--interactive-ttl") && more) {
      Modes.interactive_ttl = atoi(argv[++j]);
    } else if (!strcmp(argv[j],"--lat") && more) {
      Modes.fUserLat = atof(argv[++j]);
    } else if (!strcmp(argv[j],"--lon") && more) {
      Modes.fUserLon = atof(argv[++j]);
    } else if (!strcmp(argv[j],"--debug") && more) {
      char *f = argv[++j];
      while(*f) {
        switch(*f) {
        case 'D': Modes.debug |= MODES_DEBUG_DEMOD; break;
        case 'd': Modes.debug |= MODES_DEBUG_DEMODERR; break;
        case 'C': Modes.debug |= MODES_DEBUG_GOODCRC; break;
        case 'c': Modes.debug |= MODES_DEBUG_BADCRC; break;
        case 'p': Modes.debug |= MODES_DEBUG_NOPREAMBLE; break;
        case 'n': Modes.debug |= MODES_DEBUG_NET; break;
        case 'j': Modes.debug |= MODES_DEBUG_JS; break;
        default:
          fprintf(stderr, "Unknown debugging flag: %c\n", *f);
          exit(1);
          break;
        }
        f++;
      }
    } else if (!strcmp(argv[j],"--stats")) {
      Modes.stats = 1;
    } else if (!strcmp(argv[j],"--snip") && more) {
      snipMode(atoi(argv[++j]));
      exit(0);
    } else if (!strcmp(argv[j],"--help")) {
      showHelp();
      exit(0);
    } else if (!strcmp(argv[j],"--ppm") && more) {
      Modes.ppm_error = atoi(argv[++j]);
    } else if (!strcmp(argv[j],"--quiet")) {
      Modes.quiet = 1;
    } else if (!strcmp(argv[j],"--mlat")) {
      Modes.mlat = 1;
    } else if (!strcmp(argv[j],"--interactive-rtl1090")) {
      Modes.interactive = 1;
      Modes.interactive_rtl1090 = 1;
    } else {
      fprintf(stderr,
              "Unknown or not enough arguments for option '%s'.\n\n",
              argv[j]);
      showHelp();
      exit(1);
    }
  }

  // Initialization
  modesInit();
  if (Modes.debug & MODES_DEBUG_BADCRC) {
    testAndTimeBitCorrection();
  }
  if (Modes.net_only) {
    fprintf(stderr,"Net-only mode, no RTL device or file open.\n");
  } else if (Modes.filename == NULL) {
    modesInitRTLSDR();
  } else {
    if (Modes.filename[0] == '-' && Modes.filename[1] == '\0') {
      Modes.fd = STDIN_FILENO;
    } else if ((Modes.fd = open(Modes.filename,O_RDONLY)) == -1) {
      perror("Opening data file");
      exit(1);
    }
  }
  if (Modes.net) modesInitNet();

  /* If the user specifies --net-only, just run in order to serve network
   * clients without reading data from the RTL device. */
  while (Modes.net_only) {
    if (Modes.exit) exit(0); // If we exit net_only nothing further in main()
    backgroundTasks();
    usleep(100000);
  }

  /* Create the thread that will read the data from the device. */
  pthread_create(&Modes.reader_thread, NULL, readerThreadEntryPoint, NULL);

  pthread_mutex_lock(&Modes.data_mutex);
  while(1) {
    if (!Modes.data_ready) {
      pthread_cond_wait(&Modes.data_cond,&Modes.data_mutex);
      continue;
    }
    computeMagnitudeVector();
    Modes.stSystemTimeBlk = Modes.stSystemTimeRTL;

    /* Signal to the other thread that we processed the available data
     * and we want more (useful for --ifile). */
    Modes.data_ready = 0;
    pthread_cond_signal(&Modes.data_cond);

    /* Process data after releasing the lock, so that the capturing
     * thread can read data while we perform computationally expensive
     * stuff * at the same time. (This should only be useful with very
     * slow processors). */
    pthread_mutex_unlock(&Modes.data_mutex);
    detectModeS(Modes.magnitude, MODES_ASYNC_BUF_SAMPLES);
    Modes.timestampBlk += (MODES_ASYNC_BUF_SAMPLES*6);
    backgroundTasks();
    pthread_mutex_lock(&Modes.data_mutex);
    if (Modes.exit) break;
  }

  // If --stats were given, print statistics
  if (Modes.stats) {
    printf("\n\n");
    printf("%d ModeA/C detected\n",                           Modes.stat_ModeAC);
    printf("%d valid Mode-S preambles\n",                     Modes.stat_valid_preamble);
    printf("%d DF-?? fields corrected for length\n",          Modes.stat_DF_Len_Corrected);
    printf("%d DF-?? fields corrected for type\n",            Modes.stat_DF_Type_Corrected);
    printf("%d demodulated with 0 errors\n",                  Modes.stat_demodulated0);
    printf("%d demodulated with 1 error\n",                   Modes.stat_demodulated1);
    printf("%d demodulated with 2 errors\n",                  Modes.stat_demodulated2);
    printf("%d demodulated with > 2 errors\n",                Modes.stat_demodulated3);
    printf("%d with good crc\n",                              Modes.stat_goodcrc);
    printf("%d with bad crc\n",                               Modes.stat_badcrc);
    printf("%d errors corrected\n",                           Modes.stat_fixed);
    for (j = 0;  j < MODES_MAX_BITERRORS;  j++) {
      printf("   %d with %d bit %s\n", Modes.stat_bit_fix[j], j+1, (j==0)?"error":"errors");
    }
    if (Modes.phase_enhance) {
      printf("%d phase enhancement attempts\n",                 Modes.stat_out_of_phase);
      printf("%d phase enhanced demodulated with 0 errors\n",   Modes.stat_ph_demodulated0);
      printf("%d phase enhanced demodulated with 1 error\n",    Modes.stat_ph_demodulated1);
      printf("%d phase enhanced demodulated with 2 errors\n",   Modes.stat_ph_demodulated2);
      printf("%d phase enhanced demodulated with > 2 errors\n", Modes.stat_ph_demodulated3);
      printf("%d phase enhanced with good crc\n",               Modes.stat_ph_goodcrc);
      printf("%d phase enhanced with bad crc\n",                Modes.stat_ph_badcrc);
      printf("%d phase enhanced errors corrected\n",            Modes.stat_ph_fixed);
      for (j = 0;  j < MODES_MAX_BITERRORS;  j++) {
        printf("   %d with %d bit %s\n", Modes.stat_ph_bit_fix[j], j+1, (j==0)?"error":"errors");
      }
    }
    printf("%d total usable messages\n",                      Modes.stat_goodcrc + Modes.stat_ph_goodcrc + Modes.stat_fixed + Modes.stat_ph_fixed);
  }

  if (Modes.filename == NULL) {
    rtlsdr_cancel_async(Modes.dev);  // Cancel rtlsdr_read_async will cause data input thread to terminate cleanly
    rtlsdr_close(Modes.dev);
  }
  pthread_cond_destroy(&Modes.data_cond);     // Thread cleanup
  pthread_mutex_destroy(&Modes.data_mutex);
  pthread_join(Modes.reader_thread,NULL);     // Wait on reader thread exit
  pthread_exit(0);
}

#include "ten90.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>


typedef struct {
  int lat;
  int lon;
} Cpr;

typedef struct {
  double lat;
  double lon;
} Pos;


void AssertPos(Pos a, Pos b) {
  assert(a.lat == b.lat);
  assert(a.lon == b.lon);
}

void AssertBytesEqual(unsigned char *expected, unsigned char *actual, int len) {
  int i;
  for (i = 0; i < len; i++) {
    if (expected[i] != actual[i]) {
      fprintf(
          stderr,
          "Byte sequence differs in position %d: expected 0x%x != "
          "actual 0x%x\n",
          i, expected[i], actual[i]);
      assert(expected[i] == actual[i]);
    }
  }
}


void TestBasicDecoding() {
  Ten90Context context;
  Ten90Frame f;
  int num_bytes;

  Ten90ContextInit(
      &context, kTen90DefaultIcaoCacheSize, kTen90DefaultIcaoCacheTtl);

  {
    unsigned char msg[] = {0x59, 0xa9, 0x37, 0x80, 0xf4, 0xa6, 0xda};
    num_bytes = Ten90DecodeFrame(msg, &context, &f);
    assert(num_bytes == 7);
    Ten90DisplayFrame(&f);
    assert(f.addr == 0xa93780);
    assert(f.msg_type == 11);
    assert(f.crc == 0);
    assert(f.crc_ok);
  }
  {
    unsigned char msg[] = {0x02, 0xe9, 0x96, 0x19, 0xfa, 0xcd, 0xae};
    num_bytes = Ten90DecodeFrame(msg, &context, &f);
    assert(num_bytes == 7);
    Ten90DisplayFrame(&f);
    assert(f.addr == 0x29400e);
    assert(f.msg_type == 0);
    assert(f.crc == 0x29400e);
    assert(!f.crc_ok);
  }

  Ten90ContextDestroy(&context);
}


void TestFixingBitErrors() {
  Ten90Context context;
  Ten90Frame f;
  int error;
  int num_bytes;
  Ten90ContextInit(
      &context, kTen90DefaultIcaoCacheSize, kTen90DefaultIcaoCacheTtl);

  {
    unsigned char wrong_bytes[] = {0x8d, 0x71, 0xbf, 0x5d, 0xd8, 0x29, 0xa2,
                                   0xad, 0xbd, 0xc6, 0xbd, 0xe3, 0xbe, 0x1d};
    unsigned char right_bytes[] = {0x8d, 0x71, 0xbf, 0x55, 0x58, 0x29, 0xa2,
                                   0xad, 0xbd, 0xc6, 0xbd, 0xe3, 0xbe, 0x1d};
    context.max_crc_bit_corrections = 2;

    // First confirm that if we see a frame with errors and the ICAO
    // isn't in the cache/whitelist, then number_corrected_bits == 0
    num_bytes = Ten90DecodeFrame(wrong_bytes, &context, &f);
    assert(num_bytes == 14);
    Ten90DisplayFrame(&f);
    assert(f.addr == 0x71bf55);
    assert(f.msg_type == 17);
    assert(f.crc == 0x365413);
    assert(!f.crc_ok);
    assert(f.number_corrected_bits == 0);
    AssertBytesEqual(right_bytes, f.msg, sizeof(right_bytes));

    // Decode a frame with no errors to put the ICAO in the
    // whitelist.
    num_bytes = Ten90DecodeFrame(right_bytes, &context, &f);
    assert(num_bytes == 14);
    assert(f.number_corrected_bits == 0);
    assert(f.crc_ok);
    AssertBytesEqual(right_bytes, f.msg, sizeof(right_bytes));

    // Now when we see a frame for that ICAO that has errors the
    // number_corrected_bits will be reported.
    num_bytes = Ten90DecodeFrame(wrong_bytes, &context, &f);
    assert(num_bytes == 14);
    Ten90DisplayFrame(&f);
    assert(f.addr == 0x71bf55);
    assert(f.msg_type == 17);
    assert(f.crc == 0x365413);
    assert(!f.crc_ok);
    assert(f.number_corrected_bits == 2);
    AssertBytesEqual(right_bytes, f.msg, sizeof(right_bytes));

    context.max_crc_bit_corrections = 0;
    num_bytes = Ten90DecodeFrame(wrong_bytes, &context, &f);
    assert(num_bytes == 14);
  }
  {
    context.max_crc_bit_corrections = 2;
    unsigned char msg[] = {0x02, 0xe9, 0x96, 0x19, 0xfa, 0xcd, 0xae};
    Ten90DecodeFrame(msg, &context, &f);
    Ten90DisplayFrame(&f);
    assert(f.addr == 0x29400e);
    assert(f.msg_type == 0);
    assert(f.crc == 0x29400e);
    assert(!f.crc_ok);
  }

  Ten90ContextDestroy(&context);
}


void TestCprDecoding() {
  {
    Cpr even = {109227, 0};
    Cpr odd = {9124, 65537};
    Pos expected = {-84.99898619570973, -179.99908447265625};
    Pos actual;
    int error = Ten90DecodeCpr(
        even.lat, even.lon, odd.lat, odd.lon,
        0.0, 0.0,
        1, 0,
        &actual.lat, &actual.lon);
    assert(!error);
    AssertPos(expected, actual);
  }
  {
    Cpr even = {43693, 16966};
    Cpr odd = {27694, 114128};
    Pos expected = {44.00108208090572, 93.17766462053571};
    Pos actual;
    int error = Ten90DecodeCpr(
        even.lat, even.lon, odd.lat, odd.lon,
        0.0, 0.0,
        1, 0,
        &actual.lat, &actual.lon);
    assert(!error);
    AssertPos(expected, actual);
  }
  {
    // Crosses transition boundary.
    Cpr even = {108148, 123838};
    Cpr odd = {130397, 123848};
    Pos actual;
    int error = Ten90DecodeCpr(
        even.lat, even.lon, odd.lat, odd.lon,
        0.0, 0.0,
        1, 0,
        &actual.lat, &actual.lon);
    assert(error);
  }
}


int main(void) {
  TestBasicDecoding();
  TestFixingBitErrors();
  TestCprDecoding();
  return 0;
}

#include "ten90.h"

#include "gtest/gtest.h"

#include <assert.h>
#include <math.h>


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


TEST(CprTest, AbsoluteDecoding) {
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
    EXPECT_FALSE(error);
    EXPECT_EQ(expected.lat, actual.lat);
    EXPECT_EQ(expected.lon, actual.lon);
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
    EXPECT_FALSE(error);
    EXPECT_EQ(expected.lat, actual.lat);
    EXPECT_EQ(expected.lon, actual.lon);
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
     EXPECT_TRUE(error);
  }
}


// int main(void) {
//   Ten90Context context;
//   Ten90ContextInit(
//       &context, kTen90DefaultIcaoCacheSize, kTen90DefaultIcaoCacheTtl);
//   /*
//     CRC:          000000 (ok)
//     DF 11:        All Call Reply.
//     Capability:   1 (Level 2 (DF0,4,5,11))
//     ICAO Address: a93780
//     IID:          II-00
//   */
//   Ten90Frame mm;
//   int error;

//   {
//     unsigned char msg[] = {0x59, 0xa9, 0x37, 0x80, 0xf4, 0xa6, 0xda};
//     Ten90DecodeFrame(msg, &context, &mm);
//     Ten90DisplayFrame(&mm);
//     assert(mm.addr == 0xa93780);
//     assert(mm.msg_type == 11);
//   }

//   {
//     unsigned char msg[] = {0x02, 0xe9, 0x96, 0x19, 0xfa, 0xcd, 0xae};
//     Ten90DecodeFrame(msg, &context, &mm);
//     Ten90DisplayFrame(&mm);
//     assert(mm.addr == 0x29400e);
//     assert(mm.msg_type == 0);
//   }

//   TestCprDecoding();

//   Ten90ContextDestroy(&context);
//   return 0;
// }

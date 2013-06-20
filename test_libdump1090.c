#include "dump1090.h"

#include <assert.h>


int main(void) {
  struct ten90_context context;
  ten90_init_context(&context);
  /*
    CRC:          000000 (ok)
    DF 11:        All Call Reply.
    Capability:   1 (Level 2 (DF0,4,5,11))
    ICAO Address: a93780
    IID:          II-00
  */
  char *s = "*59a93780f4a6da;";
  struct modesMessage mm;
  ten90_decode_hex_message(&mm, s, &context);
  assert(mm.addr == 0xa93780);
  assert(mm.msgtype == 11);
  return 0;
}

#include "ten90.h"

#include <assert.h>


int main(void) {
  ten90_context context;
  ten90_context_init(&context);
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
  ten90_context_destroy(&context);
  return 0;
}

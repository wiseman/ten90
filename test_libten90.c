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
  ten90_mode_s_message mm;
  int error;
  char *s;

  s = "*59a93780f4a6da;";
  error = ten90_decode_hex_message(&mm, s, &context);
  assert(!error);
  ten90_display_mode_s_message(&mm);
  assert(mm.addr == 0xa93780);
  assert(mm.msgtype == 11);

  s = "*02E99619FACDAE;";
  error = ten90_decode_hex_message(&mm, s, &context);
  assert(!error);
  ten90_display_mode_s_message(&mm);
  assert(mm.addr == 0x29400e);
  assert(mm.msgtype == 0);

  s = "@01020304050691929394959697;";
  error = ten90_decode_hex_message(&mm, s, &context);
  assert(!error);
  ten90_display_mode_s_message(&mm);

  ten90_context_destroy(&context);
  return 0;
}

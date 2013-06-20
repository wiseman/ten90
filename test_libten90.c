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

  {
    unsigned char msg[] = {0x59, 0xa9, 0x37, 0x80, 0xf4, 0xa6, 0xda};
    ten90_decode_mode_s_message(&mm, msg, &context);
    ten90_display_mode_s_message(&mm);
    assert(mm.addr == 0xa93780);
    assert(mm.msgtype == 11);
  }

  {
    unsigned char msg[] = {0x02, 0xe9, 0x96, 0x19, 0xfa, 0xcd, 0xae};
    ten90_decode_mode_s_message(&mm, msg, &context);
    ten90_display_mode_s_message(&mm);
    assert(mm.addr == 0x29400e);
    assert(mm.msgtype == 0);
  }

  ten90_context_destroy(&context);
  return 0;
}

#include "ten90.h"

#include <assert.h>


int main(void) {
  Ten90Context context;
  Ten90ContextInit(&context);
  /*
    CRC:          000000 (ok)
    DF 11:        All Call Reply.
    Capability:   1 (Level 2 (DF0,4,5,11))
    ICAO Address: a93780
    IID:          II-00
  */
  Ten90Frame mm;
  int error;

  {
    unsigned char msg[] = {0x59, 0xa9, 0x37, 0x80, 0xf4, 0xa6, 0xda};
    Ten90DecodeFrame(msg, &context, &mm);
    Ten90DisplayFrame(&mm);
    assert(mm.addr == 0xa93780);
    assert(mm.msg_type == 11);
  }

  {
    unsigned char msg[] = {0x02, 0xe9, 0x96, 0x19, 0xfa, 0xcd, 0xae};
    Ten90DecodeFrame(msg, &context, &mm);
    Ten90DisplayFrame(&mm);
    assert(mm.addr == 0x29400e);
    assert(mm.msg_type == 0);
  }

  Ten90ContextDestroy(&context);
  return 0;
}

/**
  ******************************************************************************
  * @file    protocol.h
  * @brief   ASCII command protocol over USB CDC (virtual COM port).
  *
  *          Frame format:  <CMD><payload>\n
  *            S<speed>     set motor A speed, per mille [-1000,1000]; B = k*A
  *                           e.g. "S+500\n"  or  "S-350\n"
  *            K<ratio>     set coupling ratio in percent [0,100]
  *                           e.g. "K50\n" -> kB = 0.50
  *            R            request status -> reply "A=%d B=%d K=%d.%02d\n"
  *            X            immediate stop (brake both)
  *            ?            help string
  *
  *          Parsing is line-buffered: bytes accumulate until '\n', then one
  *          complete frame is decoded. Partial / malformed frames are ignored.
  ******************************************************************************
  */
#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define PROTO_LINE_MAX  64

void PROTO_Init(void);
/* Feed one received byte from USB CDC. Returns 1 when a full frame was parsed. */
int  PROTO_PushByte(uint8_t b);
/* Emit a status reply over CDC (uses CDC_Transmit_FS). */
void PROTO_ReplyStatus(void);

#ifdef __cplusplus
}
#endif
#endif /* __PROTOCOL_H */

/**
  ******************************************************************************
  * @file    protocol.h
  * @brief   ASCII command protocol over USB CDC (virtual COM port).
  *
  *          Frame format: <CMD><payload>\n
  *            S<speed>  set motor A, per mille [-1000,1000]
  *            K<percent> set k in percent [1,100]
  *            T<0|1>    telemetry on/off
  *            L<0|1>    encoder speed closed loop on/off
  *            R         basic status   "A=.. B=.. K=.."
  *            V         extended status with commands, measurements and state
  *            E         encoder counts and speeds
  *            X         brake both
  *            ?         help
  *
  *          Line-buffered: bytes accumulate until '\n', then one frame decodes.
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
int  PROTO_PushByte(uint8_t b);
void PROTO_ReplyStatus(void);
void PROTO_ReplyExtended(void);
/* Call every 50 ms. Emits a telemetry line when telemetry is enabled. */
void PROTO_TelemetryTick(void);

#ifdef __cplusplus
}
#endif
#endif /* __PROTOCOL_H */

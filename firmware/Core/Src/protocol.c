/**
  ******************************************************************************
  * @file    protocol.c
  * @brief   Line-buffered ASCII command parser for the dual-motor board.
  ******************************************************************************
  */
#include "protocol.h"
#include "motor.h"
#include "usbd_cdc_if.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

static char   s_line[PROTO_LINE_MAX];
static uint8_t s_len;

void PROTO_Init(void)
{
    s_len = 0;
    memset(s_line, 0, sizeof(s_line));
}

/* Minimal signed integer parser: optional +/- then digits. Returns 0 on fail. */
static int parse_int(const char *p, int *out)
{
    int sign = 1;
    int val = 0;
    int any = 0;

    if (*p == '+' || *p == '-') {
        if (*p == '-') sign = -1;
        p++;
    }
    while (*p >= '0' && *p <= '9') {
        val = val * 10 + (*p - '0');
        any = 1;
        p++;
    }
    if (!any) return 0;
    *out = sign * val;
    return 1;
}

static void cdc_send(const char *s)
{
    CDC_Transmit_FS((uint8_t *)s, (uint16_t)strlen(s));
}

/* Decode one complete line (without trailing '\n'). */
static void decode(const char *ln)
{
    int value;

    if (ln[0] == '\0') return;

    switch (ln[0]) {
    case 'S': {                       /* S<speed> : set A, B solved from sin(v1)=k*cos(v2) */
        if (parse_int(ln + 1, &value)) {
            Motor_SetSpeedA(value);
            cdc_send("OK\r\n");
        } else {
            cdc_send("ERR\r\n");
        }
        break;
    }
    case 'K': {                       /* K<percent> : set k = percent/100, 1..100 */
        if (parse_int(ln + 1, &value)) {
            Motor_SetRatio(value * 10);   /* percent -> permille */
            cdc_send("OK\r\n");
        } else {
            cdc_send("ERR\r\n");
        }
        break;
    }
    case 'R':                          /* R : status */
        PROTO_ReplyStatus();
        break;
    case 'X':                          /* X : immediate stop */
        Motor_Stop();
        cdc_send("STOPPED\r\n");
        break;
    case '?':                          /* ? : help */
        cdc_send("CMD: S+500 K100 R X ?\r\n");
        break;
    default:
        cdc_send("ERR\r\n");
        break;
    }
}

int PROTO_PushByte(uint8_t b)
{
    /* Accept both LF and CR as frame end; ignore stray CR in CRLF. */
    if (b == '\n' || b == '\r') {
        if (s_len == 0) return 0;         /* ignore empty / trailing CR */
        s_line[s_len] = '\0';
        decode(s_line);
        s_len = 0;
        memset(s_line, 0, sizeof(s_line));
        return 1;
    }

    if (s_len < PROTO_LINE_MAX - 1u) {
        s_line[s_len++] = (char)b;
    } else {
        s_len = 0;                        /* overflow: drop the line */
    }
    return 0;
}

void PROTO_ReplyStatus(void)
{
    char buf[48];
    int a = Motor_GetSpeedA();
    int b = Motor_GetSpeedB();
    int k = Motor_GetRatio();             /* permille */
    /* "A=%+d B=%+d K=%d.%02d\r\n" */
    int n = snprintf(buf, sizeof(buf), "A=%+d B=%+d K=%d.%02d\r\n",
                     a, b, k / 1000, (k % 1000) / 10);
    if (n > 0) CDC_Transmit_FS((uint8_t *)buf, (uint16_t)n);
}

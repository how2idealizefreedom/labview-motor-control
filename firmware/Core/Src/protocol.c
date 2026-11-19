/**
  ******************************************************************************
  * @file    protocol.c
  * @brief   Line-buffered ASCII command parser for the dual-motor board.
  ******************************************************************************
  */
#include "protocol.h"
#include "motor.h"
#include "encoder.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>

static char   s_line[PROTO_LINE_MAX];
static uint8_t s_len;
static uint8_t s_drop = 0;      /* drop rest of an oversized line */
static int    s_telemetry = 0;

void PROTO_Init(void)
{
    s_len = 0;
    s_drop = 0;
    s_telemetry = 0;
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

static int parse_flag(const char *p, int *out)
{
    int v;
    if (!parse_int(p, &v)) return 0;
    if (v != 0 && v != 1) return 0;
    *out = v;
    return 1;
}

static void cdc_send(const char *s)
{
    CDC_Transmit_FS((uint8_t *)s, (uint16_t)strlen(s));
}

static char state_char(int st)
{
    if (st == MOTOR_STATE_RUN) return 'R';
    if (st == MOTOR_STATE_TIMEOUT) return 'T';
    return 'I';
}

static void decode(const char *ln)
{
    int value;

    if (ln[0] == '\0') return;
    Motor_FeedWatchdog();

    switch (ln[0]) {
    case 'S':
        if (parse_int(ln + 1, &value)) { Motor_SetSpeedA(value); cdc_send("OK\r\n"); }
        else cdc_send("ERR\r\n");
        break;
    case 'K':
        if (parse_int(ln + 1, &value)) { Motor_SetRatio(value * 10); cdc_send("OK\r\n"); }
        else cdc_send("ERR\r\n");
        break;
    case 'T':
        if (parse_flag(ln + 1, &value)) { s_telemetry = value; cdc_send("OK\r\n"); }
        else cdc_send("ERR\r\n");
        break;
    case 'L':
        if (parse_flag(ln + 1, &value)) { Motor_SetClosedLoop(value); cdc_send("OK\r\n"); }
        else cdc_send("ERR\r\n");
        break;
    case 'R':
        PROTO_ReplyStatus();
        break;
    case 'V':
        PROTO_ReplyExtended();
        break;
    case 'E': {
        char buf[64];
        int n = snprintf(buf, sizeof(buf), "CA=%d CB=%d RA=%+d RB=%+d\r\n",
                         Encoder_CountA(), Encoder_CountB(),
                         Motor_GetMeasA(), Motor_GetMeasB());
        if (n > 0) cdc_send(buf);
        break;
    }
    case 'X':
        Motor_Stop();
        cdc_send("STOPPED\r\n");
        break;
    case '?':
        cdc_send("CMD: S+500 K100 T1 L1 R V E X ?\r\n");
        break;
    default:
        cdc_send("ERR\r\n");
        break;
    }
}

int PROTO_PushByte(uint8_t b)
{
    if (b == '\n' || b == '\r') {
        if (s_drop) { s_drop = 0; s_len = 0; return 0; }
        if (s_len == 0) return 0;
        s_line[s_len] = '\0';
        decode(s_line);
        s_len = 0;
        memset(s_line, 0, sizeof(s_line));
        return 1;
    }
    if (s_drop) return 0;
    if (s_len < PROTO_LINE_MAX - 1u) s_line[s_len++] = (char)b;
    else { s_len = 0; s_drop = 1; }
    return 0;
}

void PROTO_ReplyStatus(void)
{
    char buf[48];
    int k = Motor_GetRatio();
    int n = snprintf(buf, sizeof(buf), "A=%+d B=%+d K=%d.%02d\r\n",
                     Motor_GetSpeedA(), Motor_GetSpeedB(),
                     k / 1000, (k % 1000) / 10);
    if (n > 0) cdc_send(buf);
}

void PROTO_ReplyExtended(void)
{
    char buf[128];
    int k = Motor_GetRatio();
    int n = snprintf(buf, sizeof(buf),
        "A=%+d B=%+d CA=%+d CB=%+d MA=%+d MB=%+d K=%d.%02d ST=%c L=%d\r\n",
        Motor_GetSpeedA(), Motor_GetSpeedB(),
        Motor_GetCmdA(), Motor_GetCmdB(),
        Motor_GetMeasA(), Motor_GetMeasB(),
        k / 1000, (k % 1000) / 10,
        state_char(Motor_GetState()), Motor_GetClosedLoop());
    if (n > 0) cdc_send(buf);
}

void PROTO_TelemetryTick(void)
{
    char buf[136];
    int k;

    if (!s_telemetry) return;
    k = Motor_GetRatio();
    snprintf(buf, sizeof(buf),
        ">A=%+d B=%+d CA=%+d CB=%+d MA=%+d MB=%+d K=%d.%02d ST=%c L=%d\r\n",
        Motor_GetSpeedA(), Motor_GetSpeedB(),
        Motor_GetCmdA(), Motor_GetCmdB(),
        Motor_GetMeasA(), Motor_GetMeasB(),
        k / 1000, (k % 1000) / 10,
        state_char(Motor_GetState()), Motor_GetClosedLoop());
    cdc_send(buf);
}

#ifndef _BSP_LOG_H
#define _BSP_LOG_H

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include <stdio.h>
#include <stdarg.h>

#define BUFFER_INDEX 0
#define LOG_PROTO(type, color, format, ...) \
    SEGGER_RTT_printf(BUFFER_INDEX, "  %s%s" format "\r\n%s", color, type, ##__VA_ARGS__, RTT_CTRL_RESET)
#define LOG_CLEAR() SEGGER_RTT_WriteString(0, "  " RTT_CTRL_CLEAR)
#define LOG(format, ...) LOG_PROTO("", "", format, ##__VA_ARGS__)

#if DISABLE_LOG_SYSTEM
#define LOGINFO(format, ...) ((void)0)
#define LOGWARNING(format, ...) ((void)0)
#define LOGERROR(format, ...) ((void)0)
#else
#define LOGINFO(format, ...) LOG_PROTO("I:", RTT_CTRL_TEXT_BRIGHT_GREEN, format, ##__VA_ARGS__)
#define LOGWARNING(format, ...) LOG_PROTO("W:", RTT_CTRL_TEXT_BRIGHT_YELLOW, format, ##__VA_ARGS__)
#define LOGERROR(format, ...) LOG_PROTO("E:", RTT_CTRL_TEXT_BRIGHT_RED, format, ##__VA_ARGS__)
#endif

void BSPLogInit(void);
int PrintLog(const char *fmt, ...);
void Float2Str(char *str, float va);

#endif

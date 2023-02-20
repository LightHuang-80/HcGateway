/*
 * log.c
 *
 *  Created on: 2021年4月4日
 *      Author: Administrator
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "main.h"
#include "log.h"

uint8_t g_LogLevel;
uint8_t g_LogOn;
uint8_t g_LogOutput;

void LOG_Init(uint8_t level, uint8_t on, uint8_t output)
{
	g_LogLevel = level;
	g_LogOn = on;
	g_LogOutput = output;
}

void LOG_OnOff(uint8_t on)
{
	g_LogOn = on;
}

void LOG_SetLevel(uint8_t level)
{
	if (level > LOG_DbgLevel) return;
	g_LogLevel = level;
}


void LOG_Print(uint8_t level, const char* fmt, ...)
{
	if (!g_LogOn) return;
	if (level > g_LogLevel) return;

	va_list v1;
	va_start(v1, fmt);

	char buffer[64];
	memset(buffer, 0, 64);

	vsnprintf(buffer, 64, fmt, v1);

	if (g_LogOutput == LOG_MonOut){
		// print to rdi monito
		printf(buffer);
	}else if (g_LogOutput == LOG_USARTOut) {
		// print to usart
		// JNT_dbgOutput(buffer, strlen(buffer));
	}
	va_end(v1);
}


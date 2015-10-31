/*
 * Utilities.cpp
 *
 *  Created on: 25 oct. 2015
 *      Author: Christophe
 */
#include <stdio.h>
#include <stdarg.h>

void format(char *fmt,char *buf, ...)
{

	va_list args;
	va_start(args, fmt);
	vsnprintf(buf, 128, fmt, args);
	va_end(args);
}




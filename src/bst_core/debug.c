#include "debug.h"
#include <stdio.h>
#include <stdlib.h>

#if !defined ARCH_stm32f4 && !defined ARCH_stm32f1
#include <unistd.h>
#endif

#ifndef VERBOSE
/* Nothing. pmesg has been "defined away" in debug.h already. */

#else

int8_t verbose = VERBOSE_NONE; /* the higher, the more messages... */

#if !defined ARCH_stm32f4 && !defined ARCH_stm32f1
#define FG_RESET   "\033[0m"
#define FG_RED     "\033[31m"
#define FG_GREEN   "\033[32m"
#define FG_YELLOW  "\033[33m"
#define FG_BLUE    "\033[34m"
#define FG_MAGENTA "\033[35m"
#define FG_CYAN    "\033[36m"
#define FG_GRAY    "\033[37m"
#define FG_DEFAULT "\033[39m"

#else
#define MAX_OUT_STR 256
char str_val[MAX_OUT_STR];
#endif

extern "C" {
	float getElapsedTime(); // defined elsewhere
}

void pmesg(VerboseLevel_t level, const char* format, ...) {
	/* Empty body, so a good compiler will optimise calls to pmesg away */

	if( (verbose < 0 && level != -verbose) || level > abs(verbose) )
		return;

#if !defined ARCH_stm32f4 && !defined ARCH_stm32f1
	if( isatty(fileno(stdout)) ) {
		switch(level) {
			case VERBOSE_ERROR:
				fprintf(stdout, FG_RED " [ERROR] " FG_RESET);
				break;

			case VERBOSE_WARN:
				fprintf(stdout, FG_YELLOW "  [WARN] " FG_RESET);
				break;

			case VERBOSE_STATUS:
				fprintf(stdout, FG_GREEN "[STATUS] " FG_RESET);
				break;

			case VERBOSE_INFO:
				fprintf(stdout, FG_CYAN "  [INFO] " FG_RESET);
				break;

			case VERBOSE_FP:
				fprintf(stdout, FG_GRAY " %.3f [FP] " FG_RESET, getElapsedTime());
				break;

			case VERBOSE_PARAM:
				fprintf(stdout, FG_GRAY " [PARAM] " FG_RESET);
				break;

			case VERBOSE_ALLOC:
				fprintf(stdout, FG_BLUE " [ALLOC] " FG_RESET);
				break;

			case VERBOSE_SENSORS:
				fprintf(stdout, FG_BLUE "[SENSORS]" FG_RESET);
				break;

			case VERBOSE_PAYLOAD:
				fprintf(stdout, FG_BLUE "[PAYLOAD]" FG_RESET);
				break;

			case VERBOSE_PACKETS:
				fprintf(stdout, FG_MAGENTA "  [PCKT] " FG_RESET);
				break;

			case VERBOSE_EKF:
				fprintf(stdout, FG_MAGENTA "   [EKF] " FG_RESET);
				break;

			case VERBOSE_ALL_PACKETS:
				fprintf(stdout, FG_MAGENTA " [PCKTS] " FG_RESET);
				break;

			case VERBOSE_CAN:
				fprintf(stdout, FG_BLUE " [CAN] " FG_RESET);
				break;

			case VERBOSE_ALL:
				fprintf(stdout, FG_GRAY "   [ALL] " FG_RESET);
				break;

			default:
				break;
		}
	}

	va_list args;
	va_start(args, format);
	vfprintf(stdout, format, args);
	fflush(stdout);
	va_end(args);
#else

	va_list args;
	va_start(args, format);
	vsnprintf(str_val, MAX_OUT_STR, format, args);
	va_end(args);

	consoleout(str_val);
#endif

}
#endif /* VERBOSE */

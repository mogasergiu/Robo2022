#include <stdarg.h>
#include <stdio.h>

#ifndef DEBUG_H
#define DEBUG_H

#define DEBUG 1

#if DEBUG
static inline void pr_debug(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}
#else
static inline void pr_debug(const char *fmt, ...) {
}
#endif

#endif  /* DEBUG_H */
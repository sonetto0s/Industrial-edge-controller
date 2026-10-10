#define _GNU_SOURCE
#include "timeutil.h"
#include <time.h>
#include <errno.h>
#include <stdint.h>

int64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

void sleep_ms(int ms)
{
    struct timespec req;
    req.tv_sec = ms / 1000;
    req.tv_nsec = (long)(ms % 1000) * 1000000L;
    while (req.tv_sec > 0 || req.tv_nsec > 0)
    {
        if (nanosleep(&req, &req) == 0)
            break;
        if (errno != EINTR)
            break;
    }
}

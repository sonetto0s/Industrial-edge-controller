#ifndef CONFIG_H
#define CONFIG_H

#include "serial.h"

typedef struct
{
    SerialConfig serial;
    int timeout_ms;
    int retries;
    int gap_ms;
}BusConfig;

int busconfig_load(BusConfig *cfg, const char *path);

#endif

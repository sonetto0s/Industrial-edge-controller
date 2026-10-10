#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>

typedef enum
{
    SERIAL_OK = 0,
    SERIAL_ERR_PARAM = -1,
    SERIAL_ERR_OPEN = -2,
    SERIAL_ERR_STATE = -3,
    SERIAL_ERR_CONFIG = -4,
    SERIAL_ERR_DATA = -5,
    SERIAL_ERR_GPIO = -6,
    SERIAL_ERR_IO = -7,
} SerialResult;

typedef enum
{
    RS485_AUTO = 0,
    RS485_GPIO = 1,
} RS485Mode;

typedef struct
{
    char device[64];
    int baud_rate;
    int databits;
    char parity;
    int stopbits;
    int rs485_mode;
    int dir_gpio;

} SerialConfig;

typedef enum
{
    FLUSH_RX = 0,
    FLUSH_TX,
    FLUSH_BOTH
} FlushDir;

typedef struct SerialPort SerialPort;

SerialPort *serial_create(void);
void serial_destroy(SerialPort *sp);
SerialResult serial_set_config(SerialPort *sp, const SerialConfig *cfg);
SerialResult serial_open(SerialPort *sp);
SerialResult serial_shut(SerialPort *sp);
int serial_is_open(SerialPort *sp);
SerialResult serial_send(SerialPort *sp, const uint8_t *data, int len);
int serial_recv(SerialPort *sp, uint8_t *buf, int max, int timeout_ms);
SerialResult serial_flush(SerialPort *sp, FlushDir dir);
int serial_get_baud(SerialPort *sp);
const char *serial_result_str(SerialResult r);

#endif


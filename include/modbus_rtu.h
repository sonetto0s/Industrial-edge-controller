#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include "serial.h"
#include <stdint.h>

typedef enum
{
    MODBUS_OK = 0,
    MODBUS_ERR_PARAM = -1,
    MODBUS_ERR_TIMEOUT = -2,
    MODBUS_ERR_STATE = -3,
    MODBUS_ERR_ADDR = -4,
    MODBUS_ERR_PROTOCOL = -5,
    MODBUS_ERR_EXCEPTION = -6,
    MODBUS_ERR_CRC = -7,
    MODBUS_ERR_IO = -8,
} ModbusResult;

#define MODBUS_FUNC_READ_COILS 0x01
#define MODBUS_FUNC_READ_DISCRETE 0x02
#define MODBUS_FUNC_READ_HOLDING 0x03
#define MODBUS_FUNC_READ_INPUT 0x04
#define MODBUS_FUNC_WRITE_COIL 0x05
#define MODBUS_FUNC_WRITE_INPUT 0x06
#define MODBUS_FUNC_WRITE_COILS 0x0F
#define MODBUS_FUNC_WRITE_MULTIPLE 0x10


typedef struct ModbusMaster{
    SerialPort *sp;
    int slave_addr;
    int timeout_ms;
    int retries;
    int gap_ms;
    uint8_t last_exception;
    int debug;
} ModbusMaster;

ModbusMaster *modbus_create(SerialPort *sp, uint8_t slave_addr);
void modbus_destroy(ModbusMaster *m);
void modbus_set_slave(ModbusMaster *m, uint16_t slave_addr);
void modbus_set_timeout(ModbusMaster *m, int timeout_ms, int retries);

int modbus_read_coils(ModbusMaster *m, uint16_t start, uint16_t count, uint8_t *out);
int modbus_read_discrete_inputs(ModbusMaster *m, uint16_t start, uint16_t count, uint8_t *out);
int modbus_read_holding_regs(ModbusMaster *m, uint16_t start, uint16_t count, uint16_t *out);
int modbus_read_input_regs(ModbusMaster *m, uint16_t start, uint16_t count, uint16_t *out);

int modbus_write_coil(ModbusMaster *m, uint16_t addr, int value);
int modbus_write_holding_reg(ModbusMaster *m, uint16_t addr, uint16_t value);
int modbus_write_coils(ModbusMaster *m, uint16_t start, uint16_t count, const uint8_t *bits);
int modbus_write_holding_regs(ModbusMaster *m, uint16_t start, uint16_t count, const uint16_t *values);

uint16_t modbus_crc16(const uint8_t *data, int len);

const char *modbus_result_str(ModbusResult r);
const char *modbus_exception_str(uint8_t code);

#endif

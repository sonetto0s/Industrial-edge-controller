#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "modbus_rtu.h"
#include "crc16.h"
#include "timeutil.h"

#define MODBUS_BUFF_SIZE 260

#define MODBUS_MAX_READ_BITS    2000
#define MODBUS_MAX_READ_REGS    125
#define MODBUS_MAX_WRITE_BITS   1968
#define MODBUS_MAX_WRITE_REGS   123

uint16_t modbus_crc16(const uint8_t *data, int len)
{
    return crc16(data, len);
}

const char *modbus_result_str(ModbusResult r)
{
    switch (r)
    {
    case MODBUS_OK:
        return "成功";
    case MODBUS_ERR_PARAM:
        return "参数错误";
    case MODBUS_ERR_STATE:
        return "串口未打开";
    case MODBUS_ERR_TIMEOUT:
        return "从站无应答(超时)";
    case MODBUS_ERR_CRC:
        return "应答CRC校验失败";
    case MODBUS_ERR_ADDR:
        return "应答地址不匹配";
    case MODBUS_ERR_EXCEPTION:
        return "从站返回异常";
    case MODBUS_ERR_PROTOCOL:
        return "应答格式错误";
    case MODBUS_ERR_IO:
        return "串口IO错误";
    default:
        return "未知错误";
    }
}
const char *modbus_exception_str(uint8_t code)
{
    switch (code)
    {
    case 0x01:
        return "非法功能码(从站不支持)";
    case 0x02:
        return "非法数据地址(地址不存在)";
    case 0x03:
        return "非法数据值(数量超限等)";
    case 0x04:
        return "从站设备故障";
    case 0x05:
        return "确认(已收到,还需时间)";
    case 0x06:
        return "从站忙";
    case 0x08:
        return "存储奇偶校验错误";
    case 0x0A:
        return "网关路径不可用";
    case 0x0B:
        return "网关目标无响应";
    default:
        return "未知异常码";
    }
}

ModbusMaster *modbus_create(SerialPort *sp, uint8_t slave_addr)
{
    if (sp == NULL)
        return NULL;

    ModbusMaster *m = (ModbusMaster *)calloc(1, sizeof(ModbusMaster));
    if (m == NULL)
        return NULL;

    m->slave_addr = slave_addr;
    m->timeout_ms = 500;
    m->retries = 1;
    m->debug = 0;
    m->gap_ms = 10;
    m->sp = sp;
    return m;
}

void modbus_destroy(ModbusMaster *m)
{
    free(m);
}

void modbus_set_slave(ModbusMaster *m, uint16_t slave_addr)
{
    if (m)
        m->slave_addr = slave_addr;
}

void modbus_set_timeout(ModbusMaster *m, int timeout_ms, int retries)
{
    if (m)
    {
        if (retries >= 0)
            m->retries = retries;
        if (timeout_ms > 0)
            m->timeout_ms = timeout_ms;
    }
}

static void hex_dump(const char *tag, const uint8_t *d, int len, int debug)
{
    if (!debug)
        return;
    char buff[1024];
    int off = snprintf(buff, sizeof(buff), "[%s]", tag);
    for (int i = 0; i < len && off < (int)sizeof(buff) - 4; i++)
    {
        off += snprintf(buff + off, sizeof(buff) - off, " %02X", d[i]);
    }
    printf("%s (%d byte)\n", buff, len);
}

static int modbus_expected_len(const uint8_t *data, int have)
{
    if (have < 2)
        return -1;
    if (data[1] & 0x80)
        return 5;
    switch (data[1])
    {
        case 0x01:
        case 0x02:
        case 0x03:
        case 0x04:
            if (have < 3)
                return -1;
            return 5 + data[2];

        case 0x05:
        case 0x06:
        case 0x0F:
        case 0x10:
            return 8;
        default :
            return -1;
    }
}

static int baud_tran_time(int nbytes, int baud)
{
    if (baud <= 0)
        baud = 115200;
    return nbytes * 11 * 1000 / baud + 20;
}

static int modbus_transaction(ModbusMaster *m, const uint8_t *req, int req_len, uint8_t *resq, int *resq_len)
{
    if (m == NULL || req == NULL || resq == NULL || resq_len == NULL)
        return MODBUS_ERR_PARAM;

    if (!serial_is_open(m->sp))
        return MODBUS_ERR_STATE;

    if (req_len < 4)
        return MODBUS_ERR_PARAM;

    serial_flush(m->sp, FLUSH_RX);
    if (serial_send(m->sp, req, req_len) != SERIAL_OK)
        return MODBUS_ERR_IO;

    hex_dump("TX", req, req_len, m->debug);
    if (m->slave_addr == 0)
    {
        *resq_len = 0;
        return MODBUS_OK;
    }

    int total = 0;
    int64_t deadline = now_ms() + m->timeout_ms;
    while (total < MODBUS_BUFF_SIZE)
    {
        int wait;
        if (total == 0)
        {
            wait = (int)(deadline - now_ms());
            if (wait <= 0)
                return MODBUS_ERR_TIMEOUT;
        }
        else
        {
            int exp = modbus_expected_len(resq, total);
            wait = (exp > 0) ? baud_tran_time(exp - total, serial_get_baud(m->sp)) : m->gap_ms;
        }

        int n = serial_recv(m->sp, resq + total, MODBUS_BUFF_SIZE - total, wait);
        if (n < 0)
            return MODBUS_ERR_IO;
        if (n == 0)
        {
            if (total == 0)
                return MODBUS_ERR_TIMEOUT;
            break;
        }
        total += n;
        int exp = modbus_expected_len(resq, total);
        if (exp > 0 &&total >= exp)
            break;
    }
    hex_dump("RX", resq, total, m->debug);
    if (total < 5)
        return MODBUS_ERR_PROTOCOL;

    uint16_t crc_calc = crc16(resq, total - 2);
    uint16_t crc_recv = (uint16_t)((resq[total - 2])  | (resq[total - 1] << 8));

    if (crc_calc != crc_recv)
        return MODBUS_ERR_CRC;
    if (resq[0] != m->slave_addr)
        return MODBUS_ERR_ADDR;
    if (resq[1] != req[1])
    {
        if (resq[1] == (uint8_t)((req[1]) | 0x80) && total >= 5)
        {
            m->last_exception = resq[2];
            return MODBUS_ERR_EXCEPTION;
        }
        return MODBUS_ERR_PROTOCOL;
    }
    *resq_len = total;
    return MODBUS_OK;
}

static int modbus_transaction_with_retries(ModbusMaster *m, uint8_t *req, int req_len, uint8_t *resq, int *resq_len)
{
    int ret = MODBUS_ERR_TIMEOUT;
    for (int attempt = 0; attempt <= m->retries; attempt++)
    {
        ret = modbus_transaction(m, req, req_len, resq, resq_len);
        if (ret == MODBUS_OK)
            return MODBUS_OK;

        if (ret == MODBUS_ERR_PARAM || ret == MODBUS_ERR_STATE || ret == MODBUS_ERR_PROTOCOL || ret == MODBUS_ERR_EXCEPTION || ret == MODBUS_ERR_ADDR || ret == MODBUS_ERR_IO)
            return ret;

        if (attempt < m->retries)
            printf("[%s 第%d次失败，重试...]", modbus_result_str(ret), attempt + 1);
    }
    return ret;
}

static void modbus_build_req8(uint8_t *req, uint8_t slave, uint8_t func, uint16_t f1, uint16_t f2)
{
    req[0] = slave;
    req[1] = func;
    req[2] = (uint8_t)(f1 >> 8);
    req[3] = (uint8_t)(f1 & 0xFF);
    req[4] = (uint8_t)(f2 >> 8);
    req[5] = (uint8_t)(f2 & 0xFF);
    uint16_t crc = crc16(req, 6);
    req[6] = (uint8_t)(crc & 0xFF);
    req[7] = (uint8_t)(crc >> 8);
}


static int modbus_read_bits(ModbusMaster *m, uint8_t func, uint16_t start, uint16_t count, uint8_t *out)
{
   if (m == NULL || out == NULL)
       return MODBUS_ERR_PARAM;
   if (m->slave_addr == 0)
       return MODBUS_ERR_PARAM;
   if (count == 0 || count > MODBUS_MAX_READ_BITS)
       return MODBUS_ERR_PARAM;

    uint8_t req[8];
    modbus_build_req8(req, m->slave_addr, func, start, count);

    uint8_t resq[MODBUS_BUFF_SIZE];
    int resq_len = 0;
    int ret = modbus_transaction_with_retries(m, req, 8, resq, &resq_len);
    if (ret != MODBUS_OK)
        return ret;

    int bc = (count + 7) / 8;
    if (resq[2] != (uint8_t)bc || resq_len != 3 + bc + 2)
        return MODBUS_ERR_PROTOCOL;
    memcpy(out, resq + 3, (size_t)bc);
    return MODBUS_OK;
}


static int modbus_read_regs(ModbusMaster *m, uint8_t func, uint16_t start, uint16_t count, uint16_t *out)
{
   if (m == NULL || out == NULL)
       return MODBUS_ERR_PARAM;
   if (m->slave_addr == 0)
       return MODBUS_ERR_PARAM;
   if (count == 0 || count > MODBUS_MAX_READ_REGS)
       return MODBUS_ERR_PARAM;

    uint8_t req[8];
    modbus_build_req8(req, m->slave_addr, func, start, count);

    uint8_t resq[MODBUS_BUFF_SIZE];
    int resq_len = 0;
    int ret = modbus_transaction_with_retries(m, req, 8, resq, &resq_len);
    if (ret != MODBUS_OK)
        return ret;

    if (resq[2] != (uint8_t)(count * 2) || resq_len != 3 + count * 2 + 2)
        return MODBUS_ERR_PROTOCOL;
    for (uint16_t i = 0; i < count; i++)
    {
        out[i] = (uint16_t)(((uint16_t)resq[3 + i * 2] << 8) | resq[4 + i * 2]);
    }
    return MODBUS_OK;
}

static int modbus_write_single(ModbusMaster *m, uint8_t func, uint16_t addr, uint16_t value)
{
    if (m == NULL)
        return MODBUS_ERR_PARAM;

    uint8_t req[8];
    modbus_build_req8(req, m->slave_addr, func, addr, value);

    uint8_t resq[MODBUS_BUFF_SIZE];
    int resq_len = 0;
    int ret = modbus_transaction_with_retries(m, req, 8, resq, &resq_len);
    if (ret != MODBUS_OK)
        return ret;
    if (m->slave_addr == 0)
        return MODBUS_OK;
    if (resq_len != 8 || memcmp(resq, req, 6) != 0)
        return MODBUS_ERR_PROTOCOL;

    return MODBUS_OK;
}

static int modbus_write_multiple(ModbusMaster *m, uint8_t func, uint16_t start, uint16_t count, const void *data)
{
    if (m == NULL || data == NULL)
        return MODBUS_ERR_PARAM;

    uint8_t req[MODBUS_BUFF_SIZE];
    int bc;

    req[0] = m->slave_addr;
    req[1] = func;
    req[2] = (uint8_t)(start >> 8);
    req[3] = (uint8_t)(start & 0xFF);
    req[4] = (uint8_t)(count >> 8);
    req[5] = (uint8_t)(count & 0xFF);

    if (func == MODBUS_FUNC_WRITE_COILS)
    {
        if (count == 0 || count > MODBUS_MAX_WRITE_BITS)
            return MODBUS_ERR_PARAM;
        bc = (count + 7) / 8;
        memcpy(req + 7, data, (size_t)bc);

    }
    else
    {
        if (count == 0 || count > MODBUS_MAX_WRITE_REGS)
            return MODBUS_ERR_PARAM;
        bc = count * 2;
        const uint16_t *values = (const uint16_t *)data;
        for (uint16_t i = 0; i < count; i++)
        {
            req[7 + i * 2] = (uint8_t)(values[i] >> 8);
            req[8 + i * 2] = (uint8_t)(values[i] & 0xFF);
        }
    }
    req[6] = (uint8_t)bc;
    int req_len = 7 + bc + 2;
    if (req_len > MODBUS_BUFF_SIZE)
        return MODBUS_ERR_PARAM;

    uint16_t crc = crc16(req, req_len - 2);
    req[req_len - 2] = (uint8_t)(crc & 0xFF);
    req[req_len - 1] = (uint8_t)(crc >> 8);

    uint8_t resq[MODBUS_BUFF_SIZE];
    int resq_len = 0;
    int ret = modbus_transaction_with_retries(m, req, req_len, resq, &resq_len);
    if (ret != MODBUS_OK)
        return ret;
    if (m->slave_addr == 0)
        return MODBUS_OK;
    if (resq_len != 8 || memcmp(resq + 2, req + 2, 4) != 0)
        return MODBUS_ERR_PROTOCOL;

    return MODBUS_OK;
}

int modbus_read_coils(ModbusMaster *m, uint16_t start, uint16_t count, uint8_t *out)
{
    return modbus_read_bits(m, MODBUS_FUNC_READ_COILS, start, count, out);
}


int modbus_read_discrete_inputs(ModbusMaster *m, uint16_t start, uint16_t count, uint8_t *out)
{
    return modbus_read_bits(m, MODBUS_FUNC_READ_DISCRETE, start, count, out);
}


int modbus_read_holding_regs(ModbusMaster *m, uint16_t start, uint16_t count, uint16_t *out)
{
    return modbus_read_regs(m, MODBUS_FUNC_READ_HOLDING, start, count, out);
}


int modbus_read_input_regs(ModbusMaster *m, uint16_t start, uint16_t count, uint16_t *out)
{
    return modbus_read_regs(m, MODBUS_FUNC_READ_INPUT, start, count, out);
}

int modbus_write_coil(ModbusMaster *m, uint16_t addr, int value)
{
    return modbus_write_single(m, MODBUS_FUNC_WRITE_COIL, addr, value ? 0xFF00 : 0x0000);
}

int modbus_write_holding_reg(ModbusMaster *m, uint16_t addr, uint16_t value)
{
    return modbus_write_single(m, MODBUS_FUNC_WRITE_INPUT, addr, value);
}

int modbus_write_coils(ModbusMaster *m, uint16_t start, uint16_t count, const uint8_t *bits)
{
    return modbus_write_multiple(m, MODBUS_FUNC_WRITE_COILS, start, count, bits);
}

int modbus_write_holding_regs(ModbusMaster *m, uint16_t start, uint16_t count, const uint16_t *values)
{
    return modbus_write_multiple(m, MODBUS_FUNC_WRITE_MULTIPLE, start, count, values);
}




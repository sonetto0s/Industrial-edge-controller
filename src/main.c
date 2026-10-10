#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <stdlib.h>

#include "serial.h"
#include "config.h"
#include "modbus_rtu.h"

static volatile sig_atomic_t is_exit = 0;

static void on_handler(int sig)
{
    (void)sig;
    is_exit = 1;
}

static void print_help(void)
{
    printf("\n========== Modbus RTU 主站  ==========\n");
    printf(" a <slave>              切换从机地址 (1~247，0=广播仅写)\n");
    printf(" c1 <start> <count>     读线圈        (0x01)\n");
    printf(" c2 <start> <count>     读离散输入    (0x02)\n");
    printf(" r <start> <count>      读保持寄存器  (0x03)\n");
    printf(" i <start> <count>      读输入寄存器  (0x04)\n");
    printf(" wc <addr> <0|1>        写单个线圈    (0x05)\n");
    printf(" w <addr> <value>       写单个寄存器  (0x06)\n");
    printf(" m <start> <v...>       写多个寄存器  (0x10)\n");
    printf(" t <ms> <retries>       设置超时和重试\n");
    printf(" v                      开/关帧十六进制打印\n");
    printf(" k                      CRC自检\n");
    printf(" h                      帮助\n");
    printf(" q                      退出\n");
    printf("=======================================\n\n");
}

static void print_result(const char *op, int ret, const ModbusMaster *m)
{
    if (ret == MODBUS_OK)
        printf("%s 成功\n", op);
    else if (ret == MODBUS_ERR_EXCEPTION)
    {
        printf("%s 失败: %s (异常码为0x%02X),%s \n", op, modbus_result_str(ret), m->last_exception, modbus_exception_str(m->last_exception));
    }
    else
    {
        printf("%s 失败: %s \n", op, modbus_result_str(ret));
    }
}

int main(int argc, char *argv[])
{
    const char *config_path = (argc > 1) ? argv[1] : "config/config.conf";

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_handler;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    BusConfig cfg;
    if (busconfig_load(&cfg, config_path) != 0)
    {
        fprintf(stderr, "配置不正确:%s \n", config_path);
        return 1;
    }
    printf("串口: %s %d %d%c%d  485模式:%s 方向脚:%d\n", cfg.serial.device, cfg.serial.baud_rate, cfg.serial.databits, cfg.serial.parity, cfg.serial.stopbits, cfg.serial.rs485_mode == RS485_GPIO ? "GPIO手动" : "自动", cfg.serial.dir_gpio);

    SerialPort *sp = serial_create();
    if (sp == NULL)
    {
        fprintf(stderr, "创建串口失败\n");
        return 1;
    }
    serial_set_config(sp, &cfg.serial);
    SerialResult sr = serial_open(sp);
    if (sr != SERIAL_OK)
    {
        fprintf(stderr, "打开串口失败: %s \n", serial_result_str(sr));
        serial_destroy(sp);
        return 1;
    }
    ModbusMaster *m = modbus_create(sp, 1);
    if (m == NULL)
    {
        serial_shut(sp);
        serial_destroy(sp);
        return 1;
    }
    modbus_set_timeout(m, cfg.timeout_ms, cfg.retries);
    printf("\n就绪: 从机=%u 超时=%dms 重试=%d  (h=帮助)\n\n", m->slave_addr, m->timeout_ms, m->retries);

    char line[512];
    while (!is_exit)
    {
        printf("modbus> ");
        fflush(stdout);
        if (fgets(line, sizeof(line), stdin) == NULL)
            break;

        size_t l = strlen(line);
        while (l > 0 && (line[l - 1] == '\n' || line[l - 1] == '\r'))
            line[--l] = '\0';

        char *p = line;
        while (*p == ' ' || *p == '\t')
            p++;

        if (*p == '\0')
            continue;

        char cmd = *p;
        const char *args = p + 1;

        if (cmd == 'q')
            break;
        if (cmd == 'h' || cmd == '?')
        {
            print_help();
            continue;
        }

        if (cmd == 'k')
        {
            uint8_t sample[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02};
            uint16_t crc = modbus_crc16(sample, sizeof(sample));
            printf("01 03 00 00 00 02 -> CRC=0x%04X ", crc);
            printf(crc == 0x0BC4 ? "[正确0]\n" : "[错误|应为0x0BC4]\n");
            continue;
        }

        if (cmd == 'v')
        {
            m->debug = !m->debug;
            printf("帧打印: %s \n", m->debug ? "开" : "关");
            continue;
        }

        if(cmd=='a')
        {
            int addr = -1;
            if (sscanf(args, "%d", &addr) == 1 && addr >= 0 && addr <= 247)
            {
                modbus_set_slave(m,(uint16_t)addr);
                printf("从机地址 = %d%s \n", addr, addr == 0 ? "(广播，仅写有效)" : "");
            }
            else
            {
                printf("用法: a <slave> (0~247) \n");
            }
            continue;
        }

        if (cmd == 't')
        {
            int to = 0, rt = -1;
            if (sscanf(args, "%d %d", &to, &rt) == 2 && to > 0 && rt >= 0)
            {
                modbus_set_timeout(m, to, rt);
                printf("超时: %dms,重试: %d\n", to, rt);
            }
            else
            {
                printf("用法: t <timeout> <retries> \n");
            }
            continue;
        }

        if (cmd == 'r' || cmd == 'i')
         {
            unsigned start = 0, count = 0;
            if (sscanf(args, "%u %u", &start, &count) != 2 || count == 0 || count > 125)
            {
                printf("用法: %c <start> <count> (1~125)\n", cmd);
                continue;
            }
            uint16_t regs[125];
            int ret = (cmd == 'r') ? modbus_read_holding_regs(m, (uint16_t)start, (uint16_t)count, regs) : modbus_read_input_regs(m, (uint16_t)start, (uint16_t)count, regs);
            if (ret == MODBUS_OK)
            {
                for (unsigned i = 0; i < count; i++)
                    printf(" [%u] = %u (0x%04X)\n", start + i, regs[i], regs[i]);
            }
            else if (ret == MODBUS_ERR_EXCEPTION)
            {
                printf("读失败: %s (0x%02X %s)\n", modbus_result_str(ret), m->last_exception, modbus_exception_str(m->last_exception));
            }
            else
            {
                printf("读失败: %s\n", modbus_result_str(ret));
            }
            continue;
        }
        if (cmd == 'c')
        {
            char sub = p[1];
            unsigned start = 0, count = 0;
            if ((sub != '1' && sub != '2') || sscanf(args + 1, "%u %u", &start, &count) != 2 || count == 0 || count > 2000)
            {
                printf("用法: c1/c2 <start> <count> (1~2000)\n");
                continue;
            }
            uint8_t bits[250];
            int ret = (sub == '1') ? modbus_read_coils(m, (uint16_t)start, (uint16_t)count, bits): modbus_read_discrete_inputs(m, (uint16_t)start, (uint16_t)count, bits);
            if (ret == MODBUS_OK)
            {
                for (unsigned i = 0; i < count; i++)
                    printf(" [%u] = %d\n", start + i, (bits[i / 8] >> (i % 8)) & 1);
            }
            else
                printf("读失败: %s\n", modbus_result_str(ret));
            continue;
        }

        if (cmd == 'w')
        {

            if (p[1] == 'c')
            {
                unsigned addr = 0;
                int v = 0;
                if (sscanf(args + 1, "%u %d", &addr, &v) != 2)
                {
                    printf("用法: wc <addr> <0|1>\n");
                    continue;
                }
                print_result("写线圈",modbus_write_coil(m, (uint16_t)addr, v), m);
            }
            else
            {
                unsigned addr = 0, value = 0;
                if (sscanf(args, "%u %u", &addr, &value) != 2)
                {
                    printf("用法: w <addr> <value>\n");
                    continue;
                }
                print_result("写单个", modbus_write_holding_reg(m, (uint16_t)addr, (uint16_t)value), m);
            }
            continue;
        }

        if (cmd == 'm')
        {
            char *tok, *save = NULL;
            char buf[512];
            snprintf(buf, sizeof(buf), "%s", args);

            tok = strtok_r(buf, " \t", &save);
            if (tok == NULL)
            {
                printf("用法: m <start> <v1> <v2> ...\n");
                continue;
            }
            char *end = NULL;
            errno = 0;
            unsigned long start = strtoul(tok, &end, 0);

            if (errno != 0 || end == tok || *end != '\0' || start > 0xFFFF)
            {
                printf("起始地址非法\n");
                continue;
            }
            uint16_t vals[123];
            int n = 0;
            while ((tok = strtok_r(NULL, " \t", &save)) != NULL && n < 123)
            {
                errno = 0;
                unsigned long v = strtoul(tok, &end, 0);
                if (errno != 0 || end == tok || *end != '\0' || v > 0xFFFF)
                {
                    printf("值 '%s' 非法\n", tok);
                    n = -1;
                    break;
                }
                vals[n++] = (uint16_t)v;
            }
            if (n <= 0)
                continue;
            print_result("写多个", modbus_write_holding_regs(m, (uint16_t)start, (uint16_t)n, vals), m);
            continue;
        }

        printf("未知命令 '%c' (h=帮助) \n", cmd);
    }
    printf("\n退出...\n");
    modbus_destroy(m);
    serial_shut(sp);
    serial_destroy(sp);
    return 0;
}

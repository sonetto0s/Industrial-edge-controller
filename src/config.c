#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>


#include "config.h"

static char *trim(char *s)
{
    while (isspace((unsigned char)*s))
        s++;
    if (*s == '\0')
        return s;
    char *end = s + strlen(s) - 1;
    while (end > s && isspace((unsigned char)*end))
        *end-- = '\0';
    return s;
}

static int parse_int(const char *s, int *out)
{
    char *end = NULL;
    errno = 0;
    long v = strtol(s, &end, 10);
    if (errno != 0 || end == s || *end != '\0')
        return -1;
    *out = (int)v;
    return 0;
}

static void busconfig_dafault(BusConfig *cfg)
{
    if (cfg == NULL)
        return;
    snprintf(cfg->serial.device, sizeof(cfg->serial.device), "/dev/ttyS7");
    cfg->serial.baud_rate = 115200;
    cfg->serial.databits = 8;
    cfg->serial.parity = 'N';
    cfg->serial.stopbits = 1;
    cfg->serial.rs485_mode = RS485_AUTO;
    cfg->serial.dir_gpio = -1;
    cfg->timeout_ms = 500;
    cfg->retries = 1;
    cfg->gap_ms = 10;
}

int busconfig_load(BusConfig *cfg, const char *path)
{
    if (cfg == NULL || path == NULL)
        return -1;
    busconfig_dafault(cfg);

    FILE *fp = fopen(path, "r");
    if (fp == NULL)
    {
        printf("警告:配置文件 %s 不存在，已使用默认配置\n", path);
        return 0;
    }

    char *key = NULL;
    char *val = NULL;
    char line[1024];
    while (fgets(line, sizeof(line), fp))
    {
        char *hash = strchr(line, '#');
        if (hash)
            *hash = '\0';
        char *eq = strchr(line, '=');
        if (!eq)
            continue;
        *eq = '\0';

        key = trim(line);
        val = trim(eq + 1);
        if (*key == '\0' || *val == '\0')
            continue;

        int v = 0;
        if (strcmp(key, "device") == 0)
        {
            snprintf(cfg->serial.device, sizeof(cfg->serial.device), "%s", val);
        }
        else if (strcmp(key, "baud_rate") == 0)
        {
            if (parse_int(val, &v) != 0 || v <= 0)
                goto fail;
            cfg->serial.baud_rate = v;
        }
        else if (strcmp(key, "databits") == 0)
        {
            if (parse_int(val, &v) != 0 || v < 5 || v > 8)
                goto fail;
            cfg->serial.databits = v;
        }
        else if (strcmp(key, "parity") == 0)
        {
            char c = *val;
            if (c >= 'a' &&c <= 'z')
                c = (char)c - 32;
            if (c != 'E' && c != 'N' && c != 'O')
                goto fail;
            cfg->serial.parity = c;
        }
        else if (strcmp(key, "stopbits") == 0)
        {
            if (parse_int(val, &v) != 0 || (v != 1 && v != 2))
                goto fail;
            cfg->serial.stopbits = v;
        }
        else if (strcmp(key, "rs485_mode") == 0)
        {
            if (parse_int(val, &v) != 0 || v < 0 || v > 1)
                goto fail;
            cfg->serial.rs485_mode = v;
        }
        else if (strcmp(key, "dir_gpio") == 0)
        {
            if (parse_int(val, &v) != 0 || v < -1)
                goto fail;
            cfg->serial.dir_gpio = v;
        }
        else if (strcmp(key, "timeout_ms") == 0)
        {
            if (parse_int(val, &v) != 0 || v <= 0)
                goto fail;
            cfg->timeout_ms = v;
        }
        else if (strcmp(key, "retries") == 0)
        {
            if (parse_int(val, &v) != 0 || v < 0)
                goto fail;
            cfg->retries = v;
        }
        else if (strcmp(key, "gap_ms") == 0)
        {
            if (parse_int(val, &v) != 0 || v <= 0)
                goto fail;
            cfg->gap_ms = v;
        }
        else
        {
            printf("[警告]未知配置 %s，已忽略\n", key);
        }
    }

    fclose(fp);
    return 0;
fail:
    printf("[警告] %s 配置错误\n",key);
    fclose(fp);
    return -1;
}

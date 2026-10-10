#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>

#include "serial.h"
#include "timeutil.h"

#define GPIO_SYS_ROOT "/sys/class/gpio"

struct SerialPort{
    SerialConfig cfg;
    int fd;
    int gpio_fd;
    int is_open;
};

const char *serial_result_str(SerialResult r)
{
    switch (r)
    {
    case SERIAL_OK:
        return "成功";
    case SERIAL_ERR_PARAM:
        return "参数错误";
    case SERIAL_ERR_OPEN:
        return "打开串口失败";
    case SERIAL_ERR_STATE:
        return "串口未打开或被其他程序占用";
    case SERIAL_ERR_CONFIG:
        return "串口参数配置失败";
    case SERIAL_ERR_DATA:
        return "波特率等参数值非法";
    case SERIAL_ERR_GPIO:
        return "485方向脚操作失败";
    case SERIAL_ERR_IO:
        return "串口读写失败";
    default:
        return "未知错误";
    }
}

static int baud_to_speed(int baud, speed_t *out)
{
    switch (baud)
    {
          case 1200:
              *out = B1200;
              return 0;
          case 2400:
              *out = B2400;
              return 0;
          case 4800:
              *out = B4800;
              return 0;
          case 9600:
              *out = B9600;
              return 0;
          case 19200:
              *out = B19200;
              return 0;
          case 38400:
              *out = B38400;
              return 0;
          case 57600:
              *out = B57600;
              return 0;
          case 115200:
              *out = B115200;
              return 0;
          default:
              return -1;
          }

}

static int gpio_write_val(SerialPort *sp, int level)
{

    char b[2] = {level ? '1' : '0', '\n'};
    if (lseek(sp->gpio_fd, 0, SEEK_SET) < 0)
        return -1;
    return (write(sp->gpio_fd, b, 2) == 2 ? 0 : -1);
}

static SerialResult gpio_init(SerialPort *sp)
{
    char path[96];
    int fd = open(GPIO_SYS_ROOT "/export", O_WRONLY);
    if (fd >= 0)
    {
        dprintf(fd, "%d", sp->cfg.dir_gpio);
        close(fd);
    }

    snprintf(path, sizeof(path), "%s/gpio%d/direction", GPIO_SYS_ROOT, sp->cfg.dir_gpio);

    fd = open(path, O_WRONLY);
    if (fd < 0)
        return SERIAL_ERR_GPIO;

    if (write(fd, "out", 3) != 3)
    {
        close(fd);
        return SERIAL_ERR_GPIO;
    }
    close(fd);

    snprintf(path, sizeof(path), "%s/gpio%d/value", GPIO_SYS_ROOT, sp->cfg.dir_gpio);

    sp->gpio_fd = open(path, O_RDWR);
    if (sp->gpio_fd < 0)
        return SERIAL_ERR_GPIO;
    return (gpio_write_val(sp, 0) == 0 ? SERIAL_OK : SERIAL_ERR_GPIO);
}

static void gpio_deinit(SerialPort *sp)
{
    if (sp->gpio_fd >= 0)
    {
        gpio_write_val(sp, 0);
        close(sp->gpio_fd);
        sp->gpio_fd = -1;
    }
}

SerialPort *serial_create(void)
{
    SerialPort *sp = (SerialPort *)calloc(1, sizeof(SerialPort));
    if (sp == NULL)
        return NULL;

    sp->fd = -1;
    sp->gpio_fd = -1;
    snprintf(sp->cfg.device, sizeof(sp->cfg.device), "/dev/ttyS7");
    sp->cfg.baud_rate = 115200;
    sp->cfg.databits = 8;
    sp->cfg.parity = 'N';
    sp->cfg.stopbits = 1;
    sp->cfg.rs485_mode = RS485_AUTO;
    sp->cfg.dir_gpio = -1;

    return sp;
}


void serial_destroy(SerialPort *sp)
{
    if (sp == NULL)
        return;
    if (sp->is_open)
        serial_shut(sp);
    free(sp);
}



SerialResult serial_set_config(SerialPort *sp, const SerialConfig *cfg)
{
    if (sp == NULL || cfg == NULL)
        return SERIAL_ERR_PARAM;
    if (sp->is_open)
        return SERIAL_ERR_STATE;
    sp->cfg = *cfg;
    return SERIAL_OK;
}

SerialResult serial_open(SerialPort *sp)
{
    if (sp == NULL)
        return SERIAL_ERR_PARAM;
    if (sp->is_open) 
        return SERIAL_ERR_STATE;
    if (sp->cfg.rs485_mode == RS485_GPIO &&sp->cfg.dir_gpio < 0)
        return SERIAL_ERR_PARAM;

    speed_t speed;
    if (baud_to_speed(sp->cfg.baud_rate, &speed) != 0)
        return SERIAL_ERR_DATA;
    sp->fd = open(sp->cfg.device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (sp->fd < 0)
        return SERIAL_ERR_OPEN;

    struct flock lk;
    memset(&lk, 0, sizeof(lk));
    lk.l_type = F_WRLCK;
    lk.l_whence = SEEK_SET;
    lk.l_start = 0;
    lk.l_len = 0;
    if (fcntl(sp->fd, F_SETLK, &lk) < 0)
    {
        close(sp->fd);
        sp->fd = -1;
        return SERIAL_ERR_STATE;
    }

    struct termios tty;
    if (tcgetattr(sp->fd, &tty) < 0)
    {
        close(sp->fd);
        sp->fd = -1;
        return SERIAL_ERR_CONFIG;
    }

    tty.c_iflag &= ~(unsigned)IGNBRK;
    tty.c_iflag &= ~(unsigned)BRKINT;
    tty.c_iflag &= ~(unsigned)PARMRK;
    tty.c_iflag &= ~(unsigned)ISTRIP;
    tty.c_iflag &= ~(unsigned)INLCR;
    tty.c_iflag &= ~(unsigned)IGNCR;
    tty.c_iflag &= ~(unsigned)ICRNL;
    tty.c_iflag &= ~(unsigned)IXON;

    tty.c_oflag &= ~(unsigned)OPOST;

    tty.c_lflag &= ~(unsigned)ECHO;
    tty.c_lflag &= ~(unsigned)ECHONL;
    tty.c_lflag &= ~(unsigned)ICANON;
    tty.c_lflag &= ~(unsigned)ISIG;
    tty.c_lflag &= ~(unsigned)IEXTEN;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag &= ~(unsigned)CSIZE;
    switch (sp->cfg.databits)
    {
    case 5:
        tty.c_cflag |= (unsigned)CS5;
        break;
    case 6:
        tty.c_cflag |= (unsigned)CS6;
        break;
    case 7:
        tty.c_cflag |= (unsigned)CS7;
        break;
    case 8: default:
        tty.c_cflag |= (unsigned)CS8;
        break;
    }

    switch (sp->cfg.parity)
    {
    case 'E':
        tty.c_cflag |=  (unsigned)PARENB;
        tty.c_cflag &= ~(unsigned)PARODD;
        break;
    case 'O':
        tty.c_cflag |=  (unsigned)PARENB;
        tty.c_cflag |=  (unsigned)PARODD;
        break;
    case 'N': default:
        tty.c_cflag &= ~(unsigned)PARENB;
        break;
    }

    if (sp->cfg.stopbits == 2)
        tty.c_cflag |= (unsigned)CSTOPB;
    else
        tty.c_cflag &= ~(unsigned)CSTOPB;

    tty.c_cflag |= (unsigned)CREAD;
    tty.c_cflag |= (unsigned)CLOCAL;

#ifdef CRTSCTS
    tty.c_cflag &= ~(unsigned)CRTSCTS;
#endif

    tcflush(sp->fd, TCIOFLUSH);
    if (tcsetattr(sp->fd, TCSANOW, &tty) != 0)
    {
        close(sp->fd);
        sp->fd = -1;
        return SERIAL_ERR_CONFIG;
    }
    sp->gpio_fd = -1;
    if (sp->cfg.rs485_mode == RS485_GPIO)
    {
        SerialResult r = gpio_init(sp);
        if (r != SERIAL_OK)
        {
            close(sp->fd);
            sp->fd = -1;
            return r;
        }
    }
    sp->is_open = 1;
    return SERIAL_OK;
}

SerialResult serial_shut(SerialPort *sp)
{
    if (sp == NULL)
        return SERIAL_ERR_PARAM;
    if (!sp->is_open)
        return SERIAL_ERR_STATE;

    gpio_deinit(sp);
    tcdrain(sp->fd);
    tcflush(sp->fd, TCIOFLUSH);
    close(sp->fd);
    sp->fd = -1;
    sp->is_open = 0;
    return SERIAL_OK;
}

int serial_is_open(SerialPort *sp)
{
    return sp ? sp->is_open : 0;
}

SerialResult serial_send(SerialPort *sp, const uint8_t *data, int len)
{
    if (sp == NULL || data == NULL || len <= 0)
        return SERIAL_ERR_PARAM;
    if (!sp->is_open)
        return SERIAL_ERR_STATE;

    int manual_dir = (sp->cfg.rs485_mode == RS485_GPIO);
    if (manual_dir)
    {
        if (sp->gpio_fd < 0)
            return SERIAL_ERR_GPIO;
        if (gpio_write_val(sp, 1) < 0)
            return SERIAL_ERR_GPIO;
    }

    SerialResult rc = SERIAL_OK;
    int sent = 0;

    while (sent < len)
    {
        ssize_t n = write(sp->fd, data + sent, (size_t)(len - sent));

        if (n > 0)
        { sent += (int)n;
            continue;
        }
        if (n < 0 && errno == EINTR)
            continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
            struct pollfd pfd = { .fd = sp->fd, .events = POLLOUT };
            int pr = poll(&pfd, 1, 1000);
            if (pr < 0 && errno == EINTR)
                continue;
            if (pr <= 0)
            {
                rc = SERIAL_ERR_IO;
                break;
            }
            continue;
        }
        rc = SERIAL_ERR_IO;
        break;
    }

    if (manual_dir)
    {
        if (rc == SERIAL_OK)
            tcdrain(sp->fd);
        gpio_write_val(sp, 0);
    }
    return rc;
}

int serial_recv(SerialPort *sp, uint8_t *buf, int max, int timeout_ms)
{
    if (sp == NULL || buf == NULL || max <= 0 || !sp->is_open)
        return -1;
    if (timeout_ms <= 0)
        return 0;

    int total = 0;
    int64_t deadline = now_ms() + timeout_ms;
    while (total < max)
    {
        int wait = (int)(deadline - now_ms());
        if (wait <= 0)
            break;
        struct pollfd pfd = {.fd = sp->fd, .events = POLLIN};
        int pr = poll(&pfd, 1, wait);
        if (pr < 0)
        {
            if (errno == EINTR)
                continue;
            return -1;
        }
        if (pr == 0)
            break;
        ssize_t r = read(sp->fd, buf + total, (size_t)(max - total));
        if (r > 0)
        { total += (int)r;
            continue;
        }
        if (r < 0 &&(errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK))
            continue;
        return -1;
    }
    return total;
}


SerialResult serial_flush(SerialPort *sp, FlushDir dir)
{
    if (sp == NULL)
        return SERIAL_ERR_PARAM;
    int q;
    switch (dir)
    {
        case FLUSH_RX:
            q = TCIFLUSH;
            break;
        case FLUSH_TX:
            q = TCOFLUSH;
            break;
        default:
            q = TCIOFLUSH;
            break;
    }
    tcflush(sp->fd, q);
    return SERIAL_OK;
}

int serial_get_baud(SerialPort *sp)
{

    return sp ? sp->cfg.baud_rate : 0;
}

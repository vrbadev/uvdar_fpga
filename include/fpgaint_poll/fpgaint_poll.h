#ifndef FPGAINT_POLL_H_
#define FPGAINT_POLL_H_

#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>
#include <stdbool.h>

#define SYSFS_FILE_FPGAINTxx "/sys/devices/platform/soc/soc:fpgaint/%02d"

typedef struct {
    int num;
    int fd;
    char* sysfs_path;
    struct pollfd ufds[1];
    char irq_count_str[10];
} fpgaint_poll_t;

#define FPGAINT_POLL_NEW(fpgaint_num) ((fpgaint_poll_t) { .num = fpgaint_num, .fd = 0, .sysfs_path = NULL, .ufds = {0, 0, 0}, .irq_count_str = {0} })

bool fpgaint_poll_init(fpgaint_poll_t* fpgaint_poll);
void fpgaint_poll_deinit(fpgaint_poll_t* fpgaint_poll);
unsigned int fpgaint_poll_wait(fpgaint_poll_t* fpgaint_poll, int timeout_ms);

#endif

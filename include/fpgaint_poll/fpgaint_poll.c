#include "fpgaint_poll.h"


bool fpgaint_poll_init(fpgaint_poll_t* fpgaint_poll)
{
    fpgaint_poll->sysfs_path = (char*) calloc(sizeof(SYSFS_FILE_FPGAINTxx), sizeof(char));
    sprintf(fpgaint_poll->sysfs_path, SYSFS_FILE_FPGAINTxx, fpgaint_poll->num);
    if ((fpgaint_poll->fd = open(fpgaint_poll->sysfs_path, O_RDONLY)) < 0) {
        return false;
    } else {
        read(fpgaint_poll->fd, fpgaint_poll->irq_count_str, sizeof(fpgaint_poll->irq_count_str));
    }
    return true;
}

void fpgaint_poll_deinit(fpgaint_poll_t* fpgaint_poll)
{
    free(fpgaint_poll->irq_count_str);
    close(fpgaint_poll->fd);
}

unsigned int fpgaint_poll_wait(fpgaint_poll_t* fpgaint_poll, int timeout_ms)
{
    unsigned int count;
    fpgaint_poll->ufds[0].fd = fpgaint_poll->fd;
    fpgaint_poll->ufds[0].events = POLLPRI | POLLERR;
    fpgaint_poll->ufds[0].revents = 0;
    int ret = poll(fpgaint_poll->ufds, 1, timeout_ms);

    if ((fpgaint_poll->ufds[0].revents & POLLPRI) && ret > 0) {
        close(fpgaint_poll->fd);
        fpgaint_poll->fd = open(fpgaint_poll->sysfs_path, O_RDONLY);
        read(fpgaint_poll->fd, fpgaint_poll->irq_count_str, sizeof(fpgaint_poll->irq_count_str));
        sscanf(fpgaint_poll->irq_count_str, "%u", &count);
    }
    return count;
}

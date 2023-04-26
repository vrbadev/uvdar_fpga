#ifndef DEVMEM_H_
#define DEVMEM_H_

#include <stdint.h>

int32_t devmem_map(int32_t addr, int32_t span, void** ptr);
int32_t devmem_unmap(int32_t devmem_fd, void* ptr, int32_t span);

#endif

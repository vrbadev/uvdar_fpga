#include <unistd.h>
#include <fcntl.h> 
#include <sys/mman.h> 

#include "devmem.h"

int32_t devmem_map(int32_t addr, int32_t span, void** ptr)
{
    int32_t devmem_fd;

    if ((devmem_fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) {
        return -1;
    }

    int32_t pagesize = sysconf(_SC_PAGESIZE);
    int32_t aligned_offset = addr & -pagesize;
    int32_t span_add = addr - aligned_offset; 
    void* map = mmap(NULL, span + span_add, PROT_READ | PROT_WRITE, MAP_SHARED, devmem_fd, aligned_offset);
    if (map == MAP_FAILED) {
        close(devmem_fd);
        return -2;
    }
    *ptr = (void*) (map + span_add); 

    return devmem_fd;
}

int32_t devmem_unmap(int32_t devmem_fd, void* ptr, int32_t span)
{
    int32_t pagesize = sysconf(_SC_PAGESIZE);
    int32_t aligned = ((int32_t) ptr) & -pagesize;
    int32_t span_add = ((int32_t) ptr) - aligned; 
    if (munmap((void*) (aligned), span + span_add) != 0) {
		return -1;
    }
	close(devmem_fd);
    return 0;
}

#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>

// Mock _close
int _close(int file) {
    return -1;
}

// Mock _lseek
off_t _lseek(int file, off_t ptr, int dir) {
    return 0;
}

// Mock _read
ssize_t _read(int file, void *ptr, size_t len) {
    return 0;
}

// Mock _write
ssize_t _write(int file, const void *ptr, size_t len) {
    return len; // Pretend we wrote everything
}

// Mock _sbrk for memory allocation
caddr_t _sbrk(int incr) {
    extern char _end; // Defined in the linker script
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = &_end;
    }
    prev_heap_end = heap_end;

    // Check if the stack and heap collide
    if (heap_end + incr > (char *)__get_MSP()) {
        errno = ENOMEM;
        return (caddr_t)-1;
    }

    heap_end += incr;
    return (caddr_t)prev_heap_end;
}

// Mock _fstat
int _fstat(int file, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

// Mock _isatty
int _isatty(int file) {
    return 1;
}

// Mock _getpid
int _getpid(void) {
    return 1;
}

// Mock _kill
int _kill(int pid, int sig) {
    errno = EINVAL;
    return -1;
}

int __get_MSP(void) {
    __asm("MRS r0, MSP");
    __asm("BX lr");
}

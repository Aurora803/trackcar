#if defined(__GNUC__)
#include <errno.h>
#include <sys/stat.h>

int _close(int file)
{
    (void)file;
    errno = EBADF;
    return -1;
}

int _fstat(int file, struct stat *st)
{
    (void)file;
    if (st != 0)
    {
        st->st_mode = S_IFCHR;
    }
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}

int _lseek(int file, int ptr, int dir)
{
    (void)file;
    (void)ptr;
    (void)dir;
    return 0;
}

int _read(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    (void)len;
    errno = EAGAIN;
    return -1;
}
#endif
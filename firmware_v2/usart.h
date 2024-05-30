#ifndef USART_H
#define USART_H

#include <sys/stat.h>

void usart_comm_init(void);

int _write(int fd, char *ptr, int len);
int _close(int file);
int _fstat(int file, struct stat *st);
int _isatty(int file);
int _lseek(int file, int ptr, int dir);
int _read(int file, char *ptr, int len);
int _getpid(void);
int _kill(int pid, int sig);

#endif

#ifndef _SERIAL_UTIL_H
#define _SERIAL_UTIL_H

#include <termios.h>

int serial_setup_tio(int fd, int bauderate, int c_flow, 
                     int databits, char parity, int stopbits);
                     
int serial_open(const char *fn, struct termios *tio_saved);

void serial_close(int fd, const struct termios *tio_saved);

int serial_receive(int fd, char *rcv_buf, int data_len);

int serial_send(int fd, char *send_buf, int data_len) ;

#endif	/* _SERIAL_UTIL_H */

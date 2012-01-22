#ifndef __SERIAL_H__
#define __SERIAL_H__

#include "packet.h"

int serial_open(const char *device);
void serial_close(int fd);

void serial_sync(int fd);
void serial_send_packet(int fd, packet_buffer* packet);

#endif

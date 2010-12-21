#ifndef __SERIAL_H__
#define __SERIAL_H__

#include "packet.h"

int serial_open();
void serial_close();

void serial_sync();
void serial_send_packet(packet_buffer* packet);

void serial_send_str(char* msg, int num_bytes);

#endif

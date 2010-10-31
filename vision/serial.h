#ifndef __SERIAL_H__
#define __SERIAL_H__


int serial_open();
void serial_close();
void serial_send_str(char* msg, int num_bytes);

#endif

#include "serial.h"
#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h> 
#include <unistd.h>

const char *device = "/dev/ttyUSB0";


int fd;


int serial_open(){
    fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1) {
        printf( "failed to open port\n" );
        return 0;
    }

    struct termios  config;
    if(!isatty(fd)) { printf("error: not a tty!"); return 0;}
    if(tcgetattr(fd, &config) < 0) { printf("error: couldn't get attr"); return 0;}
    //
    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    //
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                        INLCR | PARMRK | INPCK | ISTRIP | IXON);
    //
    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    //
    // config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
    //                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
    config.c_oflag = 0;
    //
    // No line processing:
    // echo off, echo newline off, canonical mode off, 
    // extended input processing off, signal chars off
    //
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    //
    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    //
    config.c_cflag &= ~(CSIZE | PARENB);
    config.c_cflag |= CS8;
    //
    // One input byte is enough to return from read()
    // Inter-character timer off
    //
    config.c_cc[VMIN]  = 1;
    config.c_cc[VTIME] = 0;
    //
    // Communication speed (simple version, using the predefined
    // constants)
    //
    if(cfsetispeed(&config, B19200) < 0 || cfsetospeed(&config, B19200) < 0) {
         printf("error: couldn't set baud!\n");
         return 0;
    }
    //
    // Finally, apply the configuration
    //
    if(tcsetattr(fd, TCSAFLUSH, &config) < 0) { printf("error: couldn't set serial attrs\n"); }

    //write(fd,"A",1);
    return 1;
}


void serial_sync(){
    const int length=32;
    const int sync_byte=0;
    uint8_t sync[length];
    int i;
    for (i=0; i<length; i++)
      sync[i] = sync_byte;
    write(fd, sync, length);
}
void serial_send_packet(packet_buffer* packet){
    //printf("send packet: 0x%08lx\n", *((uint32_t*)packet));
    //printf("  size: %u\n", sizeof(packet_buffer));
    uint8_t len = sizeof(packet_buffer);
    write(fd, &len, 1);
    write(fd, packet, sizeof(packet_buffer));
    usleep(20000);
}

void serial_send_str(char *msg, int num_bytes){
    write(fd, msg, num_bytes);
}

void serial_close(){
    close(fd);
}


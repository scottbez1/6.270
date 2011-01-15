#include "serial.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

const char *device = 0;

int fd;

#ifndef DARWIN
#define DEV_NAME "ttyUSB"
#else
#define DEV_NAME "tty.usbserial"
#endif

void findDevice() {
    printf("Searching /dev/%s*...\n", DEV_NAME);
    DIR *dirp = opendir(".");
    struct dirent *dp;
    while ((dp = readdir(dirp)) != NULL) {
        if (!strncmp(dp->d_name, DEV_NAME, sizeof(DEV_NAME))) {
            printf("Using serial device %s\n", dp->d_name);
            // apparently Linux fails at strdup
            device = (const char*)strcpy((char*)malloc(strlen(dp->d_name)+1), dp->d_name);
            break;
        }
    }
    closedir(dirp);
}

int serial_open(const char *ttyDevice){
    device = ttyDevice;
    fd = -1;
    if (!device)
        findDevice();
    if (!device) {
        printf("No serial device.\n");
        return 0;
    }

    fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1) {
        printf( "Failed to open serial device %s\n", device );
        return 0;
    }

    struct termios options;
    memset(&options,0,sizeof(struct termios));
    if(!isatty(fd)) { printf("error: not a tty!"); return 0;}
#ifdef DARWIN
	cfmakeraw(&options);
	cfsetspeed(&options, 19200);
	options.c_cflag = CREAD | CLOCAL;
	options.c_cflag |= CS8;
	options.c_cc[VMIN] = 0;
	options.c_cc[VTIME] = 10;
	ioctl(fd, TIOCSETA, &options);
#else
    if(tcgetattr(fd, &options) < 0) { printf("error: couldn't get attr"); return 0;}
    //
    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    //
    options.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                        INLCR | PARMRK | INPCK | ISTRIP | IXON);
    //
    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    //
    // options.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
    //                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
    options.c_oflag = 0;
    //
    // No line processing:
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    //
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    //
    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    //
    options.c_cflag &= ~(CSIZE | PARENB);
    options.c_cflag |= CS8;
    //
    // One input byte is enough to return from read()
    // Inter-character timer off
    //
    options.c_cc[VMIN]  = 1;
    options.c_cc[VTIME] = 0;
    //
    // Communication speed (simple version, using the predefined
    // constants)
    //
    if(cfsetispeed(&options, B19200) < 0 || cfsetospeed(&options, B19200) < 0) {
         printf("error: couldn't set baud!\n");
         return 0;
    }
    //
    // Finally, apply the optionsuration
    //
    if(tcsetattr(fd, TCSAFLUSH, &options) < 0) { printf("error: couldn't set serial attrs\n"); }
#endif

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
}

void serial_send_str(char *msg, int num_bytes){
    write(fd, msg, num_bytes);
}

void serial_close(){
    close(fd);
}


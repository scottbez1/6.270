#ifndef _TABLE_H_INC_
#define _TABLE_H_INC_

#include <stdint.h>

// Generated automatically

extern uint16_t hamming_codes[65536];

#define HAMMING_DECODE(_num_, _pid_, _porient_, _pdist_) { \
    int entry = hamming_codes[_num_];  \
    *(_pid_) = entry & 0x1F;           \
    *(_porient_) = (entry & 0x60) >> 5; \
    *(_pdist_) = (entry & 0x380) >> 7; \
}

#endif

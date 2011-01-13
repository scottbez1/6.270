#ifndef _UTIL_H_INC_
#define _UTIL_H_INC_

#include "vision.h"

int clamp(int x, int low, int high);
double dist_sq(CvPoint *a, CvPoint *b);
int boundedRandom(int min, int max);
int get_5pixel_avg(IplImage *img, int x, int y);
double timeNow();

#endif

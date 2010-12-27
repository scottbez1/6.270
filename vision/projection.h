#ifndef __PROJECTION_H__
#define __PROJECTION_H__

#include <cv.h>

CvMat *projection;


void projection_init(CvPoint tl, CvPoint tr, CvPoint br, CvPoint bl, const int X_MIN, const int X_MAX, const int Y_MIN, const int Y_MAX);
CvPoint project(CvPoint point);
void projection_destroy();

#endif

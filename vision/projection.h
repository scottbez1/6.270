#ifndef __PROJECTION_H__
#define __PROJECTION_H__

#include <cv.h>

void projection_init(CvMat *projection, CvPoint tl, CvPoint tr, CvPoint br, CvPoint bl, const int X_MIN, const int X_MAX, const int Y_MIN, const int Y_MAX, const int inverse);
CvPoint project(CvMat *projection, CvPoint point);
void projection_destroy(CvMat *projection);

#endif

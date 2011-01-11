#ifndef __PROJECTION_H__
#define __PROJECTION_H__

#include <cv.h>

void projection_init(CvMat **projection, CvMat **invProjection, CvPoint tl, CvPoint tr, CvPoint br, CvPoint bl, const int X_MIN, const int X_MAX, const int Y_MIN, const int Y_MAX);
CvPoint2D32f project(CvMat *projection, CvPoint2D32f point);

#endif

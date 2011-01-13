#ifndef __PROJECTION_H__
#define __PROJECTION_H__

#include <cv.h>

void projection_init(CvMat **projection, CvMat **invProjection, CvPoint2D32f points[4], float bounds[4]);
CvPoint2D32f project(CvMat *projection, CvPoint2D32f point);

#endif

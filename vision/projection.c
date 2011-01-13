#include <stdio.h>
#include <cv.h>
#include <assert.h>
#include "projection.h"

//See http://stackoverflow.com/questions/169902/projective-transformation/2551747#2551747

// points are given in the order
// top left, top right, bottom right, bottom left
// bounds are given in the order
// X_MIN, X_MAX, Y_MIN, Y_MAX
void projection_init(CvMat **projection, CvMat **invProjection, CvPoint2D32f points[4], float bounds[4]) {
    if (*projection) cvReleaseMat(projection);
    if (*invProjection) cvReleaseMat(invProjection);

    CvPoint2D32f *src = points;
    CvPoint2D32f dst[4] = {{bounds[0],bounds[3]},{bounds[1],bounds[3]},{bounds[1],bounds[2]},{bounds[0],bounds[2]}};

    *projection = cvCreateMat(3,3,CV_32FC1);
    *invProjection = cvCreateMat(3,3,CV_32FC1);

    *projection = cvGetPerspectiveTransform(src, dst, *projection);
    *invProjection = cvGetPerspectiveTransform(dst, src, *invProjection);
}

CvPoint2D32f project(CvMat *projection, CvPoint2D32f point){
    assert(projection != NULL);

    CvPoint2D32f outputPoint;
    CvMat input = cvMat(1,1,CV_32FC2,&point), output = cvMat(1,1,CV_32FC2,&outputPoint);
    cvPerspectiveTransform(&input, &output, projection);

    return outputPoint;
}


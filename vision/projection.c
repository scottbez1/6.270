#include <stdio.h>
#include <cv.h>
#include <assert.h>
#include "projection.h"

//See http://stackoverflow.com/questions/169902/projective-transformation/2551747#2551747

void projection_init(CvMat **projection, CvMat **invProjection, CvPoint tl, CvPoint tr, CvPoint br, CvPoint bl, const int X_MIN, const int X_MAX, const int Y_MIN, const int Y_MAX){
    if (*projection != NULL){
        cvReleaseMat(projection);
    }

    CvPoint2D32f src[4] = {{tl.x, tl.y},{tr.x,tr.y},{br.x,br.y},{bl.x,bl.y}};
    CvPoint2D32f dst[4] = {{X_MIN,Y_MAX},{X_MAX,Y_MAX},{X_MAX,Y_MIN},{X_MIN,Y_MIN}};
    
    *projection = cvCreateMat(3,3,CV_32FC1);
    *invProjection = cvCreateMat(3,3,CV_32FC1);

    *projection = cvGetPerspectiveTransform(src, dst, *projection);
    *invProjection = cvGetPerspectiveTransform(dst, src, *invProjection);


    printf("project init. proj:%p, invProj:%p\n", *projection, *invProjection);
}

CvPoint2D32f project(CvMat *projection, CvPoint2D32f point){
    assert(projection != NULL);

    CvPoint2D32f inputPoint[] = {point};
    CvMat input = cvMat(1,1,CV_32FC2,inputPoint);

    CvPoint2D32f outputPoint[] = {{0,0}};
    CvMat output = cvMat(1,1,CV_32FC2,outputPoint);

    cvPerspectiveTransform(&input, &output, projection);
    
    return outputPoint[0];
}

void projection_destroy(CvMat *projection){
    cvReleaseMat(&projection);
}



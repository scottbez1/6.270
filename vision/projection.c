#include <cv.h>
#include <assert.h>
#include "projection.h"

//See http://stackoverflow.com/questions/169902/projective-transformation/2551747#2551747



void projection_init(CvPoint tl, CvPoint tr, CvPoint br, CvPoint bl, const int X_MIN, const int X_MAX, const int Y_MIN, const int Y_MAX){
    printf("project init\n");
    if (projection != NULL){
        cvReleaseMat(&projection);
    }

    projection = cvCreateMat(2,4,CV_64F);

    double d[] = {  X_MIN, Y_MAX,
                    X_MAX, Y_MAX,
                    X_MAX, Y_MIN,
                    X_MIN, Y_MIN};
    CvMat destination = cvMat(4,2,CV_64F,d);


    double i[] = { tl.x, tl.y, tl.x * tl.y, 1,
                   tr.x, tr.y, tr.x * tr.y, 1,
                   br.x, br.y, br.x * br.y, 1,
                   bl.x, bl.y, bl.x * bl.y, 1};
    CvMat input = cvMat(4,4,CV_64F,i);

    CvMat* input_inv = cvCreateMat(4,4,CV_64F);
    cvInvert(&input,input_inv, CV_LU); 
    
    CvMat* projection_transpose = cvCreateMat(4,2,CV_64F);
    cvMatMul(input_inv, &destination, projection_transpose);

    cvTranspose(projection_transpose, projection);

    cvReleaseMat(&projection_transpose);
    cvReleaseMat(&input_inv);
}

CvPoint project(CvPoint point){
    assert(projection != NULL);

    double i[] = { point.x,
                point.y,
                point.x * point.y,
                1};
    CvMat input = cvMat(4,1,CV_64F,i);

    double o[] = { 0,
                0};
    CvMat output = cvMat(2,1,CV_64F,o);

    cvMatMul(projection, &input, &output);
    
    return cvPoint(o[0], o[1]);
}

void projection_destroy(){
    cvReleaseMat(&projection);
}



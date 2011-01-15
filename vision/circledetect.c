//include files
#include "cv.h"
#include "highgui.h"
#include "math.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

int main (int argc, char** argv ) {
        int px[0], py[0];

        int edge_thresh = 80;
        float radius = 6;
        IplImage *src = 0, *csrc, *gray, *lab, *edge;
        CvMemStorage* cstorage = cvCreateMemStorage(0);
        CvCapture* capture=cvCaptureFromCAM(0);
        cvNamedWindow("src",1);

        for(;;){
                src=cvQueryFrame(capture);
                if (!csrc)
                    csrc = cvCreateImage(cvSize(src->width,src->height), 8, 3);
                if (!gray)
                    gray = cvCreateImage(cvSize(src->width,src->height), 8, 1);
                if (!lab)
                    lab = cvCreateImage(cvSize(src->width,src->height), 8, 3);
                if (!edge)
                    edge = cvCreateImage(cvSize(src->width,src->height), 8, 1);
                //cvConvertScale(src, csrc, 1./255., 0);
                csrc = cvCloneImage(src);
                cvCvtColor(csrc,lab,CV_BGR2Lab);
                //cvCvtColor(csrc,gray,CV_BGR2GRAY);
                gray->origin=1;
                cvSmooth( lab, lab, CV_GAUSSIAN, 11, 11, 0, 0);
                cvSplit(lab, gray, 0, 0, 0);
                CvSeq* circles =  cvHoughCircles( gray, cstorage, CV_HOUGH_GRADIENT, 2, 1.8*radius, edge_thresh, radius * 1.56, radius-2, radius+2); //get circles
                printf("%d\n", circles->total);
                for(int i = 0; circles->total>=2?i<2:i < circles->total; i++) { //just make a filter to limit only <=2 ciecles to draw
                     float* p = (float*)cvGetSeqElem( circles, i );
                     cvCircle( csrc, cvPoint(cvRound(p[0]),cvRound(p[1])), 3, CV_RGB(255,0,0), -1, 8, 0 );
                     cvCircle( csrc, cvPoint(cvRound(p[0]),cvRound(p[1])), cvRound(p[2]), CV_RGB(200,0,0), 1, 8, 0 );
                     px[i]=cvRound(p[0]); py[i]=cvRound(p[1]);
                }

                cvShowImage("src",csrc);

                cvReleaseImage(&csrc);
                cvClearMemStorage( cstorage );

                int c=cvWaitKey(10);
                if(c==27)break;
        }

        cvReleaseCapture( &capture);
        cvDestroyAllWindows();
}

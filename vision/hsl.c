//include files
#include "cv.h"
#include "highgui.h"
#include "math.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>

CvFont font;
int cvPrintf(IplImage *img, CvPoint pt, CvScalar color, const char *format, ...) {
    static char buffer[2048];
    va_list ap;
    int count;

    va_start(ap, format);
    count = vsnprintf(buffer, 2048, format, ap);
    va_end(ap);

    cvPutText(img, buffer, pt, &font, color);
    return count;
}

int main (int argc, char** argv ) {
        int px[0], py[0];

        int w = 600;
        int edge_thresh = 80;
        float radius = 6;
        IplImage *hsv = cvCreateImage(cvSize(w,w),8,3);
        IplImage *rgb = cvCreateImage(cvSize(w,w),8,3);
        cvNamedWindow("src",1);
        for (int i=0; i<w; i++) {
            for (int j=0; j<w; j++) {
                float x = (j-w/2.)/(w/2.);
                float y = (w/2.-i)/(w/2.);
                float theta = atan2(x, y)/M_PI;
                float r = MIN(sqrt(x*x+y*y), 1.0);
                if (r<1.0) {
                    int hue = (int)((theta*.5 + 1)*16+.5) % 16;
                    theta = (float)hue/8.;
                    int sat = MIN((int)(r * 16),15);
                    r = sat / 16.;
                    cvSet2D(hsv, i, j, cvScalar(90*theta,r*255.,255,0));
                } else {
                    cvSet2D(hsv, i, j, cvScalar(0,0,0,0));
                }
            }
        }
        cvCvtColor(hsv, rgb, CV_HSV2BGR);
        cvInitFont(&font, CV_FONT_VECTOR0, 0.5, 0.5, 0, 2, CV_AA);
        for (int i=0; i<16; i++) {
            float theta = M_PI*(float)i/8.;
            float x = w*(.5+.44*sin(theta));
            float y = w*(.5-.47*cos(theta));
            cvPrintf(rgb, cvPoint(x-30,y+5), CV_RGB(0,0,0), "Hue=%2d", i);
        }
        for (int j=0; j<16; j++) {
            float i =-.25;
            float theta = M_PI*(float)i/8.;
            float r = j / 16.0;
            float x = w*(.5+r*.5*sin(theta));
            float y = w*(.5-r*.5*cos(theta));
            cvPrintf(rgb, cvPoint(x-25,y), CV_RGB(0,0,0), "%s%2d", j==0 ? "Sat=" : "", j);
        }
        cvShowImage("src", rgb);

        for (;;) {
            int c=cvWaitKey(10);
            if(c==27)break;
        }

        cvReleaseImage(&hsv);
        cvReleaseImage(&rgb);
        cvDestroyAllWindows();
}

// int hue = (int)((h*16/180.0)+.5) % 16;
// int sat = (int)(s/16.0);

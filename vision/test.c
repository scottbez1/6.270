/*
 * The MIT License
 *
 * Copyright (c) 2010 Scott Bezek, MIT 6.270 Robotics Competition
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


//Fiducial pattern tracker, adapted from the OpenCV square detection demo.

#ifdef _CH_
#pragma package <opencv>
#endif

#define CV_NO_BACKWARD_COMPATIBILITY

#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

IplImage* img = 0;
const char* wndname = "6.270 Vision System";


int main(int argc, char** argv)
{
    int i, c;

    CvCapture* capture = 0;

    if( argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
        capture = cvCaptureFromCAM( argc == 2 ? argv[1][0] - '0' : 0 );
    else if( argc == 2 )
        capture = cvCaptureFromAVI( argv[1] );

    if( !capture )
    {
        fprintf(stderr,"Could not initialize capturing...\n");
        return -1;
    }


    cvNamedWindow( wndname, 1 );

    while(1){
           IplImage* frame = 0;

        frame = cvQueryFrame( capture );
        if( !frame ){
            fprintf(stderr,"cvQueryFrame failed!\n");
            continue;
            break;
        }

        img = cvCloneImage( frame );
        // show the resultant image
        //cvShowImage( wndname, img );
        printf("val: %i\n", cvGet2D(img, 10, 10).val[0]);
        cvReleaseImage( &img );



        // Also the function cvWaitKey takes care of event processing
        c = cvWaitKey(5);
        // clear memory storage - reset free space position
        if( (char)c == 27 )
            break;
    }

    cvDestroyWindow( wndname );

    return 0;
}


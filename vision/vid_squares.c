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

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

int threshold = 144;
//int thresh = 50;
IplImage* img = 0;
CvMemStorage* storage = 0;
const char* wndname = "6.270 Vision System";

#define WND_FILTERED "Filtered Video"
#define WND_CONTROLS "Controls"
#define TRK_THRESHOLD "Threshold"
#define TRK_TOLERANCE "Side length tolerance"
int side_tolerance = 60;

int show_filtered = 1;

const char *device = "/dev/ttyUSB0";

CvFont font;

// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
double angle( CvPoint* pt1, CvPoint* pt2, CvPoint* pt0 )
{
    double dx1 = pt1->x - pt0->x;
    double dy1 = pt1->y - pt0->y;
    double dx2 = pt2->x - pt0->x;
    double dy2 = pt2->y - pt0->y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

IplImage* filter_image( IplImage* img ){
    CvSize sz = cvSize( img->width & -2, img->height & -2 );
   
    IplImage* timg = cvCloneImage( img ); // make a copy of input image
    IplImage* pyr = cvCreateImage( cvSize(sz.width/2, sz.height/2), 8, 3 );
    IplImage* tgray;
   
    // select the maximum ROI in the image
    // with the width and height divisible by 2
    cvSetImageROI( timg, cvRect( 0, 0, sz.width, sz.height ));

    // down-scale and upscale the image to filter out the noise
    //cvPyrDown( timg, pyr, 7 );
    //cvPyrUp( pyr, timg, 7 );
    tgray = cvCreateImage( sz, 8, 1 );

    cvCvtColor(timg, tgray, CV_BGR2GRAY);
    cvReleaseImage( &pyr );
    cvReleaseImage( &timg );


    return tgray;
}

// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
CvSeq* findSquares4( IplImage* tgray, CvMemStorage* storage )
{
    CvSeq* contours;
    int i, c, l, N = 5;//11;
    CvSize sz = cvSize( img->width & -2, img->height & -2 );
    IplImage* gray = cvCreateImage( sz, 8, 1 );
    CvSeq* result;
    double s, t;
    // create empty sequence that will contain points -
    // 4 points per square (the square's vertices)
    CvSeq* squares = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvPoint), storage );


        // try several threshold levels
        //for( l = 0; l < N; l++ )
        //{
            /*
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                cvCanny( tgray, gray, 0, thresh, 5 );
                // dilate canny output to remove potential
                // holes between edge segments
                cvDilate( gray, gray, 0, 1 );
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                cvThreshold( tgray, gray, (l+1)*255/N, 255, CV_THRESH_BINARY );
            }
            */
           
            cvThreshold( tgray, gray, threshold, 255, CV_THRESH_BINARY );
            
            if (show_filtered)
                cvShowImage( WND_FILTERED, gray );

            // find contours and store them all as a list
            cvFindContours( gray, storage, &contours, sizeof(CvContour),
                CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );

            // test each contour
            while( contours )
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                result = cvApproxPoly( contours, sizeof(CvContour), storage,
                    CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.02, 0 );
                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( result->total == 4 &&
                    cvContourArea(result,CV_WHOLE_SEQ,0) > 1000 &&
                    cvContourArea(result,CV_WHOLE_SEQ,0) < 8000 &&
                    cvCheckContourConvexity(result) )
                {
                    s = 0;

                    for( i = 0; i < 5; i++ )
                    {
                        // find minimum angle between joint
                        // edges (maximum of cosine)
                        if( i >= 2 )
                        {
                            t = fabs(angle(
                            (CvPoint*)cvGetSeqElem( result, i ),
                            (CvPoint*)cvGetSeqElem( result, i-2 ),
                            (CvPoint*)cvGetSeqElem( result, i-1 )));
                            s = s > t ? s : t;
                        }
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( s < 0.3 )
                        for( i = 0; i < 4; i++ )
                            cvSeqPush( squares,
                                (CvPoint*)cvGetSeqElem( result, i ));
                }

                // take the next contour
                contours = contours->h_next;
            }
        //}
    // release all the temporary images
    cvReleaseImage( &gray );
    return squares;
}

// returns the squared euclidian distance between two points
double dist_sq(CvPoint* a, CvPoint* b){
    double dx = a->x - b->x;
    double dy = a->y - b->y;
    return dx*dx+dy*dy;
}

// the function draws all the squares in the image
//TODO: refactor to separate robot detection from drawing
void drawSquares( IplImage* img, IplImage* grayscale, CvSeq* squares )
{
    CvSeqReader reader;
    IplImage* cpy = cvCloneImage( img );
    int i;

    // initialize reader of the sequence
    cvStartReadSeq( squares, &reader, 0 );
    // read 4 sequence elements at a time (all vertices of a square)
    for( i = 0; i < squares->total; i += 4 )
    {
        CvPoint pt[4], *rect = pt;
        int count = 4;

        // read 4 vertices
        CV_READ_SEQ_ELEM( pt[0], reader );
        CV_READ_SEQ_ELEM( pt[1], reader );
        CV_READ_SEQ_ELEM( pt[2], reader );
        CV_READ_SEQ_ELEM( pt[3], reader );


        //calculate the length of each side
        double side_len[4];
        side_len[0] = dist_sq(&pt[0],&pt[1]);
        side_len[1] = dist_sq(&pt[1],&pt[2]);
        side_len[2] = dist_sq(&pt[2],&pt[3]);
        side_len[3] = dist_sq(&pt[3],&pt[0]);
        
        double tolerance = (double)side_tolerance / 100.;
        //check to make sure all sides are approx. the same length as side 0
        if (fabs(side_len[0] - side_len[1])/side_len[0] > tolerance ||
            fabs(side_len[0] - side_len[2])/side_len[0] > tolerance ||
            fabs(side_len[0] - side_len[3])/side_len[0] > tolerance) {
            // at least one of the sides is significantly different than side 0, so skip this square
            //printf("sides: %f %f %f %f\n", side_len[0], side_len[1], side_len[2], side_len[3]);
            continue;
        }

        // draw the square as a closed polyline
        cvPolyLine( cpy, &rect, &count, 1, 1, CV_RGB(0,0,255), 2, CV_AA, 0 );
       
        CvPoint corner_pt[4];
        corner_pt[0].x = (pt[0].x*3+pt[2].x*13)/16;
        corner_pt[0].y = (pt[0].y*3+pt[2].y*13)/16;
        corner_pt[1].x = (pt[1].x*3+pt[3].x*13)/16;
        corner_pt[1].y = (pt[1].y*3+pt[3].y*13)/16;
        corner_pt[2].x = (pt[2].x*3+pt[0].x*13)/16;
        corner_pt[2].y = (pt[2].y*3+pt[0].y*13)/16;
        corner_pt[3].x = (pt[3].x*3+pt[1].x*13)/16;
        corner_pt[3].y = (pt[3].y*3+pt[1].y*13)/16;

        CvSize sz = cvSize( grayscale->width & -2, grayscale->height & -2 );

        CvScalar corner[4];
        corner[0] = cvGet2D(grayscale, corner_pt[0].y, corner_pt[0].x);
        corner[1] = cvGet2D(grayscale, corner_pt[1].y, corner_pt[1].x);
        corner[2] = cvGet2D(grayscale, corner_pt[2].y, corner_pt[2].x);
        corner[3] = cvGet2D(grayscale, corner_pt[3].y, corner_pt[3].x);

        int best_corner = 0;
        int j;
        for (j=1; j<4; j++){
          if(corner[best_corner].val[0] < corner[j].val[0]){
            best_corner = j;
          }
        }

        CvPoint center = cvPoint((pt[0].x + pt[1].x + pt[2].x + pt[3].x)/4,(pt[0].y + pt[1].y + pt[2].y + pt[3].y)/4);

        //corner_idx is used to map corner indices in relation to the registration corner to absolute corner indices
        //e.g. if corner_pt[3] is the registration corner, then corner_idx[0]=3, corner_idx[1]=0, corner_idx[2]=1, etc

        int corner_idx[4];
        corner_idx[0]=(0+best_corner)%4;
        corner_idx[1]=(1+best_corner)%4;
        corner_idx[2]=(2+best_corner)%4;
        corner_idx[3]=(3+best_corner)%4;
        //TODO: check assumption that corners are in a consistent order, e.g. clockwise

        //make sure the other 3 corners are dark, otherwise ignore this square
        if (cvGet2D(grayscale, corner_pt[corner_idx[1]].y, corner_pt[corner_idx[1]].x).val[0] > threshold ||
            cvGet2D(grayscale, corner_pt[corner_idx[2]].y, corner_pt[corner_idx[2]].x).val[0] > threshold ||
            cvGet2D(grayscale, corner_pt[corner_idx[3]].y, corner_pt[corner_idx[3]].x).val[0] > threshold){
            //continue;
        }


        CvPoint bit_pt[4];
        bit_pt[0].x = (pt[corner_idx[0]].x*3+pt[corner_idx[2]].x*5)/8;
        bit_pt[0].y = (pt[corner_idx[0]].y*3+pt[corner_idx[2]].y*5)/8;
        bit_pt[2].x = (pt[corner_idx[1]].x*3+pt[corner_idx[3]].x*5)/8;
        bit_pt[2].y = (pt[corner_idx[1]].y*3+pt[corner_idx[3]].y*5)/8;
        bit_pt[3].x = (pt[corner_idx[2]].x*3+pt[corner_idx[0]].x*5)/8;
        bit_pt[3].y = (pt[corner_idx[2]].y*3+pt[corner_idx[0]].y*5)/8;
        bit_pt[1].x = (pt[corner_idx[3]].x*3+pt[corner_idx[1]].x*5)/8;
        bit_pt[1].y = (pt[corner_idx[3]].y*3+pt[corner_idx[1]].y*5)/8;


        cvCircle(cpy, bit_pt[0], 3, CV_RGB(255,0,0),-1,8,0);
        cvCircle(cpy, bit_pt[1], 3, CV_RGB(0,255,0),-1,8,0);
        cvCircle(cpy, bit_pt[2], 3, CV_RGB(0,0,255),-1,8,0);
        cvCircle(cpy, bit_pt[3], 3, CV_RGB(255,0,255),-1,8,0);

        int id =    ((cvGet2D(img, bit_pt[0].y, bit_pt[0].x).val[0] >= threshold) << 3) +
                    ((cvGet2D(img, bit_pt[1].y, bit_pt[1].x).val[0] >= threshold) << 2) +
                    ((cvGet2D(img, bit_pt[2].y, bit_pt[2].x).val[0] >= threshold) << 1) +
                    ((cvGet2D(img, bit_pt[3].y, bit_pt[3].x).val[0] >= threshold) << 0);
        //printf("Found robot %i\n", id);

        //Show the robot's ID next to it
        char buffer[20];
        sprintf(buffer,"Robot %i",id);
        cvPutText(cpy, buffer, cvPoint(center.x-20, center.y+50), &font, cvScalar(255,255,0,0));
        
        
        //printf("Square 0 at: %i,%i\n", (pt[0].x+pt[1].x+pt[2].x+pt[3].x)/4, (pt[0].y+pt[1].y+pt[2].y+pt[3].y)/4);
        
        //make a dot in the registration corner
        cvCircle(cpy, corner_pt[best_corner], 6, CV_RGB(255,0,0),-1,8,0);
        
    }

    // show the resultant image
    cvShowImage( wndname, cpy );
    cvReleaseImage( &cpy );
}


int fd;


void openSerial(){
    fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1) {
        printf( "failed to open port\n" );
    }

    struct termios  config;
    if(!isatty(fd)) { printf("error: not a tty!"); }
    if(tcgetattr(fd, &config) < 0) { printf("error: couldn't get attr"); }
    //
    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    //
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                        INLCR | PARMRK | INPCK | ISTRIP | IXON);
    //
    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    //
    // config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
    //                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
    config.c_oflag = 0;
    //
    // No line processing:
    // echo off, echo newline off, canonical mode off, 
    // extended input processing off, signal chars off
    //
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    //
    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    //
    config.c_cflag &= ~(CSIZE | PARENB);
    config.c_cflag |= CS8;
    //
    // One input byte is enough to return from read()
    // Inter-character timer off
    //
    config.c_cc[VMIN]  = 1;
    config.c_cc[VTIME] = 0;
    //
    // Communication speed (simple version, using the predefined
    // constants)
    //
    if(cfsetispeed(&config, B19200) < 0 || cfsetospeed(&config, B19200) < 0) {
         printf("error: couldn't set baud!\n");
    }
    //
    // Finally, apply the configuration
    //
    if(tcsetattr(fd, TCSAFLUSH, &config) < 0) { printf("error: couldn't set serial attrs\n"); }

    write(fd,"A",1);
}

void closeSerial(){
    close(fd);
}

int main(int argc, char** argv)
{
    openSerial();
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 2, CV_AA);
    int i, c;
    // create memory storage that will contain all the dynamic data
    storage = cvCreateMemStorage(0);

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
    cvNamedWindow( WND_CONTROLS, CV_WINDOW_AUTOSIZE);
    cvNamedWindow( WND_FILTERED, CV_WINDOW_AUTOSIZE);
    cvCreateTrackbar( TRK_THRESHOLD, WND_CONTROLS, &threshold, 255, NULL);
    cvCreateTrackbar( TRK_TOLERANCE, WND_CONTROLS, &side_tolerance, 300, NULL);


    while(1){
           IplImage* frame = 0;

        frame = cvQueryFrame( capture );
        if( !frame ){
            fprintf(stderr,"cvQueryFrame failed!\n");
            continue;
            break;
        }

        img = cvCloneImage( frame );

        IplImage* grayscale = filter_image(img);

        // find and draw the squares
        CvSeq* squares = findSquares4( grayscale, storage );

        drawSquares( img, grayscale, squares );

        cvReleaseImage( &grayscale );

        // Also the function cvWaitKey takes care of event processing
        c = cvWaitKey(5);
        // release both images
        cvReleaseImage( &img );
        // clear memory storage - reset free space position
        cvClearMemStorage( storage );
        if( (char)c == 27 )
            break;
    }

    cvDestroyWindow( wndname );

    closeSerial();

    return 0;
}


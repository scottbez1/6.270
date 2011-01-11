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
#include "serial.h"
#include "projection.h"
#include <pthread.h>

#include <time.h>

#define MATCH_LEN_SECONDS 60

#define MATCH_ENDED 0
#define MATCH_RUNNING 1
int matchState = MATCH_ENDED;
time_t matchStartTime;

int sendStartPacket = 0;   //flag to have a start packet sent ASAP

#define PI 3.14159265

//size of space to project onto
#define X_MIN -2048
#define X_MAX 2047

#define Y_MIN -2048
#define Y_MAX 2047


int threshold = 100;

IplImage* img = 0;
CvMemStorage* storage = 0;
const char* WND_MAIN = "6.270 Vision System";
const char* WND_FILTERED = "Filtered Video";
const char* WND_CONTROLS = "Controls";
const char* TRK_THRESHOLD = "Threshold";
const char* TRK_TOLERANCE = "Side length tolerance";
const char* TRK_MIN_AREA = "Min square area";
const char* TRK_MAX_AREA = "Max square area";
const char* TRK_ROBOT_A_ID = "Robot A id";
const char* TRK_ROBOT_B_ID = "Robot B id";

const char* TRK_RAND_GOAL_SEED = "Random goal seed";
int randomGoalSeed;


int side_tolerance = 20;

int min_area = 500;
int max_area = 2000;

CvFont font;

typedef struct fiducial {
    CvPoint tl;
    CvPoint tr;
    CvPoint bl;
    CvPoint br;
} fiducial_t;

typedef struct robot {
    int id;
    signed x : 12;
    signed y : 12;
    signed theta : 12;
    pthread_mutex_t lock;
} robot_t;


#define MAX_ROBOT_ID 16
robot_t robot_a;
robot_t robot_b;

int clamp(int x, int low, int high){
    return x < low ? low : (x > high ? high : x);
}


// returns the squared euclidian distance between two points
double dist_sq(CvPoint* a, CvPoint* b){
    double dx = a->x - b->x;
    double dy = a->y - b->y;
    return dx*dx+dy*dy;
}

int get_5pixel_avg(IplImage* img, int x, int y){
    int sum = 0;
    int num = 0;

    sum += cvGet2D(img, y, x).val[0];
    num ++;

    if (x-1 > 0){
        sum += cvGet2D(img, y, x-1).val[0];
        num++;
    }
    if (x+1 < img->width){
        sum += cvGet2D(img, y, x+1).val[0];
        num++;
    }
    if (y-1 > 0){
        sum += cvGet2D(img, y-1, x).val[0];
        num++;
    }
    if (y+1 < img->height){
        sum += cvGet2D(img, y+1, x).val[0];
        num++;
    }
    return sum/num;
}



#define GOAL_TOLERANCE 50
int score = 0;

CvPoint goal = {0,0};

void reseedRandom(){
    srand(randomGoalSeed);
}

void resetScore(){
    score = 0;
}

int boundedRandom(int min, int max){
    return min + (int)((rand()/(RAND_MAX +1.0))*(max-min));
}

void pickNewGoal(){
    int x = boundedRandom(-2048+600, 2047-600);
    int y = boundedRandom(-2048+600, 0 - 600);
    goal = cvPoint(x,y);
}

void checkGoals(){
    CvPoint robotPt = cvPoint(robot_a.x, robot_a.y);
    if (sqrt(dist_sq(&goal, &robotPt)) <= GOAL_TOLERANCE){
        score++;
        pickNewGoal();
        printf("GOAL!\n");
    }
}






CvMat *projection;
CvMat *invProjection;

CvPoint projectionPoints[4];

#define MOUSE_NA 0
#define MOUSE_PROJECT_1 1
#define MOUSE_PROJECT_2 2
#define MOUSE_PROJECT_3 3
#define MOUSE_PROJECT_4 4

int mouseState = MOUSE_NA;
void mouseHandler(int event, int x, int y, int flags, void* param){
    if (event == CV_EVENT_LBUTTONDOWN){
        switch(mouseState){
            case MOUSE_PROJECT_1:
                projectionPoints[0] = cvPoint(x*4,y*4);
                mouseState = MOUSE_PROJECT_2;
                break;
            case MOUSE_PROJECT_2:
                projectionPoints[1] = cvPoint(x*4,y*4);
                mouseState = MOUSE_PROJECT_3;
                break;
            case MOUSE_PROJECT_3:
                projectionPoints[2] = cvPoint(x*4,y*4);
                mouseState = MOUSE_PROJECT_4;
                break;
            case MOUSE_PROJECT_4:
                projectionPoints[3] = cvPoint(x*4,y*4);
                mouseState = MOUSE_NA;
                
                projection_init(&projection,
                                &invProjection,
                                projectionPoints[0],
                                projectionPoints[1],
                                projectionPoints[2],
                                projectionPoints[3],
                                X_MIN, X_MAX, Y_MIN, Y_MAX);
                printf("projection: %p\n", projection);
                break;
        }
    }
}




static void fiducial_clear(fiducial_t *f){
    f->tl = cvPoint(0,0);
    f->tr = cvPoint(0,0);
    f->bl = cvPoint(0,0);
    f->br = cvPoint(0,0);
}

static CvPoint2D32f fiducial_center(fiducial_t f){
    return cvPoint2D32f( f.tl.x + f.tr.x + f.bl.x + f.br.x,
                    f.tl.y + f.tr.y + f.bl.y + f.br.y);
}


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
    int i;
    CvSize sz = cvSize( img->width & -2, img->height & -2 );
    IplImage* gray = cvCreateImage( sz, 8, 1 );
    CvSeq* result;
    double s, t;
    // create empty sequence that will contain points -
    // 4 points per square (the square's vertices)
    CvSeq* squares = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvPoint), storage );

            cvThreshold( tgray, gray, threshold, 255, CV_THRESH_BINARY );
            
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
                    -cvContourArea(result,CV_WHOLE_SEQ,1) > min_area &&
                    -cvContourArea(result,CV_WHOLE_SEQ,1) < max_area &&
                    cvCheckContourConvexity(result))
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
                    // vertices to resultant sequence in clockwise order
                    if( s < 0.3 )
                        for( i = 0; i < 4; i++ )
                            cvSeqPush( squares,
                                (CvPoint*)cvGetSeqElem( result, 3-i ));
                }

                // take the next contour
                contours = contours->h_next;
            }
    // release all the temporary images
    cvReleaseImage( &gray );
    return squares;
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

        float l = -.3, r = 4.3;
        CvPoint2D32f src[4] = {{l,l},{r,l},{r,r},{l,r}};
        CvPoint2D32f dst[4] = {{pt[0].x,pt[0].y},{pt[1].x,pt[1].y},{pt[2].x,pt[2].y},{pt[3].x,pt[3].y}};
        CvMat* H = cvCreateMat(3,3,CV_32FC1);
        H = cvGetPerspectiveTransform(src, dst, H);

        //calculate the coordinates of each bit
        CvPoint2D32f bit_pt_raw[16];
        int j;
        for (j=0; j<16; j++)
            bit_pt_raw[j] = cvPoint2D32f(.5 + j%4, .5 + j/4);
        CvMat pts = cvMat(1, 16, CV_32FC2, bit_pt_raw);
        cvPerspectiveTransform(&pts, &pts, H);

        cvReleaseMat(&H);

        int bit_raw[16];
        for (j=0; j<16; j++)
            bit_raw[j] = (get_5pixel_avg(img, bit_pt_raw[j].x, bit_pt_raw[j].y) > threshold);

        int corner[4];
        corner[0] = bit_raw[0];
        corner[1] = bit_raw[3];
        corner[2] = bit_raw[15];
        corner[3] = bit_raw[12];

        // registration corner is the white corner whose clockwise neighbor is black
        int best_corner = -1;
        for (j=0; j<4; j++){
          if(corner[j] && !corner[(j+1) % 4]){
            if (best_corner != -1) { // corner is ambiguous
                best_corner = -1;
                break;
            }
            best_corner = j;
          }
        }

        if (best_corner == -1){
            continue;
        }

        CvPoint center = cvPoint((pt[0].x + pt[1].x + pt[2].x + pt[3].x)/4,(pt[0].y + pt[1].y + pt[2].y + pt[3].y)/4);

        // shift indices so that best_corner -> 0
        CvPoint2D32f bit_pt_true[16];
        int bit_true[16];
        for (j=0; j<16; j++) {
            int x = j%4, y=j/4;
            int xp, yp;
            switch (best_corner) {
                case 0:
                    xp = x; yp = y;
                    break;
                case 1:
                    xp = 3-y; yp = x;
                    break;
                case 2:
                    xp = 3-x; yp = 3-y;
                    break;
                case 3:
                    xp = y; yp = 3-x;
                    break;
            }
            int j_raw = xp + yp*4;
            bit_pt_true[j] = bit_pt_raw[j_raw];
            bit_true[j] = bit_raw[j_raw];
        }

        //for debugging, draw a dot over each bit location
        cvCircle(cpy, cvPoint(bit_pt_true[5].x, bit_pt_true[5].y), 3, CV_RGB(255,0,0),-1,8,0);
        cvCircle(cpy, cvPoint(bit_pt_true[6].x, bit_pt_true[6].y), 3, CV_RGB(0,255,0),-1,8,0);
        cvCircle(cpy, cvPoint(bit_pt_true[9].x, bit_pt_true[9].y), 3, CV_RGB(0,0,255),-1,8,0);
        cvCircle(cpy, cvPoint(bit_pt_true[10].x, bit_pt_true[10].y), 3, CV_RGB(255,0,255),-1,8,0);

        //read fiducial bits into "id"
        int id = (bit_true[5] << 0) + (bit_true[6] << 1) + (bit_true[9] << 2) + (bit_true[10] << 3) +
                 (bit_true[1] << 4) + (bit_true[2] << 5) + (bit_true[4] << 6) + (bit_true[7] << 7) +
                 (bit_true[8] << 8) + (bit_true[11] << 9) + (bit_true[13] << 10) + (bit_true[14] << 11) +
                 ((bit_true[12] + bit_true[15]) << 12);

        //Show the robot's ID next to it
        {
            char buffer[20];
            sprintf(buffer,"Robot %i",id);
            cvPutText(cpy, buffer, cvPoint(center.x-20, center.y+50), &font, cvScalar(255,255,0,0));
        }

        fiducial_t fiducial;

        if (id >= 0 && id < MAX_ROBOT_ID){
            fiducial.tl.x = bit_pt_true[0].x;
            fiducial.tl.y = bit_pt_true[0].y;
            fiducial.tr.x = bit_pt_true[3].x;
            fiducial.tr.y = bit_pt_true[3].y;
            fiducial.br.x = bit_pt_true[15].x;
            fiducial.br.y = bit_pt_true[15].y;
            fiducial.bl.x = bit_pt_true[12].x;
            fiducial.bl.y = bit_pt_true[12].y;
        }

        if (projection != NULL){
            CvPoint2D32f center = project(projection, fiducial_center(fiducial));

            //to find the heading, "extend" the top and bottom edges 4x to the right and take 
            //  the average endpoint, then project this and take the dx and dy in the projected 
            //  space to find the angle it makes

            int extended_top_x = fiducial.tr.x + (fiducial.tr.x - fiducial.tl.x)*4;
            int extended_top_y = fiducial.tr.y + (fiducial.tr.y - fiducial.tl.y)*4;

            int extended_bottom_x = fiducial.br.x + (fiducial.br.x - fiducial.bl.x)*4;
            int extended_bottom_y = fiducial.br.y + (fiducial.br.y - fiducial.bl.y)*4;

            int extended_avg_x = (extended_top_x+extended_bottom_x)/2;
            int extended_avg_y = (extended_top_y+extended_bottom_y)/2;

            //draw a white circle at the extended point
            cvCircle(cpy, cvPoint(extended_avg_x,extended_avg_y), 5, CV_RGB(255,255,255),-1,8,0);

            //project into coordinate space
            CvPoint2D32f projected_extension = project(projection, cvPoint2D32f(extended_avg_x*4,extended_avg_y*4));

            //find the dx and dy with respect to the fiducial's center point
            float dx = ((float)projected_extension.x-(float)center.x);
            float dy = ((float)projected_extension.y-(float)center.y);

            float theta = atan2(dy,dx);

            robot_t *robot;
            if (robot_a.id == id){
                robot = &robot_a;
            } else if (robot_b.id == id){
                robot = &robot_b;
            } else { //not a recognized robot id, ignore it
                printf("Ignoring detected robot %i\n", id);
                continue;
            }

            //store robot coordinates
            pthread_mutex_lock( &robot->lock);
            robot->x = clamp(center.x, X_MIN, X_MAX);
            robot->y = clamp(center.y, Y_MIN, Y_MAX);
            robot->theta = theta / PI * 2048; //change theta from +/- PI to +/-2048 (signed 12 bit int)
            pthread_mutex_unlock( &robot->lock);

            if (id == 11){
                printf("X: %04i, Y: %04i, theta: %04i, theta_act: %f, proj_x:%f, proj_y:%f \n", robot->x, robot->y, robot->theta, theta, projected_extension.x, projected_extension.y);
            }

        }

        //make a dot in the registration corner
        cvCircle(cpy, cvPoint(bit_pt_true[0].x, bit_pt_true[0].y), 6, CV_RGB(255,0,0),-1,8,0);
    }


    switch (mouseState) {
        case MOUSE_PROJECT_1:
            cvPutText(cpy, "Init Projection: Click the TOP LEFT corner", cvPoint(2, 20), &font, cvScalar(0,255,0,0));
            break;
        case MOUSE_PROJECT_2:
            cvPutText(cpy, "Init Projection: Click the TOP RIGHT corner", cvPoint(2, 20), &font, cvScalar(0,255,0,0));
            break;
        case MOUSE_PROJECT_3:
            cvPutText(cpy, "Init Projection: Click the BOTTOM RIGHT corner", cvPoint(2, 20), &font, cvScalar(0,255,0,0));
            break;
        case MOUSE_PROJECT_4:
            cvPutText(cpy, "Init Projection: Click the BOTTOM LEFT corner", cvPoint(2, 20), &font, cvScalar(0,255,0,0));
            break;
        default:
            i=0; //HACK to get the compiler to not complain about having a "label" for a declaration
            //draw projection boundary line
            CvPoint imageProjectionPoints[4];  //scale down the projection points since they're all 4x larger than they are on the image
            imageProjectionPoints[0] = cvPoint(projectionPoints[0].x/4,projectionPoints[0].y/4);
            imageProjectionPoints[1] = cvPoint(projectionPoints[1].x/4,projectionPoints[1].y/4);
            imageProjectionPoints[2] = cvPoint(projectionPoints[2].x/4,projectionPoints[2].y/4);
            imageProjectionPoints[3] = cvPoint(projectionPoints[3].x/4,projectionPoints[3].y/4);

            cvLine(cpy, imageProjectionPoints[0], imageProjectionPoints[1], CV_RGB(30,30,200), 2, 8, 0);
            cvLine(cpy, imageProjectionPoints[1], imageProjectionPoints[2], CV_RGB(30,30,200), 2, 8, 0);
            cvLine(cpy, imageProjectionPoints[2], imageProjectionPoints[3], CV_RGB(30,30,200), 2, 8, 0);
            cvLine(cpy, imageProjectionPoints[3], imageProjectionPoints[0], CV_RGB(30,30,200), 2, 8, 0);
            break;
    }

    char scoreString[200];
    sprintf(scoreString, "Score: %i", score);
    cvPutText(cpy, scoreString, cvPoint(5, 440), &font, cvScalar(0,255,255,0));


    if (matchState == MATCH_ENDED){
        cvPutText(cpy, "Match ended.  Press <r> to start a new match.", cvPoint(5, 460), &font, cvScalar(0, 255, 255, 0));
    } else if (matchState == MATCH_RUNNING){
        char timeString[200];
        
        //grab the currentTime
        time_t now;
        time(&now);
        sprintf(timeString, "Remaining time: %i seconds.", MATCH_LEN_SECONDS - (int)(now - matchStartTime));
        
        
        cvPutText(cpy, timeString, cvPoint(5, 460), &font, cvScalar(0,255,255,0));
        
        //draw circle at goal
        CvPoint2D32f goalPt = cvPoint2D32f(goal.x, goal.y);
        goalPt = project(invProjection, goalPt);
        cvCircle(cpy, cvPoint(goalPt.x/4, goalPt.y/4), 6, CV_RGB(0,0,255),-1,8,0);
    }

    // show the resultant image
    cvShowImage( WND_MAIN, cpy );
    cvReleaseImage( &cpy );
}

void* runSerial(void* params){


    packet_buffer position;
    while(1){
       position.type = POSITION;
       position.address = 0xFF;
       
        pthread_mutex_lock( &robot_a.lock );
            position.payload.coords[0].id = robot_a.id;
            position.payload.coords[0].x = robot_a.x;
            position.payload.coords[0].y = robot_a.y;
            position.payload.coords[0].theta = robot_a.theta;
        pthread_mutex_unlock(&robot_a.lock);
        
        pthread_mutex_lock( &robot_b.lock );
            position.payload.coords[1].id = robot_b.id;
            position.payload.coords[1].x = robot_b.x;
            position.payload.coords[1].y = robot_b.y;
            position.payload.coords[1].theta = robot_b.theta;
        pthread_mutex_unlock(&robot_b.lock);
        
        
        //put goal position as object 2:
        position.payload.coords[2].id = 100;
        position.payload.coords[2].x = clamp(goal.x, X_MIN, X_MAX);
        position.payload.coords[2].y = clamp(goal.y, Y_MIN, Y_MAX);


        serial_send_packet(&position);

        if (sendStartPacket){
            packet_buffer startPacket;
            startPacket.type = START;
            startPacket.payload.array[0] = robot_a.id;
            startPacket.payload.array[1] = robot_b.id;

            serial_send_packet(&startPacket);

            sendStartPacket = 0;
        }

        //usleep(50000);
    }
}


int main(int argc, char** argv)
{
    if (!serial_open()){
        fprintf(stderr, "Could not open serial port!\n");
    }
    serial_sync();
    
    
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 2, CV_AA);
    int c;
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


    //initialize mutexes
    pthread_mutex_init(&robot_a.lock, NULL);
    pthread_mutex_init(&robot_b.lock, NULL);

    robot_a.id=7;
    robot_b.id=14;

    //setup camera properties
    /*
    cvSetCaptureProperty(capture, CV_CAP_PROP_BRIGHTNESS, 0.75);
    cvSetCaptureProperty(capture, CV_CAP_PROP_CONTRAST, 1);
    cvSetCaptureProperty(capture, CV_CAP_PROP_SATURATION, 0);
    cvSetCaptureProperty(capture, CV_CAP_PROP_BRIGHTNESS, 0.25);
    //cvSetCaptureProperty(capture, CV_CAP_PROP_EXPOSURE, 0.2);
    //cvSetCaptureProperty(capture, CV_CAP_PROP_GAIN, 0);

    //printf("PROPERTY: %f\n",cvGetCaptureProperty( capture, CV_CAP_PROP_MODE ));
    */
    cvNamedWindow( WND_MAIN, 1 );
    cvNamedWindow( WND_CONTROLS, 1);
    cvResizeWindow( WND_CONTROLS, 200, 400);
    cvNamedWindow( WND_FILTERED, CV_WINDOW_AUTOSIZE);
    cvCreateTrackbar( TRK_THRESHOLD, WND_CONTROLS, &threshold, 255, NULL);
    cvCreateTrackbar( TRK_TOLERANCE, WND_CONTROLS, &side_tolerance, 300, NULL);
    cvCreateTrackbar( TRK_MIN_AREA, WND_CONTROLS, &min_area, 10000, NULL);
    cvCreateTrackbar( TRK_MAX_AREA, WND_CONTROLS, &max_area, 10000, NULL);
    cvCreateTrackbar( TRK_ROBOT_A_ID, WND_CONTROLS, &robot_a.id, MAX_ROBOT_ID-1, NULL);
    cvCreateTrackbar( TRK_ROBOT_B_ID, WND_CONTROLS, &robot_b.id, MAX_ROBOT_ID-1, NULL);

    cvCreateTrackbar( TRK_RAND_GOAL_SEED, WND_CONTROLS, &randomGoalSeed, 5000, NULL);

    //setup mouse handler
    cvSetMouseCallback(WND_MAIN,mouseHandler, NULL);

    printf("To initialize coordinate projection, press <i>\n");

    //start the serial comm thread
    pthread_t serialThread;
    pthread_create( &serialThread, NULL, &runSerial, NULL);


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
        if( (char)c == 27 ){  //ESC
            break;
        } else if ( (char) c == 'i' ){
            mouseState = MOUSE_PROJECT_1;
        } else if ( (char) c == 'r' ){
            time(&matchStartTime); //set the match start time
            matchState = MATCH_RUNNING;
            sendStartPacket = 1; //set flag for start packet to be sent
            reseedRandom();
            pickNewGoal();
            resetScore();
        }


        time_t now;
        time(&now);
        if ((int)(now - matchStartTime) >= MATCH_LEN_SECONDS){
            matchState = MATCH_ENDED;
        } else {
            checkGoals();
        }

    }


    if (projection)
        projection_destroy(projection);

    cvDestroyWindow( WND_MAIN);

    serial_close();

    return 0;
}


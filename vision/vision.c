#include "vision.h"

double matchStartTime;
int matchState = MATCH_ENDED;
int sendStartPacket = 0;   //flag to have a start packet sent ASAP
int score = 0;
CvPoint goal;
robot_t robot_a;
robot_t robot_b;
pthread_mutex_t serial_lock;

float bounds[4] = {X_MIN, X_MAX, Y_MIN, Y_MAX};

#define SHOW_FILTERED_OUTPUT 0

const char *WND_MAIN = "6.270 Vision System";
const char *WND_FILTERED = "Filtered Video";
const char *WND_CONTROLS = "Controls";
const char *TRK_THRESHOLD = "Threshold";
const char *TRK_TOLERANCE = "Side length tolerance";
const char *TRK_MIN_AREA = "Min square area";
const char *TRK_MAX_AREA = "Max square area";
const char *TRK_ROBOT_A_ID = "Robot A id";
const char *TRK_ROBOT_B_ID = "Robot B id";
const char *TRK_RAND_GOAL_SEED = "Random goal seed";

const char *mouseCornerLabel[4] = {"TOP LEFT", "TOP RIGHT", "BOTTOM RIGHT", "BOTTOM LEFT"};

int threshold = 100;
int randomGoalSeed = 1337;
int side_tolerance = 50;
int min_area = 800; // ~ square of (fraction of frame width in 1/1000s)
int max_area = 5800; // will be corrected for resolution

CvFont font;
CvMemStorage *storage;
CvCapture *capture;

CvPoint pickNewGoal() {
    const int inset = 600;
    const int min_sep = 768; //require goals to be at least 18in. (768 ticks) apart
    CvPoint newGoal;

    do {
        int x = boundedRandom(X_MIN+inset, X_MAX-inset);
        int y = boundedRandom(Y_MIN+inset,     0-inset);
        newGoal = cvPoint(x,y);
    } while (dist_sq(&newGoal, &goal) < min_sep*min_sep);
    return newGoal;
}

void checkGoals() {
    CvPoint robotPt = cvPoint(robot_a.x, robot_a.y);
    if (sqrt(dist_sq(&goal, &robotPt)) <= GOAL_TOLERANCE) {
        score++;
        goal = pickNewGoal();
        printf("GOAL!\n");
    }
}

CvMat *projection;
CvMat *invProjection;

CvPoint2D32f projectionPoints[4];

int nextMousePoint = 4;
void mouseHandler(int event, int x, int y, int flags, void *param) {
    CvPoint2D32f point = cvPoint2D32f(x,y);
    if (event == CV_EVENT_LBUTTONDOWN && nextMousePoint < 4) {
        projectionPoints[nextMousePoint++] = point;
        if (nextMousePoint == 4) {
            projection_init(&projection, &invProjection, projectionPoints, bounds);
            printf("project init %s\n", (projection && invProjection) ? "succeeded" : "failed");
        }
    }
}

IplImage *filter_image( IplImage *img ) {
    CvSize sz = cvSize( img->width & -2, img->height & -2 );

    //IplImage *timg = cvCloneImage( img ); // make a copy of input image
    //IplImage *pyr = cvCreateImage( cvSize(sz.width/2, sz.height/2), 8, 3 );
    IplImage *tgray;

    // select the maximum ROI in the image
    // with the width and height divisible by 2
    cvSetImageROI( img, cvRect( 0, 0, sz.width, sz.height ));

    // down-scale and upscale the image to filter out the noise
    //cvPyrDown( timg, pyr, 7 );
    //cvPyrUp( pyr, timg, 7 );
    tgray = cvCreateImage( sz, 8, 1 );

    cvCvtColor(img, tgray, CV_BGR2GRAY);
    //cvReleaseImage( &pyr );
    //cvReleaseImage( &timg );

    return tgray;
}

// returns sequence of squares detected on the image.
// the sequence is stored in the shared memory storage
CvSeq *findCandidateSquares(IplImage *tgray) {
    CvSeq *contours;
    int i;
    CvSize sz = cvSize( tgray->width & -2, tgray->height & -2 );
    IplImage *gray = cvCreateImage( sz, 8, 1 );
    CvSeq *result;
    double s, t;
    // create empty sequence that will contain points -
    // 4 points per square (the square's vertices)
    CvSeq *squares = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvPoint), storage );

    cvThreshold( tgray, gray, threshold, 255, CV_THRESH_BINARY );

#if SHOW_FILTERED_OUTPUT
    cvShowImage( WND_FILTERED, gray );
#endif

    // find contours and store them all as a list
    cvFindContours( gray, storage, &contours, sizeof(CvContour),
            CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );

    // test each contour
    while( contours ) {
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
                cvCheckContourConvexity(result)) {
            s = 0;

            for( i = 2; i < 5; i++ ) {
                // find minimum angle between joint
                // edges (maximum of cosine)
                t = fabs(cosAngle((CvPoint*)cvGetSeqElem( result, i ),
                            (CvPoint*)cvGetSeqElem( result, i-2 ),
                            (CvPoint*)cvGetSeqElem( result, i-1 )));
                s = s > t ? s : t;
            }

            // if cosines of all angles are small (angles are ~90 degrees)
            if( s < 0.3 ) {
                CvPoint pt[4];
                for (i=0; i<4; i++)
                    pt[i] = *(CvPoint*)cvGetSeqElem(result, 3-i);

                // calculate the length of each side
                double side_len[4];
                side_len[0] = dist_sq(&pt[0],&pt[1]);
                side_len[1] = dist_sq(&pt[1],&pt[2]);
                side_len[2] = dist_sq(&pt[2],&pt[3]);
                side_len[3] = dist_sq(&pt[3],&pt[0]);

                double tolerance = (double)side_tolerance / 100.;
                // check to make sure all sides are approx. the same length as side 0
                if (fabs(side_len[0] - side_len[1])/side_len[0] <= tolerance &&
                        fabs(side_len[0] - side_len[2])/side_len[0] <= tolerance &&
                        fabs(side_len[0] - side_len[3])/side_len[0] <= tolerance) {
                    // then write quandrange vertices to resultant sequence in clockwise order
                    for( i = 0; i < 4; i++ )
                        cvSeqPush( squares, &pt[i] );
                }
            }
        }

        // take the next contour
        contours = contours->h_next;
    }
    // release all the temporary images
    cvReleaseImage( &gray );
    return squares;
}

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

void drawSquare(IplImage *img, CvPoint pt[4], CvPoint2D32f bit_pt_true[16], int id, CvPoint2D32f orientationHandle) {
    // draw the square as a closed polyline
    CvPoint *rect = pt;
    int count = 4;
    cvPolyLine(img, &rect, &count, 1, 1, CV_RGB(0,0,255), 2, CV_AA, 0 );

    if (id != -1) {
        // for debugging, draw a dot over each bit location
        cvCircle(img, cvPoint(bit_pt_true[5].x, bit_pt_true[5].y), 3, CV_RGB(255,0,0),-1,8,0);
        cvCircle(img, cvPoint(bit_pt_true[6].x, bit_pt_true[6].y), 3, CV_RGB(0,255,0),-1,8,0);
        cvCircle(img, cvPoint(bit_pt_true[9].x, bit_pt_true[9].y), 3, CV_RGB(0,0,255),-1,8,0);
        cvCircle(img, cvPoint(bit_pt_true[10].x, bit_pt_true[10].y), 3, CV_RGB(255,0,255),-1,8,0);

        // Show the robot's ID next to it
        CvPoint center = cvPoint((pt[0].x + pt[1].x + pt[2].x + pt[3].x)/4,(pt[0].y + pt[1].y + pt[2].y + pt[3].y)/4);
        cvPrintf(img, cvPoint(center.x-20, center.y+50), CV_RGB(255,255,0), "Robot %i", id);

        // draw a white circle at the extended point
        cvCircle(img, cvPoint(orientationHandle.x,orientationHandle.y), 5, CV_RGB(255,255,255),-1,8,0);

        //make a dot in the registration corner
        cvCircle(img, cvPoint(bit_pt_true[0].x, bit_pt_true[0].y), 6, CV_RGB(255,0,0),-1,8,0);
    }
}

void getBitSamplingTransform(CvPoint pt[4], CvMat **H) {
    const float l = -.5, r = 4.5;
    CvPoint2D32f src[4] = {{l,l},{r,l},{r,r},{l,r}};
    CvPoint2D32f dst[4] = {{pt[0].x,pt[0].y},{pt[1].x,pt[1].y},{pt[2].x,pt[2].y},{pt[3].x,pt[3].y}};
    *H = cvCreateMat(3,3,CV_32FC1);
    *H = cvGetPerspectiveTransform(src, dst, *H);
}

int getOrientationFromBits(int bit_raw[16], int *orientation) {
    int corner[4];
    corner[0] = bit_raw[0];
    corner[1] = bit_raw[3];
    corner[2] = bit_raw[15];
    corner[3] = bit_raw[12];

    // registration corner is the white corner whose clockwise neighbor is black
    *orientation = -1;
    for (int j=0; j<4; j++) {
        if(corner[j] && !corner[(j+1) % 4]) {
            if (*orientation != -1) { // corner is ambiguous
                *orientation = -1;
                break;
            }
            *orientation = j;
        }
    }
    return *orientation != -1;
}

int getIDFromBits(int bit_true[16], int *id) {
    *id = (bit_true[5] << 0) + (bit_true[6] << 1) + (bit_true[9] << 2) + (bit_true[10] << 3) +
        (bit_true[1] << 4) + (bit_true[2] << 5) + (bit_true[4] << 6) + (bit_true[7] << 7) +
        (bit_true[8] << 8) + (bit_true[11] << 9) + (bit_true[13] << 10) + (bit_true[14] << 11) +
        ((bit_true[12] + bit_true[15]) << 12);
    return 1;
}

void rotateBitsToOrientation(CvPoint2D32f bit_pt_raw[16], int bit_raw[16], int orientation, CvPoint2D32f bit_pt_true[16], int bit_true[16]) {
    // shift indices so that orientation -> 0
    for (int j=0; j<16; j++) {
        int x = j%4, y=j/4;
        int xp, yp;
        switch (orientation) {
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
}

int readPattern(IplImage *img, CvPoint pt[4], CvPoint2D32f bit_pt_true[16], int *id) {
    CvPoint2D32f bit_pt_raw[16];
    int bit_raw[16], bit_true[16];

    CvMat *H;
    getBitSamplingTransform(pt, &H);
    //calculate the coordinates of each bit
    for (int j=0; j<16; j++)
        bit_pt_raw[j] = cvPoint2D32f(.5 + j%4, .5 + j/4);
    CvMat pts = cvMat(1, 16, CV_32FC2, bit_pt_raw);
    cvPerspectiveTransform(&pts, &pts, H);
    for (int j=0; j<16; j++)
        bit_raw[j] = (get_5pixel_avg(img, bit_pt_raw[j].x, bit_pt_raw[j].y) > threshold);
    cvReleaseMat(&H);

    int orientation;
    if (!getOrientationFromBits(bit_raw, &orientation)) return 0;
    rotateBitsToOrientation(bit_pt_raw, bit_raw, orientation, bit_pt_true, bit_true);
    if (!getIDFromBits(bit_true, id)) return 0;
    return 1;
}

void getCenterFromBits(CvPoint2D32f bit_pt_true[16], CvPoint2D32f *trueCenter) {
    CvPoint2D32f rawCenter = cvPoint2D32f((bit_pt_true[0].x + bit_pt_true[3].x + bit_pt_true[15].x + bit_pt_true[12].x)/4,
            (bit_pt_true[0].y + bit_pt_true[3].y + bit_pt_true[15].y + bit_pt_true[12].y)/4);
    *trueCenter = project(projection, rawCenter);
}

float getThetaFromAffine(CvPoint2D32f bit_pt_true[16]) {
    const float l = 0.5, r = 3.5;
    CvPoint2D32f src[4] = {{l,l},{r,l},{r,r},{l,r}};
    CvPoint2D32f dst[4] = {bit_pt_true[0],bit_pt_true[3],bit_pt_true[15],bit_pt_true[12]};
    CvMat *A = cvCreateMat(2,3,CV_32FC1);

    CvMat srcM = cvMat(1, 4, CV_32FC2, src);
    CvMat dstM = cvMat(1, 4, CV_32FC2, dst);
    cvEstimateRigidTransform(&srcM, &dstM, A, 1);

    // use affine approximation and SVD to determine angle
    float A22_mat[2][2] = {{cvmGet(A, 0, 0),cvmGet(A, 0, 1)},{cvmGet(A, 1, 0),cvmGet(A, 1, 1)}};
    CvMat A22  = cvMat(2,2,CV_32FC1,A22_mat);
    float U_mat[2][2], W_mat[2][2], V_mat[2][2], R_mat[2][2];
    CvMat U = cvMat(2,2,CV_32FC1,U_mat);
    CvMat W = cvMat(2,2,CV_32FC1,W_mat);
    CvMat V = cvMat(2,2,CV_32FC1,V_mat);
    CvMat R = cvMat(2,2,CV_32FC1,R_mat);
    cvSVD(&A22, &W, &U, &V, CV_SVD_U_T|CV_SVD_V_T); // A = U D V^T
    cvTranspose(&U, &U);
    cvMatMulAdd(&U, &V, 0, &R);
    float theta = atan2(R_mat[1][0], R_mat[0][0]);
    cvReleaseMat(&A);
    return theta;
}

float getThetaFromExtension(CvPoint2D32f bit_pt_true[16], CvPoint2D32f trueCenter) {
    //to find the heading, "extend" the top and bottom edges 4x to the right and take
    //  the average endpoint, then project this and take the dx and dy in the projected
    //  space to find the angle it makes

    fiducial_t fiducial;
    fiducial.corners[0] = bit_pt_true[0];
    fiducial.corners[1] = bit_pt_true[3];
    fiducial.corners[2] = bit_pt_true[15];
    fiducial.corners[3] = bit_pt_true[12];

    int extended_top_x = fiducial.corners[TR].x + (fiducial.corners[TR].x - fiducial.corners[TL].x)*4;
    int extended_top_y = fiducial.corners[TR].y + (fiducial.corners[TR].y - fiducial.corners[TL].y)*4;

    int extended_bottom_x = fiducial.corners[BR].x + (fiducial.corners[BR].x - fiducial.corners[BL].x)*4;
    int extended_bottom_y = fiducial.corners[BR].y + (fiducial.corners[BR].y - fiducial.corners[BL].y)*4;

    int extended_avg_x = (extended_top_x+extended_bottom_x)/2;
    int extended_avg_y = (extended_top_y+extended_bottom_y)/2;

    //project into coordinate space
    CvPoint2D32f projected_extension = project(projection, cvPoint2D32f(extended_avg_x,extended_avg_y));

    //find the dx and dy with respect to the fiducial's center point
    float dx = ((float)projected_extension.x-(float)trueCenter.x);
    float dy = ((float)projected_extension.y-(float)trueCenter.y);

    return atan2(dy,dx);
}

void processRobotDetection(CvPoint2D32f trueCenter, float theta, int id, CvPoint2D32f *orientationHandle) {
    *orientationHandle = cvPoint2D32f(trueCenter.x + FOOT*cos(theta), trueCenter.y + FOOT*sin(theta));
    *orientationHandle = project(invProjection, *orientationHandle);

    robot_t *robot;
    if (robot_a.id == id)
        robot = &robot_a;
    else if (robot_b.id == id)
        robot = &robot_b;
    else {
        //not a recognized robot id, ignore it
        printf("Ignoring detected robot %i\n", id);
        return;
    }

    //store robot coordinates
    pthread_mutex_lock( &serial_lock);
    robot->x = clamp(trueCenter.x, X_MIN, X_MAX);
    robot->y = clamp(trueCenter.y, Y_MIN, Y_MAX);
    robot->theta = theta / M_PI * 2048; //change theta from +/- PI to +/-2048 (signed 12 bit int)
    pthread_mutex_unlock( &serial_lock);

    if (id == 11)
        printf("X: %04i, Y: %04i, theta: %04i, theta_act: %f, proj_x:%f, proj_y:%f \n", robot->x, robot->y, robot->theta, theta, orientationHandle->x, orientationHandle->y);
}

void updateHUD(IplImage *img) {
    static double last_frame = 0.0;
    static float last_fps = 0.0;
    double now = timeNow();
    float fps = 1.0/(now-last_frame);
    last_frame = now;
    fps = (10*last_fps + fps) / 11.;
    last_fps = fps;

    if (nextMousePoint!=4)
        cvPrintf(img, cvPoint(2, 20), CV_RGB(0,255,0), "Init Projection: Click the %s corner", mouseCornerLabel[nextMousePoint]);
    else if (projection) {
        CvPoint corners[4], *rect = corners;
        int cornerCount = 4;
        for (int i=0; i<4; i++)
            corners[i] = cvPoint(projectionPoints[i].x, projectionPoints[i].y);
        cvPolyLine(img, &rect, &cornerCount, 1, 1, CV_RGB(30,30,200), 2, 8, 0);
    }

    cvPrintf(img, cvPoint(5, img->height-40), CV_RGB(255,255,0), "Score: %i", score);

    CvPoint textPoint = cvPoint(5, img->height-20);
    CvScalar textColor = CV_RGB(255,255,0);
    if (matchState == MATCH_ENDED)
        cvPrintf(img, textPoint, textColor, "Match ended.  Press <r> to start a new match.  %.1f FPS", fps);
    else if (matchState == MATCH_RUNNING) {
        cvPrintf(img, textPoint, textColor, "Remaining time: 00:%6.3f seconds.  %.1f FPS", MATCH_LEN_SECONDS - (now - matchStartTime), fps);

        //draw circle at goal
        CvPoint2D32f goalPt = cvPoint2D32f(goal.x, goal.y);
        goalPt = project(invProjection, goalPt);
        cvCircle(img, cvPoint(goalPt.x, goalPt.y), 6, CV_RGB(0,0,255),-1,8,0);
    }
}

// detects robots and draws the HUD
void processSquares( IplImage *img, IplImage *grayscale, CvSeq *squares ) {
    CvSeqReader reader;
    IplImage *cpy = cvCloneImage( img );

    CvPoint pt[4];
    CvPoint2D32f bit_pt_true[16], trueCenter, orientationHandle;
    int id;
    float theta;

    // initialize reader of the sequence
    cvStartReadSeq( squares, &reader, 0 );
    // read 4 sequence elements at a time (all vertices of a square)
    for(int i=0; i<squares->total; i+=4) {
        for (int j=0; j<4; j++)
            CV_READ_SEQ_ELEM( pt[j], reader );

        if (!readPattern(img, pt, bit_pt_true, &id)) continue;

        getCenterFromBits(bit_pt_true, &trueCenter);

        if (0)
            theta = getThetaFromAffine(bit_pt_true);
        else
            theta = getThetaFromExtension(bit_pt_true, trueCenter);
        processRobotDetection(trueCenter, theta, id, &orientationHandle);

        drawSquare(cpy, pt, bit_pt_true, id, orientationHandle);
    }

    updateHUD(cpy);

    // show the resultant image
    cvShowImage( WND_MAIN, cpy );
    cvReleaseImage( &cpy );
}

void *runSerial(void *params){
    packet_buffer position;
    while(1) {
        position.type = POSITION;
        position.address = 0xFF;

        pthread_mutex_lock( &serial_lock );
        position.payload.coords[0].id = robot_a.id;
        position.payload.coords[0].x = robot_a.x;
        position.payload.coords[0].y = robot_a.y;
        position.payload.coords[0].theta = robot_a.theta;

        position.payload.coords[1].id = robot_b.id;
        position.payload.coords[1].x = robot_b.x;
        position.payload.coords[1].y = robot_b.y;
        position.payload.coords[1].theta = robot_b.theta;
        pthread_mutex_unlock(&serial_lock);

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

int initSerial(const char *device) {
    if (!serial_open(device))
        fprintf(stderr, "Could not open serial port!\n");
    serial_sync();

    //initialize mutexes
    pthread_mutex_init(&serial_lock, NULL);

    //start the serial comm thread
    pthread_t serialThread;
    pthread_create( &serialThread, NULL, &runSerial, NULL);

    return 0;
}

void cleanupSerial() {
    serial_close();
}

int initUI() {
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 2, CV_AA);
    cvNamedWindow( WND_MAIN, 1 );
    cvNamedWindow( WND_CONTROLS, 1);
    cvResizeWindow( WND_CONTROLS, 200, 400);
#if SHOW_FILTERED_OUTPUT
    cvNamedWindow( WND_FILTERED, CV_WINDOW_AUTOSIZE);
#endif
    cvCreateTrackbar( TRK_THRESHOLD, WND_CONTROLS, &threshold, 255, NULL);
    cvCreateTrackbar( TRK_TOLERANCE, WND_CONTROLS, &side_tolerance, 300, NULL);
    cvCreateTrackbar( TRK_MIN_AREA, WND_CONTROLS, &min_area, 10000, NULL);
    cvCreateTrackbar( TRK_MAX_AREA, WND_CONTROLS, &max_area, 10000, NULL);
    cvCreateTrackbar( TRK_ROBOT_A_ID, WND_CONTROLS, &robot_a.id, MAX_ROBOT_ID-1, NULL);
    cvCreateTrackbar( TRK_ROBOT_B_ID, WND_CONTROLS, &robot_b.id, MAX_ROBOT_ID-1, NULL);
    cvCreateTrackbar( TRK_RAND_GOAL_SEED, WND_CONTROLS, &randomGoalSeed, 5000, NULL);

    cvSetMouseCallback(WND_MAIN,mouseHandler, NULL);

    return 0;
}

void cleanupUI() {
    cvDestroyWindow( WND_MAIN);
}

float frameWidth;
float frameHeight;
int initCV(char *source) {
    // create memory storage for contours
    storage = cvCreateMemStorage(0);

    capture = 0;

    int i = 0;
    if (source && sscanf(source, "%d", &i) != 1)
        i = -1;

    if (i!=-1)
        capture = cvCaptureFromCAM(i);
    else if (source)
        capture = cvCaptureFromAVI( source );

    if( !capture ) {
        fprintf(stderr,"Could not initialize capturing...\n");
        return -1;
    }

    /*
    //setup camera properties
    cvSetCaptureProperty(capture, CV_CAP_PROP_BRIGHTNESS, 0.75);
    cvSetCaptureProperty(capture, CV_CAP_PROP_CONTRAST, 100);
    cvSetCaptureProperty(capture, CV_CAP_PROP_SATURATION, 0);
    cvSetCaptureProperty(capture, CV_CAP_PROP_BRIGHTNESS, 0.25);
    cvSetCaptureProperty(capture, CV_CAP_PROP_EXPOSURE, 0.2);
    cvSetCaptureProperty(capture, CV_CAP_PROP_GAIN, 0);

    //printf("PROPERTY: %f\n",cvGetCaptureProperty( capture, CV_CAP_PROP_MODE ));
    */
    frameWidth = cvGetCaptureProperty(capture,CV_CAP_PROP_FRAME_WIDTH);
    frameHeight = cvGetCaptureProperty(capture,CV_CAP_PROP_FRAME_HEIGHT);

    cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, 960);
    cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, 720);
    frameWidth = 960;
    frameHeight = 720;
    printf("%f, %f\n", frameWidth, frameHeight);
    min_area *= (frameWidth*frameWidth)/(1000*1000);
    max_area *= (frameWidth*frameWidth)/(1000*1000);

    return 0;
}

void cleanupCV() {
    if (projection)
        cvReleaseMat(&projection);
    if (invProjection)
        cvReleaseMat(&invProjection);
}

int initGame() {
    robot_a.id=14;
    robot_b.id=7;
    return 0;
}

int handleKeypresses() {
    char c = cvWaitKey(5); // cvWaitKey takes care of event processing
    if( c == 27 )  //ESC
        return -1;
    else if ( c == 'i' || (c == 'r' && (!projection || nextMousePoint != 4)))
        nextMousePoint = 0;
    else if ( c == 'r' ) {
        matchStartTime = timeNow(); //set the match start time
        matchState = MATCH_RUNNING;
        sendStartPacket = 1; //set flag for start packet to be sent
        srand(randomGoalSeed); // reseed random
        goal = cvPoint(X_MIN, Y_MIN); // don't let pickNewGoal choose a point near the start
        goal = pickNewGoal();
        score = 0;
    }
    return 0;
}

void updateGame() {
    double now = timeNow();
    if ((now - matchStartTime) >= MATCH_LEN_SECONDS)
        matchState = MATCH_ENDED;
    else
        checkGoals();
}

int main(int argc, char** argv) {
    if (initSerial(argc>2 ? argv[2] : NULL)) return -1;
    if (initUI()) return -1;
    if (initCV(argc>1 ? argv[1] : NULL)) return -1;
    if (initGame()) return -1;

    printf("To initialize coordinate projection, press <i>\n");

    projectionPoints[0] = cvPoint2D32f(0, 0);
    projectionPoints[1] = cvPoint2D32f(frameWidth, 0);
    projectionPoints[2] = cvPoint2D32f(frameWidth, frameHeight);
    projectionPoints[3] = cvPoint2D32f(0, frameHeight);
    projection_init(&projection, &invProjection, projectionPoints, bounds);
    while(1) {
        IplImage *frame = cvQueryFrame( capture );
        if( !frame ) {
            fprintf(stderr,"cvQueryFrame failed!\n");
            continue;
        }

        IplImage *img = cvCloneImage(frame); // can't modify original
        IplImage *grayscale = filter_image(img);
        CvSeq *squares = findCandidateSquares(grayscale);
        processSquares(img, grayscale, squares);

        if (handleKeypresses())
            break;
        updateGame();

        cvReleaseImage(&grayscale);
        cvReleaseImage(&img);
        cvClearMemStorage(storage); // clear memory storage - reset free space position
    }

    cleanupCV();
    cleanupUI();
    cleanupSerial();

    return 0;
}

#include "util.h"

int clamp(int x, int low, int high) {
    return x < low ? low : (x > high ? high : x);
}

// returns the squared euclidian distance between two points
double dist_sq(CvPoint *a, CvPoint *b) {
    double dx = a->x - b->x;
    double dy = a->y - b->y;
    return dx*dx+dy*dy;
}

int boundedRandom(int min, int max) {
    return min + (int)((rand()/(float)RAND_MAX)*(max-min));
}

int get_5pixel_avg(IplImage *img, int x, int y) {
    float sum = 0;
    int num = 0;

    sum += cvGet2D(img, y, x).val[0];
    num ++;

    if (x-1 > 0) {
        sum += cvGet2D(img, y, x-1).val[0];
        num++;
    }
    if (x+1 < img->width) {
        sum += cvGet2D(img, y, x+1).val[0];
        num++;
    }
    if (y-1 > 0) {
        sum += cvGet2D(img, y-1, x).val[0];
        num++;
    }
    if (y+1 < img->height) {
        sum += cvGet2D(img, y+1, x).val[0];
        num++;
    }
    return sum/num;
}

double timeNow() {
    struct timeval t;
    gettimeofday(&t, NULL);
    return t.tv_sec + .000001 * t.tv_usec;
}

// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
double cosAngle( CvPoint *pt1, CvPoint *pt2, CvPoint *pt0 ) {
    double dx1 = pt1->x - pt0->x;
    double dy1 = pt1->y - pt0->y;
    double dx2 = pt2->x - pt0->x;
    double dy2 = pt2->y - pt0->y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

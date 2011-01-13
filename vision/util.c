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

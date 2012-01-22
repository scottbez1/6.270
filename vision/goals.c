#include "goals.h"
#include <stdlib.h>
#include "util.h"

static int score = 0;

static CvPoint goal;

void pickNewGoal();



void reseedRandom(int seed){
    goal = cvPoint(X_MIN, Y_MIN);
    srand(seed);
}

void resetScore(){
    score = 0;
}

void resetRound(int randomSeed) {
    resetScore();
    reseedRandom(randomSeed);
    pickNewGoal();
}

int getScore() {
    return score;
}

CvPoint getGoal() {
    return goal;
}


void pickNewGoal(){
    CvPoint newGoal;

    do {
        int x = boundedRandom(-2048+600, 2047-600);
        int y = boundedRandom(-2048+600, 2047-600);
        newGoal = cvPoint(x,y);

    } while (dist_sq(newGoal, goal) < 1024*1024); //require goals to be at least 18in. (1024 ticks) apart
    goal = newGoal;
}

void checkGoals(int x, int y){
    CvPoint robotPt = cvPoint(x, y);
    if (sqrt(dist_sq(goal, robotPt)) <= GOAL_TOLERANCE){
        score++;
        pickNewGoal();
        printf("GOAL!\n");
    }
}

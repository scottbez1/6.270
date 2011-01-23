#ifndef _GAME_H_INC_
#define _GAME_H_INC_

#define MATCH_LEN_SECONDS 120

#define MATCH_ENDED 0
#define MATCH_RUNNING 1

//size of space to project onto
#define X_MIN -2048
#define X_MAX 2047

#define Y_MIN -2048
#define Y_MAX 2048

#define FOOT 512.0

#define GOAL_TOLERANCE (FOOT/6.)

#define MAX_ROBOT_ID 32

#endif

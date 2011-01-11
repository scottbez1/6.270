cc = gcc
cxx = g++

UNAME := $(shell uname)
ifeq ($(UNAME), Linux)
location=/usr/local
libs=-lm -lcv -lcxcore -lhighgui -lcvaux
endif
ifeq ($(UNAME), Darwin)
location=/usr
libs=-lm -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_video -lopencv_calib3d
endif


all:
	$(cc) -ggdb projection.c serial.c vid_squares.c -o vision -I $(location)/include/opencv/ -L$(location)/lib $(libs)

calibrate: calibrate.cc
	$(cxx) -ggdb calibrate.cc -o calibrate -I $(location)/include/opencv/ -L$(location)/lib $(libs)

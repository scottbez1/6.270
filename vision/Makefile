CC = gcc
CXX = g++
RM = rm
MAKE = make
CFLAGS = -std=c99 $(DEBUG)
CXXFLAGS = $(DEBUG)
HEADERS = vision.h projection.h serial.h util.h game.h
VISION_OBJS = projection.o serial.o vision.o util.o
CALIBRATE_OBJS = calibrate.o

UNAME := $(shell uname)
ifeq ($(UNAME), Linux)
location=/usr/local
libs=-lm -lcv -lcxcore -lhighgui -lcvaux
endif
ifeq ($(UNAME), Darwin)
location=/usr
libs=-lm -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_video -lopencv_calib3d
endif

CFLAGS += -I $(location)/include/opencv/
CXXFLAGS += -I $(location)/include/opencv/

%.o : %.c $(HEADERS)
	$(CC) $(CFLAGS) -c -o $@ $<

%.o : %.cc
	$(CXX) $(CXXFLAGS) -c -o $@ $<

all: vision calibrate

vision: $(VISION_OBJS)
	$(CC) $(CFLAGS) $(VISION_OBJS) -o $@ -I $(location)/include/opencv/ -L$(location)/lib $(libs)

debug:
	$(MAKE) DEBUG="-ggdb -D DEBUG"

calibrate: $(CALIBRATE_OBJS)
	$(CXX) $(CXXFLAGS) $(CALIBRATE_OBJS) -o $@ -L$(location)/lib $(libs)

.PHONY: clean
clean:
	$(RM) -r *.o calibrate vision

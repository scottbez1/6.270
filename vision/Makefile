CC = gcc
CXX = g++
RM = rm
MAKE = make
CFLAGS = -std=c99 $(DEBUG)
CXXFLAGS = $(DEBUG)
HEADERS = vision.h projection.h serial.h util.h game.h packet.h
VISION_OBJS = projection.o serial.o vision.o util.o
CALIBRATE_OBJS = calibrate.o
OUTPUTS = vision calibrate

UNAME := $(shell uname)
ifeq ($(UNAME), Linux)
LOCATION=/usr/local
LIBS=-lm -lcv -lcxcore -lhighgui -lcvaux
endif
ifeq ($(UNAME), Darwin)
LOCATION=/usr
LIBS=-lm -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_video -lopencv_calib3d
CFLAGS += -D DARWIN
endif

CFLAGS += -I $(LOCATION)/include/opencv/
CXXFLAGS += -I $(LOCATION)/include/opencv/

%.o : %.c $(HEADERS)
	$(CC) $(CFLAGS) -c -o $@ $<

%.o : %.cc
	$(CXX) $(CXXFLAGS) -c -o $@ $<

all: $(OUTPUTS)

vision: $(VISION_OBJS)
	$(CC) $(CFLAGS) $(VISION_OBJS) -o $@ -I $(LOCATION)/include/opencv/ -L$(LOCATION)/lib $(LIBS)

debug:
	$(MAKE) DEBUG="-ggdb -D DEBUG"

calibrate: $(CALIBRATE_OBJS)
	$(CXX) $(CXXFLAGS) $(CALIBRATE_OBJS) -o $@ -L$(LOCATION)/lib $(LIBS)

.PHONY: clean
clean:
	$(RM) -r *.o $(OUTPUTS)

cc = gcc

UNAME := $(shell uname)
ifeq ($(UNAME), Linux)
location=/usr/local
endif
ifeq ($(UNAME), Darwin)
location=/opt/local
endif


all:
	$(cc) -Wall projection.c serial.c vid_squares.c -o vision -I $(location)/include/opencv/ -L$(location)/lib -lm -lcv -lcxcore -lhighgui -lcvaux

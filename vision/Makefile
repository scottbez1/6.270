cc = gcc

UNAME := $(shell uname)
ifeq ($(UNAME), Linux)
location=/usr/local
endif
ifeq ($(UNAME), Darwin)
location=/opt/local
endif


all:
	$(cc) -Wall serial.c vid_squares.c -o vid_squares -I $(location)/include/opencv/ -L$(location)/lib -lm -lcv -lcxcore -lhighgui -lcvaux

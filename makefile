OBJS = simple.o nnlib.o roboteq.o joystick.o
CC = gcc
CFLAGS = -Wall -c
LFLAGS = -Wall -lwinmm

testrobot : $(OBJS)
	$(CC) $(OBJS) $(LFLAGS) -o testrobot

simple.o : simple.c nnlib.h roboteq.h joystick.h
	$(CC) $(CFLAGS) simple.c

roboteq.o : roboteq.h roboteq.c nnlib.h
	$(CC) $(CFLAGS) roboteq.c

nnlib.o : nnlib.h nnlib.c
	$(CC) $(CFLAGS) nnlib.c

joystick.o : joystick.h joystick.c
	$(CC) $(CFLAGS) $(LFLAGS) joystick.c

clean :
	\rm *.o *~ testrobot
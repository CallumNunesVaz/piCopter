LDLIBS=-lbcm2835
CFLAGS=-Wall -g

SRC=$(wildcard libs/*.c) cli/main.c
MAIN_EXEC=piCopter.run 

default:
	gcc $(SRC) -o $(MAIN_EXEC) 


LDLIBS=-lbcm2835
CFLAGS=-Wall -g

SRC=$(wildcard libs/*.c) cli/main.c
MAIN_EXEC=piCopter.run 

default:
	sudo gcc $(SRC) -o $(MAIN_EXEC) $(LDLIBS)
	@echo "Build Complete! :)"

clean:
	sudo rm piCopter.run
	@echo "Clean Complete! :)"

run:
	sudo ./piCopter.run

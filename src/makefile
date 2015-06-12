IDIR =../include
CC=gcc
CFLAGS=-I$(IDIR)

ODIR=obj
LDIR =../lib

LIBS=-lbcm2835

_DEPS = piCopter.h ak8975.h mpu9150.h  easySerialComms.h  mpl3115a2.h

DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = piCopter.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))


$(ODIR)/%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

piCopter: $(OBJ)
	gcc -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~

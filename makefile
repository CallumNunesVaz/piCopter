LIBS=libs/ak8975 libs/easySerialComms libs/mpl3115a2 libs/mpu9150
INCLUDE_PATH=libs/

cli_exec: libraries
	gcc cli/main.c $(wildcard libraries/*) -I$(INCLUDE_PATH) -o piCopter.run -l bcm2835


libraries:
	mkdir libraries/
	for dir in $(LIBS); do \
		cd $$dir; \
		gcc -c *.c -I../ -l bcm2835; \
		mv *.o ../../libraries; \
		cd -; \
	done

clean:
	rm -rf libraries/ cli_exec
	

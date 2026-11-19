CC ?= gcc
INC = -I../firmware/Core/Inc
SRC = ../firmware/Core/Src/coupling.c ../firmware/Core/Src/pid.c

.PHONY: test test-c clean

test:
	python3 test_firmware.py

test-c:
	$(CC) -std=c99 -Wall $(INC) test_algorithm.c $(SRC) -lm -o test_algorithm
	./test_algorithm

clean:
	rm -f test_algorithm

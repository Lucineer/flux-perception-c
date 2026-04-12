CC ?= gcc
CFLAGS = -std=c11 -Wall -Wextra -Wpedantic -O2
AR ?= ar

all: libperception.a test

perception.o: perception.c perception.h
	$(CC) $(CFLAGS) -c perception.c -o perception.o

libperception.a: perception.o
	$(AR) rcs libperception.a perception.o

test: test_perception.c perception.c perception.h
	$(CC) $(CFLAGS) -o test_perception test_perception.c perception.c -lm
	./test_perception

clean:
	rm -f *.o *.a test_perception

.PHONY: all clean test

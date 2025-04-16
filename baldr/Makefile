CC=g++

CFLAGS  = -std=c++17 -Wall -Wextra -g -O3 -ggdb -I/Documents/dcs/libImageStreamIO -I/usr/include/eigen3
#pthread is only needed for ImageStreamIO
LDFLAGS = -lcommander -lm -lzmq -lImageStreamIO -lb64 -lboost_program_options -lfmt -lpthread -lcfitsio
EXEC    = baldr
OBJECTS = baldr.o rtc.o telemetry.o

# PREFIX is environment variable, but if it is not set, then set default value
ifeq ($(PREFIX),)
    PREFIX := /usr/local
endif

# the makefile instructions

all: $(EXEC)

baldr: $(OBJECTS)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.cpp 
	$(CC) -o $@ -c $< $(CFLAGS)

clean:
	rm -rf *.o *.so
	rm -rf *~

mrproper: clean
	rm -rf $(EXEC)

install:
	install -D $(EXEC) $(PREFIX)/bin/

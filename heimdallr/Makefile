CC=g++

CFLAGS  = -std=c++17 -Wall -Wextra -ggdb -O1
LDFLAGS = -lcommander -lm -lzmq -lImageStreamIO -lb64 -lpthread -lboost_program_options -lfmt -lfftw3  
EXEC    = heimdallr
OBJECTS = heimdallr.o forward_ft.o fringe_tracker.o

# PREFIX is environment variable, but if it is not set, then set default value
ifeq ($(PREFIX),)
    PREFIX := /usr/local
endif

# the makefile instructions

all: $(EXEC)

heimdallr: $(OBJECTS)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.cpp heimdallr.h
	$(CC) -o $@ -c $< $(CFLAGS)

clean:
	rm -rf *.o *.so
	rm -rf *~

mrproper: clean
	rm -rf $(EXEC)

install:
	install -D $(EXEC) $(PREFIX)/bin/

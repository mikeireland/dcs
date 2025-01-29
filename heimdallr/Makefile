CC=g++

CFLAGS  = -std=c++17 -Wall -ggdb 
LDFLAGS = -lcommander -lm -lzmq -lImageStreamIO -lpthread -lboost_program_options -lfmt -lfftw3
EXEC    = heimdallr
OBJECTS = heimdallr.o

# PREFIX is environment variable, but if it is not set, then set default value
ifeq ($(PREFIX),)
    PREFIX := /usr/local
endif

# the makefile instructions

all: $(EXEC)

heimdallr: $(OBJECTS)
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

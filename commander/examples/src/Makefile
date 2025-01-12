CC=g++
CFLAGS=-std=c++17 -I../../include
BIN=client example test
LFLAGS=-L../../../lib -lcommander -lboost_program_options -lfmt -lzmq

all: $(BIN)

%: %.cpp
	$(CC) -o $@ $< $(CFLAGS) $(LFLAGS)
	
clean:
	rm -f $(BIN)
	


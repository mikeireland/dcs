CC=g++
CFLAGS=-std=c++17 -I../include
OBJ=server/interactive.o server/single_command.o server/socket.o client/socket.o registry.o server.o

all: $(OBJ)
	ar rcs ../../lib/libcommander.a $(OBJ)

%.o: %.cpp
	$(CC) -c -o $@ $< $(CFLAGS)
	
clean:
	rm -f *.o
	rm -f server/*.o
	rm -f clieng/*.o
	


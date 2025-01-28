CC=g++

# FYI: it turns out that the order in which the libraries are linked matters!
# they probably are linked in the reverse order, and lpthread should be after
# ImageStreamIO.

# Do I reall need to link to boost_program_options?

CFLAGS  = -std=c++17 -W -Wall -ggdb 
LDFLAGS = -lcommander -lm -lzmq -lImageStreamIO -lpthread -lboost_program_options -lfmt 
EXEC    = asgard_cam_server
OBJECTS = asgard_ZMQ_CRED1_server.o

# PREFIX is environment variable, but if it is not set, then set default value
ifeq ($(PREFIX),)
    PREFIX := /usr/local
endif

# adding the BMC specific things as they are setup on this machine

# CFLAGS  += -I $(BMC_INCDIR)
# LDFLAGS += -L $(BMC_LIBDIR) -lBMC -lBMC_PCIeAPI -lBMC_USBAPI

# adding the PCI EDT specific compilation options
CFLAGS  += -I /opt/EDTpdv
LDFLAGS += -L /opt/EDTpdv -lpdv

# the makefile instructions

all: $(EXEC)


asgard_cam_server: $(OBJECTS)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) -o $@ -c $< $(CFLAGS)

clean:
	rm -rf *.o *.so
	rm -rf *~

mrproper: clean
	rm -rf $(EXEC)

install:
	install -D $(EXEC) $(PREFIX)/bin/

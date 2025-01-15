CC=gcc

CFLAGS  = -W -Wall -ggdb
LDFLAGS = -lpthread -lm -lzmq -lImageStreamIO
EXEC    = asgard_DM_server
OBJECTS = asgard_ZMQ_MDM_server.o

# PREFIX is environment variable, but if it is not set, then set default value
ifeq ($(PREFIX),)
    PREFIX := /usr/local
endif

# adding the BMC specific things as they are setup on this machine

BMC_LIBDIR = /opt/Boston\ Micromachines/lib
BMC_INCDIR = /opt/Boston\ Micromachines/include/

CFLAGS  += -I $(BMC_INCDIR)
LDFLAGS += -L $(BMC_LIBDIR) -lBMC -lBMC_PCIeAPI -lBMC_USBAPI

# the makefile instructions

all: $(EXEC)


asgard_DM_server: $(OBJECTS)
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

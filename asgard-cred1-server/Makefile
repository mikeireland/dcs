CC=gcc

# FYI: it turns out that the order in which the libraries are linked matters!
# they probably are linked in the reverse order, and lpthread should be the last
# or linking errors do occur

CFLAGS  = -W -Wall -ggdb
LDFLAGS = -lm -lzmq -lImageStreamIO -lpthread
EXEC    = asgard_cam_server
OBJECTS = asgard_ZMQ_CRED1_server.o

# PREFIX is environment variable, but if it is not set, then set default value
ifeq ($(PREFIX),)
    PREFIX := /usr/local
endif

# adding the BMC specific things as they are setup on this machine

# CFLAGS  += -I $(BMC_INCDIR)
# LDFLAGS += -L $(BMC_LIBDIR) -lBMC -lBMC_PCIeAPI -lBMC_USBAPI

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

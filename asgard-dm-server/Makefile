CC=g++

CFLAGS  = -std=c++17 -W -Wall -Wextra
LDFLAGS = -lcommander -lpthread -lzmq -lImageStreamIO -lboost_program_options -lfmt
EXEC    = asgard_DM_server
# OBJECTS = asgard_commander_MDM_server.o
OBJECTS = asgard_commander_MDM_high_perf_server.o

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
	install -D 17DW019* /opt/Boston\ Micromachines/Profiles/

/* =========================================================================
 * Asgard camera control server - Frantz Martinache
 * 
 * The control server is ZMQ server so that other processes can easily 
 * interface to it, exchanging simple text messages.
 * 
 * The program mostly handles the creation, update and destruction of the
 * shared memory data structure (ImageStreamIO library by O. Guyon) used to
 * share frames acquired with the camera through the PCI ET interface.
 * ========================================================================= */

#include <stdio.h>

#ifdef __GNUC__
#  if(__GNUC__ > 3 || __GNUC__ ==3)
#	define _GNUC3_
#  endif
#endif

#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <math.h>
#include <pthread.h>
#include <zmq.h>
#include <assert.h>
#include <unistd.h>

// #include <ImageStruct.h>    // libImageStreamIO header
#include <ImageStreamIO.h>  // libImageStreamIO header

// #include <edtinc.h>         // EDT-PCI board API


/* =========================================================================
 *                            Global variables
 * ========================================================================= */
#define LINESIZE 256
#define CMDSIZE 200

int simmode = 1;   // flag to set to "1" to *not* connect to the board!
int baud = 115200; // serial connexion baud rate
// EdtDev *ed;        // handle for the serial connexion to camera
// PdvDev *pdv_p;     // handle for data camera link

IMAGE *imarray = NULL;    // shared memory img pointer
int width, height, depth; // frame info from camera link
char *cameratype;         // from camera link

int keepgoing = 0; // flag to control the image fetching loop
char dashline[80] =
  "-----------------------------------------------------------------------------\n";

/* =========================================================================
 *                           function prototypes
 * ========================================================================= */
void *fetch_imgs();
int shm_setup();

/* =========================================================================
 *      Allocates shared memory data structures for the new settings
 * ========================================================================= */
int shm_setup() {
  int NBIMG = 1;
  int shared = 1;
  int NBkw = 10;
  long naxis = 2;
  uint8_t atype = _DATATYPE_UINT16;
  uint32_t *imsize;
  char shmname[20];

  imsize = (uint32_t *) malloc(sizeof(uint32_t) * naxis);
  imsize[0] = 320;
  imsize[1] = 256;

  if (imarray != NULL) {
    ImageStreamIO_destroyIm(imarray);
    free(imarray);
    imarray = NULL;
  }
  imarray = (IMAGE*) malloc(sizeof(IMAGE) * NBIMG);
  sprintf(shmname, "%s", "cred1"); // root-name of the shm
  ImageStreamIO_createIm_gpu(imarray, shmname, naxis, imsize, atype, -1,
			     shared, IMAGE_NB_SEMAPHORE, NBkw, MATH_DATA);
  free(imsize);

  // =========================== add keywords ==============================
  return 0;
}

/* =========================================================================
 *                     Camera image fetching thread
 * ========================================================================= */
void* fetch_imgs() {
  int ii = 0;
  
  // =====================================
  /* Set up higher priority to the attached process?

  uid_t ruid; // Real UID (= user launching process at startup)
  uid_t euid; // Effective UID (= owner of executable at startup)
  uid_t suid; // Saved UID (= owner of executable at startup)
        
  int RT_priority = 70; //any number from 0-99
  struct sched_param schedpar;
  int ret;

  getresuid(&ruid, &euid, &suid);
  ret = seteuid(ruid);   // normal user privileges

  schedpar.sched_priority = RT_priority;
#ifndef __MACH__
  ret = seteuid(euid); //This goes up to maximum privileges
  sched_setscheduler(0, SCHED_FIFO, &schedpar); //other option is SCHED_RR, might be faster
  ret = seteuid(ruid); //Go back to normal privileges
#endif
  */
  // =====================================

  // ----- image fetching loop starts here -----
  while (keepgoing > 0) {
    ii += 1;
    sleep(0.01);
    printf("\rcntr = %5d", ii);
    fflush(stdout);
  }
  printf("\nFetching stopped\n");
  return NULL;
}

/* =========================================================================
 *                               Main program
 * ========================================================================= */
int main() {
  char cmdstring[CMDSIZE]; // holder for the commands sent to the server

  int cmdOK = 0;
  pthread_t tid_fetch; // thread ID for the image fetching to SHM

  // ----- ZMQ setup specifics -----
  int server_port = 6667;  // ZMQ server port number
  char address[15];        // to construct full ZMQ server TCP address
  char status[8] = "idle"; // to answer ZMQ status queries
  
  sprintf(address, "tcp://*:%d", server_port);
  void *context = zmq_ctx_new();
  void *responder = zmq_socket(context, ZMQ_REP);
  int rc = zmq_bind(responder, address);
  assert(rc == 0);

  shm_setup();

  // --------------------- set-up the prompt --------------------
  printf("%s", dashline);
  printf("ZMQ MDM driver server on port %d !\n", server_port);
  printf("%s", dashline);

  // ------------------ start the command line ------------------
  for (;;) {
    cmdOK = 0; // ready to accept a new command
    zmq_recv(responder, cmdstring, 10, 0); // received a command!

    // -------------- process the ZMQ commands ------------------

    // =====================================================
    if (cmdOK == 0)
      if (strncmp(cmdstring, "coucou", strlen("coucou")) == 0) {
	cmdOK = 1;
	zmq_send(responder, "coucou", 6, 0);
	sprintf(status, "%s", "idle");
      }

    // =====================================================
    if (cmdOK == 0)
      if (strncmp(cmdstring, "fetch", strlen("fetch")) == 0) {
	cmdOK = 1;
	if (keepgoing == 0) {
	  keepgoing = 1; // raise the flag
	  printf("Triggering the fetching data\n");
	  pthread_create(&tid_fetch, NULL, fetch_imgs, NULL);
	}
	zmq_send(responder, "OK", 2, 0);
	sprintf(status, "%s", "running");
      }

    // =====================================================
    if (cmdOK == 0)
      if (strncmp(cmdstring, "stop", strlen("stop")) == 0) {
	cmdOK = 1;
	if (keepgoing == 1) {
	  keepgoing = 0;
	}
	zmq_send(responder, "OK", 2, 0);
	sprintf(status, "%s", "idle");
      }

    // =====================================================
    if (cmdOK == 0)
      if (strncmp(cmdstring, "status", strlen("status")) == 0) {
	cmdOK = 1;
	zmq_send(responder, status, strlen(status), 0);
      }

    // =====================================================
    if (cmdOK == 0)
      if (strncmp(cmdstring, "quit", strlen("quit")) == 0) {
	cmdOK = 1;

	if (imarray != NULL) {
	  ImageStreamIO_destroyIm(imarray);
	  free(imarray);
	  imarray = NULL;
	}
	
	zmq_send(responder, "BYE!", 4, 0);
	break;
      }

    // =====================================================    
    if (cmdOK == 0) {
      printf("Unkown command\n");
      zmq_send(responder, "WTF?", 4, 0);
    }
  }
  // -------------------------
  // clean-end of the program
  // -------------------------  
  printf("%s\n", status);
  exit(0);
}

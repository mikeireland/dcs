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
 *          Local data structure to keep track of camera settings
 * ========================================================================= */
typedef struct {		
  int FGchannel;          // frame grabber channel 
  int width, height, depth;
  int timeout;
  char cameratype[16];
  float tint;             // integration time for each read	
  int   NDR;              // number of reads per reset	
  char readmode[16];      // readout mode
  char status[16];        // camera overall status
  float temperature;      // current cryostat temperature
  float maxFps;           // maximum frame rate
  float fps;              // current number of frames in Hz
  float gain;             // 
  // cropping parameters
  int cropmode;          // 0: OFF, 1: ON
  int row0; // range 1 - 256 (granularity = 1)
  int row1; // range 1 - 256 (granularity = 1)
  int col0; // range 1 -  10 (granularity = 32)
  int col1; // range 1 -  10 (granularity = 32)

  int sensibility;
  // 0: low
  // 1: medium
  // 2: high
  
  long frameindex;
  
} CREDSTRUCT;

/* =========================================================================
 *                           function prototypes
 * ========================================================================= */
void *fetch_imgs();
int init_cam_configuration(CREDSTRUCT *camconf);
int shm_setup(CREDSTRUCT *camconf);
int read_pdv_cli(EdtDev *ed, char *outbuf);
int camera_command(EdtDev *ed, const char *cmd);
float camera_query_float(EdtDev *ed, const char *cmd);
int processing_cli_commands(char* cmd);

/* =========================================================================
 *                            Global variables
 * ========================================================================= */
#define LINESIZE 256
#define CMDSIZE 200
#define OUTSIZE 200  // not sure a dedicated constant is warranted, but...

int simmode = 1;   // flag to set to "1" to *not* connect to the board!
int verbose = 0;   // flag to output messages in shell
int baud = 115200; // serial connexion baud rate
EdtDev *ed;        // handle for the serial connexion to camera CLI
PdvDev *pdv_p;     // handle for data camera link

int unit = 0, channel = 0;

IMAGE *imarray = NULL;    // shared memory img pointer
int width, height, depth; // frame info from camera link
char *cameratype;         // from camera link
int keepgoing = 0;        // flag to control the image fetching loop

CREDSTRUCT *camconf;     // structure holding relevant camera configuration options

char dashline[80] =
  "-----------------------------------------------------------------------------\n";


/* =========================================================================
 *       Open the serial CLI and the data-link communication channels
 * ========================================================================= */

// ----- start with the serial CLI -----
ed = pdv_open_channel(EDT_INTERFACE, unit, channel);
if (ed == NULL) {
  pdv_perror(EDT_INTERFACE);
  return -1;
 }
printf("device name is: %s\n", ed->edt_devname);
pdv_set_baud(ed, baud);
printf("serial timeout: %d\n", ed->dd_p->serial_timeout);

// ----- and then, the data-link -----
pdv_p = pdv_open_channel(EDT_INTERFACE, unit, channel);
if (pdv_p == NULL) {
  sprintf(errstr, "pdv_open_channel(%s%d_%d)", edt_devname, unit, channel);
  pdv_perror(errstr);
  return (1);
 }
pdv_flush_fifo(pdv_p);
  

/* =========================================================================
 *             initializes internal camera configuration data structure
 * by getting info from the data link?
 * ========================================================================= */
int init_cam_configuration(CREDSTRUCT *camconf) {
  camconf = (CREDSTRUCT*) malloc(sizeof(CREDSTRUCT));
  char cmd_cli[CMDSIZE];  // holder for commands sent to the camera CLI
  char out_cli[OUTSIZE];  // holder for CLI responses

  // shot in the dark here: I'm not sure the cameralink can know its size without
  // having been told first. The size information may need to be recovered from
  // serial CLI queries

  printf("\n%s", dashline);
  printf("Querying the data-link first...\n\n");
  printf("\n%s", dashline);
  camconf->width = pdv_get_width(pdv_p);
  camconf->height = pdv_get_height(pdv_p);
  camconf->depth = pdv_get_depth(pdv_p);

  camconf->timeout = pdv_get_timeout(pdv_p);
  sprintf(camconf->cameratype, "%s", pdv_get_cameratype(pdv_p));
  
  printf("image size  : %d x %d\n", width, height);
  printf("Timeout     : %d\n", timeout);
  printf("Camera type : %s\n", cameratype);

  printf("\n%s", dashline);
  printf("Querying the serial CLI then\n\n");
  printf("\n%s", dashline);

  sprintf(cmd_cli, "status raw");
  camera_command(ed, cmd_cli);
  read_pdv_cli(ed, out_cli);
  sprintf("status = %s\n", out_cli);
  printf("\n%s", dashline);  
}

/* =========================================================================
 *      Allocates shared memory data structures for the new settings
 * 
 * Parameters: (xsz, ysz) the total (x, y) camera readout dimensions
 * ========================================================================= */
int shm_setup(CREDSTRUCT* camconf) {
  int NBIMG = 1;
  int shared = 1;
  int NBkw = 10;
  long naxis = 2;
  uint8_t atype = _DATATYPE_UINT16;
  uint32_t *imsize;
  char shmname[20];

  imsize = (uint32_t *) malloc(sizeof(uint32_t) * naxis);
  imsize[0] = camconf->width;
  imsize[1] = camconf->height;

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

/* /\* ========================================================================= */
/*  *                      read pdv command line response */
/*  * ========================================================================= *\/ */
/* int read_pdv_cli(EdtDev *ed, char *outbuf) { */
/*   int     ret = 0; */
/*   u_char  lastbyte, waitc; */
/*   int     length=0; */
  
/*   outbuf[0] = '\0'; */
/*   do { */
/*     ret = pdv_serial_read(ed, buf, SERBUFSIZE); */
/*     if (verbose) */
/*       printf("read returned %d\n", ret); */
	
/*     if (*buf) */
/*       lastbyte = (u_char)buf[strlen(buf)-1]; */
	
/*     if (ret != 0) { */
/*       buf[ret + 1] = 0; */
/*       strcat(outbuf, buf); */
/*       length += ret; */
/*     } */
	
/*     if (ed->devid == PDVFOI_ID) */
/*       ret = pdv_serial_wait(ed, 500, 0); */
/*     else if (pdv_get_waitchar(ed, &waitc) && (lastbyte == waitc)) */
/*       ret = 0; /\* jump out if waitchar is enabled/received *\/ */
/*     else ret = pdv_serial_wait(ed, 500, 64); */
/*   } while (ret > 0); */
/* } */

/* /\* ========================================================================= */
/*  *                    generic send a camera CLI command */
/*  * ========================================================================= *\/ */
/* int camera_command(EdtDev *ed, const char *cmd) { */
/*   char tmpbuf[SERBUFSIZE]; */
/*   char outbuf[2000]; */

/*   read_pdv_cli(ed, outbuf); // flush */
/*   sprintf(tmpbuf, "%s\r", cmd); */
/*   pdv_serial_command(ed, tmpbuf); */
/*   if (verbose) */
/*     printf("command: %s", tmpbuf); */
/*   return 0; */
/* } */

/* /\* ========================================================================= */
/*  *                 generic camera CLI query (expects float) */
/*  * ========================================================================= *\/ */
/* float server_query_float(EdtDev *ed, const char *cmd) { */
/*   char outbuf[2000]; */
/*   float fval; */

/*   server_command(ed, cmd); */
/*   usleep(100000); // why this much? to be adjusted! */
/*   read_pdv_cli(ed, outbuf); */
/*   sscanf(outbuf, "%f", &fval); */

/*   return fval; */
/* } */

/* =========================================================================
 *                     Camera image fetching thread
 * ========================================================================= */
void* fetch_imgs() {
  int ii = 0;
  int overrun = 0, overruns = 0;
  int timeout, timeouts, last_timeouts = 0;
  
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
 *                      Processing of FLI's CLI commands
 * ========================================================================= */
int processing_cli_commands(char* cmd) {
  printf("FLI CLI >> %s\n", cmd);

  /* -----------------------------------------------------
     Take care of the commands sent to the camera


     ----------------------------------------------------- */
  return 0;
}

/* =========================================================================
 *                               Main program
 * ========================================================================= */
int main() {
  char cmdstring[CMDSIZE]; // holder for commands sent to this program
  char serialcmd[CMDSIZE]; // holder for commands sent to the camera CLI
  int cmdOK = 0;
  int res = 0;
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

  init_cam_configuration(camconf);
  shm_setup(camconf);

  // --------------------- set-up the prompt --------------------
  
  //-

  printf("\n%s", dashline);
  printf(" _   _      _               _       _ _           ____        _     _      \n");
  printf("| | | | ___(_)_ __ ___   __| | __ _| | |_ __     | __ )  __ _| | __| |_ __ \n");
  printf("| |_| |/ _ \\ | '_ ` _ \\ / _` |/ _` | | | '__|____|  _ \\ / _` | |/ _` | '__|\n");
  printf("|  _  |  __/ | | | | | | (_| | (_| | | | | |_____| |_) | (_| | | (_| | |   \n");
  printf("|_| |_|\\___|_|_| |_|_|_|\\__,_|\\__,_|_|_|_|       |____/ \\__,_|_|\\__,_|_|   \n");
  printf("                 / ___|__ _ _ __ ___   ___ _ __ __ _                       \n");
  printf("                | |   / _` | '_ ` _ \\ / _ \\ '__/ _` |                      \n");
  printf("                | |__| (_| | | | | | |  __/ | | (_| |                      \n");
  printf("                 \\____\\__,_|_| |_| |_|\\___|_|  \\__,_|                      \n");
  printf("\n%s", dashline);
  printf("ZMQ MDM driver server on port # %4d !\n", server_port);
  printf("%s", dashline);

  // ------------------ start the command line ------------------
  for (;;) {
    cmdOK = 0; // ready to accept a new command
    zmq_recv(responder, cmdstring, 10, 0); // received a command!

    // -------------- process the ZMQ commands ------------------

    // First Light CLI commands will be prefixed by a "!"
    // =====================================================
    if (cmdOK == 0)
      if (strncmp(cmdstring, "!", 1) == 0) {
	cmdOK = 1;
	memmove(serialcmd, cmdstring + 1, strlen(cmdstring) - 1);
	res = processing_cli_commands(serialcmd);
	if (res == 0)
	  zmq_send(responder, "OK", 2, 0);
	else
	  zmq_send(responder, "WTF?", 4, 0);
      }

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

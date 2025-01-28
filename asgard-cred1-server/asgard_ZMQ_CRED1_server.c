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
#define SIMULATE

#ifdef __GNUC__
#  if(__GNUC__ > 3 || __GNUC__ ==3)
#	define _GNUC3_
#  endif
#endif

// With commander, are all of these needed???
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

#include <commander/commander.h> // commander header

// #include <ImageStruct.h>    // libImageStreamIO header
#include <ImageStreamIO.h>  // libImageStreamIO header

#include <edtinc.h>         // EDT-PCI board API


/* =========================================================================
 *          Local data structure to keep track of camera settings
 * ========================================================================= */
typedef struct {		
  int FGchannel;          // frame grabber channel 
  uint32_t width, height, depth;
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
int init_cam_configuration();
void shm_setup();
int read_pdv_cli(EdtDev ed, char *outbuf);
int camera_command(EdtDev ed, const char *cmd);
float camera_query_float(EdtDev ed, const char *cmd);
int processing_cli_commands(char* cmd);

/* =========================================================================
 *                            Global variables
 * ========================================================================= */
#define SERBUFSIZE 512
#define LINESIZE 256
#define CMDSIZE 50   // max ZMQ command size
#define OUTSIZE 200  // not sure a dedicated constant is warranted, but...

static char buf[SERBUFSIZE];

int simmode = 1;     // flag to set to "1" to *not* connect to the board!
int verbose = 0;     // flag to output messages in shell
int baud = 115200;   // serial connexion baud rate
EdtDev ed = NULL;    // handle for the serial connexion to camera CLI
PdvDev pdv_p = NULL; // handle for data camera link

int unit = 0;

IMAGE *shm_img = NULL;    // shared memory img pointer
int width, height, depth; // frame info from camera link
char *cameratype;         // from camera link
int keepgoing = 0;        // flag to control the image fetching loop

CREDSTRUCT *camconf;     // structure holding relevant camera configuration options

pthread_t tid_fetch; // thread ID for the image fetching to SHM
char status_cstr[8] = "idle"; // to answer ZMQ status queries

char dashline[80] =
  "-----------------------------------------------------------------------------\n";


/* =========================================================================
 *             initializes internal camera configuration data structure
 * by getting info from the data link?
 * ========================================================================= */
int init_cam_configuration() {
  camconf = (CREDSTRUCT*) malloc(sizeof(CREDSTRUCT));
  char cmd_cli[CMDSIZE];  // holder for commands sent to the camera CLI
  char out_cli[OUTSIZE];  // holder for CLI responses

  printf("\n%s", dashline);
  camconf->width = (uint32_t) pdv_get_width(pdv_p);
  camconf->height = (uint32_t) pdv_get_height(pdv_p);
  camconf->depth = pdv_get_depth(pdv_p);

  camconf->timeout = pdv_serial_get_timeout(ed);
  sprintf(camconf->cameratype, "%s", pdv_get_camera_type(pdv_p));
  
  printf("image size  : %d x %d\n", camconf->width, camconf->height);
  printf("Timeout     : %d\n", camconf->timeout);
  printf("Camera type : %s\n", camconf->cameratype);

  printf("\n%s", dashline);

  sprintf(cmd_cli, "status raw");
  camera_command(ed, cmd_cli);
  read_pdv_cli(ed, out_cli);
  printf("status = %s\n", out_cli);
  return 0;
}

/* =========================================================================
 *      Allocates shared memory data structures for the new settings
 * 
 * Parameters: (xsz, ysz) the total (x, y) camera readout dimensions
 * ========================================================================= */
void shm_setup() {
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

  if (shm_img != NULL) {
    ImageStreamIO_destroyIm(shm_img);
    free(shm_img);
    shm_img = NULL;
  }
  shm_img = (IMAGE*) malloc(sizeof(IMAGE) * NBIMG);
  sprintf(shmname, "%s", "cred1"); // root-name of the shm
  ImageStreamIO_createIm_gpu(shm_img, shmname, naxis, imsize, atype, -1,
			     shared, IMAGE_NB_SEMAPHORE, NBkw, MATH_DATA);
  free(imsize);

  // =========================== add keywords ==============================
}

/* =========================================================================
 *                      read pdv command line response
 * ========================================================================= */
int read_pdv_cli(EdtDev ed, char *outbuf) {
  int     ret = 0;
  u_char  lastbyte, waitc;
  int     length=0;
  
  outbuf[0] = '\0';
  do {
    ret = pdv_serial_read(ed, buf, SERBUFSIZE);
    if (verbose)
      printf("read returned %d\n", ret);
	
    if (*buf)
      lastbyte = (u_char)buf[strlen(buf)-1];
	
    if (ret != 0) {
      buf[ret + 1] = 0;
      strcat(outbuf, buf);
      length += ret;
    }
    
    if (pdv_serial_get_waitchar(ed, &waitc) && (lastbyte == waitc))
      ret = 0;  // Jump out if 'waitchar' is enabled or received.
    else
      ret = pdv_serial_wait(ed, 500, 64);
    
    /*if (ed->devid == PDVFOI_ID)
      ret = pdv_serial_wait(ed, 500, 0);
    else if (pdv_serial_get_waitchar(ed, &waitc) && (lastbyte == waitc))
      ret = 0; // jump out if waitchar is enabled/received
    else ret = pdv_serial_wait(ed, 500, 64);*/
  } while (ret > 0);
  return 0;
}

/* =========================================================================
 *                    generic send a camera CLI command
 * ========================================================================= */
int camera_command(EdtDev ed, const char *cmd) {
  char tmpbuf[SERBUFSIZE];
  char outbuf[2000];

  read_pdv_cli(ed, outbuf); // flush
  sprintf(tmpbuf, "%s\r", cmd);
  pdv_serial_command(ed, tmpbuf);
  if (verbose)
    printf("command: %s", tmpbuf);
  return 0;
}

/* =========================================================================
 *                 generic camera CLI query (expects float)
 * ========================================================================= */
float camera_query_float(EdtDev ed, const char *cmd) {
  char outbuf[2000];
  float fval;

  camera_command(ed, cmd);
  usleep(100000); // why this much? to be adjusted!
  read_pdv_cli(ed, outbuf);
  sscanf(outbuf, "%f", &fval);

  return fval;
}

/* =========================================================================
 *                     Camera image fetching thread
 * ========================================================================= */
void* fetch_imgs(void *dummy) {
  uint8_t *image_p = NULL;
  int width, height, nbpix;
  // unsigned short int *imageushort;
  int numbufs = 256;
  bool timeoutrecovery = false;
  int timeouts;
  
  width = camconf->width;
  height = camconf->height;
  nbpix = width * height;
    
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
    pdv_timeout_restart(pdv_p, true);
    pdv_flush_fifo(pdv_p);
    pdv_multibuf(pdv_p, numbufs);
    pdv_start_images(pdv_p, numbufs);
    timeoutrecovery = false;

    while (!timeoutrecovery) {
      image_p = pdv_wait_images(pdv_p, 1);
      pdv_start_images(pdv_p, numbufs);

      shm_img->md->write = 1;              // signaling about to write
      memcpy(shm_img->array.UI16,          // copy image to shared memory
	     (unsigned short *) image_p,
	     sizeof(unsigned short) * nbpix);
      shm_img->md->write = 0;              // signaling done writing
      ImageStreamIO_sempost(shm_img, -1);  // post semaphores
      shm_img->md->cnt0++;                 // increment internal counter
      shm_img->md->cnt1++;                 // idem

      printf("\rcntr = %10ld", shm_img->md->cnt0);
      fflush(stdout);

      timeouts = pdv_timeouts(pdv_p);
      if (timeouts > 0)
	timeoutrecovery = true;

      if (keepgoing == 0)
	break;
    }
  }
  printf("\nFetching stopped\n");
  return NULL;
}

/* =========================================================================
 *                      Processing of FLI's CLI commands
 * ========================================================================= */
int processing_cli_commands(char* cmd) {
  char out_cli[OUTSIZE];  // holder for CLI responses  
  printf("FLI CLI >> %s\n", cmd);

  /* -----------------------------------------------------
     Take care of the commands sent to the camera

     I'm not going to bother here for now. Mike will do
     things the "commander" way.

     At the moment, this just prints the CLI output to the
     terminal
     ----------------------------------------------------- */
  camera_command(ed, cmd);
  read_pdv_cli(ed, out_cli);
  printf("%s\n", out_cli);
  return 0;
}

// Here are the functions that will be registered with the commander server
void fetch(){
 	if (keepgoing == 0) {
	  keepgoing = 1; // raise the flag
	  printf("Triggering the fetching data\n");
	  pthread_create(&tid_fetch, NULL, fetch_imgs, NULL);
	}
  sprintf(status_cstr, "%s", "running");
}

std::string cli(std::string cmd) {
  char serialcmd[CMDSIZE]; // holder for commands sent to the camera CLI
  int res = 0;
  //memmove(serialcmd, cmd.c_str(), cmd.length());
  memmove(serialcmd, cmd.c_str(), cmd.length());
  //res = processing_cli_commands(serialcmd);
  return "NOT IMPLEMENTED YET";
}

std::string status() {
  return status_cstr;
}

void stop(){
  if (keepgoing == 1) {
    keepgoing = 0;
  }
  sprintf(status_cstr, "%s", "idle");
}
 
namespace co=commander;

COMMANDER_REGISTER(m)
{
  using namespace co::literals;
  m.def("fetch", fetch, "Trigger fetching data from the camera.");
  m.def("cli", cli, "Directly send a command to the camera Command Line Interface.");
  m.def("status", status, "Get the current status of the camera.");
  m.def("stop", stop, "Stop fetching data from the camera.");
}

/* =========================================================================
 *                               Main program
 * ========================================================================= */
int main(int argc, char **argv) {
  char errstr[2*CMDSIZE];
  char edt_devname[CMDSIZE];
  
#ifndef SIMULATE
  // ----- start with the serial CLI -----
  ed = pdv_open(EDT_INTERFACE, unit);
  if (ed == NULL) {
    edt_msg_perror(EDTAPP_MSG_WARNING, EDT_INTERFACE);
    return -1;
  }
  
  pdv_serial_read_enable(ed);
  pdv_serial_set_baud(ed, baud);
    
  // ----- and then, the data-link -----
  pdv_p = pdv_open(EDT_INTERFACE, unit);
  strcpy(edt_devname, EDT_INTERFACE);
  if (pdv_p == NULL) {
    sprintf(errstr, "pdv_open(%s, %d)", edt_devname, unit);
    edt_msg_perror(EDTAPP_MSG_WARNING, errstr);
    return (1);
  }
  pdv_flush_fifo(pdv_p);

  init_cam_configuration();

  shm_setup();
#endif


  //printf("shm_img naxis = %d\n", shm_img->naxis);

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
   
  // start the commander server
  co::Server s(argc, argv);

  s.run();
  
  // -------------------------
  // clean-end of the program
  // -------------------------  
  printf("%s\n", status_cstr);
  if (shm_img != NULL) {
	  ImageStreamIO_destroyIm(shm_img);
	  free(shm_img);
	  shm_img = NULL;
	}
  exit(0);
}

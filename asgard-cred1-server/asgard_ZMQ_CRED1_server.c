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
// #define SIMULATE

#ifdef __GNUC__
#  if(__GNUC__ > 3 || __GNUC__ ==3)
#	define _GNUC3_
#  endif
#endif

#include <commander/commander.h> // commander header
#include <ImageStreamIO.h>       // libImageStreamIO header
#include <edtinc.h>              // EDT-PCI board API
#include <pthread.h>
#include <unistd.h>              // for access and usleep
#include <cjson/cJSON.h>         // for configuration file(s)

/* =========================================================================
 *          Local data structure to keep track of camera settings
 * ========================================================================= */
typedef struct {		
  int FGchannel;          // frame grabber channel 
  uint32_t width, height, depth;
  int timeout;
  char cameratype[16];
  int   NDR;              // number of reads per reset	
  char readmode[16];      // readout mode
  char status[16];        // camera overall status
  float temperature;      // current cryostat temperature
  float maxfps;           // maximum frame rate
  float fps;              // current number of frames in Hz
  float gain;             // camera gain
  // cropping parameters
  int cropmode;          // 0: OFF, 1: ON
  int row0; // range 1 - 256 (granularity = 1)
  int row1; // range 1 - 256 (granularity = 1)
  int col0; // range 1 -  10 (granularity = 32)
  int col1; // range 1 -  10 (granularity = 32)

  // int sensibility;
  // 0: low
  // 1: medium
  // 2: high
  
  long frameindex;
  
} CREDSTRUCT;

/* =========================================================================
 *   Local data structure to split the shm into other shm sub-arrays
 *        for the 4 baldr beams and the 2 heimdallr interferograms
 *        these are read/written to/from a json configuration file
 *         or eventually, a toml configuration file
 * 
 * each array stores: x0, y0, x1, y1 (in that order) coordinates in the
 * full image reference frame!
 * ========================================================================= */
typedef struct {
  char name[6];
  int x0, y0, xsz, ysz;
} subarray;

/* =========================================================================
 *                           function prototypes
 * ========================================================================= */
void *fetch_imgs(void *arg);
void *split_imgs(void *arg);
int init_cam_configuration();
void shm_setup();
int read_pdv_cli(EdtDev ed, char *outbuf);
int camera_command(EdtDev ed, const char *cmd);
float camera_query_float(EdtDev ed, const char *cmd);
char *read_json_file(const char *filename);


/* =========================================================================
 *                            Global variables
 * ========================================================================= */
#define SERBUFSIZE 512
#define LINESIZE 256
#define CMDSIZE 50   // max ZMQ command size
#define OUTSIZE 200  // not sure a dedicated constant is warranted, but...

static char buf[SERBUFSIZE];

int simmode = 1;     // flag to set to "1" to *not* connect to the board!
int splitmode = 0;   // flag to check if split mode is activated
uint32_t nbreads = 100;   // the number of reads without detector reset
int verbose = 0;     // flag to output messages in shell
int baud = 115200;   // serial connexion baud rate
EdtDev ed = NULL;    // handle for the serial connexion to camera CLI
PdvDev pdv_p = NULL; // handle for data camera link

int unit = 0;

IMAGE *shm_img = NULL;    // shared memory img pointer
IMAGE *shm_ROI = NULL;    // pointer to the different shared memory ROI
int width, height, depth; // frame info from camera link
char *cameratype;         // from camera link
int keepgoing = 0;        // flag to control the image fetching loop

CREDSTRUCT *camconf;     // structure holding relevant camera configuration options

pthread_t tid_fetch; // thread ID for the image fetching to SHM
char status_cstr[8] = "idle"; // to answer ZMQ status queries

char dashline[80] =
  "-----------------------------------------------------------------------------\n";

int nroi = 6; // number of regions of interest on the detector
subarray *readout;

/* =========================================================================
 *                  JSON configuration file processing
 * ========================================================================= */

// ============================================================
char *read_json_file(const char *filename) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        perror("Failed to open file");
        return NULL;
    }
    
    fseek(file, 0, SEEK_END);
    long length = ftell(file);
    rewind(file);

    char *data = (char *)malloc(length + 1);
    if (!data) {
        perror("Failed to allocate memory");
        fclose(file);
        return NULL;
    }

    fread(data, 1, length, file);
    data[length] = '\0';  // Null-terminate the string
    fclose(file);
    return data;
}

/* =========================================================================
 *             initializes internal camera configuration data structure
 * by getting info from the data link?
 * ========================================================================= */
int init_cam_configuration() {
  camconf = (CREDSTRUCT*) malloc(sizeof(CREDSTRUCT));
  char cmd_cli[CMDSIZE];  // holder for commands sent to the camera CLI
  char out_cli[OUTSIZE];  // holder for CLI responses
  char fluff[50];         // to discard

  printf("\n%s", dashline);

  /* -----------------------------------------------------------------------
     The right way to do things would be to set the camera settings using
     the CLI first:

     After reading the relevant information in a TOML file
     (columns, rows only - the rest should be dynamically allocated?

     "set cropping columns ..."
     "set cropping rows ..."
     "set mode globalresetburts"
     "set nbreadsworeset 100" (default)
     "set rawimages on"
     "set gain x"
     "maxfps"
     "set fps maxfps"
     ---------------------------------------------------------------------- */

  // -----------------------------
  // -----------------------------
  sprintf(cmd_cli, "set cropping columns 32-223"); // on CRED3: multiples of 32
  sprintf(cmd_cli, "set cropping columns 0-639"); // on CRED3: multiples of 32
  camera_command(ed, cmd_cli);
  read_pdv_cli(ed, out_cli);  // to flush ?
  
  /* sprintf(cmd_cli, "cropping columns"); */
  /* camera_command(ed, cmd_cli); */
  /* read_pdv_cli(ed, out_cli); */
  /* printf("%s\n", out_cli); */

  sprintf(cmd_cli, "set cropping rows 20-219"); // on CRED3: multiples of 4
  sprintf(cmd_cli, "set cropping rows 0-511"); // on CRED3: multiples of 4
  camera_command(ed, cmd_cli);
  read_pdv_cli(ed, out_cli);  // to flush ?
  /* sprintf(cmd_cli, "cropping rows"); */
  /* camera_command(ed, cmd_cli); */
  /* read_pdv_cli(ed, out_cli); */
  /* printf("%s\n", out_cli); */

  usleep(100);
  
  // -----------------------------
  // sprintf(cmd_cli, "set mode globalresetbursts");
  // camera_command(ed, cmd_cli);
  // sprintf(cmd_cli, "set nbreadsworeset %d", nbreads);
  // camera_command(ed, cmd_cli);
  // sprintf(cmd_cli, "set rawimages on");
  // camera_command(ed, cmd_cli);
  
  
  // -----------------------------
  
  camconf->width = (uint32_t) pdv_get_width(pdv_p);
  camconf->height = (uint32_t) pdv_get_height(pdv_p);
  camconf->depth = pdv_get_depth(pdv_p);

  camconf->timeout = pdv_serial_get_timeout(ed);
  sprintf(camconf->cameratype, "%s", pdv_get_camera_type(pdv_p));
  
  printf("frame size  : %d x %d\n", camconf->width, camconf->height);
  printf("Timeout     : %d\n", camconf->timeout);
  printf("Camera type : %s\n", camconf->cameratype);

  printf("\n%s", dashline);

  // -----------------------------
  sprintf(cmd_cli, "cropping rows raw");
  camera_command(ed, cmd_cli);
  read_pdv_cli(ed, out_cli);
  sscanf(out_cli, "%d-%d%s", &camconf->row0, &camconf->row1, fluff);

  sprintf(cmd_cli, "cropping columns raw");
  camera_command(ed, cmd_cli);
  usleep(100);
  read_pdv_cli(ed, out_cli);
  sscanf(out_cli, "%d-%d%s", &camconf->col0, &camconf->col1, fluff);

  printf("rows = %d-%d, cols = %d-%d\n",
	 camconf->row0, camconf->row1,
	 camconf->col0, camconf->col1);
  
  camconf->width = camconf->col1 - camconf->col0 + 1;
  camconf->height = camconf->row1 - camconf->row0 + 1;

  printf(">> resized window size = %d x %d\n",
	 camconf->width, camconf->height);
  // -----------------------------
  
  sprintf(cmd_cli, "maxfps raw");
  camera_command(ed, cmd_cli);
  read_pdv_cli(ed, out_cli);
  sscanf(out_cli, "%f%s", &camconf->maxfps, fluff);
  sprintf(cmd_cli, "set fps %.1f", camconf->maxfps);
  camera_command(ed, cmd_cli);
  
  sprintf(cmd_cli, "fps raw");
  camera_command(ed, cmd_cli);
  read_pdv_cli(ed, out_cli);
  sscanf(out_cli, "%f%s", &camconf->fps, fluff);
  printf("FPS set to: %f Hz\n", camconf->fps);

  sprintf(cmd_cli, "status raw");
  camera_command(ed, cmd_cli);
  read_pdv_cli(ed, out_cli);
  //printf("status = %s\n", out_cli);
  return 0;
}

/* =========================================================================
 *      Allocates shared memory data structures for the new settings
 * 
 * Parameters: (xsz, ysz) the total (x, y) camera readout dimensions
 * ========================================================================= */
void shm_setup() {
  // int NBIMG = 1;
  int shared = 1;
  int NBkw = 10;
  long naxis = 3;
  uint8_t atype = _DATATYPE_UINT16;
  char shmname[20];
  uint32_t imsize[3] = {camconf->width, camconf->height, nbreads};
  uint32_t roisize[2];

  if (shm_img != NULL) {
    ImageStreamIO_destroyIm(shm_img);
    free(shm_img);
    shm_img = NULL;
  }

  shm_img = (IMAGE*) malloc(sizeof(IMAGE));
  sprintf(shmname, "%s", "cred1"); // root-name of the shm
  ImageStreamIO_createIm_gpu(shm_img, shmname, naxis, imsize, atype, -1,
			     shared, IMAGE_NB_SEMAPHORE, NBkw, MATH_DATA);

  // =========================== add keywords ==============================

  if (splitmode == 1) { // should be done anyways no ?

    if (shm_ROI != NULL) {
      ImageStreamIO_destroyIm(shm_ROI);
      free(shm_ROI);
      shm_ROI = NULL;
    }

    naxis = 2;
    shm_ROI = (IMAGE*) malloc(sizeof(IMAGE) * nroi);

    for (int ii = 0; ii < nroi; ii++) {
      // printf("creating the %s ROI shared memory structure\n", readout[ii].name);
      roisize[0] = readout[ii].xsz;
      roisize[1] = readout[ii].ysz;

      // should data-type change to float here after?
      ImageStreamIO_createIm_gpu(&shm_ROI[ii], readout[ii].name, naxis,
  				 roisize, atype, -1, shared,
  				 IMAGE_NB_SEMAPHORE, NBkw, MATH_DATA);
    }
  }
  printf("Shared memory structures created!\n");
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
void* fetch_imgs(void *arg) {
  uint8_t *image_p = NULL;
  unsigned short int *liveimg;
  int numbufs = 256;
  bool timeoutrecovery = false;
  int timeouts;
  unsigned int liveindex = 0;
  int nbpix = camconf->width * camconf->height;

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

      liveindex = shm_img->md->cnt1+1;
      if (liveindex >= shm_img->md->size[2])
	liveindex = 0;

      liveimg = shm_img->array.UI16 + liveindex * nbpix;  // live pointer

      image_p = pdv_wait_images(pdv_p, 1);
      pdv_start_images(pdv_p, numbufs);

      shm_img->md->write = 1;              // signaling about to write
      memcpy(liveimg,                      // copy image to shared memory
	     (unsigned short *) image_p,
	     sizeof(unsigned short) * nbpix);
      shm_img->md->write = 0;              // signaling done writing
      ImageStreamIO_sempost(shm_img, -1);  // post semaphores
      shm_img->md->cnt0++;                 // increment internal counter
      shm_img->md->cnt1 = liveindex;       // idem

      // printf("\rcntr = %10ld", shm_img->md->cnt0);
      // fflush(stdout);

      timeouts = pdv_timeouts(pdv_p);
      if (timeouts > 0)
	timeoutrecovery = true;

      if (keepgoing == 0)
	break;
    }
  }
  return NULL;
}

/* =========================================================================
 *                    Image splitting into ROIs thread
 * ========================================================================= */
void* split_imgs(void* arg) {
  int ii, jj, ri;  // ii,jj pixel indices, ri: ROI index
  int nbpix = camconf->width * camconf->height;
  unsigned short int *liveroi, *liveimg;
  int ixsz, xsz, ysz, x0, y0;
  
  while (keepgoing > 0) {
    ImageStreamIO_semwait(shm_img, 1);  // waiting for image update
    liveimg = shm_img->array.UI16 + shm_img->md->cnt1 * nbpix;
    ixsz = shm_img->md->size[0];
    
    for (ri = 0; ri < nroi; ri++) {
      liveroi = shm_ROI[ri].array.UI16; // live ROI data pointer
      xsz = shm_ROI[ri].md->size[0];
      ysz = shm_ROI[ri].md->size[1];
      x0 = readout[ri].x0;
      y0 = readout[ri].y0;

      shm_ROI[ri].md->write = 1;
      for (jj = 0; jj < ysz; jj++) {
	for (ii = 0; ii < xsz; ii++) {
	  liveroi[jj*xsz+ii] = liveimg[(jj + y0) * ixsz + ii + x0];
	}
      }
      shm_ROI[ri].md->write = 0;
      ImageStreamIO_sempost(&shm_ROI[ri], -1);
      shm_ROI[ri].md->cnt0++;
      shm_ROI[ri].md->cnt1++;
    }
  }
  return NULL;
}

/* =========================================================================
 *      Functions that will be registered with the commander server
 * ========================================================================= */
void fetch(){
  /* -------------------------------------------------------------------------
   *          Trigger the thread fetching images to shared memory
   * ------------------------------------------------------------------------- */
  if (keepgoing == 0) {
    keepgoing = 1; // raise the flag
    printf("Triggering the fetching data\n");
    pthread_create(&tid_fetch, NULL, fetch_imgs, NULL);
  }
  sprintf(status_cstr, "%s", "running");
}

std::string cli(std::string cmd) {
  /* -------------------------------------------------------------------------
   *            Processing Camera command-line interface exchanges.
   * ------------------------------------------------------------------------- */
  char out_cli[OUTSIZE];  // holder for CLI responses  
  
  camera_command(ed, cmd.c_str());
  read_pdv_cli(ed, out_cli);
  return out_cli;
}

std::string status() {
  /* -------------------------------------------------------------------------
   *                          Returns server status
   *  ------------------------------------------------------------------------- */
  return status_cstr;
}

void stop(){
  /* -------------------------------------------------------------------------
   *                      Interrupts the fetching process
   *  ------------------------------------------------------------------------- */
  if (keepgoing == 1) {
    keepgoing = 0;
  }
  sprintf(status_cstr, "%s", "idle");
}
 
void split() {
  /* -------------------------------------------------------------------------
   *      Trigger the thread splitting the raw image into its different ROI
   * ------------------------------------------------------------------------- */
  pthread_t tid_split; // thread ID for the image ROI splitting
  printf("Triggering the ROI splitting mechanism\n");
  pthread_create(&tid_split, NULL, split_imgs, NULL);
}  

namespace co=commander;

COMMANDER_REGISTER(m)
{
  using namespace co::literals;
  m.def("fetch", fetch, "Trigger fetching data from the camera.");
  m.def("cli", cli, "Directly send a command to the camera Command Line Interface.");
  m.def("status", status, "Get the current status of the camera.");
  m.def("stop", stop, "Stop fetching data from the camera.");
  m.def("split", split, "Trigger the update of ROI data shared memory");
}

/* =========================================================================
 *                               Main program
 * ========================================================================= */
int main(int argc, char **argv) {
  char errstr[2*CMDSIZE];
  char edt_devname[CMDSIZE];
  const char* homedir = getenv("HOME");
  char split_conf_fname[LINESIZE];
  int ii = 0;

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

#endif

  // --------- configuration of the image splitting -------------
  sprintf(split_conf_fname, "%s/.config/camera_split.json", homedir);

  if (access(split_conf_fname, F_OK) == 0) {
    printf("Configuration file %s found!\n", split_conf_fname);
    splitmode = 1;
    char *json_data = read_json_file(split_conf_fname);
    cJSON *json_root = cJSON_Parse(json_data);
    free(json_data);

    ii = 0;
    readout = (subarray *) malloc(nroi * sizeof(subarray));
    // iterate over objects
    cJSON *item = NULL;
    cJSON_ArrayForEach(item, json_root) {
      const char *name = item->string;
      sprintf(readout[ii].name, "%s", name);
      readout[ii].x0  = cJSON_GetObjectItem(item, "x0")->valueint;
      readout[ii].y0  = cJSON_GetObjectItem(item, "y0")->valueint;
      readout[ii].xsz = cJSON_GetObjectItem(item, "xsz")->valueint;
      readout[ii].ysz = cJSON_GetObjectItem(item, "ysz")->valueint;
      ii++;
    }
  } else {
    printf("%s doesn't exist!\n", split_conf_fname);
    splitmode = 0;
  }
  
  shm_setup();
  
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
  free(readout);
  if (shm_img != NULL) {
    ImageStreamIO_destroyIm(shm_img);
    free(shm_img);
    shm_img = NULL;
  }

  if (shm_ROI != NULL) {
    for (ii= 0; ii < nroi; ii++) {
      ImageStreamIO_destroyIm(&shm_ROI[ii]);
    }
    // ImageStreamIO_destroyIm(shm_ROI);
    free(shm_ROI);
    shm_ROI = NULL;
  }

  exit(0);
}

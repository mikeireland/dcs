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
#include <fitsio.h>
#include <time.h>
#include <sys/stat.h>

/* =========================================================================
 *          Local data structure to keep track of camera settings
 * ========================================================================= */
typedef struct {		
  int FGchannel;          // frame grabber channel 
  uint32_t width, height, depth;
  uint32_t nbpix_frm;     // the number of pixels in a "frame"
  uint32_t nbpix_cub;     // the number of pixels in a "cube"
  uint32_t nbreads;       // the number of reads without detector reset
  uint32_t nbr_hlf;       // the number of reads saved in a FITS cube
  int timeout;
  char cameratype[16];
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

int verbose = 0;     // flag to output messages in shell
int baud = 115200;   // serial connexion baud rate
EdtDev ed = NULL;    // handle for the serial connexion to camera CLI
PdvDev pdv_p = NULL; // handle for data camera link

int unit = 0;

IMAGE *shm_img = NULL;    // shared memory img pointer
IMAGE *shm_ROI = NULL;    // pointer to the different shared memory ROI
unsigned short *tosave = NULL; // holder for the cube to be saved to FITS

int keepgoing = 0;        // flag to control the image fetching loop
int savemode = 0;         // flag to control the FITS data save mode
long savecube_index = 0;   // to keep track of saved cubes
CREDSTRUCT *camconf;      // structure holding relevant camera configuration options

pthread_t tid_fetch; // thread ID for the image fetching to SHM
pthread_t tid_split; // thread ID for the image ROI splitting

char status_cstr[8] = "idle"; // to answer ZMQ status queries

char dashline[80] =
  "-----------------------------------------------------------------------------\n";

char savedir[200];  // the directory where the data will be saved

int nroi = 6; // number of regions of interest on the detector
subarray *readout = NULL;

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
 *      Split configuration is read (for now) from a JSON file.
 *   The procedure updates a shared readout configuration data structure.
 * ========================================================================= */
void refresh_image_splitting_configuration() {
  char split_conf_fname[LINESIZE];
  int ii;
  sprintf(split_conf_fname, "%s/.config/cred1_split.json", getenv("HOME"));
  if (access(split_conf_fname, F_OK) == 0) {
    printf("Configuration file %s found!\n", split_conf_fname);
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
    splitmode = 1;

  } else {
    printf("%s doesn't exist!\n", split_conf_fname);
    splitmode = 0;
  }
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

  sprintf(cmd_cli, "set cropping columns 1-10"); // on CRED1: groupings of 32
  camera_command(ed, cmd_cli);
  read_pdv_cli(ed, out_cli);  // to flush ?
  
  sprintf(cmd_cli, "set cropping rows 0-255"); // on CRED1
  camera_command(ed, cmd_cli);
  read_pdv_cli(ed, out_cli);  // to flush ?

  usleep(100);
  
  // -----------------------------
  // sprintf(cmd_cli, "set mode globalresetbursts");
  // camera_command(ed, cmd_cli);
  // sprintf(cmd_cli, "set nbreadsworeset %d", nbreads);
  // camera_command(ed, cmd_cli);
  // sprintf(cmd_cli, "set rawimages on");
  // camera_command(ed, cmd_cli);
  
  
  // -----------------------------
  camconf->nbreads = 200;
  camconf->nbr_hlf = 100;
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
  /*
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
  camconf->width = 32 * (camconf->col1 - camconf->col0 + 1);
  camconf->height = camconf->row1 - camconf->row0 + 1;

  */
  
  camconf->nbpix_frm = camconf->width * camconf->height;
  camconf->nbpix_cub = camconf->nbpix_frm * camconf->nbr_hlf;

  printf(">> resized window size = %d x %d\n",
	 camconf->width, camconf->height);
  printf(">> saving cubes of %d images\n", camconf->nbr_hlf);
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
  return 0;
}

/* =========================================================================
 *      Allocates shared memory data structures for the new settings
 * 
 * Parameters: (xsz, ysz) the total (x, y) camera readout dimensions
 * ========================================================================= */
void shm_setup() {
  int shared = 1;
  int NBkw = 10;
  long naxis = 3;
  uint8_t atype = _DATATYPE_UINT16;
  char shmname[20];
  uint32_t imsize[3] = {camconf->width, camconf->height, camconf->nbreads};
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
  /*
  strcpy(shm_img[0].kw[0].name, "fps");
  shm_img[0].kw[0].type = 'D';
  shm_img[0].kw[0].value.numf = camconf->fps;
  strcpy(shm_img[0].kw[0].comment, "frame rate (in Hz)");
  */
  if (splitmode == 1) { // should be done anyways no ?

    if (shm_ROI != NULL) {
      ImageStreamIO_destroyIm(shm_ROI);
      free(shm_ROI);
      shm_ROI = NULL;
    }

    naxis = 2;
    shm_ROI = (IMAGE*) malloc(sizeof(IMAGE) * nroi);

    for (int ii = 0; ii < nroi; ii++) {
      roisize[0] = readout[ii].xsz;
      roisize[1] = readout[ii].ysz;

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
 *                     Save as a fits data-cube thread
 * ========================================================================= */
void* save_cube_to_fits(void *_cube) {
  fitsfile *fptr;
  int status;
  int bitpix = USHORT_IMG;
  long naxis = 3;
  long naxes[3] = {camconf->width, camconf->height, camconf->nbr_hlf};

  long fpixel; //, nel;
  unsigned short *cube = (unsigned short*) _cube;
  struct timespec tnow;  // time since epoch
  struct tm *uttime;     // calendar time

  struct stat st;
  
  char fname[300];  // path + name of the fits file to write
  // get the time - to give the file a unique name
  clock_gettime(CLOCK_REALTIME, &tnow);  // elapsed time since epoch
  uttime = gmtime(&tnow.tv_sec);         // translate into calendar time

  // cubes saved in ~/Music/ for now
  sprintf(savedir, "%s/Music/%04d%02d%02d/", getenv("HOME"),
	  1900 + uttime->tm_year, 1 + uttime->tm_mon, uttime->tm_mday); 

  if (stat(savedir, &st) == -1)
    mkdir(savedir, 0700);
  sprintf(fname, "%s/T%02d:%02d:%02d.%09ld.fits", savedir,
	  uttime->tm_hour, uttime->tm_min, uttime->tm_sec, tnow.tv_nsec);

  // create a fits file
  fpixel = 1; // first pixel to write
  fits_create_file(&fptr, fname, &status);
  fits_create_img(fptr, bitpix, naxis, naxes, &status);
  
  fits_write_img(fptr, TUSHORT, fpixel, camconf->nbpix_cub, cube, &status);

  // fill in the proper keywords
  // fits_update_key(fptr, TLONG, "EXPOSURE", &tint, "Exposure time", &status);
  fits_update_key(fptr, TLONG, "CINDEX", &savecube_index, "Cube index", &status);
  fits_close_file(fptr, &status);
  savecube_index++;
  return NULL;
}

/* =========================================================================
 *                     Camera image fetching thread
 * ========================================================================= */
void* fetch_imgs(void *arg) {
  uint8_t *image_p = NULL;
  unsigned short *liveimg;
  int numbufs = 256;
  bool timeoutrecovery = false;
  int timeouts;
  unsigned int liveindex = 0;
  long nbpix_frm = camconf->nbpix_frm;
  long nbpix_cub = camconf->nbpix_cub;

  pthread_t tid_save; // thread ID for the saving of a data-cube
  
  // ----- image fetching loop starts here -----

  while (keepgoing > 0) {
    pdv_timeout_restart(pdv_p, true);
    pdv_flush_fifo(pdv_p);
    pdv_multibuf(pdv_p, numbufs);
    pdv_start_images(pdv_p, numbufs);
    timeoutrecovery = false;

    while (!timeoutrecovery) {

      liveimg = shm_img->array.UI16 + liveindex * nbpix_frm;  // live pointer

      image_p = pdv_wait_images(pdv_p, 1);
      pdv_start_images(pdv_p, numbufs);

      shm_img->md->write = 1;              // signaling about to write
      memcpy(liveimg,                      // copy image to shared memory
	     (unsigned short *) image_p,
	     sizeof(unsigned short) * nbpix_frm);
      shm_img->md->write = 0;              // signaling done writing
      ImageStreamIO_sempost(shm_img, -1);  // post semaphores
      shm_img->md->cnt0++;                 // increment internal counter
      
      liveindex = shm_img->md->cnt0 % shm_img->md->size[2];
      shm_img->md->cnt1 = liveindex;       // idem

      if (savemode > 0) {
	if (liveindex == camconf->nbr_hlf) {
	  // save the first half of the live data-cube
	  memcpy(tosave, (unsigned short *) shm_img->array.UI16,
		 nbpix_cub * sizeof(unsigned short));
	  pthread_create(&tid_save, NULL, save_cube_to_fits, tosave);
	}
	if (liveindex == 0) {
	  // save the second half of the live data-cube
	  memcpy(tosave, (unsigned short *) (shm_img->array.UI16 + nbpix_cub),
		 nbpix_cub * sizeof(unsigned short));
	  pthread_create(&tid_save, NULL, save_cube_to_fits, tosave);
	}
      }
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

    if (splitmode == 1) {
      printf("ROI splitting mechanism is ON\n");
      free(readout);
      refresh_image_splitting_configuration();
      pthread_create(&tid_split, NULL, split_imgs, NULL);
    }
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
 
void update_fps(float fps) {
    /* -------------------------------------------------------------------------
     * server level command to update the camera fps and synchronize the
     * shared memory with the new settings
     * ------------------------------------------------------------------------- */
  int wasrunning = 0;  // to keep track
  char cmd_cli[CMDSIZE];  // holder for commands sent to the camera CLI
  char out_cli[OUTSIZE];  // holder for CLI responses

  if (keepgoing == 1) {
    keepgoing = 0; // to interrupt the fetching process
    wasrunning = 1; //
  }
  sprintf(cmd_cli, "set fps %.2f", fps);
  camera_command(ed, cmd_cli);
  read_pdv_cli(ed, out_cli);

  if (wasrunning == 1) fetch();
}

void set_savemode(int _mode) {
    /* -------------------------------------------------------------------------
     * Start or interrupt the FITS saving of data cubes acquired by the camera
     * ------------------------------------------------------------------------- */
  if (_mode <= 0) {
    savemode = 0;
    printf("Savemode was turned OFF\n");
  }
  else {
    savemode = 1;
    printf("Savemode was turned ON\n");
  }
}

void set_splitmode(int _mode) {
  /* -------------------------------------------------------------------------
   *      Trigger the thread splitting the raw image into its different ROI
   * ------------------------------------------------------------------------- */
  if (_mode <= 0) {
    splitmode = 0;
    printf("Splitmode was turned OFF\n");
  }
  else {
    splitmode = 1;
    printf("Splitmode was turned ON\n");
  }
}


namespace co=commander;

COMMANDER_REGISTER(m)
{
  using namespace co::literals;
  m.def("fetch", fetch, "Trigger fetching data from the camera.");
  m.def("cli", cli, "Directly send a command to the camera Command Line Interface.");
  m.def("status", status, "Get the current status of the camera.");
  m.def("stop", stop, "Stop fetching data from the camera.");
  m.def("split", set_splitmode, "Trigger the update of ROI data shared memory");
  m.def("fps", update_fps, "Updates the camera FPS and syncs SHM");
  m.def("savemode", set_savemode, "Set/unset the FITS cube savemode");
}

/* =========================================================================
 *                               Main program
 * ========================================================================= */
int main(int argc, char **argv) {
  char errstr[2*CMDSIZE];
  char edt_devname[CMDSIZE];
  const char* homedir = getenv("HOME");
  int ii = 0;

  sprintf(savedir, "%s/Music/", homedir);  // cubes saved in ~/Music/ for now
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
  refresh_image_splitting_configuration();
  
  shm_setup();
  tosave = (unsigned short int*) malloc(camconf->nbpix_cub * sizeof(unsigned short int));

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
    free(shm_ROI);
    shm_ROI = NULL;
  }

  free(tosave);  // the holder for the data to save
  exit(0);
}

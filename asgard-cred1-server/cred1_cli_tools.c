#include "cred1_cli_tools.h"     // command line interface (CLI) tools
#include <unistd.h>              // for access and usleep

#ifdef __GNUC__
#  if(__GNUC__ > 3 || __GNUC__ ==3)
#	define _GNUC3_
#  endif
#endif

#define SERBUFSIZE 512
static char buf[SERBUFSIZE];


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
    /* if (verbose) */
    /*   printf("read returned %d\n", ret); */
	
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

  sleep(0.1);
  read_pdv_cli(ed, outbuf); // flush
  sleep(0.1);
  sprintf(tmpbuf, "%s\n", cmd);
  pdv_serial_command(ed, tmpbuf);
  sleep(0.1);
  return 0;
}

/* =========================================================================
 *          generic camera CLI query (float and integer versions)
 * ========================================================================= */
void trim(char *s) {
  int ii = 0, jj = 0;
  while (s[ii] == ' ') ii++;
  while (s[jj++] = s[ii++]);
}

float camera_query_float(EdtDev ed, const char *cmd) {
  char outbuf[200];
  char fluff[50];         // to discard  
  float fval;

  camera_command(ed, cmd);
  read_pdv_cli(ed, outbuf);
  trim(outbuf);

  sscanf(outbuf, "%f%s", &fval, fluff);
  return fval;
}

int camera_query_int(EdtDev ed, const char *cmd) {
  char outbuf[200];
  char fluff[50];         // to discard  
  int ival;

  camera_command(ed, cmd);
  read_pdv_cli(ed, outbuf);
  trim(outbuf);
  
  sscanf(outbuf, "%d%s", &ival, fluff);
  return ival;
}

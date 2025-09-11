/* =========================================================================
   The tools to interact with the CRED1 command line interface (CLI) through
   the PCI EDT board
 * ========================================================================= */

#include <edtinc.h>              // EDT-PCI board API

// generic camera serial interaction commands
int read_pdv_cli(EdtDev ed, char *outbuf);
int camera_command(EdtDev ed, const char *cmd);

// specific type query cases
float camera_query_float(EdtDev ed, const char *cmd);
int camera_query_int(EdtDev ed, const char *cmd);



#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <sys/time.h>
#include <time.h>
#include "ImageStruct.h"
#include "ImageStreamIO.h"

#define NDMS 4
#define NCH 4
#define NBkw 0
//gcc -O2 -std=c17 -o sim_mdm_server sim_mdm_server.c -lImageStreamIO -lm -lpthread
int main() {
    IMAGE shmarray[NDMS][NCH + 1];
    char shmname[32];

    int naxis = 2;
    uint32_t imsize[2] = {12, 12};
    uint8_t atype = _DATATYPE_UINT16;
    int shared = 1;

    for (int kk = 0; kk < NDMS; kk++) {
        // individual channels
        for (int ii = 0; ii < NCH; ii++) {
            sprintf(shmname, "dm%ddisp%02d", kk + 1, ii);
            if (ImageStreamIO_createIm_gpu(&shmarray[kk][ii], shmname, naxis, imsize, atype, -1,
                                           shared, IMAGE_NB_SEMAPHORE, NBkw, MATH_DATA) != IMAGESTREAMIO_SUCCESS) {
                fprintf(stderr, "[ERROR] Failed to create SHM: %s\n", shmname);
                return EXIT_FAILURE;
            }
            printf("[SIM MDM Server] Created SHM: %s\n", shmname);
        }
        // the combined array
        sprintf(shmname, "dm%d", kk + 1);
        if (ImageStreamIO_createIm_gpu(&shmarray[kk][NCH], shmname, naxis, imsize, atype, -1,
                                       shared, IMAGE_NB_SEMAPHORE, NBkw, MATH_DATA) != IMAGESTREAMIO_SUCCESS) {
            fprintf(stderr, "[ERROR] Failed to create SHM: %s\n", shmname);
            return EXIT_FAILURE;
        }
        printf("[SIM MDM Server] Created SHM: %s\n", shmname);
    }

    // Initialize to zero once
    for (int kk = 0; kk < NDMS; kk++) {
        for (int jj = 0; jj < imsize[0] * imsize[1]; ++jj) {
            shmarray[kk][NCH].array.UI16[jj] = 0;
        }
        ImageStreamIO_sempost(&shmarray[kk][NCH], 0);
    }

    // Main loop: just listen / keep SHMs alive
    while (1) {
        sleep(0.000001); // 10 ms
    }


    for (int kk = 0; kk < NDMS; kk++) {
        for (int ii = 0; ii <= NCH; ii++) {
            ImageStreamIO_destroyIm(&shmarray[kk][ii]);
        }
    }

    return 0;
}

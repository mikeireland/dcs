/*
 * Example code to write image in shared memory
 *
 * compile with:
 * gcc ImCreate_ft.c ImageStreamIO.c -o imft -lm -lpthread -lfftw3
 *
 * Required files in compilation directory :
 * ImCreate_test.c   : source code (this file)
 * ImageStreamIO.c   : ImageStreamIO source code
 * ImageStreamIO.h   : ImageCreate function prototypes
 * ImageStruct.h     : Image structure definition
 *
 * EXECUTION:
 * ./a.out
 * (no argument)
 *
 * Creates an image imtest00 in shared memory
 * Updates the image every ~ 10ms, forever...
 * A disk is rotating around the center of the image
 *
 */
#define SZ 32
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <complex.h>
#include <fftw3.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "ImageStreamIO.h"

/* [geometry] 
hole_diameter   = 9e-3
beam_x          = [-15.6e-3,15.6e-3,0,0]
beam_y          = [-9e-3,-9e-3,0,18e-3]
pix = 24.0
 */

const float hole_radius = 1.65;      // Radius of the hole in pupil pixels
//const float hole_x[4] = {-3.46*hole_radius, 0,0,3.46*hole_radius}; // Hole x center in pupil pixels
//const float hole_y[4] = {-2*hole_radius, 4*hole_radius, 0 , -2*hole_radius}; // Hole y center in pupil pixels
float phase=0.0;
const float hole_x[4] = {-0.00792, -0.0085, 0.0,0.02130};
const float hole_y[4] = {0.019,-0.01484, 0.0, -0.00015};
const float s = SZ*24/2.1;

int make_image(IMAGE *imarray, fftw_complex *pupil, fftw_complex *image, fftw_plan plan, float wavenum_scale, float flux_scale)
{
    float x,y, rnoise;                    // Image column and row indices
    fftw_complex hole_phasors[4];              // Phasors for the holes

    imarray->md->write = 1;         // Poor-man's mutex when writing

    // Make our hole phasors
    for (int kk=0; kk<4; kk++)
        hole_phasors[kk] = cexp(2.5*I*kk*sin(phase)*wavenum_scale);

    // Fill the pupil with the holes
    for(int jj=0; jj<SZ; jj++)            // loop rows
    {
        y = (((jj + SZ/2) % SZ) - SZ/2.0);
        for(int ii=0; ii<SZ; ii++)     // loop columns
        {
            x = (((ii + SZ/2) % SZ) - SZ/2.0);
            pupil[jj*SZ + ii] = 0.0;
            for (int kk=0; kk<4; kk++)
            {
                if (sqrt((x-s*hole_x[kk]*wavenum_scale)*(x-s*hole_x[kk]*wavenum_scale) + (y-s*hole_y[kk]*wavenum_scale)*(y-s*hole_y[kk]*wavenum_scale)) < 1.5*wavenum_scale)
                {
                    pupil[jj*SZ + ii] += hole_phasors[kk];
                }
            }
        }
    }
    // Use our FFT plan to transform the pupil to the image
    fftw_execute(plan);

    // Now take the square of the electric field and copy to the image
    // ->array is union; ->array.F is float pointer to image
    float* dotF = imarray->array.F;
    for (int ii=0; ii<SZ; ii++)
    {
        for (int jj=0; jj<SZ; jj++)
        {
            rnoise = rand()/(float)RAND_MAX - 0.5;
            *(dotF++) = powf(cabsf(image[((ii + SZ/2) % SZ)*SZ + (jj + SZ/2) % SZ]),2)*flux_scale + rnoise*5;
        }
    }

    // Post all semaphores (index = -1)
    ImageStreamIO_sempost(imarray, -1);

    imarray->md->cnt0++;
    imarray->md->cnt1++;

    return 0;
}

int main()
{
    fftw_complex *pupil_K1, *image_K1, *pupil_K2, *image_K2;
    fftw_plan plan_K1, plan_K2;
    IMAGE imarray[2];              // pointer to array of images
    uint32_t imsize[2] = { SZ, SZ }; // image size is 512 x 512
    long naxis = sizeof(imsize) / (sizeof *imsize);  // # of axes

    // Data type; see file ImageStruct.h for list of supported types
    uint8_t atype = _DATATYPE_FLOAT;

    int shared = 1;                // 1 if image in shared mem
    int NBkw = 10;                 // number of keywords allowed
    int dtus = 10000;              // Wait 10ms = 10,000 microseconds

    // allocate the fftw arrays
    pupil_K1 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * SZ * SZ);
    pupil_K2 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * SZ * SZ);
    image_K1 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * SZ * SZ);
    image_K2 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * SZ * SZ);
    plan_K1 = fftw_plan_dft_2d(SZ, SZ, pupil_K1, image_K1, FFTW_FORWARD, FFTW_MEASURE);
    plan_K2 = fftw_plan_dft_2d(SZ, SZ, pupil_K2, image_K2, FFTW_FORWARD, FFTW_MEASURE);

    // create an image in shared memory
    ImageStreamIO_createIm(imarray, "shei_k1", naxis, imsize, atype, shared, NBkw);
    ImageStreamIO_createIm(imarray+1, "shei_k2", naxis, imsize, atype, shared, NBkw);


    // Writes an image, with random noise on top.
    while (1)
    {
        make_image(imarray, pupil_K1, image_K1, plan_K1, 1.0, 1.0);
        make_image(imarray+1, pupil_K2, image_K2, plan_K2, 0.9, 0.6);
        phase += 0.02;
        phase = fmod(phase, 2*M_PI);
        usleep(dtus);           // Wait 10ms
    }
    return 0;
}

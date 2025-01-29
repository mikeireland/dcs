#include <complex.h> //Needed so that fftw complex numbers can be added.
#include <fftw3.h>
#include <pthread.h>
#include <ImageStreamIO.h>

class ForwardFt {
public:
    int subim_sz;
    double *subim;
    fftw_complex *ft;
    fftw_plan plan;
    pthread_t thread;

    ForwardFt(IMAGE * subarray) {
        // Sanity check that we actually have a 2D , square image
        if (subarray->md->naxis != 2) {
            throw std::runtime_error("Subarray is not 2D");
        }
        subim_sz = subarray->md->size[0];
        if (subarray->md->size[1] != subim_sz) {
            throw std::runtime_error("Subarray is not square");
        }
        // Allocate memory for the subimage and the Fourier transform
        subim = (double*) fftw_malloc(sizeof(double) * subim_sz * subim_sz);
        ft = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * subim_sz * (subim_sz / 2 + 1));
        // Create the plan
        plan = fftw_plan_dft_r2c_2d(subim_sz, subim_sz, subim, ft, FFTW_MEASURE);
    }
    
    void spawn() {
        pthread_create(&thread, NULL, &ForwardFt::start, this);
    }
    static void* start(void* arg) {
        return NULL;
    }
};
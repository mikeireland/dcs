#include "heimdallr.h"

ForwardFt::ForwardFt(IMAGE * subarray_in) {
    subarray = subarray_in;
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
    window = (double*) fftw_malloc(sizeof(double) * subim_sz * subim_sz);
    power_spectrum = (double*) fftw_malloc(sizeof(double) * subim_sz * (subim_sz / 2 + 1));
    power_spectra[0] = (double*) fftw_malloc(sizeof(double) * subim_sz * (subim_sz / 2 + 1) * MAX_N_PS_BOXCAR);
    // Initialise the window to 1.0 everywhere
    for (unsigned int ii=0; ii<subim_sz; ii++) {
        for (unsigned int jj=0; jj<subim_sz; jj++) {
            window[ii*subim_sz + jj] = 1.0;
        }
        // Also initialise the power spectrum array
        for (unsigned int jj=0; jj<subim_sz * (subim_sz / 2 + 1); jj++) {
            for (int kk=0; kk<MAX_N_PS_BOXCAR; kk++) {
                power_spectra[kk][ii*subim_sz + jj] = 0.0;
            }
            power_spectrum[ii*subim_sz + jj] = 0.0;
        }
    }
    ft = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * subim_sz * (subim_sz / 2 + 1));
    // Create the plan
    plan = fftw_plan_dft_r2c_2d(subim_sz, subim_sz, subim, ft, FFTW_MEASURE);
}
    
void ForwardFt::spawn() {
    pthread_create(&thread, NULL, &ForwardFt::start, this);
}

void ForwardFt::join() {
    mode = FT_STOPPING;
    pthread_join(thread, NULL);
}

void* ForwardFt::start(void* arg) {
    int ii_shift, jj_shift;
    ForwardFt* self = static_cast<ForwardFt*>(arg);
    self->cnt = self->subarray->md->cnt0;
    while (self->mode != FT_STOPPING) {
        if (self->subarray->md->cnt0 != self->cnt) {
            if ((self->subarray->md->cnt0 > self->cnt+1) && (self->mode == FT_RUNNING)) {
                std::cout << "Missed frame" << self->subarray->md->cnt0 << self->cnt << std::endl;
                self->nerrors++;
            }
            self->mode = FT_RUNNING;

            // Copy the data from the IMAGE subarray to the subimage
            for (unsigned int ii=0; ii<self->subim_sz; ii++) {
                for (unsigned int jj=0; jj<self->subim_sz; jj++) {
                    ii_shift = (ii + self->subim_sz/2) % self->subim_sz;
                    jj_shift = (jj + self->subim_sz/2) % self->subim_sz;
                    self->subim[ii_shift*self->subim_sz + jj_shift] = 
                        self->subarray->array.F[ii*self->subim_sz + jj]
                            * self->window[ii*self->subim_sz + jj];
                }
            }
            // Do the FFT, and then indicate that the frame has been processed
            fftw_execute(self->plan);

            // Compute the power spectrum. Other than SNR purposes, this isn't 
            // time critical, but doesn't take long wo we can do it here.
            int next_ps_ix = (self->subarray->md->cnt0 + 1) % MAX_N_PS_BOXCAR;
            for (unsigned int ii=0; ii<self->subim_sz; ii++) {
                for (unsigned int jj=0; jj<self->subim_sz / 2 + 1; jj++) {
                    self->power_spectra[next_ps_ix][ii*self->subim_sz + jj] = 
                        self->ft[ii*(self->subim_sz/2 + 1) + jj][0] * self->ft[ii*(self->subim_sz/2 + 1) + jj][0] +
                        self->ft[ii*(self->subim_sz/2 + 1) + jj][1] * self->ft[ii*(self->subim_sz/2 + 1) + jj][1];
                    self->power_spectrum[ii*self->subim_sz + jj] += 
                        self->power_spectra[next_ps_ix][ii*self->subim_sz + jj]/MAX_N_PS_BOXCAR -
                        self->power_spectra[self->ps_index][ii*self->subim_sz + jj]/MAX_N_PS_BOXCAR;
                }
            }
            self->ps_index = next_ps_ix;
            self->cnt++;

            //std::cout << self->subarray->name << ": " << self->cnt << std::endl;
        } else usleep(100); //!!! Need a semaphore here if "nearly" ready for the FT
    }
    return NULL;
}
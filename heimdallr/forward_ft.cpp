#include "heimdallr.h"
//#define PRINT_TIMING

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
    // Allocate memory for the Fourier transform and plan it.
    ft = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * subim_sz * (subim_sz / 2 + 1));
    subim = (double*) fftw_malloc(sizeof(double) * subim_sz * subim_sz);
 
    // Create the plan
    plan = fftw_plan_dft_r2c_2d(subim_sz, subim_sz, subim, ft, FFTW_MEASURE);

    // Allocate memory for the subimage
    window = (double*) fftw_malloc(sizeof(double) * subim_sz * subim_sz);
    power_spectrum = (double*) fftw_malloc(sizeof(double) * subim_sz * (subim_sz / 2 + 1));
    //power_spectra[0] = (double*) fftw_malloc(sizeof(double) * subim_sz * (subim_sz / 2 + 1) * MAX_N_PS_BOXCAR);
    for (int ii=0; ii<MAX_N_PS_BOXCAR; ii++) {
        power_spectra[ii] = (double*) fftw_malloc(sizeof(double) * subim_sz * (subim_sz / 2 + 1));
        //power_spectra[ii-1] + subim_sz * (subim_sz / 2 + 1);
    }
    // Initialise the window to 1.0 everywhere
    for (unsigned int ii=0; ii<subim_sz; ii++) {
        for (unsigned int jj=0; jj<subim_sz; jj++) {
            window[ii*subim_sz + jj] = 1.0;
        }
        // Also initialise the power spectrum array
        for (unsigned int jj=0; jj<subim_sz / 2 + 1; jj++) {
            for (int kk=0; kk<MAX_N_PS_BOXCAR; kk++) {
                power_spectra[kk][ii*(subim_sz/2+1) + jj] = 0.0;
            }
            power_spectrum[ii*(subim_sz/2+1) + jj] = 0.0;
        }
    }
}
    
void ForwardFt::start() {
    thread = std::thread(&ForwardFt::loop, this);
}

void ForwardFt::stop() {
    mode = FT_STOPPING;
    if (thread.joinable()) thread.join();
}

void ForwardFt::loop() {
#ifdef PRINT_TIMING
    timespec now, then;
#endif
    unsigned int ii_shift, jj_shift, szj;
    cnt = subarray->md->cnt0;
    while (mode != FT_STOPPING) {
        if (subarray->md->cnt0 != cnt) {
            // Put this here just in case there is a re-start with a new size. Unlikely!
            szj = subim_sz/2 + 1;
            if ((subarray->md->cnt0 > cnt+1) && (mode == FT_RUNNING)) {
                std::cout << "Missed frame" << subarray->md->cnt0 << cnt << std::endl;
                nerrors++;
            }
            mode = FT_RUNNING;

            // Copy the data from the IMAGE subarray to the subimage
#ifdef PRINT_TIMING
            clock_gettime(CLOCK_REALTIME, &then);
#endif
            for (unsigned int ii=0; ii<subim_sz; ii++) {
                for (unsigned int jj=0; jj<subim_sz; jj++) {
                    ii_shift = (ii + subim_sz/2) % subim_sz;
                    jj_shift = (jj + subim_sz/2) % subim_sz;
                    subim[ii_shift*subim_sz + jj_shift] = 
                        subarray->array.F[ii*subim_sz + jj]
                            * window[ii*subim_sz + jj];
                }
            }
            // Do the FFT, and then indicate that the frame has been processed
            fftw_execute(plan);

#ifdef PRINT_TIMING
            clock_gettime(CLOCK_REALTIME, &now);
            if (then.tv_sec == now.tv_sec)
                std::cout << "Window and FFT time: " << now.tv_nsec-then.tv_nsec << std::endl;
            then = now;
#endif

            // Compute the power spectrum. Other than SNR purposes, this isn't 
            // time critical, but doesn't take long so we can do it here. 
            int next_ps_ix = (subarray->md->cnt0 + 1) % MAX_N_PS_BOXCAR;
            for (unsigned int ii=0; ii<subim_sz; ii++) {
                for (unsigned int jj=0; jj<szj; jj++) {
                    power_spectra[next_ps_ix][ii*szj + jj] = 
                        ft[ii*szj + jj][0] * ft[ii*szj + jj][0] +
                        ft[ii*szj + jj][1] * ft[ii*szj + jj][1];
                    power_spectrum[ii*szj + jj] += 
                        power_spectra[next_ps_ix][ii*szj + jj]/MAX_N_PS_BOXCAR -
                        power_spectra[ps_index][ii*szj + jj]/MAX_N_PS_BOXCAR;
                }
            }
            // Compute the power spectrum bias and instantaneous bias.
            power_spectrum_bias=0;
            power_spectrum_inst_bias=0;
            for (unsigned int ii=subim_sz/2-subim_sz/8; ii<subim_sz/2+subim_sz/8; ii++) {
                for (unsigned int jj=szj - subim_sz/8; jj<szj; jj++) {
                    power_spectrum_bias += power_spectrum[ii*szj + jj];
                    power_spectrum_inst_bias += power_spectra[next_ps_ix][ii*szj + jj];
                }
            }
            power_spectrum_bias /= subim_sz*subim_sz/32; //two squares 1/8 of the subim
            power_spectrum_inst_bias /= subim_sz*subim_sz/32; //two squares 1/8 of the subim
            ps_index = next_ps_ix;

#ifdef PRINT_TIMING
            clock_gettime(CLOCK_REALTIME, &now);
            if (then.tv_sec == now.tv_sec)
                std::cout << "PS time: " << now.tv_nsec-then.tv_nsec << std::endl;
            then = now;
#endif

            // As long as this is the same type as cnt0, it should wrap around correctly
            // The reason it is here and not before power spectrum computation is because we need at
            // lease 1 power spectrum in order for the group delay.
            cnt++;

            //std::cout << subarray->name << ": " << cnt << std::endl;
        } else usleep(100); //!!! Need a semaphore here if "nearly" ready for the FT
    }
}
#include "heimdallr.h"

long unsigned int ft_cnt=0;
long unsigned int nerrors=0;
double x_px_K1[N_BL], y_px_K1[N_BL], x_px_K2[N_BL], y_px_K2[N_BL], sign[N_BL], gd_to_K1=1.0;
dcomp K1_phasor[N_BL], K2_phasor[N_BL];

// Initialise variables assocated with baselines.
void initialise_baselines(){
    for (int i=0; i<N_BL; i++){
        baselines[i].gd=0;
        baselines[i].pd=0;
        baselines[i].gd_snr=0;
        baselines[i].pd_snr=0;
        baselines[i].n_gd_boxcar=MAX_N_GD_BOXCAR;
        baselines[i].ix_gd_boxcar=MAX_N_GD_BOXCAR-1;
        baselines[i].gd_phasor = 0;
        for (int j=0; j<MAX_N_GD_BOXCAR; j++){
            baselines[i].gd_phasors[j] = 0;
        }
    }
    for (int i=0; i<N_CP; i++){
        bispectra[i].n_bs_boxcar=MAX_N_BS_BOXCAR;
        bispectra[i].ix_bs_boxcar=MAX_N_BS_BOXCAR-2;
        bispectra[i].bs_phasor = 0;
        bispectra[i].closure_phase = 0;
        for (int j=0; j<MAX_N_BS_BOXCAR; j++){
            bispectra[i].bs_phasors[j] = 0;
        }
    }
    float pix = config["geometry"]["pix"].value_or(24.0);
    float wave_K1 = config["wave"]["K1"].value_or(2.05);
    float wave_K2 = config["wave"]["K2"].value_or(2.25);
    gd_to_K1 = wave_K2/(wave_K1-wave_K2)/2/M_PI;
    for (int bl=0; bl<N_BL; bl++){
        float bl_x = config["geometry"]["beam_x"][baseline2beam[bl][1]].value_or(0.0) -
            config["geometry"]["beam_x"][baseline2beam[bl][0]].value_or(0.0);
        float bl_y = config["geometry"]["beam_y"][baseline2beam[bl][1]].value_or(0.0) -
            config["geometry"]["beam_y"][baseline2beam[bl][0]].value_or(0.0);
        if (bl_x < 0){
            bl_x = -bl_x;
            bl_y = -bl_y;
            sign[bl] = -1;
        } else sign[bl] = 1;
        x_px_K1[bl] = bl_x * pix / wave_K1 * K1ft->subim_sz;
        y_px_K1[bl] = bl_y * pix / wave_K1 * K1ft->subim_sz;
        x_px_K2[bl] = bl_x * pix / wave_K2 * K2ft->subim_sz;
        y_px_K2[bl] = bl_y * pix / wave_K2 * K2ft->subim_sz;
        if (bl_y < 0){
            y_px_K1[bl] += K1ft->subim_sz;
            y_px_K2[bl] += K2ft->subim_sz;
        }
        std::cout << "Baseline: " << bl << " x_px_K1: " << x_px_K1[bl] << " y_px_K1: " << y_px_K1[bl] << std::endl;
    }
}

// The main fringe tracking function
void* fringe_tracker(void* arg){
    using namespace std::complex_literals;
    long x_px, y_px, stride;
    initialise_baselines();
    ft_cnt = K1ft->cnt;
    while(servo_mode != SERVO_STOP){
        // Wait for the next frame to be ready in K1
        while(K1ft->cnt == ft_cnt || K2ft->cnt == ft_cnt){
            usleep(50); //!!! Need to be more sophisticated here
        }
        // Check for missed frames
        if (K1ft->cnt > ft_cnt+1 || K2ft->cnt > ft_cnt+1){
            std::cout << "Missed frame! K1: " << K1ft->cnt << " K2: " 
                << K2ft->cnt << " FT: " << ft_cnt << std::endl;
            nerrors++;
        }
        ft_cnt++;
        // Extract the phases from the Fourier transforms 
        //std::cout << ft_cnt << std::endl;
        for (int bl=0; bl<N_BL; bl++){
            int next_gd_ix = (baselines[bl].ix_gd_boxcar + 1) % baselines[bl].n_gd_boxcar;
            // Use the peak of the splodge to compute the phase
            x_px = lround(x_px_K1[bl]);
            y_px = lround(y_px_K1[bl]);
            stride = K1ft->subim_sz/2 + 1;
            K1_phasor[bl] = K1ft->ft[y_px*stride + x_px][0] + 
                1i*K1ft->ft[y_px*stride + x_px][1]*sign[bl];
            x_px = lround(x_px_K2[bl]);
            y_px = lround(y_px_K2[bl]);
            stride = K2ft->subim_sz/2 + 1;
            K2_phasor[bl] = K2ft->ft[y_px*stride + x_px][0] + 
                1i*K2ft->ft[y_px*stride + x_px][1]*sign[bl];
            //std::cout << stride << " " << x_px << " " << y_px << std::endl;
            //std::cout << "Baseline: " << bl << " Phase: " << std::arg(K2_phasor[bl]) << std::endl;
            //std::cout << "Baseline: " << bl << " Abs: " << std::abs(K2_phasor[bl]) << std::endl;

            // Compute the group delay - units of wavelengths at K1
            baselines[bl].gd_phasors[next_gd_ix] = 
                K1_phasor[bl] * std::conj(K2_phasor[bl]);
            baselines[bl].gd_phasor += baselines[bl].gd_phasors[next_gd_ix] - 
                baselines[bl].gd_phasors[baselines[bl].ix_gd_boxcar];
            baselines[bl].gd = std::arg(baselines[bl].gd_phasor) * gd_to_K1;
            baselines[bl].ix_gd_boxcar = next_gd_ix;

            // Compute the unwrapped phase delay and signal to noise. We don't expect to 
            // actually use this unwrapped phase delay, but it is useful for debugging.
            double pdiff = std::fmod((std::arg(K1_phasor[bl])/2/M_PI - baselines[bl].pd + 0.5), 1.0) - 0.5;
            baselines[bl].pd += pdiff;
        }
        // Now we have the group delays, we can regularise by multipliying by the  
        // I6gd matrix. Then in a Bayesian approach, we use the phase and knowledge
        // of the previous state to compute the true delay in a nonlinear way. 


        //!!! Add code to do the fringe tracking

        // Now we sanity check by computing the bispectrum and closure phases.
        // K1 only... (as this is a quick-look sanity check)
        for (int cp=0; cp<N_CP; cp++){
            int next_cp_ix = (bispectra[cp].ix_bs_boxcar + 1) % bispectra[cp].n_bs_boxcar;
            int bl1 = closure2bl[cp][0];
            int bl2 = closure2bl[cp][1];
            int bl3 = closure2bl[cp][2];
            bispectra[cp].bs_phasors[next_cp_ix] = 
                K1_phasor[bl1] * K1_phasor[bl2] * std::conj(K1_phasor[bl3]);
            bispectra[cp].bs_phasor += bispectra[cp].bs_phasors[next_cp_ix] - 
                bispectra[cp].bs_phasors[bispectra[cp].ix_bs_boxcar];
            bispectra[cp].closure_phase = std::arg(bispectra[cp].bs_phasor);
            //std::cout << "CP: " << cp << " Phase: " << bispectra[cp].closure_phase << std::endl;
        }

    }
    return NULL;
}
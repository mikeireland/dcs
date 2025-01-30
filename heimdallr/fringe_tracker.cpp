#include "heimdallr.h"

long unsigned int ft_cnt=0;
float x_px_K1[N_BL], y_px_K1[N_BL], x_px_K2[N_BL], y_px_K2[N_BL], sign[N_BL];
dcomp K1_phasor[N_BL], K2_phasor[N_BL];

// Initialise variables assocated with baselines.
void initialise_baselines(){
     for (int i=0; i<N_BL; i++){
        baselines[i].gd=0;
        baselines[i].pd=0;
        baselines[i].gd_snr=0;
        baselines[i].pd_snr=0;
        baselines[i].n_gd_boxcar=0;
        baselines[i].gd_phasor = 0;
        for (int j=0; j<MAX_N_GD_BOXCAR; j++){
            baselines[i].gd_phasors[j] = 0;
        }
    }
    float pix = config["geometry"]["pix"].value_or(24.0);
    float wave_K1 = config["wave"]["K1"].value_or(2.05);
    float wave_K2 = config["wave"]["K2"].value_or(2.25);
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
    }
}

// The main fringe tracking function
void* fringe_tracker(void* arg){
    float x_px, y_px;
    initialise_baselines();
    while(servo_mode != SERVO_STOP){
        // Wait for the next frame to be ready in K1
        while(K1ft->cnt == ft_cnt || K2ft->cnt == ft_cnt){
            usleep(50); //!!! Need to be more sophisticated here
        }
        // Extract the phases from the Fourier transforms
        for (int bl=0; bl<N_BL; bl++){
            // Use the peak of the splodge to compute the phase
        }

        //!!! Add code to do the fringe tracking
    }
    return NULL;
}
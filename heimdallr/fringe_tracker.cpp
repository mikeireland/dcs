#include "heimdallr.h"

long unsigned int ft_cnt=0, cnt_since_init=0;
long unsigned int nerrors=0;
double gd_to_K1=1.0;

// Local baseline variables
double x_px_K1[N_BL], y_px_K1[N_BL], x_px_K2[N_BL], y_px_K2[N_BL], sign[N_BL];
dcomp K1_phasor[N_BL], K2_phasor[N_BL];

// A 6x6 matrix for the weights of phase and group delay
Eigen::Matrix<double, N_BL, N_BL> Wpd, Wgd;
Eigen::Matrix<double, N_BL, 1> pd_filtered, gd_filtered;

// Convenience matrices and vectors.
// A 4x4 matrix of zeros to store the diagonal.
Eigen::Matrix<double, N_TEL, N_TEL> singularDiag = Eigen::Matrix<double, N_TEL, N_TEL>::Zero();

// A 4x4 identity matrix.
Eigen::Matrix4d I4 = Eigen::Matrix4d::Identity();

// The search vector.
Eigen::Vector4d search_vector_scale(-2.75,-1.75,1.25,3.25);

// Make the pseudo-inverse matrix needed to project onto delay line (telescope) space.
#define NUMERIC_LIMIT 2e-6
Eigen::Matrix4d make_pinv(Eigen::Matrix<double, N_BL, N_BL> W, double threshold){
    using namespace Eigen;
    // This function computes the pseudo-inverse of the matrix M^T *  W * M, using the
    // SVD method. The threshold is used to set the minimum eigenvalue, and the
    // minimum eigenvalue is used to set the minimum eigenvalue of the pseudo-inverse.
    // W * M_lacour is a 6x4 matrix, and M_lacour.transpose() * W * M_lacour is a 4x4 matrix.
    // Was ComputeThinU
   SelfAdjointEigenSolver<Matrix4d> es(M_lacour.transpose() * W * M_lacour);
    // Start with a diagonal vector of 4 zeros.
    for (int i=0; i<N_TEL; i++){
         if ((es.eigenvalues()(i) <= threshold) || (es.eigenvalues()(i) < NUMERIC_LIMIT)){
             if (threshold > 0){
                 singularDiag(i,i) = es.eigenvalues()(i)/threshold/threshold;
            } else singularDiag(i,i) = 0;
        } else {
            singularDiag(i,i) = 1.0/es.eigenvalues()(i);
        }
    }
     return  es.eigenvectors() * singularDiag * es.eigenvectors().transpose();
}

// Initialise variables assocated with baselines.
void initialise_baselines(){
    cnt_since_init = 0;
    Wpd = Eigen::Matrix<double, N_BL, N_BL>::Zero();
    Wgd = Eigen::Matrix<double, N_BL, N_BL>::Zero();
    pd_filtered = Eigen::Matrix<double, N_BL, 1>::Zero();
    gd_filtered = Eigen::Matrix<double, N_BL, 1>::Zero();
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

// Reset the search
void reset_search(){
    // This function resets the search for the delay line and piezo.
    // It sets the delay line and piezo to zero, and sets the SNR values to zero.
    beam_mutex.lock();
    control_u.dl.setZero();
    control_u.piezo.setZero();
    control_u.dm_piston.setZero();
    control_u.search.setZero();
    control_u.dl_offload.setZero();
    control_u.search_delta = 0.01;
    control_u.omega_dl = 10;
    control_u.dit = 0.001;
    control_u.search_Nsteps = 0;
    control_u.steps_to_turnaround = 0;
    beam_mutex.unlock();
}

// The main fringe tracking function
void fringe_tracker(){
    timespec now, last_dl_offload;
    last_dl_offload.tv_sec = 0;
    last_dl_offload.tv_nsec = 0;
    using namespace std::complex_literals;
    Eigen::Matrix<double, N_BL, N_BL> I6gd, I6pd;
    Eigen::Matrix4d I4_search_projection;
    Eigen::Vector4d gd_tel, pd_tel, delay_tel;
    Eigen::Matrix<double, N_BL, 1> gd_bl, pd_bl, delay_bl;
    long x_px, y_px, stride;
    initialise_baselines();
    reset_search();
    ft_cnt = K1ft->cnt;
    while(servo_mode != SERVO_STOP){
        cnt_since_init++; //This should "never" wrap around, as a long int is big.
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

            // Compute the unwrapped phase delay and signal to noise. 
            // Until SNR is high enough, pd_filtered is zero
            // The phase delay is in units of the K1 central wavelength. 
            double pdiff = std::fmod((std::arg(K1_phasor[bl])/2/M_PI - pd_filtered(bl) + 0.5), 1.0) - 0.5;
            baselines[bl].pd = pd_filtered(bl) + pdiff;

            // Now we need the gd_snr and pd_snr for this baseline. 
            baselines[bl].pd_snr = std::fabs(K1_phasor[bl])/std::sqrt(K1ft->power_spectrum_inst_bias);
            
            // The GD_phasor has a variance sqrt(baselines[bl].n_gd_boxcar) larger than a
            // single phasor, so we need to divide by that.
            baselines[bl].gd_snr = std::fabs(baselines[bl].gd_phasor)/
                std::sqrt(K1ft->power_spectrum_bias * K2ft->power_spectrum_bias)/
                std::sqrt(baselines[bl].n_gd_boxcar);    
                
            // Set the weight matriix (bl,bl) to the square of the SNR, unless 
            // the SNR is too low, in which case we set it to zero.
            Wgd(bl, bl) = baselines[bl].gd_snr*baselines[bl].gd_snr;
            Wpd(bl, bl) = baselines[bl].pd_snr*baselines[bl].pd_snr;
        }
        // Now we have the group delays and phase delays, we can regularise by using by the  
        // I6gd matrix and the I6pd matrix. No short-cuts!
        // Fill a Vector of baseline group and phase delay.
        //I6gd = M_lacour * make_pinv(Wgd, 0) * M_lacour.transpose() * Wgd;
        //I6pd = M_lacour * make_pinv(Wpd, 0) * M_lacour.transpose() * Wpd;
        I4_search_projection = I4 - M_lacour_dag * I6gd * M_lacour;
        for (int bl=0; bl<N_BL; bl++){
            gd_bl(bl) = baselines[bl].gd;
            pd_bl(bl) = baselines[bl].pd;
        }   
        gd_filtered = I6gd * gd_bl;
        gd_tel = M_lacour_dag * gd_filtered;
        pd_filtered = I6pd * pd_bl;
        pd_tel = M_lacour_dag * pd_filtered;

        // Multiply by the K1 wavelength config["wave"]["K1"].value_or(2.05)

        // Do the Fringe tracking! The error signal is the "delay" variable.
        if (servo_mode==SERVO_PID){
            // Compute the piezo control signal. !!!

            // Apply the signal to the DM! !!!

        }

        // Now do the delay line control. This is slower, so occurs after the servo.
        // Compute the search sign.
        unsigned int search_level = 0;
        unsigned int index = control_u.search_Nsteps/control_u.steps_to_turnaround + 1;
        while (index >>= 1) ++search_level;
        control_u.search += control_u.search_delta * (1.0 - (search_level % 2) * 2.0)
            * search_vector_scale;

        // Compute the delay line offload.
        control_u.dl_offload -= control_u.omega_dl * control_u.dit * control_u.dm_piston;

        // Apply the DL offload if enough time has passed since the last offload. !!! Remove 2 zeros.
        clock_gettime(CLOCK_REALTIME, &now);
        if (now.tv_sec > last_dl_offload.tv_sec || 
            (now.tv_sec == last_dl_offload.tv_sec && now.tv_nsec - last_dl_offload.tv_nsec > 1000000000)){
            set_delay_lines(control_u.dl_offload);
            last_dl_offload = now;
        }
    

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
}
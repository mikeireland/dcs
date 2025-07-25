#include "heimdallr.h"
//#define PRINT_TIMING
//#define PRINT_TIMING_ALL
//#define DEBUG
#define GD_THRESHOLD 3
#define PD_THRESHOLD 4
#define GD_SEARCH_RESET 5
#define MAX_DM_PISTON 0.3

using namespace std::complex_literals;

long unsigned int ft_cnt=0, cnt_since_init=0;
long unsigned int nerrors=0;
double gd_to_K1=1.0;

// Local baseline variables
double x_px_K1[N_BL], y_px_K1[N_BL], x_px_K2[N_BL], y_px_K2[N_BL], sign[N_BL];
dcomp K1_phasor[N_BL], K2_phasor[N_BL];

// A 6x6 matrix for the weights of phase and group delay
Eigen::Matrix<double, N_BL, N_BL> Wpd, Wgd, cov_pd, cov_gd;
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
#ifdef PRINT_TIMING_ALL
    timespec now, then;
    clock_gettime(CLOCK_REALTIME, &then);
#endif
    SelfAdjointEigenSolver<Matrix4d> es(M_lacour.transpose() * W * M_lacour);
#ifdef PRINT_TIMING_ALL
    clock_gettime(CLOCK_REALTIME, &now);
    if (then.tv_sec == now.tv_sec)
        std::cout << "SVD time: " << now.tv_nsec-then.tv_nsec << std::endl;
    then = now;
#endif
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
#ifdef PRINT_TIMING_ALL
    clock_gettime(CLOCK_REALTIME, &now);
    if (then.tv_sec == now.tv_sec)
        std::cout << "Thresholding time: " << now.tv_nsec-then.tv_nsec << std::endl;
#endif
    return  es.eigenvectors() * singularDiag * es.eigenvectors().transpose();
}

void set_dm_piston(Eigen::Vector4d dm_piston){
    // This function sets the DM piston to the given value.
    for(int i = 0; i < N_TEL; i++) {
            for (int j=0; j<144; j++){
            DMs[i].array.D[j] = dm_piston(i);
        }
        ImageStreamIO_sempost(&master_DMs[i], 1);
    }
}

// Initialise variables assocated with baselines.
void initialise_baselines(){
    cnt_since_init = 0;
    Wpd.setZero();
    Wgd.setZero();
    cov_gd.setZero();
    cov_pd.setZero();
    pd_filtered.setZero();
    gd_filtered.setZero();
    baselines.gd.setZero();
    baselines.pd.setZero();
    baselines.jump_needed.setZero();
    baselines.gd_snr.setZero();
    baselines.pd_snr.setZero();
    baselines.n_gd_boxcar=MAX_N_GD_BOXCAR;
    baselines.n_pd_boxcar=MAX_N_PD_BOXCAR;
    baselines.ix_gd_boxcar=0;
    baselines.ix_pd_boxcar=0;
    baselines.gd_phasor.setZero();
    baselines.pd_phasor.setZero();
    baselines.pd_phasor_boxcar_avg.setZero();
    baselines.pd_av_filtered.setZero();
    baselines.pd_av.setZero();

    // Reset the boxcar averages
    for (int i=0; i<MAX_N_GD_BOXCAR; i++){
        baselines.gd_phasor_boxcar[i].setZero();
    }

    for (int i=0; i<MAX_N_PD_BOXCAR; i++){
        baselines.pd_phasor_boxcar[i].setZero();
    }

    // Reset bispectra variables for K1 and K2
    for (int i=0; i<N_CP; i++){
        bispectra_K1[i].n_bs_boxcar=MAX_N_BS_BOXCAR;
        bispectra_K1[i].ix_bs_boxcar=0;
        bispectra_K1[i].bs_phasor = 0;
        bispectra_K1[i].closure_phase = 0;
        for (int j=0; j<MAX_N_BS_BOXCAR; j++){
            bispectra_K1[i].bs_phasors[j] = 0;
        }
        bispectra_K2[i].n_bs_boxcar=MAX_N_BS_BOXCAR;
        bispectra_K2[i].ix_bs_boxcar=0;
        bispectra_K2[i].bs_phasor = 0;
        bispectra_K2[i].closure_phase = 0;
        for (int j=0; j<MAX_N_BS_BOXCAR; j++){
            bispectra_K2[i].bs_phasors[j] = 0;
        }
    }
    float pix = config["geometry"]["pix"].value_or(24.0);
    float wave_K1 = config["wave"]["K1"].value_or(2.05);
    float wave_K2 = config["wave"]["K2"].value_or(2.25);
    gd_to_K1 = wave_K2/(wave_K2-wave_K1)/2/M_PI;

    for (int bl=0; bl<N_BL; bl++){
        // Set the offsets to the group delay
        baselines.gd_phasor_offset(bl) = 
            std::exp(-1.0i *config["servo"]["gd_phasor_offset"][bl].value_or(0.0)/gd_to_K1);

        // Set the x and y coordinates for extracting flux
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
        //std::cout << "Baseline: " << bl << " x_px_K1: " << x_px_K1[bl] << " y_px_K1: " << y_px_K1[bl] << std::endl;
        //std::cout << "Baseline: " << bl << " x_px_K2: " << x_px_K2[bl] << " y_px_K2: " << y_px_K2[bl] << std::endl;
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
    control_u.dit = 0.002;
    control_u.search_Nsteps = 0;
    control_u.steps_to_turnaround = 1000;
    control_u.test_beam=0;
    control_u.test_n=0;
    control_u.test_ix=0;
    control_u.test_value=0.1;
    beam_mutex.unlock();

    pid_settings.mutex.lock();
    pid_settings.kp = 0.5;
    pid_settings.ki = 0.0;
    pid_settings.kd = 0.0;
    pid_settings.integral = 0.0;
    pid_settings.dl_feedback_gain = 0.0;
    pid_settings.pd_offset_gain = 0.0;
    pid_settings.gd_gain = pid_settings.kp / MAX_N_GD_BOXCAR;
    pid_settings.mutex.unlock();
}

Eigen::Matrix<double, N_BL, 1> filter6(Eigen::Matrix<double, N_BL, N_BL> I6gd, Eigen::Matrix<double, N_BL, 1> x){
    // This function filters the input vector x using the I6gd matrix.
    // It returns the filtered vector.
    Eigen::Matrix<double, N_BL, 1> y;
    // Each positive element of x could be x-1, and each negative element
    // could be x+1. If we tried every combination, we would have 2^N_BL=64 combinations.
    y = I6gd * x;
    return y;
}

// The main fringe tracking function
void fringe_tracker(){
    timespec now, last_dl_offload;
    last_dl_offload.tv_sec = 0;
    last_dl_offload.tv_nsec = 0;
    using namespace std::complex_literals;
    Eigen::Matrix<double, N_BL, N_BL> I6gd, I6pd;
    Eigen::Matrix4d I4_search_projection;
    Eigen::Matrix<double, N_TEL, N_TEL> cov_gd_tel;
    Eigen::Matrix<double, N_TEL, N_TEL> cov_pd_tel;
    long x_px, y_px, stride;
    initialise_baselines();
    reset_search();
    set_dm_piston(Eigen::Vector4d::Zero());
    ft_cnt = K1ft->cnt;
    while(servo_mode != SERVO_STOP){
        cnt_since_init++; //This should "never" wrap around, as a long int is big.
        // Wait for the next frame to be ready in K1
        while(K1ft->cnt == ft_cnt || K2ft->cnt == ft_cnt){
            usleep(50); //!!! Need to be more sophisticated here
        }
        // Check for missed frames
        if (K1ft->cnt > ft_cnt+2 || K2ft->cnt > ft_cnt+2){
            std::cout << "Missed FT frames! K1: " << K1ft->cnt << " K2: " 
                << K2ft->cnt << " FT: " << ft_cnt << std::endl;
            nerrors++;
        }
        ft_cnt++;
#ifdef PRINT_TIMING
        timespec then;
        clock_gettime(CLOCK_REALTIME, &then);
#endif
        // Extract the phases from the Fourier transforms, one baseline
        // at a time. This could in principle be vectorised. 
        //std::cout << ft_cnt << std::endl;
        int gd_ix = baselines.ix_gd_boxcar;
        int pd_ix = baselines.ix_pd_boxcar;
        for (int bl=0; bl<N_BL; bl++){
            // Use the peak of the splodge to compute the phase
            x_px = lround(x_px_K1[bl]) % K1ft->subim_sz;
            y_px = lround(y_px_K1[bl]) % K1ft->subim_sz;
            stride = K1ft->subim_sz/2 + 1;
            K1_phasor[bl] = K1ft->ft[y_px*stride + x_px][0] + 
                1i*K1ft->ft[y_px*stride + x_px][1]*sign[bl];
            // Also fill in the V^2 from the power spectrum.
            baselines.v2_K1(bl) = (K1ft->power_spectrum[y_px*stride + x_px]-K1ft->power_spectrum_bias)
                /K1ft->power_spectrum[0] * 16;

            // Fill in the boxcar average of the K1 phasor.
            baselines.pd_phasor_boxcar_avg(bl) -= baselines.pd_phasor_boxcar[pd_ix](bl);
            baselines.pd_phasor_boxcar[pd_ix](bl) = K1_phasor[bl];
            baselines.pd_phasor_boxcar_avg(bl) += baselines.pd_phasor_boxcar[pd_ix](bl);
            baselines.pd_av(bl) = std::arg(baselines.pd_phasor_boxcar_avg(bl)) /2/M_PI;
            
            x_px = lround(x_px_K2[bl]) % K2ft->subim_sz;
            y_px = lround(y_px_K2[bl]) % K2ft->subim_sz;
            stride = K2ft->subim_sz/2 + 1;
            K2_phasor[bl] = K2ft->ft[y_px*stride + x_px][0] + 
                1i*K2ft->ft[y_px*stride + x_px][1]*sign[bl];
            // Also fill in the V^2 from the power spectrum.
            baselines.v2_K2(bl) = (K2ft->power_spectrum[y_px*stride + x_px]-K2ft->power_spectrum_bias)
                /K2ft->power_spectrum[0] * 16;

            // Compute the group delay - units of wavelengths at K1
            baselines.gd_phasor(bl) -= baselines.gd_phasor_boxcar[gd_ix](bl);
            baselines.gd_phasor_boxcar[gd_ix](bl) = 
                K1_phasor[bl] * std::conj(K2_phasor[bl]);
            baselines.gd_phasor(bl) += baselines.gd_phasor_boxcar[gd_ix](bl);  
            baselines.gd(bl) = std::arg(baselines.gd_phasor(bl)*baselines.gd_phasor_offset(bl)) * gd_to_K1;

            // Compute the unwrapped phase delay and signal to noise. 
            // Until SNR is high enough, pd_filtered is zero
            // The phase delay is in units of the K1 central wavelength. 
            // !!! The 7.5 is a John Monnnier hack, due to fmod's treatment of negative numbers.
            //double pdiff = std::fmod((std::arg(K1_phasor[bl])/2/M_PI - pd_filtered(bl) + 7.5), 1.0) - 0.5;
            //baselines.pd(bl) = pd_filtered(bl) + pdiff;
            baselines.pd(bl) = std::fmod( (std::arg(K1_phasor[bl])/2/M_PI - baselines.pd_av_filtered(bl) + 1.5), 1.0) - 0.5;
            //baselines.pd(bl) = std::arg(K1_phasor[bl])/2/M_PI; // Temporarily overwrite this. !!!

            // Now we need the gd_snr and pd_snr for this baseline. 
            baselines.pd_snr(bl) = std::fabs(K1_phasor[bl])/std::sqrt(K1ft->power_spectrum_inst_bias);
            
            // The GD_phasor has a variance sqrt(baselines[bl].n_gd_boxcar) larger than a
            // single phasor, so we need to divide by that.
            baselines.gd_snr(bl) = std::fabs(baselines.gd_phasor(bl))/
                std::sqrt(K1ft->power_spectrum_bias * K2ft->power_spectrum_bias)/
                std::sqrt(baselines.n_gd_boxcar);    
                
            // Set the weight matriix (bl,bl) to the square of the SNR, unless 
            // the SNR is too low, in which case we set it to zero.
            if (baselines.gd_snr(bl) > GD_THRESHOLD){
                Wgd(bl, bl) = baselines.gd_snr(bl)*baselines.gd_snr(bl);
                cov_gd(bl, bl) = 1/baselines.gd_snr(bl)/baselines.gd_snr(bl);
            }
            else {
                Wgd(bl, bl) = 0;
                cov_gd(bl, bl) = 1e6; //!!!
            }
            if (baselines.pd_snr(bl) > PD_THRESHOLD){
                Wpd(bl, bl) = baselines.pd_snr(bl)*baselines.pd_snr(bl);
                cov_pd(bl, bl) = 1/baselines.pd_snr(bl)/baselines.pd_snr(bl);
            }
            else{
                Wpd(bl, bl) = 0;
                cov_pd(bl, bl) = 1e6; //!!!
            }
        }
        baselines.ix_gd_boxcar = (gd_ix + 1) % baselines.n_gd_boxcar;
        baselines.ix_pd_boxcar = (pd_ix + 1) % baselines.n_pd_boxcar;

#ifndef DEBUG
        // Now we have the group delays and phase delays, we can regularise by using by the  
        // I6gd matrix and the I6pd matrix. No short-cuts!
        // Fill a Vector of baseline group and phase delay.
        I6gd = M_lacour * make_pinv(Wgd, 0) * M_lacour.transpose() * Wgd;
        I6pd = M_lacour * make_pinv(Wpd, 0) * M_lacour.transpose() * Wpd;
        I4_search_projection = I4 - M_lacour_dag * I6gd * M_lacour; 
        gd_filtered = I6gd * baselines.gd;

        // !!! The following filtering should check for phasors that are too far out.
        pd_filtered = I6pd * baselines.pd;

        // Filter the average phase delay. !!! This doesn't work. Removing for now. !!!
        //baselines.pd_av_filtered = filter6(I6gd, baselines.pd_av);
        baselines.pd_av_filtered = baselines.pd_av;

        // The covariance matrix of baselines_gd and baselines_pd is given by a diagonal
        // matrix with the inverse of the SNR squared on the diagonal. We need to find the 
        // covariance of the telescope group and phase delays. !!! Unused in main loop (but copied)
        // to search logic below.

        //cov_gd_tel = M_lacour_dag * I6gd * cov_gd * I6gd.transpose() * M_lacour_dag.transpose();
        //cov_pd_tel = M_lacour_dag * I6pd * cov_pd * I6pd.transpose() * M_lacour_dag.transpose();
        

        // Here we have a conservative phase jump.
        // !!! We actually need to use the smaller uncertainties that come
        // after fitting, and use telescope rather than baseline delays. It can also only
        // occur if the condition remains true for a while, e.g. N_TO_JUMP frames of low
        // enough delay error.
        /*
        Eigen::Matrix<double, N_BL, 1> bl_pd_offset = M_lacour * control_a.pd_offset;
        if (cnt_since_init > MAX_N_PS_BOXCAR) for (int bl=0; bl<N_BL; bl++){
            double delay_error = std::sqrt(1/baselines.pd_snr(bl)/baselines.pd_snr(bl)/4/M_PI/M_PI +
                gd_to_K1*gd_to_K1/baselines.gd_snr(bl)/baselines.gd_snr(bl));
            //fmt::print("B{} pd_snr: {:.1e} gd_snr: {:.1e} delay_error: {:.3f}\n", bl,
            //    baselines[bl].pd_snr, baselines[bl].gd_snr, delay_error);
            if (delay_error < 0.4) 
            {
                if ((std::fabs(gd_filtered(bl)-pd_filtered(bl)+bl_pd_offset(bl)) > 0.8) &&
                (std::fabs(gd_filtered(bl)-pd_filtered(bl)+bl_pd_offset(bl))/delay_error > 3.0)){
                    baselines.jump_needed(bl) += 1;
                    if (baselines.jump_needed(bl) > N_TO_JUMP){
                        fmt::print("Phase jump: B{} pd: {:.3f} pdf: {:.3f} gd: {:.3f} delay_error: {:.3f} offset: {:3f}\n", bl,
                            baselines.pd(bl), pd_filtered(bl), gd_filtered(bl), delay_error, bl_pd_offset(bl));
                        pd_filtered(bl) += std::round(gd_filtered(bl)-pd_filtered(bl));
                        baselines.jump_needed(bl) = 0;
                    }
                } else  baselines.jump_needed(bl) = 0;
            }
        }
        */

        control_a.gd = M_lacour_dag * gd_filtered;
        control_a.pd = M_lacour_dag * pd_filtered;

        // Do the Fringe tracking! The error signal is the "delay" variable.
        // Only in this part do we ultiply by the K1 wavelength 
        // config["wave"]["K1"].value_or(2.05)

        // Just use a proportional servo on group delay with fixed gain of 0.5.
        if (servo_mode==SERVO_SIMPLE){
            // Compute the piezo control signal. T
            control_u.dm_piston += (pid_settings.kp * control_a.pd + 
                pid_settings.gd_gain * control_a.gd) * config["wave"]["K1"].value_or(2.05)/OPD_PER_DM_UNIT;
            // Center the DM piston.
            control_u.dm_piston = control_u.dm_piston - control_u.dm_piston.mean()*Eigen::Vector4d::Ones();
            // Limit it to no more than +/- MAX_DM_PISTON.
            control_u.dm_piston = control_u.dm_piston.cwiseMin(MAX_DM_PISTON);
            control_u.dm_piston = control_u.dm_piston.cwiseMax(-MAX_DM_PISTON);

        }
        // Make the test pattern.
        if (control_u.test_n > 0){
            if (control_u.test_ix < control_u.test_n){
                control_u.dm_piston(control_u.test_beam) = control_u.test_value;
            } else  {
                control_u.dm_piston(control_u.test_beam) = -control_u.test_value;
            } 
            control_u.test_ix = (control_u.test_ix + 1) % (2*control_u.test_n);
        }
        // Apply the signal to the DM!
        set_dm_piston(control_u.dm_piston);

#ifdef PRINT_TIMING
        clock_gettime(CLOCK_REALTIME, &now);
        if (then.tv_sec == now.tv_sec)
            std::cout << "FT Computation time: " << now.tv_nsec-then.tv_nsec << std::endl;
        then = now;
#endif

        cov_gd_tel = M_lacour_dag * I6gd * cov_gd * I6gd.transpose() * M_lacour_dag.transpose();
        double worst_gd_var = cov_gd_tel.diagonal().minCoeff();
        if (worst_gd_var < gd_to_K1*gd_to_K1/GD_SEARCH_RESET/GD_SEARCH_RESET){
            control_u.search_Nsteps=0;
            control_u.search.setZero();
        } else {
            // Now do the delay line control. This is slower, so occurs after the servo.
            // Compute the search sign.
            unsigned int search_level = 0;
            unsigned int index = control_u.search_Nsteps/control_u.steps_to_turnaround + 1;
            //This gives a logarithm base 2, so we search twice as far each turnaround. 
            while (index >>= 1) ++search_level;
            control_u.search += control_u.search_delta * (1.0 - (search_level % 2) * 2.0)
                * search_vector_scale;
            control_u.search_Nsteps++;
        }

        // Compute the delay line offload.
        control_u.dl_offload += (control_u.dit/((double)offload_time_ms*0.001)) * control_u.dm_piston * OPD_PER_DM_UNIT;

        // Apply the DL offload if enough time has passed since the last offload. !!! Remove 2 zeros.
        clock_gettime(CLOCK_REALTIME, &now);
        if (now.tv_sec > last_dl_offload.tv_sec || 
            (now.tv_sec == last_dl_offload.tv_sec && now.tv_nsec - last_dl_offload.tv_nsec > offload_time_ms*1000000)){
            if (offload_mode == OFFLOAD_NESTED){
                fmt::print("Offload: {} {} {} {}\n", control_u.dl_offload(0), control_u.dl_offload(1),
                    control_u.dl_offload(2), control_u.dl_offload(3));
                add_to_delay_lines(-control_u.dl_offload);
                control_u.dl_offload.setZero();
            }
            else if (offload_mode == OFFLOAD_GD) //!!! Was -1...
                add_to_delay_lines(-0.5*control_a.gd * config["wave"]["K1"].value_or(2.05));
            last_dl_offload = now;
        }

        // Servo over! Compute the phase delay offsets, by servoing to the difference betwee
        // pd and gd.
        //control_a.pd_offset += (control_a.gd - control_a.pd) * pid_settings.pd_offset_gain;
    
        // Now we sanity check by computing the bispectrum and closure phases.
        // K1 and K2, in case of tracking on resolved objects... 
        for (int cp=0; cp<N_CP; cp++){
            int K1_ix = bispectra_K1[cp].ix_bs_boxcar;
            int K2_ix = bispectra_K2[cp].ix_bs_boxcar;
            int bl1 = closure2bl[cp][0];
            int bl2 = closure2bl[cp][1];
            int bl3 = closure2bl[cp][2];
            bispectra_K1[cp].bs_phasor -= bispectra_K1[cp].bs_phasors[K1_ix];
            bispectra_K2[cp].bs_phasor -= bispectra_K2[cp].bs_phasors[K2_ix];
            bispectra_K1[cp].bs_phasors[K1_ix] = 
                K1_phasor[bl1] * K1_phasor[bl2] * std::conj(K1_phasor[bl3]);
            bispectra_K2[cp].bs_phasors[K2_ix] =
                K2_phasor[bl1] * K2_phasor[bl2] * std::conj(K2_phasor[bl3]);
            bispectra_K1[cp].bs_phasor += bispectra_K1[cp].bs_phasors[K1_ix];
            bispectra_K2[cp].bs_phasor += bispectra_K2[cp].bs_phasors[K2_ix];
            // Compute the closure phase.
            bispectra_K1[cp].closure_phase = std::arg(bispectra_K1[cp].bs_phasor);
            bispectra_K2[cp].closure_phase = std::arg(bispectra_K2[cp].bs_phasor);
            // Increment the counters.
            bispectra_K1[cp].ix_bs_boxcar = (bispectra_K1[cp].ix_bs_boxcar + 1) % bispectra_K1[cp].n_bs_boxcar;
            bispectra_K2[cp].ix_bs_boxcar = (bispectra_K2[cp].ix_bs_boxcar + 1) % bispectra_K2[cp].n_bs_boxcar;
            //std::cout << "CP: " << cp << " Phase: " << bispectra[cp].closure_phase << std::endl;
        }

#endif
    }
}

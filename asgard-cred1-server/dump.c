  // =====================================
  /* Set up higher priority to the attached process?

  uid_t ruid; // Real UID (= user launching process at startup)
  uid_t euid; // Effective UID (= owner of executable at startup)
  uid_t suid; // Saved UID (= owner of executable at startup)
        
  int RT_priority = 70; //any number from 0-99
  struct sched_param schedpar;
  int ret;

  getresuid(&ruid, &euid, &suid);
  ret = seteuid(ruid);   // normal user privileges

  schedpar.sched_priority = RT_priority;
#ifndef __MACH__
  ret = seteuid(euid); //This goes up to maximum privileges
  sched_setscheduler(0, SCHED_FIFO, &schedpar); //other option is SCHED_RR, might be faster
  ret = seteuid(ruid); //Go back to normal privileges
#endif
  */
  // =====================================

// global variable
IMAGE *shm_ROI_ndmr = NULL;    // pointer to the SHM to temporarily store NDMR ROIs
// in shm_setup()
    // ======================== the case of NDMR ROI =======================
    if (shm_ROI_ndmr != NULL) {
      ImageStreamIO_destroyIm(shm_ROI_ndmr);
      free(shm_ROI_ndmr);
      shm_ROI_ndmr = NULL;
    }

    naxis = 3;
    shm_ROI_ndmr = (IMAGE*) malloc(sizeof(IMAGE) * nroi);
  
    for (int ri = 0; ri < nroi; ri++) {
      nd_roisize[0] = ROI[ri].xsz;
      nd_roisize[1] = ROI[ri].ysz;
      nd_roisize[2] = ROI[ri].nrs;

      ImageStreamIO_createIm_gpu(&shm_ROI_ndmr[ri], ROI[ri]._name, naxis,
				 nd_roisize, _DATATYPE_UINT16, -1, shared,
				 IMAGE_NB_SEMAPHORE, NBkw, MATH_DATA);
      // init all values in this shm
      for (int ii = 0; ii < ROI[ri].npx * ROI[ri].nrs; ii++)
	shm_ROI_ndmr[ri].array.UI16[ii] = 0;
    }

// in free_shm()
    if (shm_ROI_ndmr != NULL) {
      for (ii= 0; ii < nroi; ii++) {
	ImageStreamIO_destroyIm(&shm_ROI_ndmr[ii]);
      }
      free(shm_ROI_ndmr);
      shm_ROI_ndmr = NULL;
    }

// NDRM mode early implementation - a bit complicated!

	  /* else {
	    // (1) insert the latest frame into the NDMR buffer
	    fii0 = live_ROI_index[ri];
	    ndmr_live_ptr = shm_ROI_ndmr[ri].array.UI16 + fii0 * ROI[ri].npx;
	    liveroi_ptr = shm_ROI_live[ri].array.SI32; // live ROI data pointer
	    
	    shm_ROI_live[ri].md->write = 1;

	    for (jj = 0; jj < ROI[ri].ysz; jj++) {
	      for (ii = 0; ii < ROI[ri].xsz; ii++) {
		ndmr_live_ptr[jj*roi_xsz+ii] = liveimg_ptr[(jj+y0) * cam_xsz + ii+x0];
		// ---- load the current image in the live ROI
		switch (liveindex) { // to accommodate for the reset special case
		case 0:
		  liveroi_ptr[jj*xsz+ii] = 0;
		  break;
		case 1:
		  liveroi_ptr[jj * roi_xsz + ii] = ndmr_live_ptr[jj*xsz+ii];
		  break;
		default:
		  liveroi_ptr[jj * roi_xsz + ii] = tsig[0] * ndmr_live_ptr[jj*xsz+ii];
		  break;
		}
	      }
	    }

	    // (2) compute the live image
	    fii1 = (fii0 + ROI[ri].nrs - 1) % ROI[ri].nrs;
	    fii2 = (fii0 + ROI[ri].nrs - 2) % ROI[ri].nrs;

	    ndmr_live_ptr = shm_ROI_ndmr[ri].array.UI16 + fii1 * ROI[ri].npx;
	    ndmr_live_ptr2 = shm_ROI_ndmr[ri].array.UI16 + fii2 * ROI[ri].npx;
	    
	    for (jj = 0; jj < ROI[ri].ysz; jj++) {
	      for (ii = 0; ii < ROI[ri].xsz; ii++) {
		// ---- load the current image in the live ROI
		switch (liveindex) {
		case 0:
		  break; // empty frame

		case 1:
		  liveroi_ptr[jj * roi_xsz + ii] -= ndmr_live_ptr[jj * roi_xsz + ii];
		  break;

		default:
		  liveroi_ptr[jj * roi_xsz + ii] +=				\
		    tsig[1] * ndmr_live_ptr[jj * roi_xsz + ii] +	\
		    tsig[2] * ndmr_live_ptr2[jj * roi_xsz + ii];
		  break;
		}
	      }
	    }
	    // (3) push that image to the ROI_live
	    shm_ROI_live[ri].md->write = 0;
	    ImageStreamIO_sempost(&shm_ROI_live[ri], -1);
	    shm_ROI_live[ri].md->cnt0++;
	    shm_ROI_live[ri].md->cnt1++;

	    // (3) increment all relevant indices
	    live_ROI_index[ri] = (live_ROI_index[ri] + 1) % ROI[ri].nrs;
	  } */

//pthread_t tid_split; // thread ID for the image ROI splitting

/* =========================================================================
 *                     Save as a fits data-cube thread
 * ========================================================================= */
void* save_cube_to_fits(void *_cube) {
  fitsfile *fptr;
  int status;
  int bitpix = USHORT_IMG;
  long naxis = 3;
  long naxes[3] = {camconf->width, camconf->height, camconf->nbr_hlf};

  long fpixel; //, nel;
  unsigned short *cube = (unsigned short*) _cube;
  struct timespec tnow;  // time since epoch
  struct tm *uttime;     // calendar time

  struct stat st;
  
  char fname[300];  // path + name of the fits file to write
  // get the time - to give the file a unique name
  clock_gettime(CLOCK_REALTIME, &tnow);  // elapsed time since epoch
  uttime = gmtime(&tnow.tv_sec);         // translate into calendar time

  // cubes saved in ~/Music/ for now
  sprintf(savedir, "%s/Music/%04d%02d%02d/", getenv("HOME"),
	  1900 + uttime->tm_year, 1 + uttime->tm_mon, uttime->tm_mday); 

  if (stat(savedir, &st) == -1)
    mkdir(savedir, 0700);
  sprintf(fname, "%s/T%02d:%02d:%02d.%09ld.fits", savedir,
	  uttime->tm_hour, uttime->tm_min, uttime->tm_sec, tnow.tv_nsec);

  // create a fits file
  fpixel = 1; // first pixel to write
  fits_create_file(&fptr, fname, &status);
  fits_create_img(fptr, bitpix, naxis, naxes, &status);
  
  fits_write_img(fptr, TUSHORT, fpixel, camconf->nbpix_cub, cube, &status);

  // fill in the proper keywords
  // fits_update_key(fptr, TLONG, "EXPOSURE", &tint, "Exposure time", &status);
  fits_update_key(fptr, TLONG, "CINDEX", &savecube_index, "Cube index", &status);
  fits_close_file(fptr, &status);
  savecube_index++;
  return NULL;
}

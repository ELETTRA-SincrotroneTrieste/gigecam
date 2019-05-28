//template.h
#include <sys/time.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifdef LIBFIT
#include <Fit.h>
#endif 

#define IMAGEPROC_MAX_REF 4
#define RAW_CALC          0   /* raw calculation fit */

#define IMAGEPROC_MAX_LEVELS 50

template <class T>
class ImageProc
{
public:
	ImageProc(int32_t height, int32_t width, int32_t num_cores); 
	~ImageProc() { 
								 
								 delete [] image;
	               delete [] hor_profile; delete [] ver_profile; 
	               delete [] hor_fit_profile; delete [] ver_fit_profile; 		
	               delete [] hor_weight; delete [] ver_weight;								 						 
	               delete [] imagetemp; delete [] imageicon; 
#ifdef LIBFIT
	               delete gauss_fit; delete gauss_fit_asymm;
								 delete confiteor_fit;								 
#endif

							 }


	/* store image into object, used external permutation array for image rotation/mirror */
	void push_permutation(const T *data_ptr, int32_t *perm_vec, uint32_t camera_cnt, uint32_t acq_cnt);
	void push_permutation_with_background(const T *data_ptr, int32_t *perm_vec, uint32_t camera_cnt, uint32_t acq_cnt, const T *background_ptr);

	/* double to int32_t rounding */
	int32_t cint(double x);
	/* calculate mirror permutation indexes */
	void permutation_mirrorh(int32_t *perm_vec);
	/* calculate rotation permutation indexes */
	void permutation_rotation(double rotation, int32_t *perm_vec);

	/* creset image */
	void reset(void);

	/* plot offset axis */
	void set_plot_offset_axis(bool data) { plot_offset_axis=data; }
	bool get_plot_offset_axis(void) { return plot_offset_axis; }

	/* flip upside down the image or mirror from left to right */
	void set_updown(bool data) { updown=data; }
	bool get_updown(void) { return updown; }
	void set_mirror(bool data) { mirror=data; }
	bool get_mirror(void) { return mirror; }

	/* image attributes */
	T* image; 
	double *hor_profile, *ver_profile;
	double *hor_fit_profile, *ver_fit_profile;		
	double *hor_weight, *ver_weight;  
	int32_t max_value, count_max_value;

	double hor_pos, ver_pos, area;
	double phase;
	double hor_sigma, ver_sigma, covar, saturation, intensity, hor_sign, ver_sign;

	double hor_calibration, ver_calibration; /* number of pixels per mm */
	double hor_pos_offset, ver_pos_offset;           /* mechanical offset */
	double ref_hor_pos_offset, ref_ver_pos_offset;   /* reference offset */
	
	uint32_t camera_counter, acquisition_counter, binning;

	/* set image size */
	bool set_size(int32_t h, int32_t v); 
	int32_t get_height(void) { return height; }
	int32_t get_width(void)  { return width; }



	/* enable image processing */
	bool set_enable_process(bool flag) { enable_process = flag; return true;}
	bool get_enable_process(void) { return enable_process; } 

	/* enable rotation filtering */
	bool set_enable_rotation_filter(bool flag) { enable_rotation_filter = flag; return true; }
	bool get_enable_rotation_filter(void) { return enable_rotation_filter; }

	bool set_enable_dis(bool flag) { enable_dis = flag; return true; }
	bool get_enable_dis(void) { return enable_dis; }

	/* order of the low pass filter (for automatic ROI detection) */
	bool set_lp_order(int32_t order);
	int32_t get_lp_order(void) { return lp_order; }
	/* number of horizontal and vertical cells used to find automatically the ROI */
	bool set_hor_grid(int32_t hor_grid);
	int32_t get_hor_grid(void) { return num_grid_hor; }
	bool set_ver_grid(int32_t ver_grid);
	int32_t get_ver_grid(void) { return num_grid_ver; }

	/* plot references in the image */
	void get_reference(int32_t *ref_h, int32_t *ref_v);
	bool set_reference(int32_t ref_h, int32_t ref_v);
	void get_raw_pos(int32_t *hor_pos, int32_t *ver_pos) /* for DIS */
		{ *hor_pos = raw_hor_pos; *ver_pos = raw_ver_pos;}   
	void set_ref_raw_pos(int32_t hor_pos, int32_t ver_pos) /* for DIS */
		{ ref_raw_hor_pos = hor_pos; ref_raw_ver_pos = ver_pos;} 
	void get_ref_raw_pos(int32_t *hor_pos, int32_t *ver_pos) /* for DIS */
		{ *hor_pos = ref_raw_hor_pos; *ver_pos = ref_raw_ver_pos;}

#ifdef LIBFIT	
	void set_algorithm(int32_t alg);
	int32_t get_algorithm(void) { return algorithm; }
	bool img_gauss_fit(void); 
	bool img_gauss_fit_asymm(void); 			
	bool img_confiteor_fit(void);		
	bool fit_hor_ok, fit_ver_ok;
	int32_t fit_error, fit_tout;
	int32_t get_fit_error(void) { return fit_error; }
	int32_t get_fit_tout(void) { return fit_tout; }
	void set_fit_tout(int32_t val) { if (val > 0) fit_tout = val; }
#endif	
	
	/* enable automatic reduction of ROI */
	bool set_enable_auto_roi(bool auto_roi_flag);
	bool get_enable_auto_roi(void) { return auto_roi; }
	int32_t *get_auto_roi(void) { return auto_roi_param; }

	/* set the intensity threshold for ROI search */
	bool set_roi_threshold(double thres);
	double get_roi_threshold(void) { return roi_threshold; }
	/* set roi search parameters (num_step,der_gain) */
	bool set_search_background_param(double *param);
	double *get_search_background_param(void) { return search_background_param; };

	/* plot automatic area of interest and position on image */
	void set_plot_mode(short flag) { plot_mode = flag; }
	short get_plot_mode(void) { return plot_mode; }
	bool set_plot_level(short val);
	short get_plot_level(void) { return plot_level; }

	/* enable fast calculation */
	void set_enable_fast_calc(bool flag) { fast_calc = flag; }
	bool get_enable_fast_calc(void) { return fast_calc; }

	/* get the parameters of the filtered image */
	int32_t get_imageicon_height(void) { return imageicon_height; }
	int32_t get_imageicon_width(void)  { return imageicon_width; }
	int32_t *imageicon;

	/* set manually ROI (hor left, ver up, hor right,  ver down) */
	bool set_manual_roi(int32_t *roi_val);
	int32_t *get_manual_roi(void) { return manual_roi_param; }
	bool set_enable_manual_roi( bool val);
	bool get_enable_manual_roi(void) { return manual_roi; }
	double img_hor_pos_offset, img_ver_pos_offset; /* image offset */

	/* absolute acquisition time */
	struct timeval acquisition_time;
	/* processing time in us. */
	int32_t processing_time;

private:

	bool updown, mirror, enable_rotation_filter;
	bool enable_process, plot_offset_axis, enable_dis;
	int32_t height, width;
	int32_t algorithm;

	Fit *gauss_fit,
	    *gauss_fit_asymm,
			*confiteor_fit;

	int32_t reference_idx;
	int32_t reference_hor[IMAGEPROC_MAX_REF],reference_ver[IMAGEPROC_MAX_REF];

  /* manual roi variables */
	bool manual_roi;          /* enable manual roi */
	int32_t manual_roi_param[4], auto_roi_param[4]; /* hor left, ver up, hor right,  ver down */

	int32_t raw_hor_pos, raw_ver_pos,
	        ref_raw_hor_pos, ref_raw_ver_pos;

	double roi_threshold;
	double search_background_param[2]; /* [0]: number of steps to search the background level
                                              [1]: derivative coeff (0->1) */
 	bool auto_roi,  /* enable automatic roi */
	     fast_calc; /* decimate automatically the image to speedup calculations */

	short plot_mode,plot_level; /* 0=no plot, 0x1=plot position axes, 0x3=plot calculation area */


	T* imagetemp;

	int32_t lp_order,
	     icon_background,
	     num_grid_hor, num_grid_ver,
	     ver_up, ver_down,
	     hor_left,hor_right;
	int32_t imageicon_height, imageicon_width;
	int32_t max_pixel_value;
	void plot_axis(int32_t cur_thres);
	void process(void);
	void dis_process(void);
	void find_roi(void);
	
};

/* 
 * ImageProc(int32_t h, int32_t w, int32_t num_cores)
 * h: image height
 * w: image width
 * num_cores: number of cores used for parallelizing fitting algorithms
 * Constructor
 */
template <class T>
ImageProc<T>::ImageProc(int32_t h, int32_t w, int32_t num_cores)
{

	image = new T[h*w];
	hor_profile = new double[w];
	memset(hor_profile, 0, sizeof(double));
	ver_profile = new double[h];
	memset(ver_profile, 0, sizeof(double));
	hor_fit_profile = new double[w];
	memset(hor_fit_profile, 0, sizeof(double));
	ver_fit_profile = new double[h];
	memset(ver_fit_profile, 0, sizeof(double));
	hor_weight = new double[w];
	ver_weight = new double[h];

	updown=false;mirror=false;enable_rotation_filter= false;

	height = h;width = w;

	/* disable automatic ROI detection */
	auto_roi = false;
	enable_dis = false;

	plot_mode = 0x0; /* don't plot anything */
	plot_level = 0;
	plot_offset_axis = false;

#ifdef LIBFIT	
	algorithm = RAW_CALC;
	gauss_fit = new Fit(GAUSS_FIT, 1, num_cores);
	gauss_fit_asymm = new Fit(GAUSS_FIT_ASYMM, 1, num_cores);
	confiteor_fit = new Fit(CONFITEOR2_FIT, 1, num_cores);
	fit_hor_ok = false;
	fit_ver_ok = false;
	fit_error = 0;
	fit_tout = 60;
 	gsl_set_error_handler_off();
#endif	
	
	hor_sign = 0;
	ver_sign = 0;

	raw_hor_pos = 0; raw_ver_pos = 0;
	ref_raw_hor_pos = 0; ref_raw_ver_pos = 0;

	/* low pass filter order 5, grid 50 */
	roi_threshold = 1;
	lp_order = 5;
	num_grid_hor = 50;
	if (num_grid_hor > width)
		 num_grid_hor = width;
	num_grid_ver = (int32_t) ((height*50)/width);
	if (num_grid_ver > height)
		 num_grid_ver = height;
	else if (num_grid_ver <= 0)
		 num_grid_ver = height;

	search_background_param[0] = 20;  /* number of step for automatic background suppression */
	search_background_param[1] = 0.5; /* derivative coeff. for area threshold */
	icon_background = 0;   /* background level of the automatic roi icon image */


	ver_up = 0; ver_down = height;
	hor_left = 0; hor_right = width;
	imagetemp = new T[(num_grid_hor + lp_order - 1)*(num_grid_ver + lp_order - 1)];
	imageicon = new int32_t[num_grid_hor*num_grid_ver];
	imageicon_height = num_grid_ver;
	imageicon_width = num_grid_hor;

	fast_calc = false;

	phase = 0;
	hor_calibration = 1; ver_calibration = 1; /* pixel * mm */
	hor_pos_offset = 0; ver_pos_offset = 0; /* mm */
	ref_hor_pos_offset = 0, ref_ver_pos_offset = 0; /* references */

	processing_time = 0;
	hor_pos = 0; ver_pos = 0; hor_sigma = 0; ver_sigma = 0; max_value = 0;
	covar = 0; count_max_value = 0;
	area = 0; saturation = 0;
	intensity = 0;

	reference_idx = 0;
	for (int32_t i = 0; i < IMAGEPROC_MAX_REF; i++) {
		reference_hor[i]= 0;
		reference_ver[i] = 0;
	}

	img_hor_pos_offset = width/2;
	img_ver_pos_offset = height/2;

	manual_roi = false;
	manual_roi_param[0] = 0;
	manual_roi_param[1] = 0;
	manual_roi_param[2] = width;
	manual_roi_param[3] = height;

	auto_roi_param[0] = 0;
	auto_roi_param[1] = 0;
	auto_roi_param[2] = width;
	auto_roi_param[3] = height;

	if (sizeof(T) == 1)
		max_pixel_value = 255;
	else if (sizeof(T) == 2)
		max_pixel_value = 4095;


}


template <class T>
void ImageProc<T>::reset(void)
{
	memset(image,0,sizeof(T)*height*width);
}


template <class T>
bool ImageProc<T>::set_size(int32_t h, int32_t v)
{

	if ((h == width) || (v == height))
		return true;

	if (hor_profile) {
		delete [] hor_profile; hor_profile = 0;
	}
	if ((hor_profile = new double[h]) == 0)
		return false;

	if (ver_profile) {
		delete [] ver_profile; ver_profile = 0;
	}

	if ((ver_profile = new double[v]) == 0)
		return false;

	if (hor_fit_profile) {
		delete [] hor_fit_profile; hor_fit_profile = 0;
	}

	if ((hor_fit_profile = new double[h]) == 0)
		return false;

	if (ver_fit_profile) {
		delete [] ver_fit_profile; ver_fit_profile = 0;
	}

	if ((ver_fit_profile = new double[v]) == 0)
		return false;

	if (hor_weight) {
		delete [] hor_weight; hor_weight = 0;
	}

	if ((hor_weight = new double[h]) == 0)
		return false;

	if (ver_weight) {
		delete [] ver_weight; ver_weight = 0;
	}

	if ((ver_weight = new double[v]) == 0)
		return false;

	if (image) {
		delete [] image; image = 0;
	}

	if ((image = new T[h*v]) == 0)
		return false;

	width = h;
	height = v;

	memset(image,0,sizeof(T)*height*width);

	if (num_grid_hor > width)
		 num_grid_hor = width;
	num_grid_ver = (int32_t) ((height*num_grid_hor)/width);
	if (num_grid_ver > height)
		num_grid_ver = height;
	else if (num_grid_ver <= 0)
		num_grid_ver = height;

	ver_up = 0; ver_down = height;
	hor_left = 0; hor_right = width;

	if (imagetemp) {
		delete [] imagetemp; imagetemp = 0;
	}

	if ((imagetemp = new T[(num_grid_hor + lp_order - 1)*(num_grid_ver + lp_order - 1)]) == 0)
		return false;

	memset(imagetemp,0,sizeof(T)*((num_grid_ver+(lp_order-1))*(num_grid_hor+(lp_order-1))));

	if (imageicon) {
		delete [] imageicon; imageicon = 0;
	}

	if ((imageicon = new int32_t[num_grid_hor*num_grid_ver]) == 0)
		return false;

	imageicon_height = num_grid_ver;
	imageicon_width = num_grid_hor;

	manual_roi = false;
	manual_roi_param[0] = 0;manual_roi_param[2] = width;
	manual_roi_param[1] = 0;manual_roi_param[3] = height;

	return true;

}


template <class T>
bool ImageProc<T>::set_enable_manual_roi(bool val)
{
 /*     val[0] = hor left, val[1]= ver up,
        val[2] = hor right, val[3] = ver down */
	if ((manual_roi_param[0] >= 0) &&
	    (manual_roi_param[0] < manual_roi_param[2]) &&
	    (manual_roi_param[2] <= width) &&
	    (manual_roi_param[1] >= 0) &&
	    (manual_roi_param[1] < manual_roi_param[3]) &&
	    (manual_roi_param[3] <= height)) {
		manual_roi = val;
		return true;
	}
	else {
		/* disable anyway */
		if (val == false) {
			manual_roi = false;
			manual_roi_param[0] = 0;
			manual_roi_param[1] = 0;
			manual_roi_param[2] = width;
			manual_roi_param[3] = height;
		}
		return false;
	}

}


/*
 *  set_plot_level
 */
template <class T>
bool ImageProc<T>::set_plot_level(short val)
{
	if (val < max_pixel_value)
		plot_level = val;
	else
		return false;

	return true;

}


/*
 *  get_reference
 */
template <class T>
void ImageProc<T>::get_reference(int32_t *ref_h, int32_t *ref_v)
{
	ref_h = &reference_hor[0];
	ref_v = &reference_ver[0];
}

#ifdef LIBFIT

/*
 *  set_plot_level
 */
template <class T>
void ImageProc<T>::set_algorithm(int32_t alg)
{
	switch (alg) {
		case RAW_CALC:
			/* reset waveforms */
			if (algorithm != RAW_CALC) {
				memset(hor_fit_profile, 0, sizeof(double)*width);
				memset(ver_fit_profile, 0, sizeof(double)*height);
			}
		case GAUSS_FIT: 
		case GAUSS_FIT_ASYMM:		
		case CONFITEOR_FIT: algorithm = alg;break;
		default:break;
	}	
	
}


/*
 * gauss_fit()
 * Fit hor_profile, ver_profile distributions
 *
 * Attributes already set (pixel scale):
 *   hor_sigma, ver_sigma, hor_pos, ver_pos
 *   hor_left, hor_right, ver_up, ver_down
 *   hor_profile, ver_profile
 *
 * Attributes to set (pixel scale):
 *   hor_sigma, ver_sigma, hor_pos, ver_pos
 *   hor_fit_profile, ver_fit_profile
 *
 * int32_t perform(
 *          double *y_values,             //IN:
 *          double *x_values,             //IN:
 *          uint32_t num_observation, //IN: number of observation (=sizeof(x_values)=sizeof(y_values))
 *          uint32_t num_iter,        //IN: maximum number of iteration
 *          uint32_t &iter,           //OUT: number of iteration
 *          double *weights,              //IN:
 *          double *fit,                  //IN: initial guess / OUT: fitted parameters
 *          double *err,                  //OUT: fit error: err[i]=sqrt(cov_i_i))* MAX(1, chi / sqrt(num_obs-num_param))
 *          double &chisq,                //OUT: chi squared
 *          double abs_err=EPSABS,        //IN: absolute error used to exit fitting loop
 *          double rel_err=EPSREL         //IN: relative error used to exit fitting loop
 *       );
 *
 */
template <class T>
bool ImageProc<T>::img_gauss_fit(void) 
{
	double fit[GAUSS_FIT_NUM_PARAM]; /* 0=base, 1=amp, 2=center, 3=width */
	uint32_t num_iter = 20, iter;
	double err[GAUSS_FIT_NUM_PARAM],
	       chisq;
	double amp;
	int32_t center = 0;
	int32_t cnt_err = 0;

	/***************** horizontal plane ******************/
	/* base */			 				 
	fit[0] = 0;
	/* amplitude, center */
	amp = 0;

	for (int32_t i = hor_left; i < hor_right; i++) {
		if (hor_profile[i] >= amp) {
			center = i;
			amp = hor_profile[i];
		}
	}
	fit[1] = amp;       /* amplitude */
	fit[2] = center-hor_left;    /* center */
	fit[3] = hor_sigma; /* raw sigma estimation (previously calculated) */

	if ((hor_right - hor_left) < 10)	
		cnt_err++;

	/* do the fitting */	
	if ((cnt_err == 0) && ((gauss_fit->perform(hor_profile+hor_left, NULL, hor_right-hor_left,
		num_iter, fit_tout, iter, NULL, fit, err, chisq)) == 0)) {
		/* if the position error or the sigma error are less the %5 */
		if ((err[2]/fit[2] < 0.05) && (err[3]/fit[3] < 0.05)) {
			hor_pos = fit[2] + hor_left;
			hor_sigma = fit[3];									
			/* calculate the distribution */
			if (gauss_fit->compute_func(hor_fit_profile + hor_left, NULL, hor_right - hor_left, fit) < 0)
				cnt_err++;
		}	
		else
			cnt_err++;						 
	}
	else
		cnt_err++;
	
	if (cnt_err) {
		memset(hor_fit_profile, 0, sizeof(double)*width); 
		fit_hor_ok = false;
	}
	else
		fit_hor_ok = true; 

	/******************* vertical plane *******************/
	cnt_err = 0;
	fit[0] = 0;
	/* amplitude, center */
	amp = 0;
	for (int32_t i = ver_up; i < ver_down; i++) {
		if (ver_profile[i] >= amp) {
			center = i;
			amp = ver_profile[i];
		}
	}
	fit[1] = amp;       /* amplitude */
	fit[2] = center-ver_up;    /* center */
	fit[3] = ver_sigma; /* raw sigma estimation (previously calculated) */

	if ((ver_down - ver_up) < 10)	
		cnt_err++;

	/* do the fitting */	
	if ((cnt_err == 0) && ((gauss_fit->perform(ver_profile + ver_up, NULL, ver_down - ver_up,
		num_iter, fit_tout, iter, NULL, fit, err, chisq)) == 0)) {										 
		/* if the position error or the sigma error are less the %5 */
		if ((err[2]/fit[2] < 0.05) && (err[3]/fit[3] < 0.05)) {	
			ver_pos = fit[2] + ver_up;
			ver_sigma = fit[3];									
			/* calculate the distribution */
			if (gauss_fit->compute_func(ver_fit_profile + ver_up, NULL, ver_down - ver_up, fit) < 0)
				cnt_err++;
		}
		else
			cnt_err++;
	}
	else
		cnt_err++;

	if (cnt_err) {
		memset(ver_fit_profile, 0, sizeof(double)*height); 
		fit_ver_ok = false;
	}
	else
		fit_ver_ok = true; 

	if (cnt_err)
		return false;
	else
		return true;

} 



/*
 * gauss_fit_asymm()
 * Fit hor_profile, ver_profile distributions
 
 */
template <class T>
bool ImageProc<T>::img_gauss_fit_asymm(void) 			
{
	double fit[GAUSS_FIT_ASYMM_NUM_PARAM]; /* 0=base, 1=amp, 2=center, 3=width left, 4=width right */
	uint32_t num_iter = 20, iter;
	double err[GAUSS_FIT_ASYMM_NUM_PARAM],
	       chisq;
	double amp;
	int32_t center = 0;
	int32_t cnt_err = 0;
	
	/***************** horizontal plane ******************/
	/* base */			 				 
	fit[0] = 0;
	/* amplitude, center */
	amp = 0;
	for (int32_t i = hor_left; i < hor_right; i++) {
		if (hor_profile[i] >= amp) {
			center = i;
			amp = hor_profile[i];
		}
	}
	fit[1] = amp;       /* amplitude */
	fit[2] = center-hor_left;    /* center */
	fit[3] = hor_sigma*(center-hor_left)/(hor_right-hor_left)*3; /* raw sigma estimation */
	fit[4] = hor_sigma*(hor_right-center)/(hor_right-hor_left)*3; /* raw sigma estimation */
	
	if ((hor_right - hor_left) < 10)	
		cnt_err++;
	
	/* do the fitting */	
	if ((cnt_err == 0) && ((gauss_fit_asymm->perform(hor_profile+hor_left, NULL, hor_right-hor_left,
		num_iter, fit_tout, iter, NULL, fit, err, chisq) == 0))) {
		if ((err[2]/fit[2] < 0.1) && (err[3]/fit[3] < 0.1)) {
			/* calculate the distribution */
			if (gauss_fit_asymm->compute_func(hor_fit_profile + hor_left, NULL, hor_right - hor_left, fit) < 0)
				cnt_err++;
			/* calculate the rms */
			if (cnt_err == 0) {
				if (gauss_fit_asymm->compute_rms(fit, &hor_pos, &hor_sigma) < 0) {
					cnt_err++;
				}
			}
			hor_pos += hor_left;	
		}
		else 
			cnt_err++;
	}
	else
		cnt_err++;

	if (cnt_err) {
		memset(hor_fit_profile, 0, sizeof(double)*width); 
		fit_hor_ok = false;
	}
	else
		fit_hor_ok = true; 
	
	/******************* vertical plane *******************/
	cnt_err = 0;
	fit[0] = 0;
	/* amplitude, center */
	amp = 0;
	for (int32_t i = ver_up; i < ver_down; i++) {
		if (ver_profile[i] >= amp) {
			center = i;
			amp = ver_profile[i];
		}
	}
	fit[1] = amp;       /* amplitude */
	fit[2] = center-ver_up;    /* center */
	fit[3] = ver_sigma*(center-ver_up)/(ver_down-ver_up)*3; /* raw sigma estimation (previously calculated) */
	fit[4] = ver_sigma*(ver_down-center)/(ver_down-ver_up)*3;

	if ((ver_down - ver_up) < 10)	
		cnt_err++;

	/* do the fitting */	
	if ((cnt_err == 0) && ((gauss_fit_asymm->perform(ver_profile + ver_up, NULL, ver_down - ver_up,
		num_iter, fit_tout, iter, NULL, fit, err, chisq)) == 0)) {
		if ((err[2]/fit[2] < 0.1) && (err[3]/fit[3] < 0.1)) {
			/* calculate the distribution */
			if (gauss_fit_asymm->compute_func(ver_fit_profile + ver_up, NULL, ver_down - ver_up, fit) < 0)
				cnt_err++;
			/* calculate the rms */
			if (cnt_err == 0) {
				if (gauss_fit_asymm->compute_rms(fit, &ver_pos, &ver_sigma) < 0) {
					cnt_err++;
				}
			}
			ver_pos += ver_up;
		}
		else
			cnt_err++;
	}
	else
		cnt_err++;

	if (cnt_err) {
		fit_ver_ok = false;
		memset(ver_fit_profile, 0, sizeof(double)*height);
	}
	else
		fit_ver_ok = true; 

	if (cnt_err)
		return false;
	else
		return true;

}


/*
 * confiteor_fit()
 * Fit hor_profile, ver_profile distributions
 */
template <class T> 
bool ImageProc<T>::img_confiteor_fit(void)	
{

	double fit[CONFITEOR_FIT_NUM_PARAM]; /* 0=base, 1=amp, 2=center, 3=width left, 4=width right */
	uint32_t num_iter = 100, iter;
	double err[CONFITEOR_FIT_NUM_PARAM],
	       chisq;
	double amp;
	int32_t center = 0;
	int32_t cnt_err = 0;
	
	/***************** horizontal plane ******************/
	/* base */
	fit[0] = 0;
	/* amplitude, center */
	amp = 0;
	for (int32_t i = hor_left; i < hor_right; i++) {
		if (hor_profile[i] >= amp) {
			center = i;
			amp = hor_profile[i];
		}
	}
	fit[1] = amp;       /* amplitude */
	fit[2] = center-hor_left;    /* center */
	fit[3] = hor_sigma*(center-hor_left)/(hor_right-hor_left); /* raw sigma estimation */
	fit[4] = hor_sigma*(hor_right-center)/(hor_right-hor_left); /* raw sigma estimation */
	fit[5] = 2; 
	fit[6] = 2;
	
	if ((hor_right - hor_left) < 10)	
		cnt_err++;
	
	/* do the fitting */
	if ((cnt_err == 0) && ((confiteor_fit->perform(hor_profile + hor_left, NULL, hor_right - hor_left, 
	                   num_iter, fit_tout, iter, NULL, fit, err, chisq)) == 0)) {
		if ((fit[2] >= hor_left) || (fit[2] <= hor_right)) {	
			/* calculate the distribution */
			if (confiteor_fit->compute_func(hor_fit_profile + hor_left, NULL, hor_right - hor_left, fit) < 0)
				cnt_err++;
			/* calculate the rms */
			if (cnt_err == 0) {
				if (confiteor_fit->compute_rms(fit, &hor_pos, &hor_sigma) < 0) {
					cnt_err++;
				}
			}
			hor_pos += hor_left;
		}
		else 
			cnt_err++;
	}
	else
		cnt_err++;

	if (cnt_err) {
		fit_hor_ok = false;
		memset(hor_fit_profile, 0, sizeof(double)*width); 
	}
	else
		fit_hor_ok = true; 


	/******************* vertical plane *******************/
	cnt_err = 0;
	fit[0] = 0;
	/* amplitude, center */
	amp = 0;
	for (int32_t i = ver_up; i < ver_down; i++) {
		if (ver_profile[i] >= amp) {
			center = i;
			amp = ver_profile[i];
		}
	}
	fit[1] = amp;/* amplitude */
	fit[2] = center-ver_up;	/* center */
	fit[3] = ver_sigma*(center-ver_up)/(ver_down-ver_up); /* raw sigma estimation (previously calculated) */
	fit[4] = ver_sigma*(ver_down-center)/(ver_down-ver_up);
	fit[5] = 2; 
	fit[6] = 2;
	
	if ((ver_down - ver_up) < 10)	
		cnt_err++;

	/* do the fitting */
	if ((cnt_err == 0) && ((confiteor_fit->perform(ver_profile + ver_up, NULL, ver_down - ver_up,
	                   num_iter, fit_tout, iter, NULL, fit, err, chisq)) == 0)) {
		if ((fit[2] >= ver_up) || (fit[2] <= ver_down)) {
			/* calculate the distribution */
			if (confiteor_fit->compute_func(ver_fit_profile + ver_up, NULL, ver_down - ver_up, fit) < 0)
				cnt_err++;
			/* calculate the rms */
			if (cnt_err == 0) {
				if (confiteor_fit->compute_rms(fit, &ver_pos, &ver_sigma) < 0) {
					cnt_err++;
				}
			}
			ver_pos += ver_up;		
		}
		else
			cnt_err++;
	}
	else
		cnt_err++;

	if (cnt_err) {
		fit_ver_ok = false;
		memset(ver_fit_profile, 0, sizeof(double)*height); 
	}
	else
		fit_ver_ok = true; 

	if (cnt_err)
		return false;
	else
		return true;

}

#endif


/*
 *  set_plot_level
 */
template <class T>
bool ImageProc<T>::set_reference(int32_t ref_h, int32_t ref_v)
{
	if ((ref_h < 0) && (ref_h >= width))
		return false;

	if ((ref_v < 0) && (ref_v >= height))
		return false;

	reference_hor[reference_idx] = ref_h;
	reference_ver[reference_idx] = ref_v;

	reference_idx = (reference_idx + 1) % IMAGEPROC_MAX_REF;

	return true;

}


/*
 *  set_manual_roi(int32_t *val)
 *  val[0] = hor left, val[1]= ver up,
 *  val[2] = hor right, val[3] = ver down
 */
template <class T>
bool ImageProc<T>::set_manual_roi(int32_t *val)
{
	if ((val[0] >= 0) &&
	    (val[0] < val[2]) &&
	    (val[2] <= width) &&
	    (val[1] >= 0) &&
	    (val[1] < val[3]) &&
	    (val[3] <= height)) {
		for (int32_t i = 0; i < 4; i++)
			manual_roi_param[i] = val[i];
		return true;
	}
	else
		return false;

}


template <class T>
bool ImageProc<T>::set_lp_order(int32_t order)
{

	if (order == lp_order)
		return true;

	if (order <= 0)
		return false;
	/* accept only even values (1,3,5,7 ...)*/
	if (order & 0x1)
		return false;

	lp_order = order;

	if (imagetemp) {
		delete [] imagetemp; imagetemp = 0;
	}
	if ((imagetemp = new T[(num_grid_hor + lp_order - 1)*(num_grid_ver + lp_order - 1)]) == 0)
		return false;
	
	memset(imagetemp,0,sizeof(T)*((num_grid_ver+(lp_order-1))*(num_grid_hor+(lp_order-1))));

	return true;

}


template <class T>
bool ImageProc<T>::set_hor_grid(int32_t grid)
{

	if (grid == num_grid_hor)
		return true;

	if ((grid <= 0) || (grid > width))
		return false;

	num_grid_hor = grid;
        
	if (imagetemp) {
		delete [] imagetemp; imagetemp = 0;
	}
	if ((imagetemp = new T[(num_grid_hor + lp_order - 1)*(num_grid_ver + lp_order - 1)]) == 0)
		return false;

	if (imageicon) {
		delete [] imageicon; imageicon = 0;
	}
	if ((imageicon = new int32_t[num_grid_hor*num_grid_ver]) == 0)
		return false;

	memset(imagetemp,0,sizeof(T)*((num_grid_ver+(lp_order-1))*(num_grid_hor+(lp_order-1))));
	memset(imageicon,0,sizeof(int32_t)*(num_grid_ver*num_grid_hor));

	imageicon_width = num_grid_hor;

	return true;
}


template <class T>
bool ImageProc<T>::set_ver_grid(int32_t grid)
{

	if (grid == num_grid_ver)
		return true;

	if ((grid <= 0) || (grid > height))
		return false;

	num_grid_ver = grid;
        
	if (imagetemp) {
		delete [] imagetemp; imagetemp = 0;
	}
	if ((imagetemp = new T[(num_grid_hor + lp_order - 1)*(num_grid_ver + lp_order - 1)]) == 0)
		return false;

	if (imageicon) {
		delete [] imageicon; imageicon = 0;
	}
	if ((imageicon = new int32_t[num_grid_hor*num_grid_ver]) == 0)
		return false;

	memset(imagetemp,0,sizeof(T)*((num_grid_ver+(lp_order-1))*(num_grid_hor+(lp_order-1))));
	memset(imageicon,0,sizeof(int32_t)*(num_grid_ver*num_grid_hor));

	imageicon_height = num_grid_ver;

	return true;
}


template <class T>
bool ImageProc<T>::set_enable_auto_roi(bool roiflag)
{
	if ((imagetemp == 0) || (imageicon == 0)) {
		auto_roi = false;
		return false;
	}
	if (roiflag == false) {
		hor_left = 0; hor_right = width;
		ver_up = 0; ver_down = height;
	}

	auto_roi = roiflag;
	return true;

}


template <class T>
bool ImageProc<T>::set_roi_threshold(double val)
{
	if (val < 0) {
		return false;
	}
	roi_threshold = val;
	return true;
}


template <class T>
bool ImageProc<T>::set_search_background_param(double *param)
{
	search_background_param[0] = param[0];
	if (((param[1] > 0) && (param[1] < 1)) || ((param[0] == 0) && (param[1] == 0))) {
		search_background_param[1] = param[1];
		return true;
	}
	return false;
}



// load an image in the object without background subtraction
template <class T>
void ImageProc<T>::push_permutation(const T *data_ptr, int32_t *perm_vec, uint32_t camera_cnt, uint32_t acq_cnt)
{
	struct timeval start, end;
	int32_t i,j;

	gettimeofday(&start,NULL);

	camera_counter = camera_cnt;
	acquisition_counter = acq_cnt;

	memset(image,0,sizeof(T)*width*height);

	for (i = 0; i < width*height; i++) {
		if (perm_vec[i] > 0) {
			image[perm_vec[i]] = data_ptr[i];
		}
		else {
			image[perm_vec[i]] = 0;
		}
	}

	if (enable_rotation_filter) {
		for (i = 1; i < height-1; i++) {
			for (j = 1; j < width-1; j++) {
				if (perm_vec[j+i*width] == 0)
					image[j+i*width] = ((int32_t)image[j+(i-1)*width] + (int32_t)image[j+1+i*width] + (int32_t)image[j+(i+1)*width] + (int32_t)image[j-1+i*width]) >> 2;
			}
		}
	}

	if (enable_process) {
		if (auto_roi) {
			find_roi();
		}
		else {
			icon_background = 0;
		}
		if (manual_roi) {
				hor_left = manual_roi_param[0]; hor_right = manual_roi_param[2];
				ver_up = manual_roi_param[1]; ver_down = manual_roi_param[3];
		}	
		process();
	}
	gettimeofday(&end,NULL);
	processing_time = (end.tv_sec-start.tv_sec)*1000000+(end.tv_usec-start.tv_usec);

}


// load an image in the object without background subtraction
template <class T>
void ImageProc<T>::push_permutation_with_background(const T *data_ptr, int32_t *perm_vec, uint32_t camera_cnt, uint32_t acq_cnt, const T *background_ptr)
{
	struct timeval start, end;
	int32_t i,j;

	gettimeofday(&start,NULL);

	camera_counter = camera_cnt;
	acquisition_counter = acq_cnt;

	memset(image,0,sizeof(T)*width*height);

	for (i = 0; i < width*height; i++) {
		image[perm_vec[i]] = data_ptr[i];
		if (image[perm_vec[i]] > background_ptr[perm_vec[i]])
			image[perm_vec[i]] -= background_ptr[perm_vec[i]];
		else
			image[perm_vec[i]] = 0;
	}

	if (enable_rotation_filter) {
		for (i = 1; i < height-1; i++) {
			for (j = 1; j < width-1; j++) {
				if (perm_vec[j+i*width] == 0)
					image[j+i*width] = ((int32_t)image[j+(i-1)*width] + (int32_t)image[j+1+i*width] + (int32_t)image[j+(i+1)*width] + (int32_t)image[j-1+i*width]) >> 2;
			}
		}
	}

	if (enable_process) {
		if (auto_roi) {
			find_roi();
		}
		else {
			icon_background = 0;
		}
		if (manual_roi) {
			hor_left = manual_roi_param[0]; hor_right = manual_roi_param[2];
			ver_up = manual_roi_param[1]; ver_down = manual_roi_param[3];
		}
		process();
	}
	gettimeofday(&end,NULL);
	processing_time = (end.tv_sec-start.tv_sec)*1000000+(end.tv_usec-start.tv_usec);

}


// round double into int
template <class T>
int32_t ImageProc<T>::cint(double x)
{
	int32_t a = (x > 0) ? (int)(x+0.5):(int)(x-0.5);
	if ((x-a) >= 0.4999999)
		return (a+1);
	else if ((x-a) < -0.5)
		return (a-1);
	else 
		return a;
}

// calculate mirror permutation vector
template <class T>
void ImageProc<T>::permutation_mirrorh(int32_t *perm_vec)
{
	int32_t i,j,idxn;
	int32_t *perm_temp;

	perm_temp = (int32_t *) malloc(height*width*sizeof(int32_t));

	memcpy(perm_temp,perm_vec,height*width*sizeof(int32_t));

	for (i = 0; i < height; i++) {
		for (j = 0; j < width; j++) {
			idxn = i*width+width-1-j;
			perm_vec[i*width+j] = perm_temp[idxn];
		}
	}

	free(perm_temp);

}


template <class T>
void ImageProc<T>::permutation_rotation(double f, int32_t *perm_vec) 
{
	int32_t i,j,idxn;
	double ir, jr, cosf, sinf,in, jn, inc_i = 0, inc_j = 0;
	
	int32_t *perm_temp;

	perm_temp = (int32_t *) malloc(height*width*sizeof(int32_t));

	memcpy(perm_temp,perm_vec,height*width*sizeof(int32_t));

	ir=((double)height/2)-1;
	jr=((double)width/2)-1;

	/* CW rotation */
	cosf = cos(-2*3.14159265358979323846*f/360);
	sinf = sin(-2*3.14159265358979323846*f/360);

	/* to compensate rounding problems */
	if (((int)(f+360) % 360) == 0) {
		inc_i = 0;inc_j = 0;
	}
	else if (((int)(f+360) % 360) <= 90) {
		inc_i = 0;inc_j = 1;
	}
	else if (((int)(f+360) % 360) <= 180) {
		inc_i = 1;inc_j = 1;
	}
	else if (((int)(f+360) % 360) <= 270) {
		inc_i = 1;inc_j = 0;
	}
	else if (((int)(f+360) % 360) < 360) {
		inc_i = 0;inc_j = 0;
	}
	
	//printf("mod=%d inc_i=%d inc_j=%d\n",(int)(f+360) % 360 ,(int)inc_i, (int)inc_j);

	for (i = 0; i < height; i++) {
		for (j = 0; j < width; j++) {
			in = ((double)i - ir)*cosf - ((double)j - jr)*sinf + ir;
			jn = ((double)i - ir)*sinf + ((double)j - jr)*cosf + jr;
			idxn = (int32_t)((cint(in)+inc_i)*width+cint(jn)+inc_j);
			if ((idxn >= 0) && (idxn < height*width)) {
				perm_vec[i*width+j] = perm_temp[idxn];
			}
			else
				perm_vec[i*width+j] = 0; /* bad pixel after rotation */
		}
	}

	free(perm_temp);

}



// process the image 
template <class T>
void ImageProc<T>::plot_axis(int32_t cur_thres)
{
	int32_t i,j;
	int32_t x_start_h, x_end_h, x_start_v, x_end_v, x_center_h, x_center_v;
	int32_t half_left_h = 7, half_right_h = 8, full_width = 15,
		half_top_v = 7, half_bottom_v = 8, full_height = 15;
	double hor_profile_ampl_rescale = 0, ver_profile_ampl_rescale = 0;
	int32_t idx;
	bool print_hor_up, print_ver_left;
	double hor_min_val = 10000000, hor_max_val = 0, ver_min_val = 100000000, ver_max_val = 0;

	if (plot_mode & 0x1) {

	for (int32_t i = hor_left; i < hor_right; i++) {
		if (hor_profile[i] < hor_min_val)
			hor_min_val = hor_profile[i];
		if (hor_profile[i] > hor_max_val)
			hor_max_val = hor_profile[i];				
	}
		
	for (int32_t i = ver_up; i < ver_down; i++) {
		if (ver_profile[i] < ver_min_val)
			ver_min_val = ver_profile[i];
		if (ver_profile[i] > ver_max_val)
			ver_max_val = ver_profile[i];				
	}		
		
	/* set where to plot horizontal profile */
	if (ver_up > height/10) {
		print_hor_up = true;
	}
	else {
		if (ver_up < (height - ver_down))
			print_hor_up = false;	
		else
			print_hor_up = true;				
	}

	/* set where to plot vertical profile */
	if (hor_left > width/10) {
		print_ver_left = true;
	}
	else {
		if (hor_left < (width - hor_right))
			print_ver_left = false;	
		else
			print_ver_left = true;				
	}


	/******************** plot profiles ********************/
	/******* calculate horizontal rescale *******/
	if (print_hor_up) {
		if ((ver_down-ver_up) > ver_up) {
			hor_profile_ampl_rescale = (double)ver_up / hor_max_val;
		}
		else {
			hor_profile_ampl_rescale = (double)(ver_down - ver_up) / hor_max_val;	
		}		
	}
	else {
		if ((ver_down-ver_up) > (height-ver_down)) {
			hor_profile_ampl_rescale = (double)(height-ver_down) / hor_max_val;
		}
		else {
			hor_profile_ampl_rescale = (double)(ver_down - ver_up) / hor_max_val;	
		}				
	}
	/******* calculate vertical rescale *********/
	if (print_ver_left) {
		if ((hor_right-hor_left) > hor_left)
			ver_profile_ampl_rescale = (double)hor_left / ver_max_val;
		else
			ver_profile_ampl_rescale = (double)(hor_right - hor_left) / ver_max_val;			
	}
	else {
		if ((hor_right-hor_left) > (width-hor_right)) {
			ver_profile_ampl_rescale = (double)(width-hor_right) / ver_max_val;
		}
		else {
			ver_profile_ampl_rescale = (double)(hor_right - hor_left) / ver_max_val;	
		}				
	}	
	/********************************************/
	
	/******** horizontal profile, UP ********/
	/* plot the distribution */
	if (print_hor_up) {
		/* print32_t on the top of the ROI */
		for (int32_t i = hor_left; i < hor_right; i++) {
			idx = (int32_t) (width * (int32_t)(ver_up - (hor_profile[i]-hor_min_val)*hor_profile_ampl_rescale) +  i);
			if ((idx >= 0) && (idx < (width*height)))
				image[idx] = plot_level;
		}
		if ((algorithm != RAW_CALC) && (fit_hor_ok)) {
			for (int32_t i = hor_left; i < hor_right; i++) {
				idx = (int32_t) (width * (int32_t)(ver_up - (hor_fit_profile[i]-hor_min_val)*hor_profile_ampl_rescale) +  i);
				if ((idx >= 0) && (idx < (width*height)))	
					image[idx] = 150;
			}
		}		

		/* plot the center axis */		
		if (algorithm == RAW_CALC) {
			for (int32_t i = 0; i < (int32_t)(hor_profile[(int32_t)hor_pos]*hor_profile_ampl_rescale); i++) {
				idx = (int32_t) (width * (int32_t)(ver_up - (hor_profile[(int32_t)hor_pos]-hor_min_val)*hor_profile_ampl_rescale + i) + hor_pos);
				if ((idx >= 0) && (idx < (width*height)) && (i & 0x1))
					image[idx] = plot_level;				
			}	
		}
		else if (fit_hor_ok) {
			for (int32_t i = 0; i < (int32_t)(hor_fit_profile[(int32_t)hor_pos]*hor_profile_ampl_rescale); i++) {
				idx = (int32_t) (width * (int32_t)(ver_up - (hor_fit_profile[(int32_t)hor_pos]-hor_min_val)*hor_profile_ampl_rescale + i) + hor_pos);
				if ((idx >= 0) && (idx < (width*height)) && (i & 0x1))
					image[idx] = 150;				
			}			
		}
		
	}
	/******** horizontal profile, DOWN ********/	
	else {
		/* plot the distribution */
		/* print32_t on the bottom of the roi */
		for (int32_t i = hor_left; i < hor_right; i++) {
			idx = (int32_t) (width * (int32_t)(ver_down + (hor_profile[i]-hor_min_val)*hor_profile_ampl_rescale) +  i);
			if ((idx >= 0) && (idx < (width*height)))
				image[idx] = plot_level;
		}
		if ((algorithm != RAW_CALC) && (fit_hor_ok)) {
			for (int32_t i = hor_left; i < hor_right; i++) {
				idx = (int32_t) (width * (int32_t)(ver_down + (hor_fit_profile[i]-hor_min_val)*hor_profile_ampl_rescale) +  i);
				if ((idx >= 0) && (idx < (width*height)))	
					image[idx] = 150;
			}
		}	
		/* plot the center axis */	
		if (algorithm == RAW_CALC) {
			for (int32_t i = 0; i < (int32_t)((hor_profile[(int32_t)hor_pos]-hor_min_val)*hor_profile_ampl_rescale); i++) {
				idx = (int32_t) (width * (int32_t)(ver_down + i) + hor_pos);
				if ((idx >= 0) && (idx < (width*height)) && (i & 0x1))
					image[idx] = plot_level;				
			}	
		}
		else if (fit_hor_ok) {
			for (int32_t i = 0; i < (int32_t)((hor_fit_profile[(int32_t)hor_pos]-hor_min_val)*hor_profile_ampl_rescale); i++) {
				idx = (int32_t) (width * (int32_t)(ver_down + i) + hor_pos);
				if ((idx >= 0) && (idx < (width*height)) && (i & 0x1))
					image[idx] = 150;				
			}			
		}	
	}
	/******** vertical profile, LEFT ********/	
	if (print_ver_left) {	
		/* plot the distribution */
		for (int32_t i = 0; i < (ver_down-ver_up); i++) {
			idx = (int32_t) (width * (ver_down-i)+ (int32_t)(hor_left - (ver_profile[ver_down-i]-ver_min_val)*ver_profile_ampl_rescale));
			if ((idx >= 0) && 
			    (idx < (width*height)) && 
					((int32_t)(hor_left - ver_profile[ver_down-i]*ver_profile_ampl_rescale) > 0))
				image[idx] = plot_level;
		}	
		if ((algorithm != RAW_CALC) && (fit_ver_ok)) {
			for (int32_t i = 0; i < (ver_down-ver_up); i++) {
				idx = (int32_t) (width * (ver_down-i)+ (int32_t)(hor_left - (ver_fit_profile[ver_down-i]-ver_min_val)*ver_profile_ampl_rescale));
				if ((idx >= 0) && 
				    (idx < (width*height)) && 
						((int32_t)(hor_left -ver_fit_profile[ver_down-i]*ver_profile_ampl_rescale) > 0))	
					image[idx] = 150;
			}
		}
		/* plot the center axis */		
		if (algorithm == RAW_CALC) {
			for (int32_t i = 0; i < (int32_t)((ver_profile[(int32_t)ver_pos]-ver_min_val)*ver_profile_ampl_rescale); i++) {
				idx = (int32_t) (width * (int32_t)(ver_pos) + (hor_left - i));
				if ((idx >= 0) && (idx < (width*height)) && (i & 0x1) && ((hor_left - i) > 0))
					image[idx] = plot_level;				
			}	
		}
		else if (fit_hor_ok) {
			for (int32_t i = 0; i < (int32_t)((ver_fit_profile[(int32_t)ver_pos]-ver_min_val)*ver_profile_ampl_rescale); i++) {
				idx = (int32_t) (width * (int32_t)(ver_pos) + (hor_left - i));
				if ((idx >= 0) && (idx < (width*height)) && (i & 0x1)  && ((hor_left - i) > 0))
					image[idx] = 150;				
			}	
		}	
	}
	/******** vertical profile, RIGHT ********/		
	else {
		/* plot the distribution */
		for (int32_t i = 0; i < (ver_down-ver_up); i++) {
			idx = (int32_t) (width * (ver_down-i)+ (int32_t)(hor_right + (ver_profile[ver_down-i]-ver_min_val)*ver_profile_ampl_rescale));
			if ((idx >= 0) && 
			    (idx < (width*height)) && 
					((int32_t)(hor_right + ver_profile[ver_down-i]*ver_profile_ampl_rescale) < width))
				image[idx] = plot_level;
		}	
		if ((algorithm != RAW_CALC) && (fit_ver_ok)) {
			for (int32_t i = 0; i < (ver_down-ver_up); i++) {
				idx = (int32_t) (width * (ver_down-i)+ (int32_t)(hor_right + (ver_fit_profile[ver_down-i]-ver_min_val)*ver_profile_ampl_rescale));
				if ((idx >= 0) && 
				    (idx < (width*height)) && 
						((int32_t)(hor_right + ver_fit_profile[ver_down-i]*ver_profile_ampl_rescale) < width))	
					image[idx] = 150;
			}
		}
		/* plot the center axis */	
		if (algorithm == RAW_CALC) {
			for (int32_t i = 0; i < (int32_t)((ver_profile[(int32_t)ver_pos]-ver_min_val)*ver_profile_ampl_rescale); i++) {
				idx = (int32_t) (width * (int32_t)(ver_pos) + (hor_right + i));
				if ((idx >= 0) && (idx < (width*height)) && (i & 0x1) && ((hor_right + i) < width))
					image[idx] = plot_level;				
			}	
		}
		else if (fit_ver_ok) {
			for (int32_t i = 0; i < (int32_t)((ver_fit_profile[(int32_t)ver_pos]-ver_min_val)*ver_profile_ampl_rescale); i++) {
				idx = (int32_t) (width * (int32_t)(ver_pos) + (hor_right + i));
				if ((idx >= 0) && (idx < (width*height)) && (i & 0x1) && ((hor_right + i) < width))
					image[idx] = 150;					
			}		
		}					
	}
	
	} /* if (plot_mode & 0x1) */

	/******************** plot position and roi axes ******************/
	if (plot_mode & 0x1) {
		for (j = hor_left; j < hor_right; j++) {
			image[ver_up*width+j] = plot_level;image[(ver_down-1)*width+j] = plot_level;
		}
		for (j = ver_up; j < ver_down; j++) {
			image[width*j+hor_left] = plot_level;image[width*j+hor_right-1] = plot_level;
		}
		if (ver_pos == height) {
			for (j = hor_left; j < hor_right; j++) { 
				image[((int32_t)ver_pos-1)*width+j] = plot_level;
			}
		}
		else {
			for (j = hor_left; j < hor_right; j++) { 
				image[(int32_t)ver_pos*width+j] = plot_level;
			}
		}
		if (hor_pos == width) {
			for (j = ver_up; j < ver_down; j++) { 
				image[width*j+(int32_t)hor_pos-1] = plot_level;
			}
		}
		else {
			for (j = ver_up; j < ver_down; j++) { 
				image[width*j+(int32_t)hor_pos] = plot_level;
			}
		}

		/* rescale cross sizes depending on the hw roi */
		if (width <= full_width) {
			full_width = width;
			half_left_h = ((full_width - width) >> 1);
			half_right_h = full_width - half_left_h;
		}
		if (height <= full_height) {
			full_height = height;
			half_top_v = ((full_height - height) >> 1);
			half_bottom_v = full_height - half_top_v;
		}

		/* plot the reference target */
		if ((ref_hor_pos_offset != 0) && (ref_ver_pos_offset != 0)) {
			x_center_h = (long) ((ref_hor_pos_offset-hor_pos_offset+img_hor_pos_offset/(hor_calibration/(double)binning))*(hor_calibration/(double)binning));
			x_center_v = (long) ((-ref_ver_pos_offset-ver_pos_offset*ver_sign+img_ver_pos_offset/(ver_calibration/(double)binning))*(ver_calibration/(double)binning));
			
			x_start_h = x_center_h - half_left_h; x_end_h = x_center_h + half_right_h;
			x_start_v = x_center_v - half_top_v; x_end_v = x_center_v + half_bottom_v;
			
			if (x_start_h < 0) { x_start_h = 0; x_end_h = full_width; x_center_h = half_left_h;}
			else if (x_start_h > width) { x_start_h = width - full_width;  x_end_h = width; x_center_h = width - half_right_h;}
			
			if (x_end_h < 0) { x_start_h = 0; x_end_h = full_width; x_center_h = half_left_h;}
			else if (x_end_h >= width) { x_start_h = width - full_width; x_end_h = width; x_center_h = width - half_right_h;}
			
			if (x_start_v < 0) { x_start_v = 0; x_end_v = full_height; x_center_v = half_top_v;}
			else if (x_start_v > height) { x_start_v = height - full_height; x_end_v = height; x_center_v = height - half_bottom_v;}
			
			if (x_end_v < 0) { x_start_v = 0; x_end_v = full_height; x_center_v = half_top_v;}
			else if (x_end_v > height) { x_start_v = height - full_height; x_end_v = height;  x_center_v = height - half_bottom_v;}

			/* plot horizontal target line */
			for (i = x_start_h; i < x_end_h; i++) {
				image[i+x_center_v*width] = (plot_level+128) & 0xff;
				if (x_center_v > 0)
					image[i+(x_center_v-1)*width] = (plot_level+128) & 0xff;
				if (x_center_v < (height-1))
					image[i+(x_center_v+1)*width] = (plot_level+128) & 0xff;
			}
			/* plot vertical target line */
			for (i = x_start_v; i < x_end_v; i++) {
				image[i*width+x_center_h] = (plot_level+128) & 0xff;
				if (x_center_h > 0)
					image[i*width+x_center_h-1] = (plot_level+128) & 0xff;
				if (x_center_h < (width-1))
					image[i*width+x_center_h+1] = (plot_level+128) & 0xff;
			}
		}

	}

	/********************* enchance the spot area *********************/
	if (plot_mode & 0x2) {
		for (i = hor_left; i < hor_right; i++) {
			for (j = ver_up; j < ver_down; j++) {
				if (image[i+j*width] < cur_thres) {
					image[i+j*width] = plot_level;
				}
			}
		}
	}

	/********************* plot references on the images *********************/
	for (int k = 0; k < IMAGEPROC_MAX_REF; k++) {
		x_center_h = reference_hor[k]; x_center_v = reference_ver[k];
		x_start_h = x_center_h - half_left_h; x_end_h = x_center_h + half_right_h;
		x_start_v = x_center_v - half_top_v; x_end_v = x_center_v + half_bottom_v;
			
		if (x_start_h < 0) { x_start_h = 0; x_end_h = full_width; }
		else if (x_start_h > width) { x_start_h = width - full_width;  x_end_h = width;}
			
		if (x_end_h < 0) { x_start_h = 0; x_end_h = full_width; }
		else if (x_end_h > width) { x_start_h = width - full_width; x_end_h = width;}
			
		if (x_start_v < 0) { x_start_v = 0; x_end_v = full_height; }
		else if (x_start_v > height) { x_start_v = height - full_height; x_end_v = height;}
			
		if (x_end_v < 0) { x_start_v = 0; x_end_v = full_height;}
		else if (x_end_v > height) { x_start_v = height - full_height; x_end_v = height;}

		if ((x_center_h > 0) || (x_center_v > 0)) {
			/* plot horizontal target line */
			for (i = x_start_h; i < x_end_h; i++) 
				image[i+x_center_v*width] = plot_level;
			/* plot vertical target line */
			for (i = x_start_v; i < x_end_v; i++) 
				image[i*width+x_center_h] = plot_level;
		}
	}

	//printf("e\n");

	/********************* plot offset axis *********************/
	if (plot_offset_axis) {
		int32_t hor_offset_axis;
		/* vertical axis */
		if (hor_sign < 0)
			hor_offset_axis =  (int32_t)(img_hor_pos_offset - hor_pos_offset*hor_sign*hor_calibration/binning);
		else
			hor_offset_axis =  (int32_t)(img_hor_pos_offset + hor_pos_offset*hor_sign*hor_calibration/binning);
			
		if ((hor_offset_axis >=0) && (hor_offset_axis < width)) {
			for (int32_t i = 0; i < height/2-1; i++) {
				image[i*2*width + hor_offset_axis] = 128;
			}
		}
		
		/* horizontal axis */
		int32_t ver_offset_axis = (int32_t)(img_ver_pos_offset - ver_pos_offset*ver_sign*ver_calibration/binning)-1;
		if ((ver_offset_axis >=0) && (ver_offset_axis < height)) {
			for (int32_t i = 0; i < width/2; i++) {
				image[ver_offset_axis*width + i*2] = 128;
			}	
		}
	}

}


// process the image 
template <class T>
void ImageProc<T>::process(void)
{
	int32_t max_pixel = 0, count_max_pixel = 0;
	int32_t i,j;
	double sum=0, h_pos=0, v_pos=0, h_sigma=0, v_sigma=0, hv_cov=0;
	int32_t cur_thres, area_tmp = 0;
	short fast_calc_inc = 0;
	int32_t tmp,idx;
	double hor_pos_bk, ver_pos_bk, hor_sigma_bk, ver_sigma_bk;

	memset(hor_profile,0,width*sizeof(double));
	memset(ver_profile,0,height*sizeof(double));
	memset(hor_fit_profile,0,width*sizeof(double));
	memset(ver_fit_profile,0,height*sizeof(double));

	cur_thres = (int32_t) ((icon_background) / (lp_order*lp_order));

	/* subtract 1 to avoid saturation to 255/4095 and sequentially floating exception */
	if (cur_thres == max_pixel_value)
		cur_thres -= 1;

	/* reduce computation time sampling the image */
	if (fast_calc) {
		for (i = 1; i <= 5; i++) {
			if (((ver_down-ver_up)*(hor_right-hor_left)) > (300*300*i)) {
				fast_calc_inc = i;break;
			}
		}
	}

	for (i = ver_up; i < ver_down; i++) {
		for (j = hor_left; j < hor_right; j++) {
			idx = i*width+j;
			/* subtract background level to the image */
			if (image[idx] <= cur_thres)
				tmp = 0;
			else
				tmp = image[idx]-cur_thres;
      /* find the max; count the number of pixels the reach the maximum */
			if (image[idx] > max_pixel) {
				max_pixel = image[idx]; count_max_pixel = 0;
			}
			else if (image[idx] == max_pixel) {
				count_max_pixel++;
			}
			/* measure the area above the background */
			if (tmp > 0) {
				area_tmp++;
			}
			/* calculate position and profiles */
			sum+=tmp;
			hor_profile[j]+=tmp; ver_profile[i]+=tmp;
			h_pos+=(j-hor_left)*tmp; v_pos+=(i-ver_up)*tmp;
			/* sample the image (if fast_calc_inc > 0 to sample the image */
			j += fast_calc_inc;
		}
		i += fast_calc_inc;
	}

	if (sum > 0) {
		hor_pos = (h_pos / sum + 0.5) + hor_left;       /* store data into public attribute */
		ver_pos = (v_pos / sum + 0.5) + ver_up;         /* store data into public attribute */
	}
	else {
		hor_pos = 0; ver_pos = 0;
	}
	area = area_tmp * (fast_calc_inc+1)*(fast_calc_inc+1); /* store data into public attribute */
	intensity = sum * (fast_calc_inc+1)*(fast_calc_inc+1); /* store data into public attribute */

	/* compensate the profiles due computation reduction */
	if (fast_calc_inc) {
		for (i = ver_up; i < (ver_down-fast_calc_inc); i++) {
			for (j = 0; j < fast_calc_inc; j++) {
				ver_profile[j+1+i] = ver_profile[i];
			}
			i += fast_calc_inc;
		}
		for (i = hor_left; i < (hor_right-fast_calc_inc); i++) {
			for (j = 0; j < fast_calc_inc; j++) {
				hor_profile[j+1+i] = hor_profile[i];
			}
			i += fast_calc_inc;
		}
	}

	/* saturation is the ratio between the area where the pixel reached the maximum value
	   and the area of the spot */
	if (max_pixel == max_pixel_value)
		saturation = (double)count_max_pixel/(double)area;
	else
		saturation = 0; /* store data into public attribute */

	max_value = max_pixel;                /* store data into public attribute */
	count_max_value = count_max_pixel;    /* store data into public attribute */

	/* calculate the row rms */
	for (i = hor_left; i < hor_right; i++) {
		for (j = ver_up; j < ver_down; j++) {
			/* calculate sigma's of the pixels > background */
			if (image[i+j*width] <= cur_thres)
				tmp = 0;
			else
				tmp = image[i+j*width]-cur_thres;

			h_sigma+=(tmp*(i-hor_pos)*(i-hor_pos));
			v_sigma+=(tmp*(j-ver_pos)*(j-ver_pos));
			hv_cov+=(tmp*(i-hor_pos)*(j-ver_pos));

			j += fast_calc_inc;
		}
		i += fast_calc_inc;
	}

	if (sum > 0) {
		hor_sigma = pow(h_sigma/sum,0.5);     /* store data into public attribute */
		ver_sigma = pow(v_sigma/sum,0.5);     /* store data into public attribute */
		covar = ((double)hv_cov/(double)sum); /* store data into public attribute */
		phase = 0.5 *  atan ((double)((2 * covar) / (hor_sigma*hor_sigma - ver_sigma*ver_sigma))/(hor_calibration*ver_calibration/(binning*binning))) * 360 / (2*3.14159265);
	}
	else {
		hor_sigma = 0; ver_sigma = 0; covar = 0; phase= 0;
	}

	/* backup values in case fitting fails */
	if (!RAW_CALC) {
		hor_pos_bk = hor_pos;
		ver_pos_bk = ver_pos;
		hor_sigma_bk = hor_sigma;
		ver_sigma_bk = ver_sigma;
	}

#ifdef LIBFIT	
	/* hor_sigma, ver_sigma, hor_pos, ver_pos already calculated (pixel scale);
	   this values could be used for initial conditions for the fitting algorithm */
	switch(algorithm) {
		case RAW_CALC: break;
		case GAUSS_FIT: img_gauss_fit(); break;
		case GAUSS_FIT_ASYMM: img_gauss_fit_asymm(); break;			
		case CONFITEOR_FIT: img_confiteor_fit(); break;	
	}
#endif


	/* check data consistency */
	if ((fit_hor_ok == false) || isnan(hor_pos) || isnan(hor_sigma) || 
		(hor_pos <= hor_left) || (hor_pos >= hor_right) || (hor_sigma <= 0)) {
		hor_pos = hor_pos_bk;hor_sigma = hor_sigma_bk;
		fit_hor_ok = false;
	}
	if ((fit_ver_ok == false) || isnan(ver_pos) || isnan(ver_sigma) || 
		  (ver_pos <= ver_up) || (ver_pos >= ver_down) || (ver_sigma <= 0)) {		
		ver_pos = ver_pos_bk;ver_sigma = ver_sigma_bk;	
		fit_ver_ok = false;
	}

	/* set fit error flag */
	fit_error = 0;
	if (algorithm != RAW_CALC) {
		if (!fit_hor_ok)
			fit_error |= 0x1;
		if (!fit_ver_ok)
			fit_error |= 0x2;
	}	

	raw_hor_pos = (int32_t) hor_pos;
	raw_ver_pos = (int32_t) ver_pos;


	if (enable_dis && manual_roi) {
		dis_process();
	}

	/* before hor_pos/ver_pos is transformed in mm, plot axis */
	plot_axis(cur_thres);

	/* convert pixel format to calibration format */
	hor_pos = hor_pos/(hor_calibration/binning)-(hor_pos_offset+img_hor_pos_offset/(hor_calibration/binning));
	ver_pos = -(ver_pos/(ver_calibration/binning)-(-ver_pos_offset*ver_sign+img_ver_pos_offset/(ver_calibration/binning)));
	hor_sigma = hor_sigma/(hor_calibration/binning);
	ver_sigma = ver_sigma/(ver_calibration/binning);
	area = area/(hor_calibration*ver_calibration/(binning*binning));

}

// rDigital image stabilization (direct porting of MATLAB code in c++
// not the best efficient code but 100% equal to Matlab one (testing advantage
template <class T>
void ImageProc<T>::dis_process(void)
{
	int32_t orig_roi_left_up[2], orig_roi_right_down[2],
		roi_width, roi_height,
		orig_hor_pos, orig_ver_pos, cur_hor_pos, cur_ver_pos,
		diff_hor, diff_ver,
		cur_roi_left_up[2], cur_roi_right_down[2],  
		i_start = 0, j_start = 0;

	/* initialize values */
	orig_roi_left_up[0] = manual_roi_param[0];
	orig_roi_left_up[1] = manual_roi_param[1];
	orig_roi_right_down[0] = manual_roi_param[2];
	orig_roi_right_down[1] = manual_roi_param[3];
	roi_width = manual_roi_param[2] - manual_roi_param[0] - 1;
	roi_height = manual_roi_param[3] - manual_roi_param[1] - 1;
	orig_hor_pos = ref_raw_hor_pos; orig_ver_pos = ref_raw_ver_pos;
	cur_hor_pos = raw_hor_pos; cur_ver_pos = raw_ver_pos;
	diff_hor = orig_hor_pos - cur_hor_pos;
	diff_ver = orig_ver_pos - cur_ver_pos;
	cur_roi_left_up[0] = manual_roi_param[0] - diff_hor;
	cur_roi_left_up[1] = manual_roi_param[1] - diff_ver;
	cur_roi_right_down[0] = manual_roi_param[2] - diff_hor;
	cur_roi_right_down[1] = manual_roi_param[3] - diff_ver;

	hor_pos += (double)diff_hor;
	ver_pos += (double)diff_ver;

	if (abs(diff_hor) <= 1)
		diff_hor = 0;
	if (abs(diff_ver) <= 1)
		diff_ver = 0;	

	/* shift unuseful */
	if ((abs(diff_hor) <= 1) && (abs(diff_ver) <= 1)) {
		return;
	}

	//limit roi shifting if border issues
	if ((cur_roi_left_up[1]+roi_height) >= height) {
		roi_height = roi_height - ((cur_roi_left_up[1]+roi_height)-height)-1;
	}
	if ((cur_roi_left_up[0]+roi_width) >= width) {
		roi_width = roi_width - ((cur_roi_left_up[0]+roi_width)-width)-1;
	}

	if (cur_roi_left_up[1] <= 0) {
		i_start = -cur_roi_left_up[1];
	}
	if (cur_roi_left_up[0] <= 0) {
		j_start = -cur_roi_left_up[0];
	}

	if ((diff_hor > 0) && (diff_ver <= 0)) {
		// vertical is correct 
		for (int i=i_start; i <= roi_height; i++) {
		// horizontal has to be reversed
			for (int j = roi_width; j >= j_start; j--) {
				image[(orig_roi_left_up[1]+i)*width + orig_roi_left_up[0]+j] = 
					image[(cur_roi_left_up[1]+i)*width + cur_roi_left_up[0]+j];
				image[(cur_roi_left_up[1]+i)*width + cur_roi_left_up[0]+j] = 0;
			}
		}
	}

	if ((diff_hor > 0) && (diff_ver > 0)) {
		// vertical to be reversed
		for (int i=roi_height; i >= i_start; i--) {
		// horizontal has to be reversed
			for (int j = roi_width; j >= j_start; j--) {
				image[(orig_roi_left_up[1]+i)*width + orig_roi_left_up[0]+j] = 
					image[(cur_roi_left_up[1]+i)*width + cur_roi_left_up[0]+j];
				image[(cur_roi_left_up[1]+i)*width + cur_roi_left_up[0]+j] = 0;
			}
		}
	}

	if ((diff_hor <= 0) && (diff_ver <= 0)) {
		// vertical is correct 
		for (int i=i_start; i <= roi_height; i++) {
		// horizontal has to be reversed
			for (int j = j_start; j <= roi_width; j++) {
				image[(orig_roi_left_up[1]+i)*width + orig_roi_left_up[0]+j] = 
					image[(cur_roi_left_up[1]+i)*width + cur_roi_left_up[0]+j];
				image[(cur_roi_left_up[1]+i)*width + cur_roi_left_up[0]+j] = 0;
			}
		}
	}

	if ((diff_hor <= 0) && (diff_ver > 0)) {
		// vertical is correct 
		for (int i=roi_height; i >= i_start; i--) {
		// horizontal has to be reversed
			for (int j = j_start; j <= roi_width; j++) {
				image[(orig_roi_left_up[1]+i)*width + orig_roi_left_up[0]+j] = 
					image[(cur_roi_left_up[1]+i)*width + cur_roi_left_up[0]+j];
				image[(cur_roi_left_up[1]+i)*width + cur_roi_left_up[0]+j] = 0;
			}
		}
	}

	/* shift also profiles */
	if (diff_hor <= 0) {
		for (int j = j_start; j <= roi_width; j++) {
			hor_profile[orig_roi_left_up[0]+j] = hor_profile[cur_roi_left_up[0]+j];
			hor_profile[cur_roi_left_up[0]+j] = 0;
			hor_fit_profile[orig_roi_left_up[0]+j] = hor_fit_profile[cur_roi_left_up[0]+j];
			hor_fit_profile[cur_roi_left_up[0]+j] = 0;
		}
	}
	if (diff_hor > 0) {
		for (int j = roi_width; j >= j_start; j--) {
			hor_profile[orig_roi_left_up[0]+j] = hor_profile[cur_roi_left_up[0]+j];
			hor_profile[cur_roi_left_up[0]+j] = 0;
			hor_fit_profile[orig_roi_left_up[0]+j] = hor_fit_profile[cur_roi_left_up[0]+j];
			hor_fit_profile[cur_roi_left_up[0]+j] = 0;
		}
	}
	if (diff_ver <= 0) {
		for (int i=i_start; i <= roi_height; i++) {
			ver_profile[orig_roi_left_up[1]+i] = ver_profile[cur_roi_left_up[1]+i];
			ver_profile[cur_roi_left_up[1]+i] = 0;
			ver_fit_profile[orig_roi_left_up[1]+i] = ver_fit_profile[cur_roi_left_up[1]+i];
			ver_fit_profile[cur_roi_left_up[1]+i] = 0;
		}
	}
	if (diff_ver > 0) {
		for (int i=roi_height; i >= i_start; i--) {
			ver_profile[orig_roi_left_up[1]+i] = ver_profile[cur_roi_left_up[1]+i];
			ver_profile[cur_roi_left_up[1]+i] = 0;
			ver_fit_profile[orig_roi_left_up[1]+i] = ver_fit_profile[cur_roi_left_up[1]+i];
			ver_fit_profile[cur_roi_left_up[1]+i] = 0;
		}
	}

}


// reduce the image field to the spot region
template <class T>
void ImageProc<T>::find_roi(void)
{
	int32_t i,j,k,l;
	int32_t hsize = width/num_grid_hor;
  int32_t vsize = height/num_grid_ver;
	int32_t min_start_h = num_grid_hor;
	int32_t max_end_h = 0;
	int32_t min_start_v = num_grid_ver;
	int32_t max_bck_val = 0;
	int32_t max_end_v = 0;
	int32_t off = (lp_order-1)/2;
	int32_t range_th;
	int32_t num_step_th = (int32_t) search_background_param[0], idx_th = 0;
	int32_t size_step_th = 1;
	int32_t sum_th[IMAGEPROC_MAX_LEVELS], area_th[IMAGEPROC_MAX_LEVELS], area_der_th[IMAGEPROC_MAX_LEVELS], sum_der_th[IMAGEPROC_MAX_LEVELS];
	int32_t thres_th, max_der_th = 0;

	memset(imageicon,0,sizeof(int32_t)*(num_grid_ver*num_grid_hor));

	/* Sample the image to speedup comutation time
     The sampling period (num_grid_hor/num_grid_ver) must be compatible with the size of the beam */
	for (i = 0; i < num_grid_ver; i++) {
		for (j = 0; j < num_grid_hor; j++) {
			/* autoroi into manual roi; set to zero data outside the manual roi */
			if (manual_roi) {
				if (((j*hsize) >= manual_roi_param[0]) && ((j*hsize) <= manual_roi_param[2]) &&
					((i*vsize) >= manual_roi_param[1]) && ((i*vsize) <= manual_roi_param[3])) {
					imagetemp[(i+off)*(num_grid_hor+(lp_order-1)) + j+off] = image[i*vsize*width+j*hsize];
					if (imagetemp[(i+off)*(num_grid_hor+(lp_order-1)) + j+off] > max_bck_val)
						max_bck_val = imagetemp[(i+off)*(num_grid_hor+(lp_order-1)) + j+off];
				}
				else {
					imagetemp[(i+off)*(num_grid_hor+(lp_order-1)) + j+off] = 0;
				}
			}
			else {
				imagetemp[(i+off)*(num_grid_hor+(lp_order-1)) + j+off] = image[i*vsize*width+j*hsize];
				if (imagetemp[(i+off)*(num_grid_hor+(lp_order-1)) + j+off] > max_bck_val)
					max_bck_val = imagetemp[(i+off)*(num_grid_hor+(lp_order-1)) + j+off];
			}
		}
	}

	/* Filter the image with a low pass.
	   Sampling and then filtering is not a good idea because of the aliasing, but the frequency of the signal
	   we are searching is very low (a big spot on a black background), so don't care.
           Filtering and than sampling is too timing consuming */
	for (i = off; i < num_grid_ver+off; i ++) {
		for (j = off; j < num_grid_hor+off; j++) {
			for (k = 0; k < lp_order; k++) {
				for (l = 0; l < lp_order; l++) {
					imageicon[(i-off)*num_grid_hor + j-off] = imageicon[(i-off)*num_grid_hor + j-off] +
						imagetemp[(i-(k-off))*(num_grid_hor+(lp_order-1)) + j-(l-off)];
				}
			}
		}
	}

	/* Find the maximum of the filtered image. Because of the low pass filter dimension (square), we increase the pixel value
		 of the object with the same dimension and shape of the filter */
	int32_t max_h = 0;int32_t max_v = 0;
	int32_t max_val = 0, min_val = 1000000;
	int32_t count = 1;

	for (i = 0; i < num_grid_ver; i++) {
		for (j = 0; j < num_grid_hor; j++) {
			if (imageicon[i*num_grid_hor + j] > max_val) {
				max_h = j;max_v = i;
				max_val = imageicon[i*num_grid_hor + j];
				count = 1;
			}
			else if (imageicon[i*num_grid_hor + j] == max_val) {
				max_h = max_h + j; max_v = max_v + i;
				count++;
			}
			if (imageicon[i*num_grid_hor + j] < min_val) {
				min_val = imageicon[i*num_grid_hor + j];
			}
		}
	}
	/* if found more then a max value */
	if (count > 1)
		max_h = int32_t (max_h/count); max_v = int32_t (max_v/count);

	/* find the background level:
	   increase the background level until there was a big change (high derivate)
	   in the number pixels above the background. The next background level will be the good
	   background */

	/* get background starting from the maximum */
	if ((search_background_param[0] == 0) && (search_background_param[1] == 0)) {
		icon_background = (int32_t) (max_bck_val * roi_threshold * lp_order * lp_order);
	}
	else {
 		range_th = max_val - min_val;
		size_step_th = range_th / num_step_th;

		for (k = 0; k < num_step_th; k++) {
			sum_th[k] = 0;area_th[k] = 0;area_der_th[k] = 0;sum_der_th[k] = 0;
			thres_th = min_val + k*size_step_th;
			for (i = 0; i < num_grid_ver; i++) {
				for (j = 0; j < num_grid_hor; j++) {
					if (imageicon[i*num_grid_hor + j] > thres_th) {
						sum_th[k] += imageicon[i*num_grid_hor + j];
						area_th[k] += 1;
					}
				}
			}
			if (k > 0) {
				area_der_th[k] = abs(area_th[k]-area_th[k-1]);
				sum_der_th[k] = abs(sum_th[k]-sum_th[k-1]);
			}
			else {
				area_der_th[k] = 0;
				sum_der_th[k] = 0;
			}

			/* if the peak of the derivate is very close (by a factor search_background_param[1])
		  	to the next consecutive peak, the new peak will be the next one */
			if (area_der_th[k] > (max_der_th*search_background_param[1])) {
				max_der_th = area_der_th[k];
				idx_th = k;
			}
		}

		/* the good threshold will be the next */
		if (idx_th < num_step_th)
			idx_th = idx_th+1;

		/* background level of the icon */
		icon_background = (int32_t) ((min_val + idx_th*size_step_th) * roi_threshold);

	}

	/* prepare the variables for the roi find algorithm */
	int32_t start_h = max_h - 1;if (start_h < 0) start_h = 0;
	int32_t end_h = max_h + 2;if (end_h > num_grid_hor) end_h = num_grid_hor;
	int32_t start_v = max_v - 1;if (start_v < 0) start_v = 0;
	int32_t end_v = max_v + 2;if (end_v > num_grid_ver) end_v = num_grid_ver;
	bool found_hmin,found_hmax,found_vmin,found_vmax,
	     found = true;
	int32_t count_found_hmin = 0, count_found_hmax = 0,
	     count_found_vmin = 0, count_found_vmax = 0;;

	/* expand from the center until touching the background level */

	while (found) {
		found = false;
		found_hmin = false; found_hmax = false;
		found_vmin = false; found_vmax = false;
		for (i = start_v; i < end_v; i++) {
			for (j = start_h; j < end_h; j++) {
				if (imageicon[i*num_grid_hor + j] > icon_background) {
					if (i < min_start_v) {
						min_start_v = i; found_vmin = true;found = true;count_found_vmin++;
					}
					if (i > max_end_v) {
						max_end_v = i; found_vmax = true;found = true;count_found_vmax++;
					}
					if (j < min_start_h) {
						min_start_h = j; found_hmin = true;found = true;count_found_hmin++;
					}
					if (j > max_end_h) {
						max_end_h = j; found_hmax = true;found = true;count_found_hmax++;
					}
				}
			}
		}

		if (found_hmin) {
			start_h = start_h - 1;if (start_h < 0) start_h = 0;
		}
		if (found_hmax) {
			end_h = end_h + 1;if (end_h > num_grid_hor) end_h = num_grid_hor;
		}
		if (found_vmin) {
			start_v = start_v - 1;if (start_v < 0) start_v = 0;
		}
		if (found_vmax) {
			end_v = end_v + 1;if (end_v > num_grid_ver) end_v = num_grid_ver;
		}
  
		if ((start_h == 0) && (end_h == num_grid_hor) && (start_v == 0) && (end_v == num_grid_ver)) 
			found = false;

		/* expand the roi just a bit aint32_t the smoothing sides */
 		if (found == false) {
 			min_start_h = min_start_h - ((int32_t)((count_found_hmin+1)/(count_found_hmax+1)) % 2); if (start_h < 0) start_h = 0;
 			max_end_h = max_end_h + ((int32_t)((count_found_hmax+1)/(count_found_hmin+1)) % 2); if (end_h < 0) end_h = num_grid_hor;
 			min_start_v = min_start_v - ((int32_t)((count_found_vmin+1)/(count_found_vmax+1)) % 2); if (start_v < 0) start_v = 0;
 			max_end_v = max_end_v + ((int32_t)((count_found_vmax+1)/(count_found_vmin+1)) % 2); if (end_v < 0) end_v = num_grid_ver;
 		}

	}

	/* if no roi found, get all the image */
	if (min_start_h > max_end_h) {
		min_start_h = 0; max_end_h = num_grid_hor;
	}
	if (min_start_v > max_end_v) {
		min_start_v = 0; max_end_v = num_grid_ver;
	}

	/* compensate the touching of the border expanding the roi on the opposite direction */
	if (max_end_h == (num_grid_hor-1)) {
		max_end_h = num_grid_hor;
		min_start_h -= 1;
	}
	if (max_end_v == (num_grid_ver-1)) {
		max_end_v = num_grid_ver;
		min_start_v -= 1;
	}

	/* rescale to full image and get the roi */
	hor_left =  (int32_t)(((double)min_start_h)*width)/(num_grid_hor); 
	hor_right = (int32_t)(max_end_h*width)/(num_grid_hor);
	ver_up =    (int32_t)(((double)min_start_v)*height)/(num_grid_ver); 
	ver_down =  (int32_t)(max_end_v*height)/(num_grid_ver);

	/* protect against overflows */
	if (hor_left < 0) hor_left = 0;
	if (hor_right > width) hor_right = width;
	if (ver_up < 0) ver_up = 0;
	if (ver_down > height) ver_down = height;

	auto_roi_param[0] = hor_left; auto_roi_param[1] = ver_up;
	auto_roi_param[2] = hor_right;auto_roi_param[3] = ver_down;

	//printf("hor_left = %d hor_right = %d ver_up = %d ver_down = %d\n",hor_left,hor_right,ver_up,ver_down);

}









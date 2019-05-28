//
// file :         acqthread.h.cpp
//
// description :  thread class
//
// project :      TANGO Device Server
//
// $Author: giulio $ Giulio Gaio
//
// $Revision: 1.9 $ 1.0
//
// $Log: acqthread.h,v $
// Revision 1.9  2017-03-20 12:22:22  giulio
// Added aCA2000-50gm, added support for tango based trajectory feedback
//
// Revision 1.8  2013-08-07 08:00:26  giulio
// Added mutexes in get_data commands; moved to Tango8; changed shutdown timeouts
//
// Revision 1.7  2013-07-01 09:32:36  giulio
// Fixed bug in hw roi (very small roi bug)
//
// Revision 1.6  2012-09-17 11:44:53  giulio
// Major updates (hw roi, auto gain/exposure, rnm..)
//
// Revision 1.5  2012-05-30 12:42:54  giulio
// Support for 32/64 bit platforms, fixed init procedure
//
// Revision 1.4  2011-06-14 12:19:08  giulio
// Fixed GetImag16 bug
//
// Revision 1.3  2011/05/13 09:44:00  giulio
// Fixed some bugs, moved to tango7, supported Pylon 2.3.3
//
// Revision 1.2  2010/04/15 13:25:54  giulio
// New revision
//
// Revision 1.2  2009/04/22 09:46:14  giulio
// Added rt_buffer for long term acquisition
//
// Revision 1.1.1.1  2009/04/20 08:23:30  giulio
// Import iniziale di gigecam.
//
//
// copyleft :   Sincrotrone Trieste S.C.p.A.
//              Strada Statale 14 - km 163,5 in AREA Science Park
//              34012 Basovizza, Trieste ITALY
// 

#ifndef ACQTHREAD_H
#define ACQTHREAD_H

#include <omnithread.h>
#include <tango.h>
#include "GigeCam.h"

#define GIGECAM_ACQTHREAD_NUM_RX         5
#define GIGECAM_ACQTHREAD_TRIGGER_IDX    0
#define GIGECAM_ACQTHREAD_GAIN_IDX       1
#define GIGECAM_ACQTHREAD_EXPOSURE_IDX   2
#define GIGECAM_ACQTHREAD_BLACKLEVEL_IDX 3
#define GIGECAM_ACQTHREAD_IMAGEDEPTH_IDX 4

#define GIGECAM_AUTO_MODE_OFF   0
#define GIGECAM_AUTO_MODE_ONCE  1  /* run once (30 shots) */
#define GIGECAM_AUTO_MODE_ON    2  /* continuos */

class CGrabBuffer;

class acqthread : public omni_thread
{
	
	public:
		friend class CGrabBuffer;
		/* acqthread::acqthread(): initialize init loop */
		acqthread(GigeCam_ns::GigeCam *s) {myacq = s;};
		/* acqthread::~acqthread(): object destructor */
		~acqthread() {};
		void CameraStop();
		bool CameraGetTriggerFrequency(float *);
		bool CameraGetTrigger(bool *);
		bool CameraSetTrigger(bool);
		bool CameraGetImageDepth(short *);
		bool CameraSetGain(double);
		bool CameraGetGain(double *);
		bool CameraSetExposure(int32_t);
		bool CameraGetExposure(int32_t *);
		bool CameraSetBlackLevel(double);
		bool CameraGetBlackLevel(double *);		
		bool CameraSetTriggerDelay(int32_t);
		bool CameraGetTriggerDelay(int32_t *);
		bool CameraSetBinning(int32_t);
		bool CameraGetBinning(int32_t *);
		void StreamGrabberDelete(void);
		void CameraDelete(void);
		void delete_data(void);
		long Width(void) {return width;};
		long Height(void){return height;};
		
	private:
		Camera_t *myCamera;
		Camera_t::StreamGrabber_t *StreamGrabber;
		std::vector<CGrabBuffer*> BufferList;
		GigeCam_ns::GigeCam *myacq;
		struct timeval camera_get_trigger,        /* 0 */
			             camera_get_gain,      /* 1 */
			             camera_get_exposure,       /* 2 */
			             camera_get_blacklevel,    /* 3 */
			             camera_get_imagedepth,    /* 4 */
			             camera_get_binning;       /* 5 */
		int32_t camera_get_counter;
		int32_t width, height, hor_grid_size, ver_grid_size;
		bool init_camera_flag, open_flag, streamgrabber_flag;
		bool CameraSetup(uint32_t);
		void CameraSetRoi(void);
		bool CameraGrab(void);
		void CameraSetAcqMode(void);
		void CameraInit(void);
		bool CameraGainRaw2dB(int32_t, double *);
		bool CameraGaindB2Raw(double, int32_t *);
		void CameraInitGainParam(void);
		void StreamGrabberInit(void);
		void ProcessImage(unsigned char *);
		bool save_data(const GrabResult *Result);
		void update_configuration(void);
#ifdef GIGECAM_RNM
		int64_t ReverseBytes64(int64_t);
#endif
		bool init_data(void);
		bool auto_gain_feedback();
		bool auto_exposure_feedback();

	protected:
		/* acqthread::run():	start init loop */
		void run(void *);
		
};	/* end class acqthread() */


#endif

/* EOF */

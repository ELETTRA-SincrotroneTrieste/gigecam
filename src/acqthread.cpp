//
// file :         acqthread.cpp
//
// description :  gige camera acquisition thread
//
// project :      TANGO Device Server
//
// $Author: giulio $ Giulio Gaio
//
// $Revision: 1.62 $ 1.0
//
// $Log: acqthread.cpp,v $
// Revision 1.62  2019-01-25 16:14:52  giulio
// Fixed bug of 180 degrees rotation + HwROI
//
// Revision 1.61  2019-01-25 08:08:54  giulio
// Support to 180 degrees HWROI support
//
// Revision 1.60  2019-01-24 15:42:29  giulio
// Added aAV2300 and acA1440-73gm
//
// Revision 1.59  2018-10-05 10:03:56  giulio
// Fixed some bugs
//
// Revision 1.58  2018-10-03 13:55:16  giulio
// Allowing set scale/offset when ccd is off, bugfixing
//
// Revision 1.57  2018-06-19 07:06:53  giulio
// Added acA2040-35-gm and acA2500-14gm support, Makefile for pylon5
//
// Revision 1.56  2017-08-25 08:53:13  giulio
// Fixed bug in exposure api
//
// Revision 1.55  2017-04-19 13:00:04  giulio
// If fitting/ sum threshold failed then state is set to ALARM
//
// Revision 1.54  2017-04-19 07:00:53  giulio
// Fixed feedback mode
//
// Revision 1.53  2017-03-20 12:22:22  giulio
// Added aCA2000-50gm, added support for tango based trajectory feedback
//
// Revision 1.52  2016-07-28 11:49:09  giulio
// Added acA1300-75gm
//
// Revision 1.51  2016-04-04 07:39:08  giulio
// Added acA1300-60gm
//
// Revision 1.50  2016-03-16 14:01:47  giulio
// Adde acA645-100gm and acA1920-50gm
//
// Revision 1.49  2016-02-03 09:29:58  giulio
// Added support for scA1600-28gm and acA1300-30gm
//
// Revision 1.48  2015-11-24 10:35:04  giulio
// Integrated acA1300-gm
//
// Revision 1.47  2015-06-10 09:43:56  giulio
// Temporary solution to fix acA1600 digital gain configuration
//
// Revision 1.46  2015-02-10 10:13:04  giulio
// Added acA1600 camera
//
// Revision 1.45  2015-01-19 10:43:31  giulio
// Added av1600 camera, solved problems with ROI
//
// Revision 1.44  2014-12-30 08:33:57  giulio
// Added Model attribute
//
// Revision 1.43  2014-08-28 07:49:45  giulio
// Added imagesum attribute and scA1400 ccd
//
// Revision 1.42  2013-12-04 09:53:20  giulio
// Moved to Pylon3, added some eth tuning properties (band reservation, interpacket delay)
//
// Revision 1.41  2013-08-21 07:59:48  giulio
// Default is with rnm enabled
//
// Revision 1.40  2013-08-07 08:00:26  giulio
// Added mutexes in get_data commands; moved to Tango8; changed shutdown timeouts
//
// Revision 1.39  2013-07-09 08:34:16  giulio
// Added packet size configuration
//
// Revision 1.38  2013-07-08 07:45:14  giulio
// Clean heap memory by using malloc_trim(0)
//
// Revision 1.37  2013-07-01 09:32:36  giulio
// Fixed bug in hw roi (very small roi bug)
//
// Revision 1.36  2013-06-10 14:50:55  giulio
// Supported 75gm 16bit data format
//
// Revision 1.35  2013-05-17 13:14:08  giulio
// Added disable trigger timeout, stream grabber configurable
//
// Revision 1.34  2013-05-16 12:41:23  giulio
// Disable hw roi when image rotated, fix hor hw roi overflow
//
// Revision 1.33  2013-04-11 14:29:53  giulio
// Added digital image stabilization, saved roi_threshold
//
// Revision 1.32  2013-02-12 09:01:37  giulio
// Minimized bug in connect-disconnect with 2.3.3
//
// Revision 1.31  2013-02-08 15:12:34  giulio
// Added autoreconnect disable + avA1000 ccd
//
// Revision 1.30  2013-01-24 08:47:29  giulio
// Added binning
//
// Revision 1.29  2013-01-23 14:01:18  giulio
// Fixed bug on ext. trigger absence (server locked)
//
// Revision 1.28  2012-11-29 15:35:11  giulio
// Fixed 16bit hwroi position rescale
//
// Revision 1.27  2012-11-29 15:30:49  giulio
// Decreased basler n_buffer to 1
//
// Revision 1.26  2012-11-02 15:00:59  giulio
// Initialize background image
//
// Revision 1.25  2012-10-09 16:13:01  giulio
// Fixed bug in the pause mode (lock detected)
//
// Revision 1.24  2012-09-28 12:16:09  giulio
// Shifted ace minimum gains to 0
//
// Revision 1.23  2012-09-28 11:32:30  giulio
// Review of gigecam gain fitting for Ace ccd
//
// Revision 1.22  2012-09-28 09:09:24  giulio
// Added support for acA780-gm camera
//
// Revision 1.21  2012-09-24 13:56:29  giulio
// Add  hw roi limits check
//
// Revision 1.20  2012-09-18 12:15:39  giulio
// Fixed error counter at first cycle
//
// Revision 1.19  2012-09-17 11:44:53  giulio
// Major updates (hw roi, auto gain/exposure, rnm..)
//
// Revision 1.18  2012-08-09 10:23:46  giulio
// Removed long sleep(500000) in acqthread
//
// Revision 1.17  2012-07-19 09:34:13  giulio
// Delete some delete attribute instructions automatically doubled by pogo7
//
// Revision 1.16  2012-07-19 07:58:03  giulio
// Sleep something after setting FAULT state in acqthread
//
// Revision 1.15  2012-07-11 10:26:54  giulio
// Commented debug stream in ccd api
//
// Revision 1.14  2012-07-02 14:48:36  giulio
// Increased erro msg info in ccd api
//
// Revision 1.13  2012-06-15 15:40:57  giulio
// Fixed bug in start device (wait link flag), splitted exceptions in thread destruction
//
// Revision 1.12  2012-05-30 12:42:54  giulio
// Support for 32/64 bit platforms, fixed init procedure
//
// Revision 1.11  2011-12-28 08:00:53  giulio
// Extended exposure timeout to its maximu value. Store all ccd parameters through SaveScale command
//
// Revision 1.10  2011/10/24 07:31:48  giulio
// Added new ccd support (sony chipset)
//
// Revision 1.9  2011/06/14 12:19:08  giulio
// Fixed GetImag16 bug
//
// Revision 1.8  2011/05/13 09:44:00  giulio
// Fixed some bugs, moved to tango7, supported Pylon 2.3.3
//
// Revision 1.7  2011/01/25 10:24:03  giulio
// Added debug thread attribute
//
// Revision 1.6  2011/01/21 09:01:34  giulio
// Fixed save property issue
//
// Revision 1.5  2010/12/15 09:40:13  giulio
// Added support for scA640 and piA1000
//
// Revision 1.4  2010/08/30 14:54:55  giulio
// Fixed get_image16 bug. Circular buffers return 0 when data is not avaliable
//
// Revision 1.3  2010/04/30 09:00:53  giulio
// Added Gaussian fitting methods
//
// Revision 1.2  2010/04/15 13:25:54  giulio
// New revision
//
// Revision 1.1.1.1  2009/04/20 08:23:30  giulio
// Import iniziale di gigecam.
//
//
// copyleft :   Sincrotrone Trieste S.C.p.A.
//              Strada Statale 14 - km 163,5 in AREA Science Park
//              34012 Basovizza, Trieste ITALY
//
#include <omnithread.h>
#include <buffer.h>
#include "GigeCam.h"
#include "acqthread.h"
#include <malloc.h>


using namespace Pylon;



class CGrabBuffer
{
    public:
        CGrabBuffer(const size_t ImageSize);
        ~CGrabBuffer();
        uint8_t* GetBufferPointer(void) { return m_pBuffer; }
        StreamBufferHandle GetBufferHandle(void) { return m_hBuffer; }
        void SetBufferHandle(StreamBufferHandle hBuffer) { m_hBuffer = hBuffer; };

    protected:
        uint8_t *m_pBuffer;
        StreamBufferHandle m_hBuffer;
};

// Constructor allocates the image buffer
CGrabBuffer::CGrabBuffer(const size_t ImageSize):
        m_pBuffer(NULL)
{
    m_pBuffer = new uint8_t[ ImageSize ];
    if (NULL == m_pBuffer)
    {
        GenICam::GenericException e("Not enough memory to allocate image buffer", __FILE__, __LINE__);
        throw e;
    }
}

// Freeing the memory
CGrabBuffer::~CGrabBuffer()
{
    if (NULL != m_pBuffer)
        delete[] m_pBuffer;
}


void SetConsoleCtrlHandler(void (*func)(int))
{
	signal(SIGINT, func);
}

// stop acquisition, shutdown the device server calling the delete_device() method
void CtrlCHandler(int32_t Signo)
{  
	//kill(parent, SIGINT);
}

void ScanName(char *input, char *mod, char* ser, char *ipaddr)
{
	int32_t cnt = 0;
	char *p, *c, tmp[GIGECAM_DESC];
	
	memset(tmp,0,GIGECAM_DESC);
	memset(ipaddr,0,GIGECAM_DESC);
	/* string example: scA780-54gm#0030530C3FBF#10.0.0.10:0 */
	
	p = strtok(input, "#");
	while (p != NULL) {
		if (cnt == 0) {
			sprintf(mod,"%s",p);cnt++;
		}
		else if (cnt == 1) {
			sprintf(ser,"%s",p);cnt++;
		}
		else if (cnt == 2) {
			sprintf(tmp,"%s",p);cnt++;
			/* terminate string at first occurrence of ':' */
			c = strchr(tmp, ':');
			if (c != NULL)
			  *c = 0;
			strncpy(ipaddr,tmp,strlen(tmp));
		}
		p = strtok(NULL, "#");
 
	}

}

#ifdef GIGECAM_RNM
int64_t acqthread::ReverseBytes64(int64_t value)
{
  return (value & 0x00000000000000FFULL) << 56 | (value & 0x000000000000FF00ULL) << 40 |
		(value & 0x0000000000FF0000ULL) << 24 | (value & 0x00000000FF000000ULL) << 8 |
		(value & 0x000000FF00000000ULL) >> 8 | (value & 0x0000FF0000000000ULL) >> 24 |
		(value & 0x00FF000000000000ULL) >> 40 | (value & 0xFF00000000000000ULL) >> 56;
}
#endif

/*
 * update_configuration
 * Update on the fly the ImageProc parameters
 */
void acqthread::update_configuration(void)
{

	/* if this is the first run */
	if (myacq->cam_param[myacq->cam_idx].configured == false) {
		/* overload manual roi param in case max image size doesn't match */
		if ((myacq->cam_param[myacq->cam_idx].manual_roi_param[0] > width) ||
				(myacq->cam_param[myacq->cam_idx].manual_roi_param[2] > width) ||
				(myacq->cam_param[myacq->cam_idx].manual_roi_param[1] > height) ||
				(myacq->cam_param[myacq->cam_idx].manual_roi_param[3] > height) ||
				(myacq->cam_param[myacq->cam_idx].manual_roi_param[0] < 0) ||
				(myacq->cam_param[myacq->cam_idx].manual_roi_param[1] < 0) ||
				(myacq->cam_param[myacq->cam_idx].manual_roi_param[2] < 0) ||
				(myacq->cam_param[myacq->cam_idx].manual_roi_param[3] < 0)) {
			myacq->cam_param[myacq->cam_idx].manual_roi_param[0] = 0;
			myacq->cam_param[myacq->cam_idx].manual_roi_param[1] = 0;
			myacq->cam_param[myacq->cam_idx].manual_roi_param[2] = width;
			myacq->cam_param[myacq->cam_idx].manual_roi_param[3] = height;
		}

		/* get sensor width and height */
		myacq->cam_param[myacq->cam_idx].max_width = myCamera->SensorWidth();
		myacq->cam_param[myacq->cam_idx].max_height = myCamera->SensorHeight();

		/* piA1000 */
		if (myCamera->SensorWidth() == 1004) {
			myCamera->GainSelector.SetValue(GainSelector_Tap1);
			myCamera->GainRaw.SetValue(0);
			myCamera->GainSelector.SetValue(GainSelector_Tap2);
			myCamera->GainRaw.SetValue(0);
			myCamera->GainSelector.SetValue(GainSelector_All);
		}
		else if ((strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler avA1000-100gm") == 0) ||
        (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler avA2300-25gm") == 0) ||
			  (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler avA1600-100gm") == 0))    {
			/* read all images in one tap */
#ifdef CONFIG_64
			myCamera->GainSelector.SetValue( GainSelector_Tap1 );
			myCamera->GainRaw.SetValue( 0 );
			myCamera->GainSelector.SetValue( GainSelector_Tap2 );
			myCamera->GainRaw.SetValue( 0 );
			myCamera->GainSelector.SetValue( GainSelector_Tap3 );
			myCamera->GainRaw.SetValue( 0 );
			myCamera->GainSelector.SetValue( GainSelector_Tap4 );
			myCamera->GainRaw.SetValue( 0 );
			myCamera->GainSelector.SetValue(GainSelector_All);
			myCamera->SensorDigitizationTaps.SetValue(SensorDigitizationTaps_One);
			myCamera->SensorDigitizationTaps.SetValue(SensorDigitizationTaps_Four);
#endif
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1600-60gm") == 0) {
			myCamera->GainSelector.SetValue(GainSelector_DigitalAll);
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1300-60gm") == 0) {
			myCamera->GainSelector.SetValue(GainSelector_DigitalAll);
		}

		/* main hw parameters */
		CameraSetExposure((int32_t)myacq->cam_param[myacq->cam_idx].exposure);
		CameraSetGain((int32_t) myacq->cam_param[myacq->cam_idx].gain);
		CameraSetTrigger((short) myacq->cam_param[myacq->cam_idx].trigger);

	}

	if (myacq->cam_param[myacq->cam_idx].img_depth == 8) {
		/* set horizontal/vertical reference position offset */
		if (myacq->take_reference == true) {
			myacq->take_reference = false;
			for (int32_t i = 0; i < myacq->imageBufferSize; i++) {
				myacq->img8[i]->ref_hor_pos_offset = myacq->img8[myacq->img_cur]->hor_pos + 2 * myacq->img8[myacq->img_cur]->hor_pos_offset;
				myacq->img8[i]->ref_ver_pos_offset = myacq->img8[myacq->img_cur]->ver_pos;					
			}
		}
		/* set image size */
		myacq->img8[myacq->img_nxt]->set_size(width, height);

		/* set low pass filter order */
		myacq->img8[myacq->img_nxt]->set_lp_order(myacq->cam_param[myacq->cam_idx].lp_order);
		/* set decimation factor */
		myacq->img8[myacq->img_nxt]->set_hor_grid(hor_grid_size);
		myacq->img8[myacq->img_nxt]->set_ver_grid(ver_grid_size);

		/* set mirror */
		myacq->img8[myacq->img_nxt]->set_mirror(myacq->cam_param[myacq->cam_idx].mirror);
		/* enable image processing */
		myacq->img8[myacq->img_nxt]->set_enable_process(myacq->cam_param[myacq->cam_idx].process_enable);
		/* set auto roi */
		myacq->img8[myacq->img_nxt]->set_enable_auto_roi(myacq->cam_param[myacq->cam_idx].auto_roi);
		/* set roi threshold */
		myacq->img8[myacq->img_nxt]->set_roi_threshold(myacq->cam_param[myacq->cam_idx].roi_threshold);
		myacq->img8[myacq->img_nxt]->set_search_background_param(myacq->cam_param[myacq->cam_idx].search_background_param);
		/* plot mode and level */
		myacq->img8[myacq->img_nxt]->set_plot_mode(myacq->cam_param[myacq->cam_idx].plot_mode);
		myacq->img8[myacq->img_nxt]->set_plot_level(myacq->cam_param[myacq->cam_idx].plot_level);
		myacq->img8[myacq->img_nxt]->set_plot_offset_axis(myacq->cam_param[myacq->cam_idx].plot_offset_axis);		
		/* enable fast calculation */
		myacq->img8[myacq->img_nxt]->set_enable_fast_calc(myacq->cam_param[myacq->cam_idx].fast_calc);
		/* set manual roi */
		myacq->img8[myacq->img_nxt]->set_enable_manual_roi(myacq->cam_param[myacq->cam_idx].sw_roi);
		myacq->img8[myacq->img_nxt]->set_manual_roi(myacq->cam_param[myacq->cam_idx].manual_roi_param);
		/* set process mode */
		myacq->img8[myacq->img_nxt]->set_algorithm(myacq->cam_param[myacq->cam_idx].process_mode);
		/* update fitting timeout */
		if (myacq->frequency > 0)
			myacq->img8[myacq->img_nxt]->set_fit_tout(1000/myacq->frequency/2); /* leave half of the time period for fitting */
		/* digital image stabilization parameters */
		myacq->img8[myacq->img_nxt]->set_ref_raw_pos(myacq->cam_param[myacq->cam_idx].ref_raw_hor_pos, myacq->cam_param[myacq->cam_idx].ref_raw_ver_pos);
		myacq->img8[myacq->img_nxt]->set_enable_dis(myacq->cam_param[myacq->cam_idx].dis_enable);


		/* if rotation not multiple of 90, enable rotation interpolation */
		if ((int32_t)myacq->cam_param[myacq->cam_idx].rotation % 90) {
			myacq->img8[myacq->img_nxt]->set_enable_rotation_filter(true);
		}
		else
			myacq->img8[myacq->img_nxt]->set_enable_rotation_filter(false);

	}
	else if (myacq->cam_param[myacq->cam_idx].img_depth == 16) {

		/* set horizontal/vertical reference position offset */
		if (myacq->take_reference == true) {
			myacq->take_reference = false;
			for (int32_t i = 0; i < myacq->imageBufferSize; i++) {
				myacq->img16[i]->ref_hor_pos_offset = myacq->img16[myacq->img_cur]->hor_pos  + 2 * myacq->img16[myacq->img_cur]->hor_pos_offset;
				myacq->img16[i]->ref_ver_pos_offset = myacq->img16[myacq->img_cur]->ver_pos;	
			}
		}

		/* set image size */
		myacq->img16[myacq->img_nxt]->set_size((int32_t)width, (int32_t) height);
		/* set low pass filter order */
		myacq->img16[myacq->img_nxt]->set_lp_order(myacq->cam_param[myacq->cam_idx].lp_order);
		/* set decimation factor */
		myacq->img16[myacq->img_nxt]->set_hor_grid(hor_grid_size);
		myacq->img16[myacq->img_nxt]->set_ver_grid(ver_grid_size);

		/* set mirror */
		myacq->img16[myacq->img_nxt]->set_mirror(myacq->cam_param[myacq->cam_idx].mirror);
		/* enable image processing */
		myacq->img16[myacq->img_nxt]->set_enable_process(myacq->cam_param[myacq->cam_idx].process_enable);
		/* set auto roi */
		myacq->img16[myacq->img_nxt]->set_enable_auto_roi(myacq->cam_param[myacq->cam_idx].auto_roi);
		/* set roi threshold */
		myacq->img16[myacq->img_nxt]->set_roi_threshold(myacq->cam_param[myacq->cam_idx].roi_threshold);
		myacq->img16[myacq->img_nxt]->set_search_background_param(myacq->cam_param[myacq->cam_idx].search_background_param);
		/* plot mode and level */
		myacq->img16[myacq->img_nxt]->set_plot_mode(myacq->cam_param[myacq->cam_idx].plot_mode);
		myacq->img16[myacq->img_nxt]->set_plot_level(myacq->cam_param[myacq->cam_idx].plot_level);
		myacq->img16[myacq->img_nxt]->set_plot_offset_axis(myacq->cam_param[myacq->cam_idx].plot_offset_axis);
		/* enable fast calculation */
		myacq->img16[myacq->img_nxt]->set_enable_fast_calc(myacq->cam_param[myacq->cam_idx].fast_calc);
		/* set manual roi */
		myacq->img16[myacq->img_nxt]->set_enable_manual_roi(myacq->cam_param[myacq->cam_idx].sw_roi);
		myacq->img16[myacq->img_nxt]->set_manual_roi(myacq->cam_param[myacq->cam_idx].manual_roi_param);
		/* set process mode */
		myacq->img16[myacq->img_nxt]->set_algorithm(myacq->cam_param[myacq->cam_idx].process_mode);
		/* update fitting timeout */
		if (myacq->frequency > 0)
			myacq->img16[myacq->img_nxt]->set_fit_tout(1000/myacq->frequency/2); /* leave half of the time period for fitting */

		/* digital image stabilization parameters */
		myacq->img16[myacq->img_nxt]->set_ref_raw_pos(myacq->cam_param[myacq->cam_idx].ref_raw_hor_pos, myacq->cam_param[myacq->cam_idx].ref_raw_ver_pos);
		myacq->img16[myacq->img_nxt]->set_enable_dis(myacq->cam_param[myacq->cam_idx].dis_enable);

		/* if rotation not multiple of 90, enable rotation interpolation */
		if ((int32_t)myacq->cam_param[myacq->cam_idx].rotation % 90)
			myacq->img16[myacq->img_nxt]->set_enable_rotation_filter(true);
		else
			myacq->img16[myacq->img_nxt]->set_enable_rotation_filter(false);

	}

	myacq->cam_param[myacq->cam_idx].configured = true;

}

/*
 * acqthread::save_data
 */
bool acqthread::save_data(const GrabResult *Result)
{
	struct timeval tv;
	bool avg_thres_ok = true;

	gettimeofday(&tv,NULL);

	/* 8 bit */
	if (myacq->cam_param[myacq->cam_idx].img_depth == 8) {
		/* rotate image */
		if (myacq->trigger_background) {
			myacq->cam_param[myacq->cam_idx].bckimg8->push_permutation((const unsigned char*) Result->Buffer(),
				myacq->cam_param[myacq->cam_idx].img_permutation,
				(uint32_t) Result->FrameNr(), (uint32_t)  myacq->bunch_number);
			myacq->trigger_background = false;
		}
		if (myacq->cam_param[myacq->cam_idx].background_subtraction) {
			myacq->img8[myacq->img_nxt]->push_permutation_with_background((const unsigned char*) Result->Buffer(), myacq->cam_param[myacq->cam_idx].img_permutation,
				(uint32_t)Result->FrameNr(), (uint32_t) myacq->bunch_number,(const unsigned char*)myacq->cam_param[myacq->cam_idx].bckimg8->image);
		}
		else {
			myacq->img8[myacq->img_nxt]->push_permutation((const unsigned char*) Result->Buffer(), myacq->cam_param[myacq->cam_idx].img_permutation, 
				(uint32_t) Result->FrameNr(), (uint32_t)myacq->bunch_number);
		}
	}
	/* 16 bit */
	else if (myacq->cam_param[myacq->cam_idx].img_depth == 16) {
		if (myacq->trigger_background) {
			myacq->cam_param[myacq->cam_idx].bckimg16->push_permutation((const unsigned short*) Result->Buffer(), 
				myacq->cam_param[myacq->cam_idx].img_permutation,
				(uint32_t) Result->FrameNr(),(uint32_t) myacq->bunch_number);
			myacq->trigger_background = false;
		}
		if (myacq->cam_param[myacq->cam_idx].background_subtraction) {
			myacq->img16[myacq->img_nxt]->push_permutation_with_background((const unsigned short*) Result->Buffer(), myacq->cam_param[myacq->cam_idx].img_permutation,
				(uint32_t) Result->FrameNr(), (uint32_t) myacq->bunch_number,(const unsigned short *)myacq->cam_param[myacq->cam_idx].bckimg16->image);
		}
		else {
			myacq->img16[myacq->img_nxt]->push_permutation((const unsigned short*) Result->Buffer(), myacq->cam_param[myacq->cam_idx].img_permutation, 
				(uint32_t) Result->FrameNr(), (uint32_t) myacq->bunch_number);
		}
	}

	myacq->img_cur = myacq->img_nxt;
	myacq->img_nxt++;

	/* save data into extended circular buffers */
	if (myacq->cam_param[myacq->cam_idx].img_depth == 8) {

		if (*myacq->attr_ImageSumSamples_read) {
			myacq->in_mutex->lock();
			myacq->imagesum_counter++;
			int img_sub_idx;
			if (myacq->imagesum_counter > *myacq->attr_ImageSumSamples_read) {
				img_sub_idx = (myacq->img_cur + myacq->imageBufferSize - *myacq->attr_ImageSumSamples_read) % myacq->imageBufferSize;
				for (int i = 0; i < myacq->img8[myacq->img_cur]->get_width()*myacq->img8[myacq->img_cur]->get_height(); i++) {
					myacq->attr_ImageSum_read[i] += (Tango::DevLong) myacq->img8[myacq->img_cur]->image[i]-(Tango::DevLong) myacq->img8[img_sub_idx]->image[i];
				}
			}
			else {
				for (int i = 0; i < myacq->img8[myacq->img_cur]->get_width()*myacq->img8[myacq->img_cur]->get_height(); i++) {
					myacq->attr_ImageSum_read[i] += (Tango::DevLong) myacq->img8[myacq->img_cur]->image[i];
				}
			}
			myacq->in_mutex->unlock();
		}

		if (myacq->circ_buffer) {
			push_data((void *) &myacq->img8[myacq->img_cur]->phase, tv, myacq->bunch_number, myacq->buf_phase);
			push_data((void *) &myacq->img8[myacq->img_cur]->area, tv, myacq->bunch_number, myacq->buf_area);
			push_data((void *) &myacq->img8[myacq->img_cur]->hor_pos, tv, myacq->bunch_number, myacq->buf_hor_pos);
			push_data((void *) &myacq->img8[myacq->img_cur]->hor_sigma, tv, myacq->bunch_number, myacq->buf_hor_sigma);
			push_data((void *) &myacq->img8[myacq->img_cur]->saturation, tv, myacq->bunch_number, myacq->buf_saturation);
			push_data((void *) &myacq->img8[myacq->img_cur]->ver_pos, tv, myacq->bunch_number, myacq->buf_ver_pos);
			push_data((void *) &myacq->img8[myacq->img_cur]->ver_sigma, tv, myacq->bunch_number, myacq->buf_ver_sigma);
			push_data((void *) &myacq->img8[myacq->img_cur]->intensity, tv, myacq->bunch_number, myacq->buf_intensity);
			push_data((void *) &myacq->img8[myacq->img_cur]->fit_error, tv, myacq->bunch_number, myacq->buf_fit_error);
			push_data((void *) &myacq->bunch_number, tv, myacq->bunch_number, myacq->buf_acquisition_counter);
			push_data((void *) &myacq->img8[myacq->img_cur]->max_value, tv, myacq->bunch_number, myacq->buf_max_val);
			push_data((void *) myacq->attr_Gain_read, tv, myacq->bunch_number, myacq->buf_gain);
			push_data((void *) myacq->attr_Exposure_read, tv, myacq->bunch_number, myacq->buf_exposure);

			double mean_intensity = 0;
			if ((myacq->img8[myacq->img_cur]->area != 0) && (myacq->img8[myacq->img_cur]->hor_calibration != 0) &&
				(myacq->img8[myacq->img_cur]->ver_calibration != 0)) {
				mean_intensity = myacq->img8[myacq->img_cur]->intensity / 
													myacq->img8[myacq->img_cur]->area /
													myacq->img8[myacq->img_cur]->hor_calibration /
													myacq->img8[myacq->img_cur]->ver_calibration;
			}
			double hor_pos = myacq->img8[myacq->img_cur]->hor_pos, ver_pos = myacq->img8[myacq->img_cur]->ver_pos;
			/* force rnm value to 0 in case of fitting error or signal under threshold */
			*myacq->attr_MeanIntensity_read = mean_intensity;
			if ((mean_intensity < *myacq->attr_MeanIntensityThreshold_read) || myacq->img8[myacq->img_cur]->fit_error) {
				hor_pos = NAN; ver_pos = NAN;
				if (*myacq->attr_FeedbackMode_read) {
					if (*myacq->attr_MeanIntensity_read < (double)*myacq->attr_MeanIntensityThreshold_read) {
						avg_thres_ok = false;
						myacq->set_status("signal under threshold");
					}
					else if (myacq->img8[myacq->img_cur]->fit_error) {
						avg_thres_ok = false;
						myacq->set_status("profile fitting failed");
					}	
				}
			}

#ifdef GIGECAM_RNM
			if ((myacq->rnmIndex >= 0) && (myacq->rnmEnabled)){
				char check = RNM_DATA_OK;
				int64_t hor_pos_big_endian = ReverseBytes64(*(int64_t *)&hor_pos);
				int64_t ver_pos_big_endian = ReverseBytes64(*(int64_t *)&ver_pos);
				int64_t mean_intensity_big_endian = ReverseBytes64(*(int64_t *)&mean_intensity);
				rnm_ccd_horpos_set_value(myacq->rnmIndex, myacq->rnmIndex+1, (void *) &hor_pos_big_endian, &check);
				rnm_ccd_verpos_set_value(myacq->rnmIndex, myacq->rnmIndex+1, (void *) &ver_pos_big_endian, &check);
				rnm_ccd_intensity_set_value(myacq->rnmIndex, myacq->rnmIndex+1, (void *) &mean_intensity_big_endian, &check);
			}
#endif

		}
	}
	else if (myacq->cam_param[myacq->cam_idx].img_depth == 16) {

		if (*myacq->attr_ImageSumSamples_read) {
			myacq->in_mutex->lock();
			myacq->imagesum_counter++;
			int img_sub_idx;
			if (myacq->imagesum_counter >= *myacq->attr_ImageSumSamples_read) {
				img_sub_idx = (myacq->img_cur + myacq->imageBufferSize - *myacq->attr_ImageSumSamples_read) % myacq->imageBufferSize;
				for (int i = 0; i < myacq->img16[myacq->img_cur]->get_width()*myacq->img16[myacq->img_cur]->get_height(); i++) {
					myacq->attr_ImageSum_read[i] += (Tango::DevLong) myacq->img16[myacq->img_cur]->image[i]-(Tango::DevLong) myacq->img16[img_sub_idx]->image[i];
				}
			}
			else {
				for (int i = 0; i < myacq->img16[myacq->img_cur]->get_width()*myacq->img16[myacq->img_cur]->get_height(); i++) {
					myacq->attr_ImageSum_read[i] += (Tango::DevLong) myacq->img16[myacq->img_cur]->image[i];
				}
			}
			myacq->in_mutex->unlock();
		}

		if (myacq->circ_buffer) {
			push_data((void *) &myacq->img16[myacq->img_cur]->phase, tv, myacq->bunch_number, myacq->buf_phase);
			push_data((void *) &myacq->img16[myacq->img_cur]->area, tv, myacq->bunch_number, myacq->buf_area);
			push_data((void *) &myacq->img16[myacq->img_cur]->hor_pos, tv, myacq->bunch_number, myacq->buf_hor_pos);
			push_data((void *) &myacq->img16[myacq->img_cur]->hor_sigma, tv, myacq->bunch_number, myacq->buf_hor_sigma);
			push_data((void *) &myacq->img16[myacq->img_cur]->saturation, tv, myacq->bunch_number, myacq->buf_saturation);
			push_data((void *) &myacq->img16[myacq->img_cur]->ver_pos, tv, myacq->bunch_number, myacq->buf_ver_pos);
			push_data((void *) &myacq->img16[myacq->img_cur]->ver_sigma, tv, myacq->bunch_number, myacq->buf_ver_sigma);
			push_data((void *) &myacq->img16[myacq->img_cur]->intensity, tv, myacq->bunch_number, myacq->buf_intensity);
			push_data((void *) &myacq->img16[myacq->img_cur]->fit_error, tv, myacq->bunch_number, myacq->buf_fit_error);
			push_data((void *) &myacq->bunch_number, tv, myacq->bunch_number, myacq->buf_acquisition_counter);
			push_data((void *) &myacq->img16[myacq->img_cur]->max_value, tv, myacq->bunch_number, myacq->buf_max_val);
			push_data((void *) myacq->attr_Gain_read, tv, myacq->bunch_number, myacq->buf_gain);
			push_data((void *) myacq->attr_Exposure_read, tv, myacq->bunch_number, myacq->buf_exposure);
		}

		double mean_intensity = 0;
		if ((myacq->img16[myacq->img_cur]->area != 0) && (myacq->img16[myacq->img_cur]->hor_calibration != 0) &&
			(myacq->img16[myacq->img_cur]->ver_calibration != 0)) {
			mean_intensity = myacq->img16[myacq->img_cur]->intensity / 
												myacq->img16[myacq->img_cur]->area /
												myacq->img16[myacq->img_cur]->hor_calibration /
												myacq->img16[myacq->img_cur]->ver_calibration;
		}
		double hor_pos = myacq->img16[myacq->img_cur]->hor_pos,
						ver_pos = myacq->img16[myacq->img_cur]->ver_pos;
		/* force rnm value to 0 in case of fitting error or signal under threshold */
		*myacq->attr_MeanIntensity_read = mean_intensity; 
		if ((mean_intensity < *myacq->attr_MeanIntensityThreshold_read) || myacq->img16[myacq->img_cur]->fit_error) {
			hor_pos = NAN; ver_pos = NAN;
			if (*myacq->attr_FeedbackMode_read) {
				if (*myacq->attr_MeanIntensity_read < (double)*myacq->attr_MeanIntensityThreshold_read) {
					avg_thres_ok = false;
					myacq->set_status("signal under threshold");
				}
				else if (myacq->img8[myacq->img_cur]->fit_error) {
					avg_thres_ok = false;
					myacq->set_status("profile fitting failed");
				}	
			}
		}

#ifdef GIGECAM_RNM
		if ((myacq->rnmIndex >= 0) && (myacq->rnmEnabled)){
			char check = RNM_DATA_OK;
			int64_t hor_pos_big_endian = ReverseBytes64(*(int64_t *)&hor_pos);
			int64_t ver_pos_big_endian = ReverseBytes64(*(int64_t *)&ver_pos);
			int64_t mean_intensity_big_endian = ReverseBytes64(*(int64_t *)&mean_intensity);
			rnm_ccd_horpos_set_value(myacq->rnmIndex, myacq->rnmIndex+1, (void *) &hor_pos_big_endian, &check);
			rnm_ccd_verpos_set_value(myacq->rnmIndex, myacq->rnmIndex+1, (void *) &ver_pos_big_endian, &check);
			rnm_ccd_intensity_set_value(myacq->rnmIndex, myacq->rnmIndex+1, (void *) &mean_intensity_big_endian, &check);
		}
#endif
	}

	if (myacq->img_nxt >= myacq->imageBufferSize)
		myacq->img_nxt = 0;

	if (avg_thres_ok) {
		*myacq->attr_FeedbackError_read = false;
	}
	else {
		*myacq->attr_FeedbackError_read = true;
	}

	return avg_thres_ok;

}

/*
 * acqthread::
 */
bool acqthread::init_data(void)
{
	myacq->in_mutex->lock();

	{ string msg_log("init_data: start"); myacq->info_log(msg_log); }

	try {
		int64_t img_width;
		int64_t img_height;
		if (myacq->cam_param[myacq->cam_idx].hw_roi == false) {
			img_width = myCamera->SensorWidth()/myacq->cam_param[myacq->cam_idx].binning;
			img_height = myCamera->SensorHeight()/myacq->cam_param[myacq->cam_idx].binning;
		}
		else {
			img_width = myacq->cam_param[myacq->cam_idx].hw_roi_param[2]-myacq->cam_param[myacq->cam_idx].hw_roi_param[0]+1;
			img_height = myacq->cam_param[myacq->cam_idx].hw_roi_param[3]-myacq->cam_param[myacq->cam_idx].hw_roi_param[1]+1;
		}
		/* vector used for coefficient permutation for image rotation */
		//myacq->cam_param[myacq->cam_idx].img_permutation = new long(img_height*img_width);

		myacq->cam_param[myacq->cam_idx].img_permutation = (int32_t *) malloc(sizeof(int32_t)*img_height*img_width);
		{ string msg_log("init_data: malloc img_permutation done"); myacq->info_log(msg_log); }
		for (int32_t i = 0; i < (img_height*img_width); i++) {
			myacq->cam_param[myacq->cam_idx].img_permutation[i] = i;
		}

		/* initialize feedback counters and flags */
		myacq->cam_param[myacq->cam_idx].exposure_auto_cnt = 0;
		myacq->cam_param[myacq->cam_idx].gain_auto_cnt = 0;
		myacq->cam_param[myacq->cam_idx].gain_auto_once = false;
		myacq->cam_param[myacq->cam_idx].exposure_auto_once = false;

		/* optimize grid sizes */
		if ((myacq->cam_param[myacq->cam_idx].gridsize != 0) && (img_width != 0) &&
			(img_width >= myacq->cam_param[myacq->cam_idx].gridsize)) {
			hor_grid_size = (int)((double)img_width/(int)(img_width/myacq->cam_param[myacq->cam_idx].gridsize));
		}
		else {
			hor_grid_size = myacq->cam_param[myacq->cam_idx].gridsize;
		}
		/* set vertical grid */
		if ((myacq->cam_param[myacq->cam_idx].gridsize != 0) && (img_height != 0) &&
			(img_height >= myacq->cam_param[myacq->cam_idx].gridsize)) {
			ver_grid_size = (int)((double)img_height/(int)(img_width/myacq->cam_param[myacq->cam_idx].gridsize));
		}
		else {
			ver_grid_size = myacq->cam_param[myacq->cam_idx].gridsize;
		}

		{ string msg_log("init_data: optimizing grid sizes done"); myacq->info_log(msg_log); }

		memset(myacq->attr_ImageSum_read, 0, img_height * img_width * sizeof(Tango::DevLong));
		myacq->imagesum_counter = 0;

		if (myacq->cam_param[myacq->cam_idx].img_depth == 8) {
			/* initialize the image structure */
			for (int32_t i = 0; i < myacq->imageBufferSize; i++) {

				ImageProc <unsigned char> *tmp = new ImageProc <unsigned char>(img_height,img_width,(int32_t)myacq->fitNumCores);
				if (!tmp)
					return false;

				tmp->set_lp_order((int32_t) myacq->cam_param[myacq->cam_idx].lp_order);
				tmp->set_hor_grid((int32_t) hor_grid_size);
				tmp->set_ver_grid((int32_t) ver_grid_size);
				tmp->binning = myacq->cam_param[myacq->cam_idx].binning;
				tmp->set_enable_auto_roi(myacq->autoRoi);
				tmp->set_enable_process(myacq->cam_param[myacq->cam_idx].process_enable);
				tmp->set_mirror(myacq->cam_param[myacq->cam_idx].mirror);
				
				tmp-> set_enable_fast_calc(myacq->fastCalculation);
				tmp->set_plot_mode(myacq->plotMode[0]);
				tmp->set_roi_threshold(myacq->autoRoiParam[2]);
				myacq->img8.push_back(tmp);

				/* set calibration and mechanical offsets */
				tmp->hor_calibration = myacq->cam_param[myacq->cam_idx].hor_calibration;
				tmp->ver_calibration = myacq->cam_param[myacq->cam_idx].ver_calibration;
				tmp->hor_pos_offset = myacq->cam_param[myacq->cam_idx].hor_pos_offset * myacq->cam_param[myacq->cam_idx].hor_sign;
				//tmp->ver_pos_offset = myacq->cam_param[myacq->cam_idx].ver_pos_offset * myacq->cam_param[myacq->cam_idx].ver_sign;
				tmp->ver_pos_offset = myacq->cam_param[myacq->cam_idx].ver_pos_offset;

				tmp->hor_sign = myacq->cam_param[myacq->cam_idx].hor_sign;
				tmp->ver_sign = myacq->cam_param[myacq->cam_idx].ver_sign;

				tmp->ref_hor_pos_offset = myacq->cam_param[myacq->cam_idx].ref_hor_pos_offset + myacq->cam_param[myacq->cam_idx].delta_ref_hor_pos_offset;
				tmp->ref_ver_pos_offset = myacq->cam_param[myacq->cam_idx].ref_ver_pos_offset + myacq->cam_param[myacq->cam_idx].delta_ref_ver_pos_offset;

				/* hw roi change image offset */
				if (myacq->cam_param[myacq->cam_idx].hw_roi == true) {
					if (myacq->cam_param[myacq->cam_idx].mirror == false) {
						tmp->img_hor_pos_offset = tmp->img_hor_pos_offset + (myCamera->SensorWidth()/2-
							tmp->img_hor_pos_offset - myacq->cam_param[myacq->cam_idx].hw_roi_param[0]);
					}
					else {
						tmp->img_hor_pos_offset = tmp->img_hor_pos_offset + (myCamera->SensorWidth()/2-
							tmp->img_hor_pos_offset - myacq->cam_param[myacq->cam_idx].hw_roi_param[0]);
					}
					tmp->img_ver_pos_offset = tmp->img_ver_pos_offset + (myCamera->SensorHeight()/2-
						tmp->img_ver_pos_offset-myacq->cam_param[myacq->cam_idx].hw_roi_param[1]);
				}
				/* binning change image offset */
				if (myacq->cam_param[myacq->cam_idx].binning > 1) {
					if (myacq->cam_param[myacq->cam_idx].mirror == false) {
						tmp->img_hor_pos_offset = tmp->img_hor_pos_offset + (myCamera->SensorWidth()/myacq->cam_param[myacq->cam_idx].binning-
							tmp->img_hor_pos_offset - myCamera->SensorWidth()/2/myacq->cam_param[myacq->cam_idx].binning);
					}
					else {
						tmp->img_hor_pos_offset = tmp->img_hor_pos_offset + (myCamera->SensorWidth()/myacq->cam_param[myacq->cam_idx].binning-
							tmp->img_hor_pos_offset - myCamera->SensorWidth()/2/myacq->cam_param[myacq->cam_idx].binning);
					}
					tmp->img_ver_pos_offset = tmp->img_ver_pos_offset + (myCamera->SensorHeight()/myacq->cam_param[myacq->cam_idx].binning-
						tmp->img_ver_pos_offset-myCamera->SensorHeight()/2/myacq->cam_param[myacq->cam_idx].binning);
				}

			}

			/* initialize permutation vector */
			if (myacq->cam_param[myacq->cam_idx].mirror)
				myacq->img8[0]->permutation_mirrorh(myacq->cam_param[myacq->cam_idx].img_permutation);
			if (myacq->cam_param[myacq->cam_idx].rotation != 0)
				myacq->img8[0]->permutation_rotation(myacq->cam_param[myacq->cam_idx].rotation, myacq->cam_param[myacq->cam_idx].img_permutation);

			/* background subtraction support variable */
			myacq->cam_param[myacq->cam_idx].bckimg8 = 
				new ImageProc <unsigned char> (img_height,img_width,(int32_t)myacq->fitNumCores);

			myacq->cam_param[myacq->cam_idx].bckimg8->reset();
	
		} /* end image 8bit */
		else if (myacq->cam_param[myacq->cam_idx].img_depth == 16) {
			/* initialize the image structure */
			for (int32_t i = 0; i < myacq->imageBufferSize; i++) {

				ImageProc <unsigned short> *tmp = new ImageProc <unsigned short>(img_height,img_width,(int32_t)myacq->fitNumCores);
				if (!tmp)
					return false;

				tmp->set_lp_order((int32_t)myacq->autoRoiParam[1]);
				tmp->set_hor_grid(hor_grid_size);
				tmp->set_ver_grid(ver_grid_size);
				tmp->binning = myacq->cam_param[myacq->cam_idx].binning;
				tmp->set_enable_auto_roi(myacq->autoRoi);
				tmp->set_enable_process(myacq->cam_param[myacq->cam_idx].process_enable);
				tmp->set_mirror(myacq->cam_param[myacq->cam_idx].mirror);

				tmp-> set_enable_fast_calc(myacq->fastCalculation);
				tmp->set_plot_mode(myacq->plotMode[0]);
				tmp->set_roi_threshold(myacq->autoRoiParam[2]);
				myacq->img16.push_back(tmp);
				/* set calibration and mechanical offsets */
				tmp->hor_calibration = myacq->cam_param[myacq->cam_idx].hor_calibration;
				tmp->ver_calibration = myacq->cam_param[myacq->cam_idx].ver_calibration;
				tmp->hor_pos_offset = myacq->cam_param[myacq->cam_idx].hor_pos_offset * myacq->cam_param[myacq->cam_idx].hor_sign;
				tmp->ver_pos_offset = myacq->cam_param[myacq->cam_idx].ver_pos_offset * myacq->cam_param[myacq->cam_idx].ver_sign;
				tmp->hor_sign = myacq->cam_param[myacq->cam_idx].hor_sign;
				tmp->ver_sign = myacq->cam_param[myacq->cam_idx].ver_sign;				

				tmp->ref_hor_pos_offset = myacq->cam_param[myacq->cam_idx].ref_hor_pos_offset + myacq->cam_param[myacq->cam_idx].delta_ref_hor_pos_offset;
				tmp->ref_ver_pos_offset = myacq->cam_param[myacq->cam_idx].ref_ver_pos_offset + myacq->cam_param[myacq->cam_idx].delta_ref_ver_pos_offset;


				/* hw roi changes image offset */
				if (myacq->cam_param[myacq->cam_idx].hw_roi == true) {
					if (myacq->cam_param[myacq->cam_idx].mirror == false) {
						tmp->img_hor_pos_offset = tmp->img_hor_pos_offset + (myCamera->SensorWidth()/2-
						tmp->img_hor_pos_offset - myacq->cam_param[myacq->cam_idx].hw_roi_param[0]);
					}
					else {
						tmp->img_hor_pos_offset = tmp->img_hor_pos_offset + (myCamera->SensorWidth()/2-
						tmp->img_hor_pos_offset - myacq->cam_param[myacq->cam_idx].hw_roi_param[0]);
					}
					tmp->img_ver_pos_offset = tmp->img_ver_pos_offset + (myCamera->SensorHeight()/2-
						tmp->img_ver_pos_offset-myacq->cam_param[myacq->cam_idx].hw_roi_param[1]);
				}
				/* binning changes image offset */
				if (myacq->cam_param[myacq->cam_idx].binning > 1) {
					if (myacq->cam_param[myacq->cam_idx].mirror == false) {
						tmp->img_hor_pos_offset = tmp->img_hor_pos_offset + (myCamera->SensorWidth()/myacq->cam_param[myacq->cam_idx].binning-
							tmp->img_hor_pos_offset - myCamera->SensorWidth()/2/myacq->cam_param[myacq->cam_idx].binning);
					}
					else {
						tmp->img_hor_pos_offset = tmp->img_hor_pos_offset + (myCamera->SensorWidth()/myacq->cam_param[myacq->cam_idx].binning-
							tmp->img_hor_pos_offset - myCamera->SensorWidth()/2/myacq->cam_param[myacq->cam_idx].binning);
					}
					tmp->img_ver_pos_offset = tmp->img_ver_pos_offset + (myCamera->SensorHeight()/myacq->cam_param[myacq->cam_idx].binning-
						tmp->img_ver_pos_offset-myCamera->SensorHeight()/2/myacq->cam_param[myacq->cam_idx].binning);
				}

			}

			/* initialize permutation vector */
			if (myacq->cam_param[myacq->cam_idx].mirror)
				myacq->img16[0]->permutation_mirrorh(myacq->cam_param[myacq->cam_idx].img_permutation);
			if (myacq->cam_param[myacq->cam_idx].rotation != 0)
				myacq->img16[0]->permutation_rotation(myacq->cam_param[myacq->cam_idx].rotation, myacq->cam_param[myacq->cam_idx].img_permutation);

			/* support variable for background subtraction */
			myacq->cam_param[myacq->cam_idx].bckimg16 = 
				new ImageProc <unsigned short> (img_height,img_width,(int32_t)myacq->fitNumCores);

			myacq->cam_param[myacq->cam_idx].bckimg16->reset();

		}

		{ string msg_log("init_data: image buffer allocated"); myacq->info_log(msg_log); }

	}
	catch (...) {
		cout << "acqthread::init_data(): exception" << endl;
	}

	myacq->in_mutex->unlock();

	return true;

}


void acqthread::delete_data(void)
{
	bool img8_flag = false,
		img16_flag = false;

	myacq->in_mutex->lock();

	try {
		if ( !myacq->img8.empty() ) {
			img8_flag = true;
			delete myacq->cam_param[myacq->cam_idx].bckimg8;
		}

		if( !myacq->img16.empty() ) {
			img16_flag = true;
			delete myacq->cam_param[myacq->cam_idx].bckimg16;
		}

		if (myacq->cam_param[myacq->cam_idx].img_permutation) {
			free((void *)myacq->cam_param[myacq->cam_idx].img_permutation);
			myacq->cam_param[myacq->cam_idx].img_permutation = 0;
		}

		/* keep permutation index and background image when disconnected */
		while( !myacq->img8.empty() ) {
			delete myacq->img8.back();
			myacq->img8.pop_back();
		}
		if (img8_flag)
			myacq->img8.clear();

		while( !myacq->img16.empty() ) {
			delete myacq->img16.back();
			myacq->img16.pop_back();
		}
		if (img16_flag)
			myacq->img16.clear();

	}
	catch (...) {
		cout << "acqthread::delete_data(): exception" << endl;
	}
	
	myacq->in_mutex->unlock();

}

// get black level value
bool acqthread::CameraGetBlackLevel(double *black_level)
{

	struct timeval now;
	gettimeofday(&now,NULL);
	if ((abs(now.tv_sec - camera_get_blacklevel.tv_sec) > 3) ||
		(((camera_get_counter % GIGECAM_ACQTHREAD_NUM_RX) == GIGECAM_ACQTHREAD_BLACKLEVEL_IDX) &&
		(abs(now.tv_sec - camera_get_blacklevel.tv_sec) > 2))) {
		gettimeofday(&camera_get_blacklevel, NULL);
		try {
			*black_level = (double) myCamera->BlackLevelRaw.GetValue();
		}
		catch (GenICam::GenericException &e) {
			cout << "CameraGetBlackLevel exception: " << e.GetDescription() << endl;
			return false;
		}
		catch (...) {
			return false;
		}
	}
	return true;

}

// set exposure value	
bool acqthread::CameraSetBlackLevel(double black_level)
{	

	try {
		myCamera->BlackLevelSelector.SetValue(BlackLevelSelector_All);
		//myCamera->ExposureTimeRaw.SetValue(exposure);
		myCamera->BlackLevelRaw.SetValue((int32_t)black_level);
	}
	catch (GenICam::GenericException &e) {
		cout << "CameraSetBlackLevel exception:" <<  e.GetDescription() << ", (" << black_level << ")" << endl;
		return false;
	}
	catch (...) {
		return false;
	}
	myacq->cam_param[myacq->cam_idx].black_level = (int32_t)black_level;
	return true;

}


// get exposure value
bool acqthread::CameraGetExposure(int32_t *exposure)
{

	struct timeval now;
	gettimeofday(&now,NULL);
	if ((abs(now.tv_sec - camera_get_exposure.tv_sec) > 3) ||
		(((camera_get_counter % GIGECAM_ACQTHREAD_NUM_RX) == GIGECAM_ACQTHREAD_EXPOSURE_IDX) &&
		(abs(now.tv_sec - camera_get_exposure.tv_sec) > 2))) {
		gettimeofday(&camera_get_exposure, NULL);
		try {
			*exposure = (int32_t) myCamera->ExposureTimeAbs.GetValue()+1;
			if (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler avA1000-100gm", GIGECAM_DESC) == 0)
				*exposure -= 1;
			else if (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler avA2300-25gm", GIGECAM_DESC) == 0)
				*exposure -= 1;
			else if (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler avA1600-100gm", GIGECAM_DESC) == 0)
				*exposure -= 1;
			else if (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA2000-50gm", GIGECAM_DESC) == 0)
				*exposure -= 1;
			else if (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1600-60gm", GIGECAM_DESC) == 0)
				*exposure -= 1;
			else if (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA780-75gm", GIGECAM_DESC) == 0)
				*exposure -= 1;
			else if (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA640-90gm", GIGECAM_DESC) == 0)
				*exposure -= 1;
			else if (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1300-30gm", GIGECAM_DESC) == 0)
				*exposure -= 1;
			else if (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1300-60gm", GIGECAM_DESC) == 0)
				*exposure -= 1;
			else if (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1300-75gm", GIGECAM_DESC) == 0)
				*exposure -= 1;
			else if (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1440-73gm", GIGECAM_DESC) == 0)
				*exposure -= 1;
			else if (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA645-100gm", GIGECAM_DESC) == 0)
				*exposure -= 1;
			else if (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1920-50gm", GIGECAM_DESC) == 0)
				*exposure -= 1;
			else if (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler 2500-14gm", GIGECAM_DESC) == 0)
				*exposure -= 1;
			else if (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler 2040-35gm", GIGECAM_DESC) == 0)
				*exposure -= 1;
		}
		catch (GenICam::GenericException &e) {
			cout << "CameraGetExposure exception: " << e.GetDescription() << endl;
			return false;
		}
		catch (...) {
			return false;
		}
	}
	return true;

}


// set exposure value	
bool acqthread::CameraSetExposure(int32_t exposure)
{	
	int32_t timebase = (exposure / 4095);
	if (timebase < 20)
		timebase = 20;
	else
		timebase = timebase + (20- (timebase % 20));

	// Enable the global reset release mode
	try {
		if (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA2500-14gm", GIGECAM_DESC) == 0) {
			// default value; supported by pylon5
			//myCamera->SensorShutterMode.SetValue(SensorShutterMode_Global);
			// exposure must be multiple of 35
			exposure = (int) (exposure / 35);
			exposure *= 35;

			myCamera->ExposureMode.SetValue(ExposureMode_Timed);
			myCamera->ExposureTimeRaw.SetValue(exposure);
			myacq->cam_param[myacq->cam_idx].exposure = exposure;
			return true;
		}
	}
	catch (GenICam::GenericException &e) {
		cout << "CameraSetExposure exception:" << e.GetDescription() << endl;
		return false;
	}
	catch (...) {
		return false;
	}	


	try {
		myCamera->ExposureMode.SetValue(ExposureMode_Timed);
		/* this model does not support this api */
		if ((strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA640-90gm", GIGECAM_DESC) != 0) &&
		    (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA2040-35gm", GIGECAM_DESC) != 0) &&
		    (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA780-75gm", GIGECAM_DESC) != 0) &&
		    (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1300-30gm", GIGECAM_DESC) != 0) &&
		    (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1300-75gm", GIGECAM_DESC) != 0) &&
		    (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1440-73gm", GIGECAM_DESC) != 0) &&
		    (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1300-60gm", GIGECAM_DESC) != 0) &&
		    (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA2000-50gm", GIGECAM_DESC) != 0) &&
		    (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1600-60gm", GIGECAM_DESC) != 0) &&
		    (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA645-100gm", GIGECAM_DESC) != 0) &&
		    (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1920-50gm", GIGECAM_DESC) != 0) &&
		    (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler avA1000-100gm", GIGECAM_DESC) != 0) &&
		    (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler avA2300-25gm", GIGECAM_DESC) != 0) &&
		    (strncmp(myacq->cam_param[myacq->cam_idx].model,"Basler avA1600-50gm", GIGECAM_DESC) != 0)) {
			myCamera->ExposureTimeBaseAbs.SetValue(timebase);
		}
		myCamera->ExposureTimeAbs.SetValue(exposure);
	}
	catch (GenICam::GenericException &e) {
		cout << "CameraSetExposure exception:" << e.GetDescription() << endl;
		return false;
	}
	catch (...) {
		return false;
	}		


	myacq->cam_param[myacq->cam_idx].exposure = exposure;
	return true;

}

//  convert raw gain to dB
void acqthread::CameraInitGainParam(void)
{
	/* 8-bit image */
	if (myacq->cam_param[myacq->cam_idx].img_depth == 8) {
		if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA2500-14gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;  
			myacq->cam_param[myacq->cam_idx].max_db_gain = 26;  
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 63;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA2040-35gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;  
			myacq->cam_param[myacq->cam_idx].max_db_gain = 36;  
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 360;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA2000-50gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 1.02;  /* removed limits */
			myacq->cam_param[myacq->cam_idx].max_db_gain = 24;  
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 36;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 512;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1300-75gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;  /* removed limits */
			myacq->cam_param[myacq->cam_idx].max_db_gain = 12;  
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 155;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 618;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1440-73gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;  /* removed limits */
			myacq->cam_param[myacq->cam_idx].max_db_gain = 20;  
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 360;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA645-100gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;  /* removed limits */
			myacq->cam_param[myacq->cam_idx].max_db_gain = 36.7;  
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 1023;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1920-50gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;  
			myacq->cam_param[myacq->cam_idx].max_db_gain = 36; 
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 360;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA780-75gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;  /* orig 9 */
			myacq->cam_param[myacq->cam_idx].max_db_gain = 27.7;  /* orig 36.7 */
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 250;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 1023;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1300-30gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;  /* orig 10.8 */
			myacq->cam_param[myacq->cam_idx].max_db_gain = 19.7; /* orig 30.5 */
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 300;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 850;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1300-60gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;  /* orig 8.3 */
			myacq->cam_param[myacq->cam_idx].max_db_gain = 12; /* orig 30.5 */
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 95;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1600-60gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;  /* orig 8.3 */
			myacq->cam_param[myacq->cam_idx].max_db_gain = 12; /* orig 30.5 */
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 95;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA640-90gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0; /* orig 6.8 */
			myacq->cam_param[myacq->cam_idx].max_db_gain = 29.9; /* orig 36.7 */
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 190;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 1023;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler avA1000-100gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0; 
			myacq->cam_param[myacq->cam_idx].max_db_gain = 21.54; 
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 600;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler scA1400-17gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_db_gain = 31;
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 192;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 1023;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler avA1600-50gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_db_gain = 16.2;
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 600;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler avA2300-25gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_db_gain = 16.2;
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 600;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler scA1600-28gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_db_gain = 20.29;
			if (myacq->cam_param[myacq->cam_idx].binning != 1)
				myacq->cam_param[myacq->cam_idx].min_raw_gain = 220;
			else
				myacq->cam_param[myacq->cam_idx].min_raw_gain = 285;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 850;
		}
		/* scout acA640 */
		else if (myCamera->SensorWidth() == 659) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_db_gain = 28.3;
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 280;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 1023;
		}
		/* scout scA780 */
		else if (myCamera->SensorWidth() == 782) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_db_gain = 25.9;
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 350;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 1023;
		}
		/* piA1000 */
		else if (myCamera->SensorWidth() == 1004) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_db_gain = 17.95;
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 500;
		}		
		/* scout scA1300 */
		else if (myCamera->SensorWidth() == 1296) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_db_gain = 25.13;
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 300;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 850;
		}
		/* scoutA1392*/
		else if (myCamera->SensorWidth() == 1392) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_db_gain = 25.13;
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 150;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 850;
		}
		/* scoutA1600 */
		else if (myCamera->SensorWidth() == 1626) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_db_gain = 20.29;
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 285;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 850;
		}
	}
	/* 16-bit image */
	else {
		if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA2500-14gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;  
			myacq->cam_param[myacq->cam_idx].max_db_gain = 26;  
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 63;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA2040-35gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;  /* removed limits */
			myacq->cam_param[myacq->cam_idx].max_db_gain = 24;  
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 240;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA2000-50gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 1.02;  /* removed limits */
			myacq->cam_param[myacq->cam_idx].max_db_gain = 24;  
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 36;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 512;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1300-75gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;  /* removed limits */
			myacq->cam_param[myacq->cam_idx].max_db_gain = 12;  
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 155;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 618;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1440-73gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;  /* removed limits */
			myacq->cam_param[myacq->cam_idx].max_db_gain = 10;  
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 240;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA645-100gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;  /* removed limits */
			myacq->cam_param[myacq->cam_idx].max_db_gain = 14.4;  
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 400;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1920-50gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;  
			myacq->cam_param[myacq->cam_idx].max_db_gain = 24; 
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 240;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA780-75gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0; /* orig 9 */
			myacq->cam_param[myacq->cam_idx].max_db_gain = 5.4; /* orig 14.4 */
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 250;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 400;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1300-30gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;  /* orig 10.8 */
			myacq->cam_param[myacq->cam_idx].max_db_gain = 3.6; /* orig 14.4 */
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 300;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 400;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1300-60gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_db_gain = 12;
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 9;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1600-60gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_db_gain = 12;
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 9;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler ac640-90gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0; /* orig 6.8 */
			myacq->cam_param[myacq->cam_idx].max_db_gain = 7.6; /* orig 914.4*/
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 190;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 400;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler avA1000-100gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0; 
			myacq->cam_param[myacq->cam_idx].max_db_gain = 21.54; 
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 600;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler scA1400-17gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_db_gain = 12.8;
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 192;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 511;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler avA1600-50gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_db_gain = 16.2;
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 600;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler avA2300-25gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_db_gain = 16.2;
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 600;
		}
		else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler scA1600-28gm") == 0) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_db_gain = 4.13;
			if (myacq->cam_param[myacq->cam_idx].binning != 1)
				myacq->cam_param[myacq->cam_idx].min_raw_gain = 220;
			else
				myacq->cam_param[myacq->cam_idx].min_raw_gain = 285;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 400;
		}
		/* scout scA640 */
		else if (myCamera->SensorWidth() == 659) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_db_gain = 10.1;
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 350;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 511;
		}
		/* scout scA780 */
		else if (myCamera->SensorWidth() == 782) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_db_gain = 7.7;
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 350;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 511;
		}
		/* piA1000 */
		else if (myCamera->SensorWidth() == 1002) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_db_gain = 14.36;
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 400;
		}
		/* scout scA1300 */
		else if (myCamera->SensorWidth() == 1296) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_db_gain = 9.0;
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 300;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 400;
		}
		/* scout scA1392 */
		else if (myCamera->SensorWidth() == 1392) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_db_gain = 9.0;
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 150;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 400;
		}
		/* scoutA1600 */
		else if (myCamera->SensorWidth() == 1626) {
			myacq->cam_param[myacq->cam_idx].min_db_gain = 0;
			myacq->cam_param[myacq->cam_idx].max_db_gain = 4.13;
			myacq->cam_param[myacq->cam_idx].min_raw_gain = 285;
			myacq->cam_param[myacq->cam_idx].max_raw_gain = 400;
		}
	}

}

//  convert raw gain to dB
bool acqthread::CameraGainRaw2dB(int32_t raw_gain, double *gain)
{
	int32_t min_raw_gain = myacq->cam_param[myacq->cam_idx].min_raw_gain,
	 max_raw_gain = myacq->cam_param[myacq->cam_idx].max_raw_gain;
	double Gc;
	 
	if (raw_gain < min_raw_gain) 
		raw_gain = min_raw_gain;
	else if (raw_gain > max_raw_gain) 
		raw_gain = max_raw_gain;

	if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA2500-14gm") == 0) {
		*gain = 0.41 * (double)raw_gain;
	}
	else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA2040-35gm") == 0) {
		*gain = 0.1 * (double)raw_gain;
	}
	else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA2000-50gm") == 0) {
		*gain = 20 * log10((double)raw_gain / 128);
	}
	else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1300-75gm") == 0) {
		*gain = 20 * log10((double)raw_gain / 128);
	}
	else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1440-73gm") == 0) {
		*gain = 0.1 * raw_gain;
	}
	else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA780-75gm") == 0) {
		/* *gain = 0.0359 * raw_gain;*/ /* orig */
		Gc = 20*log10((double)(658+min_raw_gain)/(double)(658-min_raw_gain));
		if (raw_gain < 511)
			*gain = 20 * log10((double)(658+raw_gain)/(double)(658-raw_gain))-Gc;
		else
			*gain = 0.0354 * (double)raw_gain - Gc;
	}
	else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA640-90gm") == 0) {
		/* *gain = 0.0359 * raw_gain;*/ /* orig */
		Gc = 20*log10((double)(658+min_raw_gain)/(double)(658-min_raw_gain));
		if (raw_gain < 511)
			*gain = 20 * log10((double)(658+raw_gain)/(double)(658-raw_gain))-Gc;
		else
			*gain = 0.0354 * (double)raw_gain - Gc;
	}
	else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler ac1300-30gm") == 0) {
		*gain = 0.0359 * raw_gain;
	}
	else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA645-100gm") == 0) {
		*gain = 0.0359 * raw_gain;
	}
	else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1920-50gm") == 0) {
		*gain = 0.1 * raw_gain;
	}
	/* aA1300-60gm */
	else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1300-60gm") == 0) {
		if (raw_gain < 31) {
			*gain = 20 * log10(1+((double)raw_gain)*2/64);
		}
		else {
			*gain = 20 * log10(2 * (1+ ((double)raw_gain-32)/64));
		}
	}
	/* aA1600-60gm */
	else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1600-60gm") == 0) {
		if (raw_gain < 31) {
			*gain = 20 * log10(1+((double)raw_gain)*2/64);
		}
		else {
			*gain = 20 * log10(2 * (1+ ((double)raw_gain-32)/64));
		}
	}

	/* avA1000-gm */
	else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler avA1000-100gm") == 0) {
		*gain = 0.0359 * raw_gain;
	}
	/* avA1600-gm */
	else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler avA1600-50gm") == 0) {
		*gain = 0.0359 * raw_gain;
	}
	/* avA2300-gm */
	else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler avA2300-25gm") == 0) {
		*gain = 0.0359 * raw_gain;
	}
	/* scoutA1400*/
	else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler scA1400-17gm") == 0) {
		Gc = 20*log10((double)(658+min_raw_gain)/(double)(658-min_raw_gain));
		if (raw_gain < 511) {
			*gain = 20 * log10((double)(658+raw_gain)/(double)(658-raw_gain))-Gc;
		}
		else
			*gain = 0.0354 * (double)raw_gain - Gc;
	}
	else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler scA1600-28gm") == 0) {
		Gc = 0.0359 * min_raw_gain;
		*gain = 0.0359 * (double)raw_gain - Gc;
	}
	/* scout scA640 */
	else if (myacq->cam_param[myacq->cam_idx].max_width == 659) {
		Gc = 20*log10((double)(658+min_raw_gain)/(double)(658-min_raw_gain));
		if (raw_gain < 511)
			*gain = 20 * log10((double)(658+raw_gain)/(double)(658-raw_gain))-Gc;
		else
			*gain = 0.0354 * (double)raw_gain - Gc;
	}
	/* scout scA780 */
	else if (myacq->cam_param[myacq->cam_idx].max_width == 782) {
		Gc = 20*log10((double)(658+min_raw_gain)/(double)(658-min_raw_gain));
		if (raw_gain < 511)
			*gain = 20 * log10((double)(658+raw_gain)/(double)(658-raw_gain))-Gc;
		else
			*gain = 0.0354 * (double)raw_gain - Gc;
	}
	/* piA1000 */
	else if (myacq->cam_param[myacq->cam_idx].max_width == 1004) {
		*gain = 0.0359 * raw_gain;
	}

	/* scout scA1300 */
	else if (myacq->cam_param[myacq->cam_idx].max_width == 1296) {
		Gc = 20*log10((double)(658+min_raw_gain)/(double)(658-min_raw_gain));
		if (raw_gain < 511)
			*gain = 20 * log10((double)(658+raw_gain)/(double)(658-raw_gain))-Gc;
		else
			*gain = 0.0354 * (double)raw_gain - Gc;
	}
	/* scoutA1392*/
	else if (myacq->cam_param[myacq->cam_idx].max_width == 1392) {
		*gain = 0.0359 * (double)raw_gain - 5.385;
	}
	
	return true;
}	

//  convert dB to raw gain
bool acqthread::CameraGaindB2Raw(double gain, int32_t *raw_gain)
{
	double min_db_gain = myacq->cam_param[myacq->cam_idx].min_db_gain,
	 max_db_gain = myacq->cam_param[myacq->cam_idx].max_db_gain;
	int32_t half_raw, i,
		min_raw_gain = myacq->cam_param[myacq->cam_idx].min_raw_gain,
		max_raw_gain = myacq->cam_param[myacq->cam_idx].max_raw_gain;	 
	double tmp_gain, check_gain;
	
	if (gain < min_db_gain) 
		gain = min_db_gain;
	else if (gain > max_db_gain) 
		gain = max_db_gain;

	half_raw = (max_raw_gain - min_raw_gain)/2;
	*raw_gain = min_raw_gain + half_raw;

 	for (i = 0; i < 11; i++) {
		CameraGainRaw2dB(*raw_gain, &tmp_gain);
		half_raw = (half_raw >> 1);		
		if (tmp_gain < gain) {
			*raw_gain += half_raw;
		}
		else {
			*raw_gain -= half_raw;
		}
		if (half_raw == 0)
			break;	

	}	


	/* check gain */
	CameraGainRaw2dB(*raw_gain, &check_gain);
	if (check_gain != gain) {
		for (int i = 0; i <= 12; i++) {
			CameraGainRaw2dB(*raw_gain-7+i, &check_gain);
			if (check_gain == gain) {
				*raw_gain = *raw_gain-7+i;
				break;
			}
		}
	}

	if (*raw_gain < myacq->cam_param[myacq->cam_idx].min_raw_gain)
		*raw_gain = myacq->cam_param[myacq->cam_idx].min_raw_gain;
	else if (*raw_gain > myacq->cam_param[myacq->cam_idx].max_raw_gain)
		*raw_gain = myacq->cam_param[myacq->cam_idx].max_raw_gain;	

	return true;	
}

// get gain
bool acqthread::CameraGetGain(double *gain)
{
	int32_t raw_gain;


	struct timeval now;
	gettimeofday(&now,NULL);
	if ((abs(now.tv_sec - camera_get_gain.tv_sec) > 3) ||
		(((camera_get_counter % GIGECAM_ACQTHREAD_NUM_RX) == GIGECAM_ACQTHREAD_GAIN_IDX) &&
		(abs(now.tv_sec - camera_get_gain.tv_sec) > 2))) {
		gettimeofday(&camera_get_gain, NULL);
		try {
			raw_gain = myCamera->GainRaw.GetValue();
		} /* try */
		catch (GenICam::GenericException &e) {
			cout << "CameraGetGain exception: " << e.GetDescription() << endl;
			return false;
		}
		catch (...) {
			return false;
		}
		/* convert raw gain to dB gain */
    CameraGainRaw2dB(raw_gain, gain);

	}
	
	return true;
}

// set gain	
bool acqthread::CameraSetGain(double gain)
{
	int32_t cur_raw;
	
	CameraGaindB2Raw(gain, &cur_raw);

	// temporary solution
	if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1600-60gm") == 0) {
		myCamera->GainSelector.SetValue(GainSelector_DigitalAll);
	}
	//else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1300-75gm") == 0) {
	//	myCamera->GainSelector.SetValue(GainSelector_DigitalAll);
	//}
	else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1300-60gm") == 0) {
		myCamera->GainSelector.SetValue(GainSelector_DigitalAll);
	}
	/* remove limits on gain */
	else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA645-100gm") == 0) {
		myCamera->ParameterSelector.SetValue(ParameterSelector_Gain);
		myCamera->RemoveLimits.SetValue(true);
	}

	try {
		myCamera->GainRaw.SetValue(cur_raw);
	}
	catch (GenICam::GenericException &e) {
		cout << "CameraSetGain exception: " << e.GetDescription() << endl;
		return false;
	}
	catch (...) {
		return false;
	}

	myacq->cam_param[myacq->cam_idx].gain = gain;
	return true;

}

// get trigger mode
bool acqthread::CameraGetImageDepth(short *imagedepth)
{

	struct timeval now;
	gettimeofday(&now,NULL);
	if ((abs(now.tv_sec - camera_get_imagedepth.tv_sec) > 3) ||
		(((camera_get_counter % GIGECAM_ACQTHREAD_NUM_RX) == GIGECAM_ACQTHREAD_IMAGEDEPTH_IDX) &&
		(abs(now.tv_sec - camera_get_imagedepth.tv_sec) > 2))) {
		gettimeofday(&camera_get_imagedepth, NULL);
		try {
			if (myCamera->PixelFormat.GetValue() == PixelFormat_Mono8) {
				*imagedepth = 8;
			}
			else if (myCamera->PixelFormat.GetValue() == PixelFormat_Mono16) {
				*imagedepth = 16;
			}
			else if (myCamera->PixelFormat.GetValue() == PixelFormat_Mono12) {
				*imagedepth = 16;
			}
			else if (myCamera->PixelFormat.GetValue() == PixelFormat_Mono10) {
				*imagedepth = 16;
			}
			else {
				return false;
			}
		}
		catch (GenICam::GenericException &e) {
			cout << "CameraGetImageDepth exception:" <<  e.GetDescription() << endl;
			return false;
		}
		catch (...) {
			return false;
		}
	}

	return true;

}

// get trigger mode
bool acqthread::CameraGetTrigger(bool *trigger)
{
	struct timeval now;
	gettimeofday(&now,NULL);
	if ((abs(now.tv_sec - camera_get_trigger.tv_sec) > 3) ||
		(((camera_get_counter % GIGECAM_ACQTHREAD_NUM_RX) == GIGECAM_ACQTHREAD_TRIGGER_IDX) &&
		(abs(now.tv_sec - camera_get_trigger.tv_sec) > 2))) {
		try {
			*trigger = (bool) myCamera->TriggerMode.GetValue();
		}
		catch (GenICam::GenericException &e) {
			cout << "CameraGetTrigger exception:" << e.GetDescription() << endl;
			return false;
		}
		catch (...) {
			return false;
		}
		gettimeofday(&camera_get_trigger, NULL);
	}

	return true;

}

// get trigger mode
bool acqthread::CameraSetTrigger(bool trigger)
{

	try {
		if (trigger) {
			myCamera->TriggerMode.SetValue(TriggerMode_On);
			myCamera->TriggerSource.SetValue(TriggerSource_Line1);
			myCamera->TriggerActivation.SetValue(TriggerActivation_RisingEdge);
		}
		else {
			myCamera->TriggerMode.SetValue(TriggerMode_Off);
			myCamera->AcquisitionMode.SetValue(AcquisitionMode_Continuous);
		}
	}
	catch (GenICam::GenericException &e) {
		cout << "CameraSetTrigger exception:" <<  e.GetDescription() << ", (" << trigger << ")" << endl;
		return false;
	}
	catch (...) {
		return false;
	}

	return true; 
}

// manage roi
void acqthread::CameraSetRoi(void)
{
	/* check horizontal top left */
	if (myacq->cam_param[myacq->cam_idx].hw_roi_param[0] < 0)
		myacq->cam_param[myacq->cam_idx].hw_roi_param[0] = 0;
	if (myacq->cam_param[myacq->cam_idx].hw_roi_param[0] >= myCamera->SensorWidth())
		myacq->cam_param[myacq->cam_idx].hw_roi_param[0] = myCamera->SensorWidth()-1;

	/* check vertical top left*/
	if (myacq->cam_param[myacq->cam_idx].hw_roi_param[1] < 0)
		myacq->cam_param[myacq->cam_idx].hw_roi_param[1] = 0;
	if (myacq->cam_param[myacq->cam_idx].hw_roi_param[1] >= myCamera->SensorHeight())
		myacq->cam_param[myacq->cam_idx].hw_roi_param[1] = myCamera->SensorHeight()-1;

	/* check horizontal bottom right */
	if (myacq->cam_param[myacq->cam_idx].hw_roi_param[2] < myacq->cam_param[myacq->cam_idx].hw_roi_param[0])
		myacq->cam_param[myacq->cam_idx].hw_roi_param[2] = myacq->cam_param[myacq->cam_idx].hw_roi_param[0];
	if (myacq->cam_param[myacq->cam_idx].hw_roi_param[2] >= myCamera->SensorWidth())
		myacq->cam_param[myacq->cam_idx].hw_roi_param[2] = myCamera->SensorWidth()-1;

	/* check vertical bottom right */
	if (myacq->cam_param[myacq->cam_idx].hw_roi_param[3] < myacq->cam_param[myacq->cam_idx].hw_roi_param[1])
		myacq->cam_param[myacq->cam_idx].hw_roi_param[3] = myacq->cam_param[myacq->cam_idx].hw_roi_param[1];
	if (myacq->cam_param[myacq->cam_idx].hw_roi_param[3] >= myCamera->SensorHeight())
		myacq->cam_param[myacq->cam_idx].hw_roi_param[3] = myCamera->SensorHeight()-1;

	/* set binning */
	myCamera->BinningHorizontal.SetValue(myacq->cam_param[myacq->cam_idx].binning);
	myCamera->BinningVertical.SetValue(myacq->cam_param[myacq->cam_idx].binning);

	int32_t hw_roi_param[4];
	for (int i = 0; i < 4; i++)
		hw_roi_param[i] = myacq->cam_param[myacq->cam_idx].hw_roi_param[i];

	// effettua la rotazione della ROI
	if (myacq->cam_param[myacq->cam_idx].rotation == 180) {
 		hw_roi_param[0] = myCamera->SensorWidth() - myacq->cam_param[myacq->cam_idx].hw_roi_param[2];
 		hw_roi_param[1] = myCamera->SensorHeight() - myacq->cam_param[myacq->cam_idx].hw_roi_param[3];
 		hw_roi_param[2] = myCamera->SensorWidth() - myacq->cam_param[myacq->cam_idx].hw_roi_param[0];
 		hw_roi_param[3] = myCamera->SensorHeight() - myacq->cam_param[myacq->cam_idx].hw_roi_param[1];
		// check the ROI Limits	
		myacq->set_roi_param(hw_roi_param); // these values set the maual_roi_param
		// copy again into temporary variable
		for (int i = 0; i < 4; i++) {
			hw_roi_param[i] = myacq->cam_param[myacq->cam_idx].manual_roi_param[i];
			myacq->cam_param[myacq->cam_idx].hw_roi_param[i] = hw_roi_param[i];
		}

	}

	/* binning won over hw roi */
	if (myacq->cam_param[myacq->cam_idx].binning > 1) {
		myacq->cam_param[myacq->cam_idx].hw_roi = false;
	}

	if (myacq->cam_param[myacq->cam_idx].hw_roi == false) {

		myCamera->OffsetX.SetValue(0);
		myCamera->OffsetY.SetValue(0);
		myCamera->Width.SetValue(myCamera->SensorWidth()/myacq->cam_param[myacq->cam_idx].binning);
		myCamera->Height.SetValue(myCamera->SensorHeight()/myacq->cam_param[myacq->cam_idx].binning);

	}
	else {
		int32_t roi_width = hw_roi_param[2]-hw_roi_param[0]+1;
		int32_t roi_height = hw_roi_param[3]-hw_roi_param[1]+1;

		/* set area dimension */
		myacq->cam_param[myacq->cam_idx].binning = 1;
		myCamera->OffsetX.SetValue(0);
		myCamera->OffsetY.SetValue(0);
		myCamera->Width.SetValue(roi_width);
		myCamera->Height.SetValue(roi_height);
		/* set the corner on top/left */
		if (myacq->cam_param[myacq->cam_idx].mirror == false) {
			myCamera->OffsetX.SetValue(hw_roi_param[0]);
			myCamera->OffsetY.SetValue(hw_roi_param[1]);
		}
		else {
			int32_t offset_x = myCamera->SensorWidth()-(hw_roi_param[0]+roi_width);
				/* avoid overflow */
			if ((roi_width+offset_x) > myCamera->SensorWidth()) {
				offset_x = myCamera->SensorWidth() - roi_width;
			}
			offset_x &= ~0x1; // tested for acA1440-73gm
			myCamera->OffsetX.SetValue(offset_x); /* hw roi mirroring */
			myCamera->OffsetY.SetValue(hw_roi_param[1]);

		}
		
	}

}

// set acquisition mode
void acqthread::CameraSetAcqMode(void)
{
	
	try {
		myCamera->TriggerSelector.SetValue(TriggerSelector_AcquisitionStart);
		if (myacq->cam_param[myacq->cam_idx].trigger) {
			myCamera->TriggerMode.SetValue(TriggerMode_On);
			myCamera->TriggerSource.SetValue(TriggerSource_Line1);
			myCamera->TriggerActivation.SetValue(TriggerActivation_RisingEdge);
		}
		else {
			myCamera->TriggerMode.SetValue(TriggerMode_Off);
			myCamera->AcquisitionMode.SetValue(AcquisitionMode_Continuous);
		}
		myCamera->ExposureMode.SetValue(ExposureMode_Timed);
	}
	catch (GenICam::GenericException &e) {
		cout << "CameraSetAcqMode exception:" <<  e.GetDescription() << endl;
		return;
	}
	catch (...) {
		return;
	}

}

// set acquisition mode
void acqthread::CameraInit(void)
{
	if (myCamera) {
		/* open camera communication */
		myCamera->Open(); open_flag = true;
		{ string msg_log("camera opened"); myacq->info_log(msg_log); }
		/* Set image format and AOI */
		if (myacq->cam_param[myacq->cam_idx].img_depth == 8)
			myCamera->PixelFormat.SetValue(PixelFormat_Mono8);
		else if (myacq->cam_param[myacq->cam_idx].img_depth == 16) {
			if ((strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA780-75gm") == 0) ||
				(strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA2040-35gm") == 0) ||
				(strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA2500-14gm") == 0) ||
				(strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA2000-50gm") == 0) ||
				(strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1600-60gm") == 0) ||
				(strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA645-100gm") == 0) ||
				(strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1920-50gm") == 0) ||
				(strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1300-60gm") == 0) ||
				(strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1440-73gm") == 0) ||
				(strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1300-30gm") == 0))
				myCamera->PixelFormat.SetValue(PixelFormat_Mono12);
			else if (strcmp(myacq->cam_param[myacq->cam_idx].model,"Basler acA1300-75gm") == 0)
				myCamera->PixelFormat.SetValue(PixelFormat_Mono10);
			else
				myCamera->PixelFormat.SetValue(PixelFormat_Mono16);
			{ string msg_log("set image format (8/16)"); myacq->info_log(msg_log); }
		}
		/* initialize gain parameters */
		CameraInitGainParam();		
		/* set roi hw */
		CameraSetRoi();
		{ string msg_log("camera roi initialization done"); myacq->info_log(msg_log); }
		/* Set the camera acquisition mode */
		CameraSetAcqMode();
		{ string msg_log("camera hardware parameters configured"); myacq->info_log(msg_log); }
		/* set packet size */
		myCamera->GevStreamChannelSelector.SetValue(GevStreamChannelSelector_StreamChannel0);
		myCamera->GevSCPSPacketSize.SetValue(myacq->packetSize);
		{ string msg_log("camera packet size configured"); myacq->info_log(msg_log); }
		/* set interpacket delay */
		if (myacq->interPacketDelay != -1) {
			myCamera->GevSCPD.SetValue(myacq->interPacketDelay);
			{ string msg_log("camera inter packet delay configured"); myacq->info_log(msg_log); }
		}
		/* set reserve bandwidth */
		if (myacq->reserveBandwidth != -1) {
			myCamera->GevSCBWR.SetValue(myacq->reserveBandwidth);
			{ string msg_log("camera reserve bandwidth configured"); myacq->info_log(msg_log); }
		}

		init_camera_flag = true;
	}
}

// set acquisition mode
void acqthread::StreamGrabberInit(void)
{
	/* allocate streamgrabber */
	StreamGrabber = NULL;
	StreamGrabber = new Camera_t::StreamGrabber_t(myCamera->GetStreamGrabber(0));
	if (StreamGrabber) {
 		StreamGrabber->Open();
		{ string msg_log("streamgrabber opened"); myacq->info_log(msg_log); }
		const size_t ImageSize = (size_t)(myCamera->PayloadSize.GetValue());
		// We won't use image buffers greater than ImageSize
		StreamGrabber->MaxBufferSize.SetValue(ImageSize);
		int32_t c_nBuffers;
		/* when trigger timeout disabled, ccd is normally in trigger train mode, so enlarge automatically buffers */
		if (*myacq->attr_DisableTriggerTimeout_read)
			c_nBuffers = 10;
		else
			c_nBuffers = myacq->streamGrabberBuffer;
		// We won't queue more than c_nBuffers image buffer at a time
		StreamGrabber->MaxNumBuffer.SetValue(c_nBuffers);
		// Allocate all resources for grabbing. Critical parameters like image
		// size now must not be changed until FinishGrab() is called.
		StreamGrabber->PrepareGrab();
		// Buffers used for grabbing must be registered at the stream grabber.
		// The registration returns a handle to be used for queuing the buffer
		for (int32_t i = 0; i < c_nBuffers; ++i) {
			CGrabBuffer *pGrabBuffer = new CGrabBuffer(ImageSize);
			pGrabBuffer->SetBufferHandle(StreamGrabber->RegisterBuffer(
			pGrabBuffer->GetBufferPointer(), ImageSize));
			// Put the grab buffer object into the buffer list
			BufferList.push_back(pGrabBuffer);
		}
		// Put buffer into the grab queue for grabbing
		for (std::vector<CGrabBuffer*>::const_iterator x = BufferList.begin(); x != BufferList.end(); ++x) {
			StreamGrabber->QueueBuffer((*x)->GetBufferHandle(), NULL);
		}
		streamgrabber_flag = true;
		{ string msg_log("camera streamgrabber allocated"); myacq->info_log(msg_log); }
	}
}

// delete myCamera
void acqthread::CameraDelete(void)
{	
	if (myCamera != NULL) {
		// Close camera
		if (open_flag == true) {
			myCamera->Close();
			{ string msg_log("switch off camera"); myacq->info_log(msg_log); }
		}
		// deleted mycamera object
		delete myCamera;
		{ string msg_log("camera deleted"); myacq->info_log(msg_log);}
	}
}

// delete myCamera
void acqthread::StreamGrabberDelete(void)
{
	if (StreamGrabber != NULL) {
		if (streamgrabber_flag == true) {
			StreamGrabber->CancelGrab();
			{ string msg_log("canceling streamgrabber"); myacq->info_log(msg_log); }
			// cancel grub buffer
			for (GrabResult r; StreamGrabber->RetrieveResult(r););
			{ string msg_log("retrieve all streamgrabber results"); myacq->info_log(msg_log); }
			for (std::vector<CGrabBuffer*>::iterator it = BufferList.begin(); it != BufferList.end(); it++) {
				StreamGrabber->DeregisterBuffer((*it)->GetBufferHandle());
				delete *it;
				*it = NULL;
			}
			{ string msg_log("deregister streamgrabber buffers"); myacq->info_log(msg_log);}
			// Free all resources used for grabbing
			StreamGrabber->FinishGrab();
			{ string msg_log("finishing streamgrabber"); myacq->info_log(msg_log);}
			// Close stream grabber
			StreamGrabber->Close();
			{ string msg_log("streamgrabber closed"); myacq->info_log(msg_log);}
		}
		delete StreamGrabber;
		{ string msg_log("streamgrabber deleted"); myacq->info_log(msg_log); }
	}
}

// automatic gain control feedback
bool acqthread::auto_gain_feedback()
{
	int32_t max_value = 0;
	int32_t err = 0;
	bool run_loop = false, deadband_flag = false;

	if ((myacq->cam_param[myacq->cam_idx].gain_auto == true) || 
		   myacq->cam_param[myacq->cam_idx].gain_auto_once)  {
		/* first get the maximum pixel value */
		if (myacq->cam_param[myacq->cam_idx].img_depth == 8)
			max_value = myacq->img8[myacq->img_cur]->max_value;
		else if (myacq->cam_param[myacq->cam_idx].img_depth == 16)
			max_value = myacq->img16[myacq->img_cur]->max_value;
		/* then calculate the error */
		err = myacq->cam_param[myacq->cam_idx].auto_target_value - max_value;
		/* if error over threshold, enable deadband flag */
		if (abs(err) > myacq->cam_param[myacq->cam_idx].auto_feedback_deadband) {
			deadband_flag = true;
		}
		/* if setpoint reached, disable out of deadband flag */
		if (abs(err) <= 1)
			deadband_flag = false;
		/* if out of deadband, run feedback */
		if (deadband_flag)
			run_loop = true;
		else
			run_loop = false;

		/* run once flag gain control (run once for max 30 shots) */
		if (myacq->cam_param[myacq->cam_idx].gain_auto_once) {
			if (myacq->cam_param[myacq->cam_idx].gain_once_cnt++ > 30) {
				myacq->cam_param[myacq->cam_idx].gain_auto_once = false;
				myacq->cam_param[myacq->cam_idx].gain_once_cnt = 0;
			}
			else {
				run_loop = true;
				/* trick to allow run once also when under threshold */
				if (max_value < myacq->cam_param[myacq->cam_idx].auto_feedback_target_thres)
					max_value = myacq->cam_param[myacq->cam_idx].auto_target_value/2+1;
			}
		}
		/* if signal under threshold, pause feedback */
		if (myacq->cam_param[myacq->cam_idx].auto_feedback_target_thres > max_value)
			run_loop = false;
		/* initialize minimum/maximum gain raw */
		if (myacq->cam_param[myacq->cam_idx].gain_auto_cnt == 0) {
			myacq->cam_param[myacq->cam_idx].gain_auto_coeff = 0;
		}
		/* downsampling counter */
		if ((myacq->cam_param[myacq->cam_idx].gain_auto_cnt % (myacq->cam_param[myacq->cam_idx].auto_feedback_dws+1)) != 0) 
			run_loop = false;
		/* run loop */
		if (run_loop) {
			
			int32_t gain_raw, new_gain_raw = 0, 
			gain_raw_d1 = 0, gain_raw_d2 = 0;
			double gain_d1, gain_d2, new_gain;

			try {
				gain_raw = myCamera->GainRaw.GetValue();
			}
			catch (GenICam::GenericException &e) {	
				return false;
			}
			catch (...) {
				return false;
			}
			
			int32_t delta = 2; /* kick to measure linear coeff gain_raw<->gain_db */
			/* if first loop cycle or error over a fixed threshold */ 
			if ((myacq->cam_param[myacq->cam_idx].gain_auto_coeff == 0) || 
				(abs(myacq->cam_param[myacq->cam_idx].auto_target_value-max_value) > 10)) {
				/* calculate conversion factor between gain_rw and gain_db */
				if ((gain_raw_d1 = (gain_raw+delta)) > myacq->cam_param[myacq->cam_idx].max_raw_gain)
					gain_raw_d1 = myacq->cam_param[myacq->cam_idx].max_raw_gain;				
				CameraGainRaw2dB(gain_raw_d1, &gain_d1);
				if ((gain_raw_d2 = (gain_raw-delta)) < myacq->cam_param[myacq->cam_idx].min_raw_gain)
					gain_raw_d2 = myacq->cam_param[myacq->cam_idx].min_raw_gain;	
				CameraGainRaw2dB(gain_raw_d2, &gain_d2);
				if ((gain_d1-gain_d2) != 0)
					myacq->cam_param[myacq->cam_idx].gain_auto_coeff = 
						(double)(gain_raw_d1-gain_raw_d2)/(gain_d1-gain_d2)*myacq->cam_param[myacq->cam_idx].auto_feedback_gain;
				else
					myacq->cam_param[myacq->cam_idx].gain_auto_coeff = 0;
			}

			/* feedback loop core */
			new_gain_raw = gain_raw;
			if ((max_value > 0) && (myacq->cam_param[myacq->cam_idx].auto_target_value > 0)) {
				if (err > 0) {
					new_gain_raw = gain_raw + (int)(myacq->cam_param[myacq->cam_idx].gain_auto_coeff*
						20*log10((double)myacq->cam_param[myacq->cam_idx].auto_target_value/(double)max_value));
				}
				else if (err < 0) {
					new_gain_raw = gain_raw - (int)(myacq->cam_param[myacq->cam_idx].gain_auto_coeff*
						20*log10((double)max_value/(double)myacq->cam_param[myacq->cam_idx].auto_target_value));
				}
			}
			/* if gain has to be changed */
			if (new_gain_raw != gain_raw) {
				/* set new gain */
				CameraGainRaw2dB(new_gain_raw, &new_gain);
				/* check gain feedback limits */
				if (myacq->cam_param[myacq->cam_idx].gain_auto_min < myacq->cam_param[myacq->cam_idx].min_db_gain)
					myacq->cam_param[myacq->cam_idx].gain_auto_min = myacq->cam_param[myacq->cam_idx].min_db_gain;
				if (myacq->cam_param[myacq->cam_idx].gain_auto_max > myacq->cam_param[myacq->cam_idx].max_db_gain)
					myacq->cam_param[myacq->cam_idx].gain_auto_max = myacq->cam_param[myacq->cam_idx].max_db_gain;
			  /* check gain */
				if (new_gain > myacq->cam_param[myacq->cam_idx].gain_auto_max) 
					new_gain = myacq->cam_param[myacq->cam_idx].gain_auto_max;
				else if (new_gain < myacq->cam_param[myacq->cam_idx].gain_auto_min) 
					new_gain = myacq->cam_param[myacq->cam_idx].gain_auto_min;
				if (CameraSetGain(new_gain) == true)
					*myacq->attr_Gain_read = new_gain;
			}
		} /* end if run_loop */
		/* increment loop counter */
		myacq->cam_param[myacq->cam_idx].gain_auto_cnt++;
	}

	return true;

}

// automatic exposure control feedback
bool acqthread::auto_exposure_feedback()
{
	int32_t max_value = 0;
	int32_t err = 0;
	bool run_loop = false, deadband_flag = false;

	if ((myacq->cam_param[myacq->cam_idx].exposure_auto == true) || 
		  myacq->cam_param[myacq->cam_idx].exposure_auto_once) {
		if (myacq->cam_param[myacq->cam_idx].img_depth == 8)
			max_value = myacq->img8[myacq->img_cur]->max_value;
		else if (myacq->cam_param[myacq->cam_idx].img_depth == 16) 
			max_value = myacq->img16[myacq->img_cur]->max_value;
		/* then calculate the error */
		err = myacq->cam_param[myacq->cam_idx].auto_target_value - max_value;
	/* if error over threshold, enable deadband flag */
		if (abs(err) > myacq->cam_param[myacq->cam_idx].auto_feedback_deadband) {
			deadband_flag = true;
		}
		/* if setpoint reached, disable out of deadband flag */
		if (abs(err) <= 1)
			deadband_flag = false;
		/* if out of deadband, run feedback */
		if (deadband_flag)
			run_loop = true;
		else
			run_loop = false;
		/* run once flag gain control (run once for max 30 shots) */
		if (myacq->cam_param[myacq->cam_idx].exposure_auto_once) {
			if (myacq->cam_param[myacq->cam_idx].exposure_once_cnt++ > 30) {
				myacq->cam_param[myacq->cam_idx].exposure_auto_once = false;
				myacq->cam_param[myacq->cam_idx].exposure_once_cnt = 0;
			}
			else {
				run_loop = true;
				/* trick to allow run once also when under threshold */
				if (max_value < myacq->cam_param[myacq->cam_idx].auto_feedback_target_thres)
					max_value = myacq->cam_param[myacq->cam_idx].auto_target_value/2+1;
			}
		}
		/* if signal under threshold, pause feedback */
		if (myacq->cam_param[myacq->cam_idx].auto_feedback_target_thres > max_value)
			run_loop = false;

		/* downsampling counter */
		if ((myacq->cam_param[myacq->cam_idx].exposure_auto_cnt % (myacq->cam_param[myacq->cam_idx].auto_feedback_dws+1)) != 0) 
			run_loop = false;

		/* run loop */
		if (run_loop) {
			int32_t new_exposure = myacq->cam_param[myacq->cam_idx].exposure + 
				(int32_t)((double)(myacq->cam_param[myacq->cam_idx].exposure*err)/(double)max_value*
				myacq->cam_param[myacq->cam_idx].auto_feedback_gain);

			if (new_exposure < myacq->cam_param[myacq->cam_idx].exposure_auto_min) 
				new_exposure = myacq->cam_param[myacq->cam_idx].exposure_auto_min;
			if (new_exposure > myacq->cam_param[myacq->cam_idx].exposure_auto_max) 
				new_exposure = myacq->cam_param[myacq->cam_idx].exposure_auto_max;
			if (CameraSetExposure(new_exposure) == true) {
				*myacq->attr_Exposure_read = new_exposure;
				myacq->attr_Exposure_write = *myacq->attr_Exposure_read;
			}
		} /* end if run_loop */
		/* increment loop counter */
		myacq->cam_param[myacq->cam_idx].exposure_auto_cnt++;
	}

	return true;

}

// stop streaming
void acqthread::CameraStop()
{
	myacq->run_flag = false;
}

/* 
 *	acqthread::run()
 *	Run  
 */		
void acqthread::run(void *) 
{
	int32_t cnt_err = 0, cnt_err_old = 0, cnt_frame = 0, old_frame = 65535, diff_frame,
	  bn_err = 0, bn_err_old = 0;
	struct timespec time_old, time_new, tim_bn_err, tim_cnt_err, now;
	DeviceInfoList_t lstDevices;
#ifndef CONFIG_64
	//Pylon::PylonAutoInitTerm autoInitTerm;	
#endif
	char tmp[GIGECAM_DESC],model[GIGECAM_DESC],ser_num[GIGECAM_DESC];
	int32_t curr_frame = 0;
	int32_t camera_idx = 0;
	int32_t wait_time_image = 0;
	int32_t old_bunch_number = 0;
	GrabResult Result;
	bool acq_flag = true,
			 ptl_flag = false,
	     grab_flag = false,
	     init_data_done_flag = false,
	     init_data_ok_flag = false,
	     init_buffers_done_flag = false,
	     init_buffers_ok_flag = false,
	     found_flag = false,
	     acquisition_flag = false,
	     avg_thres_ok = true,
	     sched_set_flag = false;
	struct sched_param mysched;
	CTlFactory *TlFactory;
	ITransportLayer *pTl;

	*myacq->attr_DebugThread_read = 1;
	gettimeofday(&myacq->thread_state_time,NULL);
	
	// init some flags
	myacq->link_flag = true;	
	init_camera_flag = false;
	open_flag = false;
	streamgrabber_flag = false;
	myCamera = NULL;
	StreamGrabber = NULL;
	myacq->frequency = 10;
	myacq->bunch_number = 0;
	clock_gettime(CLOCK_REALTIME, &tim_cnt_err);
	clock_gettime(CLOCK_REALTIME, &tim_bn_err);
	tim_bn_err.tv_sec -= GIGECAM_ERR_TIME_BARRIER-1;
	tim_cnt_err.tv_sec -= GIGECAM_ERR_TIME_BARRIER-1;

	{ string msg_log("Acquisition loop started"); myacq->info_log(msg_log);}

	*myacq->attr_DebugThread_read = 2; 

	// Get the transport layer factory
	try {
		TlFactory = &CTlFactory::GetInstance();
		string msg_log("Creating TlFactory"); myacq->info_log(msg_log);
		found_flag = true;
	}
	catch (GenICam::LogicalErrorException e) {
		TlFactory = NULL;
		found_flag = false;
		string msg_log("PylonLib logical exception (TlFactory):" + string(e.GetDescription()));
		myacq->info_log(msg_log);
	}
	catch (GenICam::RuntimeException e) {
		TlFactory = NULL;
		found_flag = false;
		string msg_log("PylonLib runtime exception (TlFactory):" + string(e.GetDescription()));
		myacq->info_log(msg_log);
	}
	catch (GenICam::GenericException &e) {
		TlFactory = NULL;
		found_flag = false;
		string msg_log("PylonLib generic exception (TlFactory):" + string(e.GetDescription()));
		myacq->info_log(msg_log);
	}
	catch (...) {
		TlFactory = NULL;
		found_flag = false;
		string msg_log("generic exception: failed to create TlFactory"); 
		myacq->info_log(msg_log);
	}

	*myacq->attr_DebugThread_read = 3;

	// Create the transport layer object needed to enumerate cameras
	if (found_flag) {
		try {
			pTl = NULL;
			pTl = TlFactory->CreateTl(CBaslerGigECamera::DeviceClass());
			ptl_flag = true;
			string msg_log("Creating pTl transport layer"); myacq->info_log(msg_log);
		}
		catch (GenICam::LogicalErrorException e) {
			pTl = NULL;
			found_flag = false;
			string msg_log("PylonLib logical exception (pTl transport):" + string(e.GetDescription()));
			myacq->info_log(msg_log);
		}
		catch (GenICam::RuntimeException e) {
			pTl = NULL;
			found_flag = false;
			string msg_log("PylonLib runtime exception (pTl transport):" + string(e.GetDescription()));
			myacq->info_log(msg_log);
		}
		catch (GenICam::GenericException &e) {
			pTl = NULL;
			found_flag = false;
			string msg_log("PylonLib generic exception (pTl transport):" + string(e.GetDescription()));
			myacq->info_log(msg_log);
		}
		catch (...) {
			pTl = NULL;
			found_flag = false;
			string msg_log("generic exception: failed to create pTl transport layer"); myacq->info_log(msg_log);
		}
	}

	*myacq->attr_DebugThread_read = 4;

	// check cameras on the network 
	if (found_flag) {
		try {
			pTl->EnumerateDevices(lstDevices);
			string msg_log("Looking for cameras"); myacq->info_log(msg_log);
		}
		catch (GenICam::LogicalErrorException e) {
			found_flag = false;
			string msg_log("PylonLib logical exception (EnumerateDevices):" + string(e.GetDescription()));
			myacq->info_log(msg_log);
		}
		catch (GenICam::RuntimeException e) {
			found_flag = false;
			string msg_log("PylonLib runtime exception (EnumerateDevices):" + string(e.GetDescription()));
			myacq->info_log(msg_log);
		}
		catch (GenICam::GenericException &e) {
			found_flag = false;
			string msg_log("PylonLib generic exception (pEnumerateDevices):" + string(e.GetDescription()));
			myacq->info_log(msg_log);
		}
		catch (...) {
			found_flag = false;
			string msg_log("generic exception: failed to enumerate devices");
			myacq->info_log(msg_log);
		}
	}

	*myacq->attr_DebugThread_read = 5;

  // Get all attached cameras and exit application if no camera is found
	if (found_flag) {
		if (0 == pTl->EnumerateDevices(lstDevices)) {
			myacq->set_status("No cameras found"); myacq->set_state(Tango::FAULT);
			string msg_log("No cameras found");myacq->info_log(msg_log);
			found_flag = false;
		}
		else {
			string msg_log("Some cameras found"); myacq->info_log(msg_log);
			found_flag = true;
		}
	}	

	*myacq->attr_DebugThread_read = 6;

	// The camera object is used to set and get all available camera features.
	if (found_flag) { 
		found_flag = false;
		DeviceInfoList_t::const_iterator it;	
		for ( it = lstDevices.begin(); it != lstDevices.end(); ++it ) {
			ScanName((char *)it->GetFullName().c_str(), model,  ser_num, tmp);
			if (strncmp(myacq->cam_param[myacq->cam_idx].ipaddress.c_str(),tmp,it->GetFullName().size()) == 0) {
				found_flag = true;
				strncpy(myacq->cam_param[myacq->cam_idx].model, model, GIGECAM_DESC);
				strncpy(*myacq->attr_Model_read, model, GIGECAM_DESC);
				cout << "Camera model: " << myacq->cam_param[myacq->cam_idx].model << endl;
				break;
			}
			camera_idx++;
		}
	}

	*myacq->attr_DebugThread_read = 7;

	// set device server state
	if (found_flag) {
		myacq->set_status("Camera found");myacq->set_state(Tango::INIT);
		string msg_log("Camera found " + string(tmp));myacq->info_log(msg_log);
	}
	else {
		myacq->set_status("Camera not found");myacq->set_state(Tango::FAULT);
		string msg_log("Camera not found");	myacq->info_log(msg_log);
	}


	// Found the camera with the correct ip address
	if (found_flag) {

		*myacq->attr_DebugThread_read = 8;

		try { /* logical exception, runtime exception, generic (...) */
			SetConsoleCtrlHandler(&CtrlCHandler);
			{string msg_log("SetConsoleCtrlHandler done");	myacq->info_log(msg_log);}
			*myacq->attr_DebugThread_read = 9;
			myCamera = NULL; 
			myCamera = new Camera_t(pTl->CreateDevice(lstDevices[camera_idx]));
			{string msg_log("myCamera created");	myacq->info_log(msg_log);}
			*myacq->attr_DebugThread_read = 10;
			CameraInit();
			*myacq->attr_DebugThread_read = 11;
			if (open_flag) {
				*myacq->attr_DebugThread_read = 12;
				/* initialize the image structure */
				init_data_ok_flag = init_data(); init_data_done_flag = true;
				//{ string msg_log("image buffers initialized"); myacq->info_log(msg_log); }
				*myacq->attr_DebugThread_read = 13;
				/* initialize realtime buffers */
				init_buffers_ok_flag = myacq->init_buffers(myacq); init_buffers_done_flag = true;
				{ string msg_log("acquisition buffers initialized"); myacq->info_log(msg_log); }
				*myacq->attr_DebugThread_read = 14;
				StreamGrabberInit();
				*myacq->attr_DebugThread_read = 15;
			}
			if (streamgrabber_flag && init_buffers_ok_flag && init_data_ok_flag) {
				// start image acquisition 
				*myacq->attr_DebugThread_read = 16;
				myCamera->GevStreamChannelSelector.SetValue(GevStreamChannelSelector_StreamChannel0);
				myCamera->AcquisitionStart.Execute();acquisition_flag = true;
				{ string msg_log("camera acquisition started"); myacq->info_log(msg_log); }
				*myacq->attr_DebugThread_read = 17;
			}
			else {
				myacq->run_flag = false;
			}

			acq_flag = myacq->run_flag;

			*myacq->attr_DebugThread_read = 18;
			
			gettimeofday(&myacq->thread_state_time,NULL);

			while (acq_flag) {
				int err = 0;
				acq_flag = myacq->run_flag;
				camera_get_counter++;
				*myacq->attr_DebugThread_read = 21;
				/* set scheduler inside the acquisition loop one time */
				if (sched_set_flag == false) {
					sched_getparam(0, &mysched);
					mysched.sched_priority = sched_get_priority_max(SCHED_RR)/2 - 1;
					if( (err=sched_setscheduler(0, SCHED_RR, &mysched )) < 0) {
						{ string msg_log("launch server as root to gain maximum priority!"); myacq->info_log(msg_log);}
						cout << err << endl;
					}
					// omniorb priority
					set_priority(PRIORITY_HIGH);
					sched_set_flag = true;
				}
				*myacq->attr_DebugThread_read = 22;

				if (myacq->pause_flag == true) {
					myacq->set_status("acquisition paused");
				}
				if (myacq->pause_flag == false) {
					/* wait one sec. more the exposure time */
					wait_time_image = (int32_t) ((double)myacq->attr_Exposure_write / 1000 + 1000);

					if (*myacq->attr_DisableTriggerTimeout_read)
						wait_time_image = GIGECAM_LOCK_TOUT*0.8*1000; /* trigger timeout (msec.) */

					*myacq->attr_DebugThread_read = 23;

					if (StreamGrabber->GetWaitObject().Wait(wait_time_image)) {
						grab_flag = false;
						*myacq->attr_DebugThread_read = 24;
						gettimeofday(&myacq->thread_state_time,NULL);
						StreamGrabber->RetrieveResult(Result);
						*myacq->attr_DebugThread_read = 25;
						curr_frame = Result.FrameNr();
						// get bunch number from rnm
#ifdef GIGECAM_RNM
						if (myacq->rnmEnabled) {
							char check_flag;
#ifdef CONFIG_64
							rnm_bunch_number_get_value(0, 1, &myacq->bunch_number, &check_flag);
#else
							rnm_bunch_number_get_value(0, 1, (long *) &myacq->bunch_number, &check_flag);
#endif
							/* vcheck bunch number lost */
							if ((old_bunch_number > 0) && ((myacq->bunch_number - old_bunch_number) != 1)) {
								bn_err++;
								myacq->error_counters[GIGECAM_ERR_BN_LOST]++;
							}
							old_bunch_number = myacq->bunch_number;

							
						}
						else {
							myacq->bunch_number = cnt_frame;
						}
#else
						myacq->bunch_number = cnt_frame;
#endif
						// end get bunch number from rnm

						*myacq->attr_DebugThread_read = 26;

						// check lost packets, tune fitting period timeout
						diff_frame = abs((old_frame % 0xfffe)-(curr_frame % 0xfffe));
						if (diff_frame > 1) {
							cnt_err++;
							myacq->error_counters[GIGECAM_ERR_CNT_LOST]++;
						} 
						clock_gettime(CLOCK_REALTIME, &now);

						old_frame = curr_frame; cnt_frame++;
						clock_gettime(CLOCK_REALTIME, &time_new);
						double curr_frequency = (double)diff_frame/(myacq->timespec2ms(myacq->difftime(time_old,time_new)));
						if ((curr_frequency > 0) && (curr_frequency < 700)) {
							myacq->frequency = curr_frequency*0.1+myacq->frequency*0.9; /* low pass filter */
						}
						time_old.tv_sec = time_new.tv_sec; time_old.tv_nsec = time_new.tv_nsec;
						// end check lost packets
						*myacq->attr_DebugThread_read = 27;
						if ((Grabbed == Result.Status())) {
							grab_flag = true;
							*myacq->attr_DebugThread_read = 28;
							width = Result.GetSizeX();
							*myacq->attr_DebugThread_read = 29;
							height = Result.GetSizeY();
							/* to minimizecomm hung due to cable disconnection, first set autogain, auto exposure */
							*myacq->attr_DebugThread_read = 30;
							auto_gain_feedback();
							*myacq->attr_DebugThread_read = 31;
							auto_exposure_feedback();
							*myacq->attr_DebugThread_read = 32;
							update_configuration();
							*myacq->attr_DebugThread_read = 33;
							avg_thres_ok = save_data(&Result);
							*myacq->attr_DebugThread_read = 34;
							StreamGrabber->QueueBuffer(Result.Handle(), NULL);
							*myacq->attr_DebugThread_read = 35;
							myacq->trig_err_consec_cnt = 0;
							myacq->comm_err_consec_cnt = 0;
						}
						else if (Canceled == Result.Status()) {
							cnt_err++;
							StreamGrabber->QueueBuffer(Result.Handle(), NULL);
							myacq->error_counters[GIGECAM_ERR_CNT_CANCEL]++;
						}
						else if (Failed == Result.Status()) {
							StreamGrabber->QueueBuffer(Result.Handle(), NULL);
						}
						else if (Idle == Result.Status()) {
							StreamGrabber->QueueBuffer(Result.Handle(), NULL);
						}
						else if (Queued == Result.Status()) {
							StreamGrabber->QueueBuffer(Result.Handle(), NULL);
						}
						else if (_UndefinedGrabStatus == Result.Status()) {
							StreamGrabber->QueueBuffer(Result.Handle(), NULL);
						}
						else {
							StreamGrabber->QueueBuffer(Result.Handle(), NULL);
						}
						*myacq->attr_DebugThread_read = 38;

						myacq->set_state(Tango::ON);
						if (avg_thres_ok == true) { 
							if ((myacq->timespec2ms(myacq->difftime(tim_bn_err,now)) > GIGECAM_ERR_TIME_BARRIER) && 
								(myacq->timespec2ms(myacq->difftime(tim_cnt_err,now)) > GIGECAM_ERR_TIME_BARRIER)) {
								myacq->set_status("acquisition running");
							}
							myacq->set_state(Tango::ON);
						}
						else {
							myacq->set_state(Tango::ALARM);
						}

					}
					/* data timeout */
					else if (myacq->get_state() != (Tango::FAULT)) {
						*myacq->attr_DebugThread_read = 39;
						myacq->set_state(Tango::OFF);
						if (myacq->cam_param[myacq->cam_idx].trigger) 
							myacq->set_status("acquisition failed (trigger?)");
						else
							myacq->set_status("acquisition failed, stream disabled");	
						string msg_log("camera connected,"); 
						char dbg_thread[20]; sprintf(dbg_thread,"trig_err(%d/%d)",(int)myacq->trig_err_consec_cnt,GIGECAM_TRIG_ERR_MAX_CNT);
						msg_log.append(dbg_thread); myacq->info_log(msg_log);

						*myacq->attr_DebugThread_read = 42;
						usleep(200000);
						if ((myacq->trig_err_consec_cnt++ > GIGECAM_TRIG_ERR_MAX_CNT) && (myacq->autoReconnect == false)) {
							myacq->run_flag = false;
							myacq->trig_err_consec_cnt = 0;
	 						myacq->set_state(Tango::OFF);
							myacq->set_status("acquisition stopped");
							string msg_log("Maximum number waiting trigger reached, acquisition stopped"); myacq->info_log(msg_log);
						}
					}

					gettimeofday(&myacq->thread_state_time,NULL);	
					*myacq->attr_DebugThread_read = 43;
					int err_api = 0;
					if (CameraGetBlackLevel(myacq->attr_BlackLevel_read) == false)
						err_api++;
					*myacq->attr_DebugThread_read = 44;
					if (CameraGetTrigger(myacq->attr_Trigger_read) == false)
						err_api++;
					*myacq->attr_DebugThread_read = 45;
					if (CameraGetGain(myacq->attr_Gain_read) == false)
						err_api++;
					*myacq->attr_DebugThread_read = 46;
					if (CameraGetImageDepth(myacq->attr_ImageDepth_read) == false)
						err_api++;
					*myacq->attr_DebugThread_read = 47;
#ifdef CONFIG_64
					if (CameraGetExposure(myacq->attr_Exposure_read) == false)
						err_api++;
#else
					if (CameraGetExposure((int32_t *)myacq->attr_Exposure_read) == false)
						err_api++;
#endif
					if (err_api) {
						found_flag = false; 
						myacq->set_state(Tango::FAULT);
						string msg_log("gigecam api error exit from loop");myacq->info_log(msg_log);
						break;
					}

				}
				else { /* if pause_flag */
					usleep(10000);
					gettimeofday(&myacq->thread_state_time,NULL);
				}

				/* notify frame lost error */
				stringstream err_msg_cnt, err_msg_bn;
				if (abs(cnt_err_old-cnt_err) > 0) 
					clock_gettime(CLOCK_REALTIME, &tim_cnt_err);
				if (myacq->timespec2ms(myacq->difftime(tim_cnt_err,now)) < GIGECAM_ERR_TIME_BARRIER)  
					err_msg_cnt << "missing frames error  ";

				/* notify bunch number lost error */
				if (abs(bn_err_old-bn_err) > 0) 
					clock_gettime(CLOCK_REALTIME, &tim_bn_err);
				if (myacq->timespec2ms(myacq->difftime(tim_bn_err,now)) < GIGECAM_ERR_TIME_BARRIER)  
					err_msg_bn << "bunch number error";

				/* store old error counters */
				cnt_err_old = cnt_err;bn_err_old = bn_err;
				clock_gettime(CLOCK_REALTIME, &now);

				/* make error message persistent */
				if (((myacq->timespec2ms(myacq->difftime(tim_cnt_err,now)) < GIGECAM_ERR_TIME_BARRIER) ||
						(myacq->timespec2ms(myacq->difftime(tim_bn_err,now)) < GIGECAM_ERR_TIME_BARRIER))
				    && (myacq->get_state() == (Tango::ON))) {
					stringstream err_msg;
					err_msg << err_msg_cnt.str() << err_msg_bn.str();
					myacq->set_status(err_msg.str());
				}

			} /* while runflag */

			// Stop acquisition
			if (acquisition_flag) {
				*myacq->attr_DebugThread_read = 48;
				myCamera->AcquisitionStop.Execute();
				{ string msg_log("acquisition stop"); myacq->info_log(msg_log); }
			}
		} /* try stream grabber open */
		catch (Tango::DevFailed &e) {
			found_flag = false;
			string msg_log("Tango exception:" + string(e.errors[0].desc));
			myacq->info_log(msg_log);myacq->error_counters[GIGECAM_ERR_CNT_CONN]++;
		}
		catch (GenICam::LogicalErrorException e) {
			found_flag = false;
			string msg_log("PylonLib logical exception:" + string(e.GetDescription()));
			myacq->info_log(msg_log);myacq->error_counters[GIGECAM_ERR_CNT_CONN]++;
		}
		catch (GenICam::RuntimeException e) {
			found_flag = false;
			string msg_log("PylonLib runtime exception:" + string(e.GetDescription()));
			myacq->info_log(msg_log);myacq->error_counters[GIGECAM_ERR_CNT_CONN]++;
		}
		catch (GenICam::GenericException &e) {
			found_flag = false;
			string msg_log("PylonLib generic exception:" + string(e.GetDescription()));
			myacq->info_log(msg_log);myacq->error_counters[GIGECAM_ERR_CNT_CONN]++;
			/* if exception happened inside exposure feedback, disable it */
			if ((myacq->cam_param[myacq->cam_idx].exposure_auto) && (myacq->cam_param[myacq->cam_idx].exposure_auto_cnt == 0)) {
				myacq->cam_param[myacq->cam_idx].exposure_auto = false;
				myacq->cam_param[myacq->cam_idx].exposure_auto_once = false;
			}
			/* if exception happened inside egain feedback, disable it */
			if ((myacq->cam_param[myacq->cam_idx].gain_auto) && (myacq->cam_param[myacq->cam_idx].gain_auto_cnt == 0)) {
				myacq->cam_param[myacq->cam_idx].gain_auto = false;
				myacq->cam_param[myacq->cam_idx].gain_auto_once = false;
			}
		}
		catch (...) {
			found_flag = false;
			{ string msg_log("generic exception"); myacq->info_log(msg_log);}
			myacq->error_counters[GIGECAM_ERR_CNT_CONN]++;
		}

		*myacq->attr_DebugThread_read = 49;

		/* set state */
		if (found_flag) {
			myacq->set_state(Tango::OFF); myacq->set_status("acquisition not running");
			usleep(500000); /* wait tango server calls to end (to avoid deleting data) */
			{ string msg_log("set state OFF"); myacq->info_log(msg_log);}
		}
		else {
			myacq->set_state(Tango::FAULT);myacq->set_status("camera error");
			usleep(500000); /* wait tango server calls to end (to avoid deleting data) */
			{ string msg_log("set state FAULT"); myacq->info_log(msg_log);}
		}

		*myacq->attr_DebugThread_read = 50;

		/* delete data structures */
		if (init_data_done_flag) {
			try {
				delete_data();
				init_data_done_flag = false;
				{ string msg_log("deleted data structures"); myacq->info_log(msg_log); }
			}
			catch (...) {
				string msg_log("generic error during data structures destruction");myacq->info_log(msg_log);
			}
		}

		*myacq->attr_DebugThread_read = 51;
		/* delete buffers */
		if (init_buffers_done_flag) {
			try {
				myacq->delete_buffers(myacq);
				{ string msg_log("deleted realtime buffers"); myacq->info_log(msg_log); }
			}
			catch (...) {
				string msg_log("generic error during buffer destruction");myacq->info_log(msg_log);
			}
		}
		*myacq->attr_DebugThread_read = 52;
		/* delete stream grabber */
		try {
			StreamGrabberDelete();
		}
		catch (GenICam::GenericException &e) {
			string msg_log("PylonLib runtime exception (stream grabber remove):" + string(e.GetDescription()));
			myacq->info_log(msg_log);
		}
		catch (...) {
			string msg_log("generic error during streamgrabber destruction"); myacq->info_log(msg_log);
		}
		*myacq->attr_DebugThread_read = 53;
		/* camera object destruction */
		try {
			CameraDelete();
		}
		catch (...) {
			string msg_log("generic error during mycamera destruction");myacq->info_log(msg_log);
		}

	} /* if (found_flag) */ 

	*myacq->attr_DebugThread_read = 54;

	/* check again if data structures were deleted */
	if (init_data_done_flag) {
		try {
			delete_data();
			init_data_done_flag = false;
			{ string msg_log("deleted data structures (2)"); myacq->info_log(msg_log); }
		}
		catch (...) {
			string msg_log("generic error during data structures destruction (2)");myacq->info_log(msg_log);
		}
	}

	if ((acq_flag == false) && (found_flag == true)) {
		myacq->set_state(Tango::OFF); myacq->set_status("acquisition stopped");
		string msg_log("acquisition stopped"); myacq->info_log(msg_log);
	}

	if ((found_flag == false) || (init_camera_flag == false)) {
		myacq->set_state(Tango::FAULT); myacq->set_status("communication error");
		{ string msg_log("communication error"); myacq->info_log(msg_log); }
	}

	if ((pTl != NULL) && (ptl_flag)) {
		TlFactory->ReleaseTl( pTl );
		*myacq->attr_DebugThread_read = 55;
		{ string msg_log("release camera communication layer"); myacq->info_log(msg_log); }
	}

	*myacq->attr_DebugThread_read = 56;
	{ string msg_log("exit from acquisition thread"); myacq->info_log(msg_log); }

	gettimeofday(&myacq->thread_state_time,NULL);

	malloc_trim(0); /* release heap */

	myacq->link_flag = false;
	myacq->pause_flag = false;

}


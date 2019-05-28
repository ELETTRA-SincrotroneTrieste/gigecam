/*----- PROTECTED REGION ID(GigeCam.cpp) ENABLED START -----*/
static const char *RcsId = "$Id: GigeCam.cpp,v 1.58 2019-03-09 11:32:40 giulio Exp $";
//=============================================================================
//
// file :        GigeCam.cpp
//
// description : C++ source for the GigeCam and its commands.
//               The class is derived from Device. It represents the
//               CORBA servant object which will be accessed from the
//               network. All commands which can be executed on the
//               GigeCam are implemented in this file.
//
// project :     Gigabit Camera Server.
//
// $Author: giulio $
//
// $Revision: 1.58 $
// $Date: 2019-03-09 11:32:40 $
//
// SVN only:
// $HeadURL:  $
//
// CVS only:
// $Source: /home/cvsadm/cvsroot/fermi/servers/gigecam/src/GigeCam.cpp,v $
// $Log: GigeCam.cpp,v $
// Revision 1.58  2019-03-09 11:32:40  giulio
// Rnmshare initialized in main.cpp to avoid crashes at rnmshare_init
//
// Revision 1.57  2019-01-25 08:08:54  giulio
// Support to 180 degrees HWROI support
//
// Revision 1.56  2019-01-24 15:42:29  giulio
// Added aAV2300 and acA1440-73gm
//
// Revision 1.55  2018-10-05 15:18:58  giulio
// Bugfixing when saving offsets
//
// Revision 1.54  2018-10-05 10:03:56  giulio
// Fixed some bugs
//
// Revision 1.53  2018-10-03 13:55:16  giulio
// Allowing set scale/offset when ccd is off, bugfixing
//
// Revision 1.52  2018-06-19 07:06:53  giulio
// Added acA2040-35-gm and acA2500-14gm support, Makefile for pylon5
//
// Revision 1.51  2017-04-19 07:00:53  giulio
// Fixed feedback mode
//
// Revision 1.50  2017-03-20 12:22:22  giulio
// Added aCA2000-50gm, added support for tango based trajectory feedback
//
// Revision 1.49  2016-07-28 11:49:09  giulio
// Added acA1300-75gm
//
// Revision 1.48  2016-04-04 07:39:07  giulio
// Added acA1300-60gm
//
// Revision 1.47  2016-03-16 14:01:47  giulio
// Adde acA645-100gm and acA1920-50gm
//
// Revision 1.46  2015-05-26 12:45:41  giulio
// Fixed get_area bug, fixed calib x/y when binning=2
//
// Revision 1.45  2015-02-16 13:26:53  giulio
// Temporary commit
//
// Revision 1.44  2015-02-10 10:13:04  giulio
// Added acA1600 camera
//
// Revision 1.43  2015-01-19 10:43:30  giulio
// Added av1600 camera, solved problems with ROI
//
// Revision 1.42  2014-12-30 08:33:57  giulio
// Added Model attribute
//
// Revision 1.41  2014-08-28 07:49:45  giulio
// Added imagesum attribute and scA1400 ccd
//
// Revision 1.40  2013-12-04 09:53:20  giulio
// Moved to Pylon3, added some eth tuning properties (band reservation, interpacket delay)
//
// Revision 1.39  2013-08-07 08:00:26  giulio
// Added mutexes in get_data commands; moved to Tango8; changed shutdown timeouts
//
// Revision 1.38  2013-07-09 08:34:16  giulio
// Added packet size configuration
//
// Revision 1.37  2013-07-01 09:32:36  giulio
// Fixed bug in hw roi (very small roi bug)
//
// Revision 1.36  2013-06-17 11:52:56  giulio
// Bug in get_imag8 fixed
//
// Revision 1.35  2013-06-17 07:03:23  giulio
// Fixed bug in get profiles
//
// Revision 1.34  2013-05-29 12:49:17  giulio
// Implemented hwroiparam
//
// Revision 1.33  2013-05-20 14:36:40  giulio
// Fixed bug in DIS algorithm when roi closed to the borders
//
// Revision 1.32  2013-05-17 13:14:08  giulio
// Added disable trigger timeout, stream grabber configurable
//
// Revision 1.31  2013-05-16 12:41:23  giulio
// Disable hw roi when image rotated, fix hor hw roi overflow
//
// Revision 1.30  2013-04-16 14:45:37  giulio
// Removed rpath, added rt-buffer 6,7 mode to profile data
//
// Revision 1.29  2013-04-11 14:29:53  giulio
// Added digital image stabilization, saved roi_threshold
//
// Revision 1.28  2013-02-12 09:01:37  giulio
// Minimized bug in connect-disconnect with 2.3.3
//
// Revision 1.27  2013-02-08 15:12:34  giulio
// Added autoreconnect disable + avA1000 ccd
//
// Revision 1.26  2013-01-24 08:47:29  giulio
// Added binning
//
// Revision 1.25  2013-01-23 14:01:18  giulio
// Fixed bug on ext. trigger absence (server locked)
//
// Revision 1.24  2012-09-28 09:09:24  giulio
// Added support for acA780-gm camera
//
// Revision 1.23  2012-09-17 11:44:53  giulio
// Major updates (hw roi, auto gain/exposure, rnm..)
//
// Revision 1.22  2012-07-19 09:34:13  giulio
// Delete some delete attribute instructions automatically doubled by pogo7
//
// Revision 1.21  2012-07-02 16:06:11  giulio
// Added saveparam command
//
// Revision 1.20  2012-06-15 15:40:57  giulio
// Fixed bug in start device (wait link flag), splitted exceptions in thread destruction
//
// Revision 1.19  2012-06-14 08:20:12  giulio
// Fixed bug in modify configuration (do not store ccd hw parameters)
//
// Revision 1.18  2012-06-11 11:51:50  giulio
// run-flag forced to false in stop command
//
// Revision 1.17  2012-05-30 12:42:54  giulio
// Support for 32/64 bit platforms, fixed init procedure
//
// Revision 1.16  2012-01-30 12:47:01  giulio
// Added attributes for image transfer optimization
//
// Revision 1.15  2012-01-18 11:16:28  giulio
// Fixed a minor bug in get buffer keys in bunch data mode
//
// Revision 1.14  2011/12/28 08:00:53  giulio
// Extended exposure timeout to its maximu value. Store all ccd parameters through SaveScale command
//
// Revision 1.13  2011/06/14 12:19:08  giulio
// Fixed GetImag16 bug
//
// Revision 1.12  2011/05/13 09:44:00  giulio
// Fixed some bugs, moved to tango7, supported Pylon 2.3.3
//
//
//=============================================================================
//                This file is generated by POGO
//        (Program Obviously used to Generate tango Object)
//=============================================================================


#include <tango.h>
#include <GigeCam.h>
#include <GigeCamClass.h>
#include "acqthread.h"

#include <check_get_rt_buffer.h>

/*----- PROTECTED REGION END -----*/	//	GigeCam.cpp

/**
 *  GigeCam class description:
 *    This server connects to a camera using an external thread. Inside this thread, images are acquired and stored
 *    in an array of objects that also do the image processing.
 *    Currently this server support Basler Gigabit cameras.
 *    To make the server work correctly, it is necessary to setup the ethernet interface with
 *    an mtu > 8000 and disable autonegotiation.
 */

//================================================================
//  The following table gives the correspondence
//  between command and method names.
//
//  Command name           |  Method name
//================================================================
//  State                  |  Inherited (no method)
//  Status                 |  Inherited (no method)
//  Start                  |  start
//  Stop                   |  stop
//  Reset                  |  reset
//  ListCamera             |  list_camera
//  AcquireBackground      |  acquire_background
//  GetImage8              |  get_image8
//  GetImage16             |  get_image16
//  GetHorPos              |  get_hor_pos
//  GetVerPos              |  get_ver_pos
//  GetHorSigma            |  get_hor_sigma
//  GetVerSigma            |  get_ver_sigma
//  GetArea                |  get_area
//  GetAcquisitionCounter  |  get_acquisition_counter
//  GetSaturation          |  get_saturation
//  SetReference           |  set_reference
//  GetPhase               |  get_phase
//  SaveScale              |  save_scale
//  RestoreScale           |  restore_scale
//  SetMeasurePoint        |  set_measure_point
//  ClearMeasurePoints     |  clear_measure_points
//  GetIntensity           |  get_intensity
//  GetHorProfile          |  get_hor_profile
//  GetVerProfile          |  get_ver_profile
//  GetHorFitProfile       |  get_hor_fit_profile
//  GetVerFitProfile       |  get_ver_fit_profile
//  SaveParam              |  save_param
//  ClearReference         |  clear_reference
//  ExposureAutoOnce       |  exposure_auto_once
//  GainAutoOnce           |  gain_auto_once
//  GetFitError            |  get_fit_error
//  GetBunchNumber         |  get_bunch_number
//  GetMaxVal              |  get_max_val
//  GetGain                |  get_gain
//  GetExposure            |  get_exposure
//================================================================

//================================================================
//  Attributes managed are:
//================================================================
//  HorPos                   |  Tango::DevDouble	Scalar
//  VerPos                   |  Tango::DevDouble	Scalar
//  MaxVal                   |  Tango::DevLong	Scalar
//  CountMaxVal              |  Tango::DevLong	Scalar
//  HorSigma                 |  Tango::DevDouble	Scalar
//  VerSigma                 |  Tango::DevDouble	Scalar
//  Area                     |  Tango::DevDouble	Scalar
//  HVCovar                  |  Tango::DevDouble	Scalar
//  CameraCounter            |  Tango::DevLong	Scalar
//  AcquisitionCounter       |  Tango::DevLong	Scalar
//  Height                   |  Tango::DevLong	Scalar
//  Width                    |  Tango::DevLong	Scalar
//  ProcessingTime           |  Tango::DevLong	Scalar
//  Frequency                |  Tango::DevDouble	Scalar
//  Trigger                  |  Tango::DevBoolean	Scalar
//  Exposure                 |  Tango::DevLong	Scalar
//  Gain                     |  Tango::DevDouble	Scalar
//  RoiThreshold             |  Tango::DevDouble	Scalar
//  AutoRoi                  |  Tango::DevBoolean	Scalar
//  SwRoi                    |  Tango::DevBoolean	Scalar
//  HwRoi                    |  Tango::DevBoolean	Scalar
//  PlotMode                 |  Tango::DevShort	Scalar
//  IconSize                 |  Tango::DevLong	Scalar
//  LowPassOrder             |  Tango::DevShort	Scalar
//  Rotation                 |  Tango::DevDouble	Scalar
//  Mirror                   |  Tango::DevBoolean	Scalar
//  ImageDepth               |  Tango::DevShort	Scalar
//  BackgroundSubtraction    |  Tango::DevBoolean	Scalar
//  EnableProcessing         |  Tango::DevBoolean	Scalar
//  FastProcessing           |  Tango::DevBoolean	Scalar
//  Saturation               |  Tango::DevDouble	Scalar
//  PlotLevel                |  Tango::DevShort	Scalar
//  Pause                    |  Tango::DevBoolean	Scalar
//  Label                    |  Tango::DevString	Scalar
//  HorPosRelative           |  Tango::DevDouble	Scalar
//  VerPosRelative           |  Tango::DevDouble	Scalar
//  Phase                    |  Tango::DevDouble	Scalar
//  ScaleHor                 |  Tango::DevDouble	Scalar
//  ScaleVer                 |  Tango::DevDouble	Scalar
//  OffsetHor                |  Tango::DevDouble	Scalar
//  OffsetVer                |  Tango::DevDouble	Scalar
//  PlotOffsetAxis           |  Tango::DevBoolean	Scalar
//  Intensity                |  Tango::DevDouble	Scalar
//  SignHor                  |  Tango::DevDouble	Scalar
//  SignVer                  |  Tango::DevDouble	Scalar
//  ProcessMode              |  Tango::DevLong	Scalar
//  DebugThread              |  Tango::DevLong	Scalar
//  BlackLevel               |  Tango::DevDouble	Scalar
//  ImageBufferSize          |  Tango::DevLong	Scalar
//  ImageAcquisitionCounter  |  Tango::DevLong	Scalar
//  ExposureAutoEnable       |  Tango::DevBoolean	Scalar
//  ExposureAutoMin          |  Tango::DevLong	Scalar
//  ExposureAutoMax          |  Tango::DevLong	Scalar
//  GainAutoEnable           |  Tango::DevBoolean	Scalar
//  GainAutoMin              |  Tango::DevDouble	Scalar
//  GainAutoMax              |  Tango::DevDouble	Scalar
//  AutoTargetValue          |  Tango::DevLong	Scalar
//  AutoFeedbackGain         |  Tango::DevDouble	Scalar
//  AutoFeedbackDws          |  Tango::DevLong	Scalar
//  AutoFeedbackTargetThres  |  Tango::DevLong	Scalar
//  AutoFeedbackDeadBand     |  Tango::DevLong	Scalar
//  FitError                 |  Tango::DevLong	Scalar
//  BunchNumber              |  Tango::DevLong	Scalar
//  MeanIntensityThreshold   |  Tango::DevLong	Scalar
//  Binning                  |  Tango::DevLong	Scalar
//  DISEnable                |  Tango::DevBoolean	Scalar
//  DisableTriggerTimeout    |  Tango::DevBoolean	Scalar
//  ImageSumSamples          |  Tango::DevLong	Scalar
//  Model                    |  Tango::DevString	Scalar
//  FeedbackMode             |  Tango::DevBoolean	Scalar
//  MeanIntensity            |  Tango::DevDouble	Scalar
//  FeedbackError            |  Tango::DevBoolean	Scalar
//  SearchBackgroundParam    |  Tango::DevDouble	Spectrum  ( max = 2)
//  HorProfile               |  Tango::DevDouble	Spectrum  ( max = 3000)
//  VerProfile               |  Tango::DevDouble	Spectrum  ( max = 3000)
//  ErrorCounters            |  Tango::DevLong	Spectrum  ( max = 10)
//  TimeStamp                |  Tango::DevLong	Spectrum  ( max = 2)
//  AutoRoiParam             |  Tango::DevLong	Spectrum  ( max = 4)
//  RoiParam                 |  Tango::DevLong	Spectrum  ( max = 4)
//  HorFitProfile            |  Tango::DevDouble	Spectrum  ( max = 3000)
//  VerFitProfile            |  Tango::DevDouble	Spectrum  ( max = 3000)
//  HwRoiParam               |  Tango::DevLong	Spectrum  ( max = 4)
//  Image8                   |  Tango::DevUChar	Image  ( max = 3000 x 3000)
//  Image8Icon               |  Tango::DevLong	Image  ( max = 400 x 400)
//  Image16                  |  Tango::DevUShort	Image  ( max = 3000 x 3000)
//  Image16Icon              |  Tango::DevLong	Image  ( max = 400 x 400)
//  Image8Counter            |  Tango::DevUChar	Image  ( max = 3000 x 3000)
//  Image16Counter           |  Tango::DevUShort	Image  ( max = 3000 x 3000)
//  ImageSum                 |  Tango::DevLong	Image  ( max = 3000 x 3000)
//================================================================

namespace GigeCam_ns
{
/*----- PROTECTED REGION ID(GigeCam::namespace_starting) ENABLED START -----*/

	//	static initializations

	/*----- PROTECTED REGION END -----*/	//	GigeCam::namespace_starting

//--------------------------------------------------------
/**
 *	Method      : GigeCam::GigeCam()
 *	Description : Constructors for a Tango device
 *                implementing the classGigeCam
 */
//--------------------------------------------------------
GigeCam::GigeCam(Tango::DeviceClass *cl, string &s)
 : TANGO_BASE_CLASS(cl, s.c_str())
{
	/*----- PROTECTED REGION ID(GigeCam::constructor_1) ENABLED START -----*/

	init_device();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::constructor_1
}
//--------------------------------------------------------
GigeCam::GigeCam(Tango::DeviceClass *cl, const char *s)
 : TANGO_BASE_CLASS(cl, s)
{
	/*----- PROTECTED REGION ID(GigeCam::constructor_2) ENABLED START -----*/

	init_device();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::constructor_2
}
//--------------------------------------------------------
GigeCam::GigeCam(Tango::DeviceClass *cl, const char *s, const char *d)
 : TANGO_BASE_CLASS(cl, s, d)
{
	/*----- PROTECTED REGION ID(GigeCam::constructor_3) ENABLED START -----*/

	init_device();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::constructor_3
}

//--------------------------------------------------------
/**
 *	Method      : GigeCam::delete_device()
 *	Description : will be called at device destruction or at init command
 */
//--------------------------------------------------------
void GigeCam::delete_device()
{
	DEBUG_STREAM << "GigeCam::delete_device() " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::delete_device) ENABLED START -----*/

	//	Delete device allocated objects
	//	Delete device's allocated object

	/* delete camera parameters struct */

	{ string msg_log("delete_device start"); info_log(msg_log); }
	int32_t cnt = 0;
	struct timeval start, now;

	run_flag = false;
	pause_flag = false;
	*attr_DisableTriggerTimeout_read = false;

	gettimeofday(&start,NULL);

	while (link_flag) {
		usleep(500000); 
		gettimeofday(&now,NULL);
		if ((now.tv_sec-start.tv_sec) > (0.6*GIGECAM_LOCK_TOUT)) {
			string msg_log("Failed to release acquisition thread, shutting down"); 
			info_log(msg_log);
			exit(1);
		}
	}

	delete [] cam_param;
	delete in_mutex;

#ifdef GIGECAM_RNM
	if (rnmEnabled) {
		rnmshare_deinit();
	}
#endif

	delete attr_HorPos_read;
	delete attr_VerPos_read;
	delete attr_Intensity_read;
	delete attr_HorSigma_read;
	delete attr_VerSigma_read;
	delete attr_Area_read;

	delete attr_Phase_read;
	delete attr_Gain_read;
	delete [] attr_SearchBackgroundParam_read;

	delete attr_HorPosRelative_read;
	delete attr_VerPosRelative_read;
	delete attr_ScaleHor_read;
	delete attr_ScaleVer_read;
	delete attr_OffsetHor_read;
	delete attr_OffsetVer_read;
	delete attr_HVCovar_read;
	delete attr_Frequency_read;
	delete attr_RoiThreshold_read;
	delete attr_Rotation_read;
	delete attr_SignHor_read;
	delete attr_SignVer_read;

	delete attr_DebugThread_read;
	delete attr_MaxVal_read;
	delete attr_Saturation_read;
	delete attr_CountMaxVal_read;
	delete attr_Exposure_read;
	delete attr_ProcessMode_read;
	delete [] attr_TimeStamp_read;
	delete [] attr_ErrorCounters_read;
	delete attr_CameraCounter_read;
	delete attr_AcquisitionCounter_read;
	delete attr_ProcessingTime_read;
	delete attr_Height_read;
	delete attr_Width_read;
	delete attr_IconSize_read;

	delete attr_PlotMode_read;
	delete attr_LowPassOrder_read;
	delete attr_ImageDepth_read;
	delete attr_PlotLevel_read;

	delete attr_Trigger_read;
	delete attr_AutoRoi_read;
	delete attr_SwRoi_read;
	delete attr_HwRoi_read;
	delete attr_Mirror_read;
	delete attr_BackgroundSubtraction_read;
	delete attr_EnableProcessing_read;
	delete attr_FastProcessing_read;
	delete attr_Pause_read;
	delete attr_PlotOffsetAxis_read;
 	delete attr_FitError_read;
	delete attr_BunchNumber_read;
	delete[] attr_HwRoiParam_read;

	if (*attr_Label_read) 
    delete *attr_Label_read;
  
  if (attr_Label_read) {
	  delete[] attr_Label_read;
    attr_Label_read = 0;
  }

	{
	string msg_log("delete_device end");
	info_log(msg_log);
	}

	/*----- PROTECTED REGION END -----*/	//	GigeCam::delete_device
	delete[] attr_BlackLevel_read;
	delete[] attr_ImageBufferSize_read;
	delete[] attr_ImageAcquisitionCounter_read;
	delete[] attr_ExposureAutoEnable_read;
	delete[] attr_ExposureAutoMin_read;
	delete[] attr_ExposureAutoMax_read;
	delete[] attr_GainAutoEnable_read;
	delete[] attr_GainAutoMin_read;
	delete[] attr_GainAutoMax_read;
	delete[] attr_AutoTargetValue_read;
	delete[] attr_AutoFeedbackGain_read;
	delete[] attr_AutoFeedbackDws_read;
	delete[] attr_AutoFeedbackTargetThres_read;
	delete[] attr_AutoFeedbackDeadBand_read;
	delete[] attr_MeanIntensityThreshold_read;
	delete[] attr_Binning_read;
	delete[] attr_DISEnable_read;
	delete[] attr_DisableTriggerTimeout_read;
	delete[] attr_ImageSumSamples_read;
	delete[] attr_Model_read;
	delete[] attr_FeedbackMode_read;
	delete[] attr_MeanIntensity_read;
	delete[] attr_FeedbackError_read;
	delete[] attr_ImageSum_read;
}

//--------------------------------------------------------
/**
 *	Method      : GigeCam::init_device()
 *	Description : will be called at device initialization.
 */
//--------------------------------------------------------
void GigeCam::init_device()
{
	DEBUG_STREAM << "GigeCam::init_device() create device " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::init_device_before) ENABLED START -----*/

	//	Initialization before get_device_property() call
	

	/*----- PROTECTED REGION END -----*/	//	GigeCam::init_device_before
	

	//	Get the device properties from database
	get_device_property();
	
	attr_BlackLevel_read = new Tango::DevDouble[1];
	attr_ImageBufferSize_read = new Tango::DevLong[1];
	attr_ImageAcquisitionCounter_read = new Tango::DevLong[1];
	attr_ExposureAutoEnable_read = new Tango::DevBoolean[1];
	attr_ExposureAutoMin_read = new Tango::DevLong[1];
	attr_ExposureAutoMax_read = new Tango::DevLong[1];
	attr_GainAutoEnable_read = new Tango::DevBoolean[1];
	attr_GainAutoMin_read = new Tango::DevDouble[1];
	attr_GainAutoMax_read = new Tango::DevDouble[1];
	attr_AutoTargetValue_read = new Tango::DevLong[1];
	attr_AutoFeedbackGain_read = new Tango::DevDouble[1];
	attr_AutoFeedbackDws_read = new Tango::DevLong[1];
	attr_AutoFeedbackTargetThres_read = new Tango::DevLong[1];
	attr_AutoFeedbackDeadBand_read = new Tango::DevLong[1];
	attr_MeanIntensityThreshold_read = new Tango::DevLong[1];
	attr_Binning_read = new Tango::DevLong[1];
	attr_DISEnable_read = new Tango::DevBoolean[1];
	attr_DisableTriggerTimeout_read = new Tango::DevBoolean[1];
	attr_ImageSumSamples_read = new Tango::DevLong[1];
	attr_Model_read = new Tango::DevString[1];
	attr_FeedbackMode_read = new Tango::DevBoolean[1];
	attr_MeanIntensity_read = new Tango::DevDouble[1];
	attr_FeedbackError_read = new Tango::DevBoolean[1];
	attr_ImageSum_read = new Tango::DevLong[3000*3000];
	/*----- PROTECTED REGION ID(GigeCam::init_device) ENABLED START -----*/

	//	Initialize device
	*attr_DisableTriggerTimeout_read = false;
	Tango::Database *db = new Tango::Database();
	try {
		Tango::DbData db_data;
		db_data.push_back((Tango::DbDatum("Host")));
		db_data.push_back((Tango::DbDatum("Port")));
		db->get_property("Database",db_data);
		db_data[0] >> host_rw;
		db_data[1] >> port_rw;
	}
	catch(Tango::DevFailed &e) {
	      INFO_STREAM << __FUNCTION__ << " Error reading Database property='" << e.errors[0].desc << "'";
	      delete db;
	}
	delete db;

	attr_BlackLevel_read = new Tango::DevDouble[1];
	attr_ImageBufferSize_read = new Tango::DevLong[1];
	attr_ImageAcquisitionCounter_read = new Tango::DevLong[1];
	attr_HorPos_read = new Tango::DevDouble();
	attr_VerPos_read = new Tango::DevDouble();
	attr_Intensity_read = new Tango::DevDouble();
	attr_HorSigma_read = new Tango::DevDouble();
	attr_VerSigma_read = new Tango::DevDouble();
	attr_Area_read = new Tango::DevDouble();
	attr_Saturation_read = new Tango::DevDouble();
	attr_Phase_read = new Tango::DevDouble();
	attr_Gain_read = new Tango::DevDouble();
	attr_BunchNumber_read = new Tango::DevLong();

	attr_HorPosRelative_read = new Tango::DevDouble();
	attr_VerPosRelative_read = new Tango::DevDouble();
	attr_ScaleHor_read = new Tango::DevDouble();
	attr_ScaleVer_read = new Tango::DevDouble();
	attr_OffsetHor_read = new Tango::DevDouble();
	attr_OffsetVer_read = new Tango::DevDouble();
	attr_HVCovar_read = new Tango::DevDouble();
	attr_Frequency_read = new Tango::DevDouble();
	attr_RoiThreshold_read = new Tango::DevDouble();
	attr_Rotation_read = new Tango::DevDouble();
	attr_SignHor_read = new Tango::DevDouble();
	attr_SignVer_read = new Tango::DevDouble();
	attr_SearchBackgroundParam_read = new Tango::DevDouble[2];

	attr_Exposure_read = new Tango::DevLong();
	attr_DebugThread_read = new Tango::DevLong();
	attr_FitError_read = new Tango::DevLong();
	attr_MaxVal_read = new Tango::DevLong();
	attr_CountMaxVal_read = new Tango::DevLong();
	attr_ProcessMode_read = new Tango::DevLong();
	attr_TimeStamp_read = new Tango::DevLong[2];
	attr_ErrorCounters_read = new Tango::DevLong[5];
	attr_CameraCounter_read = new Tango::DevLong();
	attr_AcquisitionCounter_read = new Tango::DevLong();
	attr_ProcessingTime_read = new Tango::DevLong();
	attr_Height_read = new Tango::DevLong();
	attr_Width_read = new Tango::DevLong();
	attr_IconSize_read = new Tango::DevLong();

	attr_PlotMode_read = new Tango::DevShort();
	attr_LowPassOrder_read = new Tango::DevShort();
	attr_ImageDepth_read = new Tango::DevShort();
	attr_PlotLevel_read = new Tango::DevShort();

	attr_Trigger_read = new Tango::DevBoolean();
	attr_AutoRoi_read = new Tango::DevBoolean();
	attr_SwRoi_read = new Tango::DevBoolean();
	attr_HwRoi_read = new Tango::DevBoolean();
	attr_Mirror_read = new Tango::DevBoolean();
	attr_BackgroundSubtraction_read = new Tango::DevBoolean();
	attr_EnableProcessing_read = new Tango::DevBoolean();
	attr_FastProcessing_read = new Tango::DevBoolean();
	attr_Pause_read = new Tango::DevBoolean();
	attr_PlotOffsetAxis_read = new Tango::DevBoolean();
	attr_HwRoiParam_read = new Tango::DevLong[4];

	*attr_Model_read = new char[GIGECAM_DESC];
	memset(*attr_Model_read, 0, GIGECAM_DESC);

	*attr_ImageSumSamples_read = 0;
	*attr_FeedbackMode_read = false;
	*attr_MeanIntensity_read = 0;
	*attr_FeedbackError_read = false;

	attr_Label_read = new char*;
	*attr_Label_read = new char[GIGECAM_DESC];

	in_mutex = new omni_mutex();

	gettimeofday(&thread_state_time,NULL);
	gettimeofday(&last_state_time,NULL);

	set_state(Tango::OFF);
	set_status("Camera not selected");
	
	/* connect by default to the first camera */
	cam_idx = 0; img_cur = 0; img_nxt = 0;
	attr_ImageAcquisitionCounter_write = 0;
	attr_Exposure_write = 0;
	auto_restart_cnt = 0;

	trigger_background = false;
	take_reference = false;
	
	for (int32_t i = 0; i < GIGECAM_ERR_CNT_SIZE; i++)
		error_counters[i] = 0;

	get_configuration();

#ifdef GIGECAM_RNM_NOCOMPILE
	if (rnmEnabled) {
		cout << "Gigecam::init_device(): before opening rnmshare" << endl;
		if (rnmshare_init(0) < 0) {
			cout << "Gigecam::init_device(): failed to open rnm" << endl;
			rnmEnabled = false;
			Tango::Except::throw_exception (
				(const char *)"Failed to conntect to rnm",
				(const char *)"Rnm error",
				(const char *)"Gigecam::init_device()");
		}
		else {
			cout << "Enable rnm bunch number acquisition" << endl;
		}		
	}
#endif

	gettimeofday(&socket_state_time, NULL);

	link_flag = false; pause_flag = false; run_flag = false;


	/* initialize buffer pointers */
	buf_phase = 0;
	buf_area = 0;
	buf_hor_pos = 0;
	buf_hor_sigma = 0;
	buf_saturation = 0;
	buf_ver_pos = 0;
	buf_ver_sigma = 0;
	buf_intensity = 0;
	buf_fit_error = 0;
	buf_acquisition_counter = 0;
	buf_max_val = 0;
	buf_gain = 0;
	buf_exposure = 0;

	/* for bunch number estimation (currently not used) */
	bunch_number_lock.old_frame_num = 0;
	bunch_number_lock.abs_frame_num = 0;
	bunch_number_lock.old_bunch_num = 0;
	bunch_number_lock.buf_idx = 0;
	for (int32_t i = 0; i < GIGECAM_LOCK_BUF_SIZE; i++) {
		bunch_number_lock.abs_frame_num_buf[i] = 0;
		bunch_number_lock.estim_bunch_num_buf[i] = 0;
		bunch_number_lock.real_bunch_num_buf[i] = 0;
	}	
	bunch_number_lock.consec_estim_err_counter = 0;

	bunch_number = 0;
	comm_err_consec_cnt = 0;
	trig_err_consec_cnt = 0;

	/* if only one camera supported start acquisition immediatly */
	/* in this case cam_idx is always 0 */
	circ_buffer = true;


	{ string msg_log("device started"); info_log(msg_log); }
	if ((num_cameras == 1) && (autoConnect == true)) {
		set_state(Tango::FAULT);
		run_flag = true;
		acqloop = NULL;
		acqloop = new acqthread(this);
		acqloop->start();
	}
	{ string msg_log("init_device end"); info_log(msg_log); }
	shutdown_flag = false;
	/*----- PROTECTED REGION END -----*/	//	GigeCam::init_device
}

//--------------------------------------------------------
/**
 *	Method      : GigeCam::get_device_property()
 *	Description : Read database to initialize property data members.
 */
//--------------------------------------------------------
void GigeCam::get_device_property()
{
	/*----- PROTECTED REGION ID(GigeCam::get_device_property_before) ENABLED START -----*/

	//	Initialize property data members
	

	/*----- PROTECTED REGION END -----*/	//	GigeCam::get_device_property_before


	//	Read device properties from database.
	Tango::DbData	dev_prop;
	dev_prop.push_back(Tango::DbDatum("Configuration"));
	dev_prop.push_back(Tango::DbDatum("AutoRoi"));
	dev_prop.push_back(Tango::DbDatum("AutoRoiParam"));
	dev_prop.push_back(Tango::DbDatum("ImageProcessing"));
	dev_prop.push_back(Tango::DbDatum("PlotMode"));
	dev_prop.push_back(Tango::DbDatum("FastCalculation"));
	dev_prop.push_back(Tango::DbDatum("ImageBufferSize"));
	dev_prop.push_back(Tango::DbDatum("DataBufferSize"));
	dev_prop.push_back(Tango::DbDatum("SearchBackgroundParam"));
	dev_prop.push_back(Tango::DbDatum("RnmEnabled"));
	dev_prop.push_back(Tango::DbDatum("AutoConnect"));
	dev_prop.push_back(Tango::DbDatum("RnmIndex"));
	dev_prop.push_back(Tango::DbDatum("ProcessMode"));
	dev_prop.push_back(Tango::DbDatum("RoiParam"));
	dev_prop.push_back(Tango::DbDatum("SwRoi"));
	dev_prop.push_back(Tango::DbDatum("HwRoi"));
	dev_prop.push_back(Tango::DbDatum("RefPosOffset"));
	dev_prop.push_back(Tango::DbDatum("GainAutoParam"));
	dev_prop.push_back(Tango::DbDatum("ExposureAutoParam"));
	dev_prop.push_back(Tango::DbDatum("HwRoiParam"));
	dev_prop.push_back(Tango::DbDatum("AutoTargetValue"));
	dev_prop.push_back(Tango::DbDatum("AutoFeedbackGain"));
	dev_prop.push_back(Tango::DbDatum("AutoFeedbackDws"));
	dev_prop.push_back(Tango::DbDatum("AutoFeedbackDeadband"));
	dev_prop.push_back(Tango::DbDatum("AutoFeedbackTargetThres"));
	dev_prop.push_back(Tango::DbDatum("Binning"));
	dev_prop.push_back(Tango::DbDatum("AutoReconnect"));
	dev_prop.push_back(Tango::DbDatum("RoiThreshold"));
	dev_prop.push_back(Tango::DbDatum("StreamGrabberBuffer"));
	dev_prop.push_back(Tango::DbDatum("PacketSize"));
	dev_prop.push_back(Tango::DbDatum("FitNumCores"));
	dev_prop.push_back(Tango::DbDatum("InterPacketDelay"));
	dev_prop.push_back(Tango::DbDatum("ReserveBandwidth"));

	//	is there at least one property to be read ?
	if (dev_prop.size()>0)
	{
		//	Call database and extract values
		if (Tango::Util::instance()->_UseDb==true)
			get_db_device()->get_property(dev_prop);
	
		//	get instance on GigeCamClass to get class property
		Tango::DbDatum	def_prop, cl_prop;
		GigeCamClass	*ds_class =
			(static_cast<GigeCamClass *>(get_device_class()));
		int	i = -1;

		//	Try to initialize Configuration from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  configuration;
		else {
			//	Try to initialize Configuration from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  configuration;
		}
		//	And try to extract Configuration value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  configuration;

		//	Try to initialize AutoRoi from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  autoRoi;
		else {
			//	Try to initialize AutoRoi from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  autoRoi;
		}
		//	And try to extract AutoRoi value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  autoRoi;

		//	Try to initialize AutoRoiParam from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  autoRoiParam;
		else {
			//	Try to initialize AutoRoiParam from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  autoRoiParam;
		}
		//	And try to extract AutoRoiParam value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  autoRoiParam;

		//	Try to initialize ImageProcessing from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  imageProcessing;
		else {
			//	Try to initialize ImageProcessing from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  imageProcessing;
		}
		//	And try to extract ImageProcessing value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  imageProcessing;

		//	Try to initialize PlotMode from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  plotMode;
		else {
			//	Try to initialize PlotMode from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  plotMode;
		}
		//	And try to extract PlotMode value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  plotMode;

		//	Try to initialize FastCalculation from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  fastCalculation;
		else {
			//	Try to initialize FastCalculation from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  fastCalculation;
		}
		//	And try to extract FastCalculation value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  fastCalculation;

		//	Try to initialize ImageBufferSize from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  imageBufferSize;
		else {
			//	Try to initialize ImageBufferSize from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  imageBufferSize;
		}
		//	And try to extract ImageBufferSize value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  imageBufferSize;

		//	Try to initialize DataBufferSize from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  dataBufferSize;
		else {
			//	Try to initialize DataBufferSize from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  dataBufferSize;
		}
		//	And try to extract DataBufferSize value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  dataBufferSize;

		//	Try to initialize SearchBackgroundParam from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  searchBackgroundParam;
		else {
			//	Try to initialize SearchBackgroundParam from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  searchBackgroundParam;
		}
		//	And try to extract SearchBackgroundParam value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  searchBackgroundParam;

		//	Try to initialize RnmEnabled from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  rnmEnabled;
		else {
			//	Try to initialize RnmEnabled from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  rnmEnabled;
		}
		//	And try to extract RnmEnabled value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  rnmEnabled;

		//	Try to initialize AutoConnect from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  autoConnect;
		else {
			//	Try to initialize AutoConnect from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  autoConnect;
		}
		//	And try to extract AutoConnect value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  autoConnect;

		//	Try to initialize RnmIndex from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  rnmIndex;
		else {
			//	Try to initialize RnmIndex from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  rnmIndex;
		}
		//	And try to extract RnmIndex value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  rnmIndex;

		//	Try to initialize ProcessMode from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  processMode;
		else {
			//	Try to initialize ProcessMode from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  processMode;
		}
		//	And try to extract ProcessMode value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  processMode;

		//	Try to initialize RoiParam from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  roiParam;
		else {
			//	Try to initialize RoiParam from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  roiParam;
		}
		//	And try to extract RoiParam value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  roiParam;

		//	Try to initialize SwRoi from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  swRoi;
		else {
			//	Try to initialize SwRoi from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  swRoi;
		}
		//	And try to extract SwRoi value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  swRoi;

		//	Try to initialize HwRoi from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  hwRoi;
		else {
			//	Try to initialize HwRoi from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  hwRoi;
		}
		//	And try to extract HwRoi value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  hwRoi;

		//	Try to initialize RefPosOffset from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  refPosOffset;
		else {
			//	Try to initialize RefPosOffset from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  refPosOffset;
		}
		//	And try to extract RefPosOffset value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  refPosOffset;

		//	Try to initialize GainAutoParam from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  gainAutoParam;
		else {
			//	Try to initialize GainAutoParam from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  gainAutoParam;
		}
		//	And try to extract GainAutoParam value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  gainAutoParam;

		//	Try to initialize ExposureAutoParam from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  exposureAutoParam;
		else {
			//	Try to initialize ExposureAutoParam from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  exposureAutoParam;
		}
		//	And try to extract ExposureAutoParam value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  exposureAutoParam;

		//	Try to initialize HwRoiParam from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  hwRoiParam;
		else {
			//	Try to initialize HwRoiParam from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  hwRoiParam;
		}
		//	And try to extract HwRoiParam value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  hwRoiParam;

		//	Try to initialize AutoTargetValue from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  autoTargetValue;
		else {
			//	Try to initialize AutoTargetValue from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  autoTargetValue;
		}
		//	And try to extract AutoTargetValue value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  autoTargetValue;

		//	Try to initialize AutoFeedbackGain from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  autoFeedbackGain;
		else {
			//	Try to initialize AutoFeedbackGain from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  autoFeedbackGain;
		}
		//	And try to extract AutoFeedbackGain value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  autoFeedbackGain;

		//	Try to initialize AutoFeedbackDws from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  autoFeedbackDws;
		else {
			//	Try to initialize AutoFeedbackDws from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  autoFeedbackDws;
		}
		//	And try to extract AutoFeedbackDws value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  autoFeedbackDws;

		//	Try to initialize AutoFeedbackDeadband from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  autoFeedbackDeadband;
		else {
			//	Try to initialize AutoFeedbackDeadband from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  autoFeedbackDeadband;
		}
		//	And try to extract AutoFeedbackDeadband value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  autoFeedbackDeadband;

		//	Try to initialize AutoFeedbackTargetThres from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  autoFeedbackTargetThres;
		else {
			//	Try to initialize AutoFeedbackTargetThres from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  autoFeedbackTargetThres;
		}
		//	And try to extract AutoFeedbackTargetThres value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  autoFeedbackTargetThres;

		//	Try to initialize Binning from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  binning;
		else {
			//	Try to initialize Binning from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  binning;
		}
		//	And try to extract Binning value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  binning;

		//	Try to initialize AutoReconnect from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  autoReconnect;
		else {
			//	Try to initialize AutoReconnect from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  autoReconnect;
		}
		//	And try to extract AutoReconnect value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  autoReconnect;

		//	Try to initialize RoiThreshold from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  roiThreshold;
		else {
			//	Try to initialize RoiThreshold from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  roiThreshold;
		}
		//	And try to extract RoiThreshold value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  roiThreshold;

		//	Try to initialize StreamGrabberBuffer from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  streamGrabberBuffer;
		else {
			//	Try to initialize StreamGrabberBuffer from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  streamGrabberBuffer;
		}
		//	And try to extract StreamGrabberBuffer value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  streamGrabberBuffer;

		//	Try to initialize PacketSize from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  packetSize;
		else {
			//	Try to initialize PacketSize from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  packetSize;
		}
		//	And try to extract PacketSize value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  packetSize;

		//	Try to initialize FitNumCores from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  fitNumCores;
		else {
			//	Try to initialize FitNumCores from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  fitNumCores;
		}
		//	And try to extract FitNumCores value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  fitNumCores;

		//	Try to initialize InterPacketDelay from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  interPacketDelay;
		else {
			//	Try to initialize InterPacketDelay from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  interPacketDelay;
		}
		//	And try to extract InterPacketDelay value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  interPacketDelay;

		//	Try to initialize ReserveBandwidth from class property
		cl_prop = ds_class->get_class_property(dev_prop[++i].name);
		if (cl_prop.is_empty()==false)	cl_prop  >>  reserveBandwidth;
		else {
			//	Try to initialize ReserveBandwidth from default device value
			def_prop = ds_class->get_default_device_property(dev_prop[i].name);
			if (def_prop.is_empty()==false)	def_prop  >>  reserveBandwidth;
		}
		//	And try to extract ReserveBandwidth value from database
		if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  reserveBandwidth;

	}

	/*----- PROTECTED REGION ID(GigeCam::get_device_property_after) ENABLED START -----*/

	//	Check device property data members init

	/*----- PROTECTED REGION END -----*/	//	GigeCam::get_device_property_after
}

//--------------------------------------------------------
/**
 *	Method      : GigeCam::always_executed_hook()
 *	Description : method always executed before any command is executed
 */
//--------------------------------------------------------
void GigeCam::always_executed_hook()
{
	DEBUG_STREAM << "GigeCam::always_executed_hook()  " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::always_executed_hook) ENABLED START -----*/

	//	code always executed before all requests
	struct timeval curr_state_time;

	gettimeofday(&curr_state_time, NULL);

	/* to avoid stopping the server after opening a gui */
	if (abs(last_state_time.tv_sec - curr_state_time.tv_sec) > (GIGECAM_LOCK_TOUT*0.7))
		thread_state_time.tv_sec = curr_state_time.tv_sec;

	last_state_time.tv_sec = curr_state_time.tv_sec;

	if (((curr_state_time.tv_sec - socket_state_time.tv_sec) > GIGECAM_RECONN_TOUT) && (run_flag == true) && (link_flag == false) && ((get_state() == Tango::FAULT))) {
		if ((comm_err_consec_cnt < GIGECAM_COM_ERR_MAX_CNT) || (autoReconnect == true)) {
			comm_err_consec_cnt++;
			acqloop = new acqthread(this);
			string msg_log("Reconnecting camera, "); 
			char dbg_thread[20]; sprintf(dbg_thread,"retry(%d/%d)",(int)comm_err_consec_cnt,GIGECAM_COM_ERR_MAX_CNT);
			msg_log.append(dbg_thread); info_log(msg_log);
			acqloop->start();
			gettimeofday(&socket_state_time,NULL);
		}
		else {
			run_flag = false;
			comm_err_consec_cnt = 0;
			set_state(Tango::OFF);
			set_status("acquisition stopped");
			string msg_log("Maximum number of automatic reconnections reached, acquisition stopped");
			info_log(msg_log);
		}
	}

	/* automatic recovery of basler lock bug */
	if (((curr_state_time.tv_sec - thread_state_time.tv_sec) > GIGECAM_LOCK_TOUT) && (*attr_DisableTriggerTimeout_read == false) &&
	  (pause_flag == false) && (link_flag == true)) {
		if (shutdown_flag == false) {
			set_state(Tango::FAULT);
			set_status("Server locked, shutting down");
			string msg_log("LibBasler err: server locked, shutting down,"); 
			char dbg_thread[20]; sprintf(dbg_thread,"debug(%d)\n",*attr_DebugThread_read);
			msg_log.append(dbg_thread);
			info_log(msg_log);
			shutdown_flag = true;
		}
		if 	((curr_state_time.tv_sec - thread_state_time.tv_sec) > (GIGECAM_LOCK_TOUT*1.2)) {
			string msg_log("LibBasler err: server locked, server exit"); info_log(msg_log);
			exit(1);
		}
	}
	else {
		shutdown_flag = false;
	}
	/*----- PROTECTED REGION END -----*/	//	GigeCam::always_executed_hook
}

//--------------------------------------------------------
/**
 *	Method      : GigeCam::read_attr_hardware()
 *	Description : Hardware acquisition for attributes
 */
//--------------------------------------------------------
void GigeCam::read_attr_hardware(TANGO_UNUSED(vector<long> &attr_list))
{
	DEBUG_STREAM << "GigeCam::read_attr_hardware(vector<long> &attr_list) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_attr_hardware) ENABLED START -----*/

	//	Add your own code
	//	Add your own code here

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_attr_hardware
}
//--------------------------------------------------------
/**
 *	Method      : GigeCam::write_attr_hardware()
 *	Description : Hardware writing for attributes
 */
//--------------------------------------------------------
void GigeCam::write_attr_hardware(TANGO_UNUSED(vector<long> &attr_list))
{
	DEBUG_STREAM << "GigeCam::write_attr_hardware(vector<long> &attr_list) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::write_attr_hardware) ENABLED START -----*/
	
	//	Add your own code
	
	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_attr_hardware
}

//--------------------------------------------------------
/**
 *	Read attribute HorPos related method
 *	Description: Horizontal position of the spot int the image. (Measure)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_HorPos(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_HorPos(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_HorPos) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_HorPos_read = img8[img_cur]->hor_pos * cam_param[cam_idx].hor_sign;
	}
	else if (img16.size() > 0) {
		*attr_HorPos_read = img16[img_cur]->hor_pos * cam_param[cam_idx].hor_sign;
	}
	else
		*attr_HorPos_read = 0;
	in_mutex->unlock();

	attr.set_value(attr_HorPos_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_HorPos
}
//--------------------------------------------------------
/**
 *	Read attribute VerPos related method
 *	Description: Horizontal position of the spot int the image.(Measure)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_VerPos(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_VerPos(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_VerPos) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_VerPos_read = img8[img_cur]->ver_pos * cam_param[cam_idx].ver_sign;
	}
	else if (img16.size() > 0) {
		*attr_VerPos_read = img16[img_cur]->ver_pos * cam_param[cam_idx].ver_sign;
	}
	else
		*attr_VerPos_read = 0;
	in_mutex->unlock();

	attr.set_value(attr_VerPos_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_VerPos
}
//--------------------------------------------------------
/**
 *	Read attribute MaxVal related method
 *	Description: Maximum pixel value.(Measure)
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_MaxVal(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_MaxVal(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_MaxVal) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_MaxVal_read = img8[img_cur]->max_value;
	}
	else if (img16.size() > 0) {
		*attr_MaxVal_read = img16[img_cur]->max_value;
	}
	else
		*attr_MaxVal_read = 0;
	in_mutex->unlock();

	attr.set_value(attr_MaxVal_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_MaxVal
}
//--------------------------------------------------------
/**
 *	Read attribute CountMaxVal related method
 *	Description: Number of pixels equal to the maximum pixel value (255/4095).(Measure)
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_CountMaxVal(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_CountMaxVal(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_CountMaxVal) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_CountMaxVal_read = img8[img_cur]->count_max_value;
	}
	else if (img16.size() > 0) {
		*attr_CountMaxVal_read = img16[img_cur]->count_max_value;
	}
	else
		*attr_CountMaxVal_read = 0;
	in_mutex->unlock();

	attr.set_value(attr_CountMaxVal_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_CountMaxVal
}
//--------------------------------------------------------
/**
 *	Read attribute HorSigma related method
 *	Description: Horizontal standard deviation of the spot in the image.(Measure)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_HorSigma(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_HorSigma(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_HorSigma) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_HorSigma_read = img8[img_cur]->hor_sigma;
	}
	else if (img16.size() > 0) {
		*attr_HorSigma_read = img16[img_cur]->hor_sigma;
	}
	else
		*attr_HorSigma_read = 0;
	in_mutex->unlock();

	attr.set_value(attr_HorSigma_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_HorSigma
}
//--------------------------------------------------------
/**
 *	Read attribute VerSigma related method
 *	Description: Vertical standard deviation of the spot in the image.(Measure)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_VerSigma(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_VerSigma(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_VerSigma) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_VerSigma_read = img8[img_cur]->ver_sigma;
	}
	else if (img16.size() > 0) {
		*attr_VerSigma_read = img16[img_cur]->ver_sigma;
	}
	else
		*attr_VerSigma_read = 0;
	in_mutex->unlock();

	attr.set_value(attr_VerSigma_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_VerSigma
}
//--------------------------------------------------------
/**
 *	Read attribute Area related method
 *	Description: Area of the spot inside the roi.(Measure)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_Area(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_Area(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_Area) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_Area_read = img8[img_cur]->area;
	}
	else if (img16.size() > 0) {
		*attr_Area_read = img16[img_cur]->area;
	}
	else
		*attr_Area_read = 0;
	in_mutex->unlock();

	attr.set_value(attr_Area_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_Area
}
//--------------------------------------------------------
/**
 *	Read attribute HVCovar related method
 *	Description: Covariance between horizontal and vertical plane.(Measure)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_HVCovar(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_HVCovar(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_HVCovar) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_HVCovar_read = img8[img_cur]->covar;
	}
	else if (img16.size() > 0) {
		*attr_HVCovar_read = img16[img_cur]->covar;
	}
	else
		*attr_HVCovar_read = 0;
	in_mutex->unlock();

	attr.set_value(attr_HVCovar_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_HVCovar
}
//--------------------------------------------------------
/**
 *	Read attribute CameraCounter related method
 *	Description: Camera counter (0-65535).(Connection Diagnostics)
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_CameraCounter(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_CameraCounter(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_CameraCounter) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_CameraCounter_read = img8[img_cur]->camera_counter;
	}
	else if (img16.size() > 0) {
		*attr_CameraCounter_read = img16[img_cur]->camera_counter;
	}
	else
		*attr_CameraCounter_read = 0;
	in_mutex->unlock();

	attr.set_value(attr_CameraCounter_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_CameraCounter
}
//--------------------------------------------------------
/**
 *	Read attribute AcquisitionCounter related method
 *	Description: Absolute acquisition image counter.(Connection Diagnostics)
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_AcquisitionCounter(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_AcquisitionCounter(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_AcquisitionCounter) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_AcquisitionCounter_read = img8[img_cur]->acquisition_counter;
	}
	else if (img16.size() > 0) {
		*attr_AcquisitionCounter_read = img16[img_cur]->acquisition_counter;
	}
	else
		*attr_AcquisitionCounter_read = 0;
	in_mutex->unlock();

	attr.set_value(attr_AcquisitionCounter_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_AcquisitionCounter
}
//--------------------------------------------------------
/**
 *	Read attribute Height related method
 *	Description: Image height
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_Height(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_Height(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_Height) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_Height_read = img8[img_cur]->get_height();
	}
	else if (img16.size() > 0) {
		*attr_Height_read = img16[img_cur]->get_height();
	}
	else
		*attr_Height_read = 0;
	in_mutex->unlock();

	attr.set_value(attr_Height_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_Height
}
//--------------------------------------------------------
/**
 *	Read attribute Width related method
 *	Description: Image width
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_Width(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_Width(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_Width) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_Width_read = img8[img_cur]->get_width();
	}
	else if (img16.size() > 0) {
		*attr_Width_read = img16[img_cur]->get_width();
	}
	else
		*attr_Width_read = 0;
	in_mutex->unlock();

	attr.set_value(attr_Width_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_Width
}
//--------------------------------------------------------
/**
 *	Read attribute ProcessingTime related method
 *	Description: Processing time in us.(Processing Setup)
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_ProcessingTime(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_ProcessingTime(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_ProcessingTime) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_ProcessingTime_read = img8[img_cur]->processing_time;
	}
	else if (img16.size() > 0) {
		*attr_ProcessingTime_read = img16[img_cur]->processing_time;
	}
	else
		*attr_ProcessingTime_read = 0;
	in_mutex->unlock();

	attr.set_value(attr_ProcessingTime_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_ProcessingTime
}
//--------------------------------------------------------
/**
 *	Read attribute Frequency related method
 *	Description: Frame rate in Hz. (Camera Setup)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_Frequency(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_Frequency(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_Frequency) ENABLED START -----*/
	*attr_Frequency_read = frequency;
	attr.set_value(attr_Frequency_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_Frequency
}
//--------------------------------------------------------
/**
 *	Read attribute Trigger related method
 *	Description: Trigger mode (Camera Setup)
 *               false = internal trigger (max frame rate)
 *               true = external trigger
 *               The trigger will be on the rising edge of the pulse.
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_Trigger(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_Trigger(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_Trigger) ENABLED START -----*/


	if ((img8.size() == 0) && (img16.size() == 0))  {
		*attr_Trigger_read = false;
	}

	attr.set_value(attr_Trigger_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_Trigger
}
//--------------------------------------------------------
/**
 *	Write attribute Trigger related method
 *	Description: Trigger mode (Camera Setup)
 *               false = internal trigger (max frame rate)
 *               true = external trigger
 *               The trigger will be on the rising edge of the pulse.
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_Trigger(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_Trigger(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevBoolean	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_Trigger) ENABLED START -----*/
	bool triggermode;
	INFO_STREAM << "Gigecam::write_Trigger(): " << w_val << endl;
	
	struct timeval t1, t2;

	if (w_val)
		triggermode = true;
	else
		triggermode = false;

	cam_param[cam_idx].trigger = (bool) triggermode;
	
	gettimeofday(&t1, NULL);

	/* trigger can be configured only when acquisition is stopped */
	if (strcmp(cam_param[cam_idx].model,"Basler acA1300-60gm") == 0) {
		stop();
		while (link_flag) {
			usleep(500000); 
			gettimeofday(&t2,NULL);
			if ((t2.tv_sec-t1.tv_sec) > (0.6*GIGECAM_LOCK_TOUT)) {
				string msg_log("Failed to release acquisition thread, shutting down"); 
				info_log(msg_log);
				exit(1);
			}
		}
		start((char *)cam_param[cam_idx].name.c_str());
	}
	else if (((img8.size() > 0) || (img16.size() > 0)) && (acqloop != NULL))  {
		acqloop->CameraSetTrigger(triggermode);
	}

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_Trigger
}
//--------------------------------------------------------
/**
 *	Read attribute Exposure related method
 *	Description: Exposure time in us.(Camera Setup)
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_Exposure(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_Exposure(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_Exposure) ENABLED START -----*/
	if ((img8.size() == 0) && (img16.size() == 0))  {
		*attr_Exposure_read = 0;
	}

	attr.set_value(attr_Exposure_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_Exposure
}
//--------------------------------------------------------
/**
 *	Write attribute Exposure related method
 *	Description: Exposure time in us.(Camera Setup)
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_Exposure(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_Exposure(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevLong	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_Exposure) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_Exposure(): " << w_val << endl;

	if (((img8.size() > 0) || (img16.size() > 0)) && (acqloop != NULL))  {

		if (*attr_ImageSumSamples_read) {
			in_mutex->lock();
			if (img8.size() > 0) {
				memset(attr_ImageSum_read, 0, img8[img_cur]->get_width()*img8[img_cur]->get_height()*4);
			}
			else if (img16.size() > 0) {
				memset(attr_ImageSum_read, 0, img16[img_cur]->get_width()*img16[img_cur]->get_height()*4);
			}
			in_mutex->unlock();
			imagesum_counter = 0;
		}

		acqloop->CameraSetExposure((int32_t)w_val);
		attr_Exposure_write = w_val;
	}

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_Exposure
}
//--------------------------------------------------------
/**
 *	Read attribute Gain related method
 *	Description: Gain value in dB.
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_Gain(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_Gain(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_Gain) ENABLED START -----*/

	if ((img8.size() == 0) && (img16.size() == 0))  {
		*attr_Gain_read = 0;
	}

	attr.set_value(attr_Gain_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_Gain
}
//--------------------------------------------------------
/**
 *	Write attribute Gain related method
 *	Description: Gain value in dB.
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_Gain(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_Gain(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevDouble	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_Gain) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_Gain(): " << w_val << endl;

	if (((img8.size() > 0) || (img16.size() > 0)) && (acqloop != NULL))  {
		if (*attr_ImageSumSamples_read) {
			in_mutex->lock();
			if (img8.size() > 0) {
				memset(attr_ImageSum_read, 0, img8[img_cur]->get_width()*img8[img_cur]->get_height()*4);
			}
			else if (img16.size() > 0) {
				memset(attr_ImageSum_read, 0, img16[img_cur]->get_width()*img16[img_cur]->get_height()*4);
			}
			imagesum_counter = 0;
			in_mutex->unlock();
		}
		acqloop->CameraSetGain(w_val);
	}

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_Gain
}
//--------------------------------------------------------
/**
 *	Read attribute RoiThreshold related method
 *	Description: Scaling factor of the automatic roi threshold.
 *               Automatic roi threshol is the background level that is automatically found by autoroi routine.(Processing Setup)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_RoiThreshold(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_RoiThreshold(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_RoiThreshold) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_RoiThreshold_read = img8[img_cur]->get_roi_threshold();
	}
	else if (img16.size() > 0) {
		*attr_RoiThreshold_read  = img16[img_cur]->get_roi_threshold();
	}
	else
		*attr_RoiThreshold_read  = 0;
	in_mutex->unlock();

	attr.set_value(attr_RoiThreshold_read );

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_RoiThreshold
}
//--------------------------------------------------------
/**
 *	Write attribute RoiThreshold related method
 *	Description: Scaling factor of the automatic roi threshold.
 *               Automatic roi threshol is the background level that is automatically found by autoroi routine.(Processing Setup)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_RoiThreshold(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_RoiThreshold(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevDouble	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_RoiThreshold) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_RoiThreshold(): " << w_val << endl;

	cam_param[cam_idx].roi_threshold = w_val;

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_RoiThreshold
}
//--------------------------------------------------------
/**
 *	Read attribute AutoRoi related method
 *	Description: Enable autoroi.\nAutoroi routine find the background level and the roi region where the spot is.\nWhen swroi is enabled also, the roi is the roi defined by the user by roiparam\nattribute but the background level is automatically calculated and taken into accout\nin position and standard deviation calculations.(Processing Setup)
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_AutoRoi(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_AutoRoi(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_AutoRoi) ENABLED START -----*/
	bool attr_bool;
	in_mutex->lock();
	if (img8.size() > 0) {
		attr_bool = img8[img_cur]->get_enable_auto_roi();
	}
	else if (img16.size() > 0) {
		attr_bool = img16[img_cur]->get_enable_auto_roi();
	}
	else
		attr_bool = false;
	in_mutex->unlock();

	if (attr_bool)
		*attr_AutoRoi_read = true;
	else
		*attr_AutoRoi_read = false;

	attr.set_value(attr_AutoRoi_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_AutoRoi
}
//--------------------------------------------------------
/**
 *	Write attribute AutoRoi related method
 *	Description: Enable autoroi.\nAutoroi routine find the background level and the roi region where the spot is.\nWhen swroi is enabled also, the roi is the roi defined by the user by roiparam\nattribute but the background level is automatically calculated and taken into accout\nin position and standard deviation calculations.(Processing Setup)
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_AutoRoi(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_AutoRoi(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevBoolean	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_AutoRoi) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_Autoroi(): " << w_val << endl;
	
	if (w_val) {
		cam_param[cam_idx].auto_roi = true;
	}
	else {
		cam_param[cam_idx].auto_roi = false;
	}

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_AutoRoi
}
//--------------------------------------------------------
/**
 *	Read attribute SwRoi related method
 *	Description: Enable roi define by the user.(Display Options) 
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_SwRoi(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_SwRoi(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_SwRoi) ENABLED START -----*/
	bool attr_bool;
	in_mutex->lock();
	if (img8.size() > 0) {
		attr_bool = img8[img_cur]->get_enable_manual_roi();
	}
	else if (img16.size() > 0) {
		attr_bool= img16[img_cur]->get_enable_manual_roi();
	}	
	else
		attr_bool = false;
	in_mutex->unlock();

	if (attr_bool)
		*attr_SwRoi_read = true;
	else
		*attr_SwRoi_read = false;

	attr.set_value(attr_SwRoi_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_SwRoi
}
//--------------------------------------------------------
/**
 *	Write attribute SwRoi related method
 *	Description: Enable roi define by the user.(Display Options) 
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_SwRoi(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_SwRoi(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevBoolean	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_SwRoi) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_PlotMode(): " << w_val << endl;
	
	if (w_val) {
		cam_param[cam_idx].sw_roi = true;
	}
	else {
		cam_param[cam_idx].sw_roi = false;
	}

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_SwRoi
}
//--------------------------------------------------------
/**
 *	Read attribute HwRoi related method
 *	Description: 
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_HwRoi(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_HwRoi(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_HwRoi) ENABLED START -----*/

	*attr_HwRoi_read = cam_param[cam_idx].hw_roi;
	attr.set_value(attr_HwRoi_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_HwRoi
}
//--------------------------------------------------------
/**
 *	Write attribute HwRoi related method
 *	Description: 
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_HwRoi(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_HwRoi(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevBoolean	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_HwRoi) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_HwRoi(): " << w_val << endl;


	if ((cam_param[cam_idx].rotation != 0) && (cam_param[cam_idx].rotation != 180)) {
		Tango::Except::throw_exception(
    	(const char *) "Failed to set hardware roi",
			(const char *) "Rotation must be set to 0 or 180",
			(const char *) "GigeCam::write_HwRoi()",Tango::ERR);
	}

	// hardware ROI must be greater then 128x128 px 
	if ((strcmp(cam_param[cam_idx].model,"Basler avA1600-50gm") == 0) && (w_val == true)) {
		if (((cam_param[cam_idx].manual_roi_param[2]-cam_param[cam_idx].manual_roi_param[0]) < 128) || ((cam_param[cam_idx].manual_roi_param[3]-cam_param[cam_idx].manual_roi_param[1]) < 128)) {
			Tango::Except::throw_exception (
				(const char *)"Width/height must be greater then 128",
				(const char *)"Invalid number of parameters",
				(const char *)"Gigecam::write_HwRoi()",Tango::ERR);
		}
	}
	else if ((strcmp(cam_param[cam_idx].model,"Basler acA1300-75gm") == 0) && (w_val == true)) {
		if (((cam_param[cam_idx].manual_roi_param[2]-cam_param[cam_idx].manual_roi_param[0]) < 256) || ((cam_param[cam_idx].manual_roi_param[3]-cam_param[cam_idx].manual_roi_param[1]) < 256)) {
			Tango::Except::throw_exception (
				(const char *)"Width/height must be greater then 256",
				(const char *)"Invalid number of parameters",
				(const char *)"Gigecam::write_HwRoi()",Tango::ERR);
		}
	}

	if (((img8.size() > 0) || (img16.size() > 0)) && (w_val !=  cam_param[cam_idx].hw_roi)) {
		cam_param[cam_idx].binning = 1;

		/* disable dis to avoid crash (to be investigated) GG */
		if (img8.size() > 0) {
			for (uint32_t j = 0; j < img8.size(); j++) 
				img8[j]->set_enable_dis(false);
		}
		else if (img16.size() > 0) {
			for (uint32_t j = 0; j < img16.size(); j++) 
				img16[j]->set_enable_dis(false);
		}
		cam_param[cam_idx].dis_enable = false; 

		/* if sw_roi not defined, do not set hw roi */
		if (w_val == true) {
			bool sw_manual_roi;
			if (img8.size() > 0) {
				sw_manual_roi = img8[img_cur]->get_enable_manual_roi();
			}
			else if (img16.size() > 0) {
				sw_manual_roi = img16[img_cur]->get_enable_manual_roi();
			}	
			else
				sw_manual_roi = false;
			if (sw_manual_roi == false) {
				Tango::Except::throw_exception(
					(const char *) "Failed to set hardware roi",
					(const char *) "User defined roi disabled",
					(const char *) "GigeCam::write_HwRoi()",Tango::ERR);
			}
		}

		stop();
		/* wait the loop stopping */
		int32_t cnt = 0;
		while (link_flag) {
			usleep(500000);
			if (cnt++ > 20)
				break;
		}
		cam_param[cam_idx].hw_roi = w_val;

		if (w_val == true) {
			for (int i = 0; i < 4; i++) {
				cam_param[cam_idx].hw_roi_param[i] = cam_param[cam_idx].manual_roi_param[i];
				cam_param[cam_idx].manual_roi_param[i] = 0;
			}
		}
		else {
			cam_param[cam_idx].manual_roi_param[0] = cam_param[cam_idx].hw_roi_param[0];
			cam_param[cam_idx].manual_roi_param[2] = cam_param[cam_idx].hw_roi_param[2];
			cam_param[cam_idx].manual_roi_param[1] = cam_param[cam_idx].hw_roi_param[1];
			cam_param[cam_idx].manual_roi_param[3] = cam_param[cam_idx].hw_roi_param[3];
		}

		start((char *)cam_param[cam_idx].name.c_str());

	}

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_HwRoi
}
//--------------------------------------------------------
/**
 *	Read attribute PlotMode related method
 *	Description: Set plotting mode.(Display Options)\n0x0: plot nothing\n0x1:plot position axis and roi region\n0x2:plot spot region\n0x3: plot both
 *
 *	Data type:	Tango::DevShort
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_PlotMode(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_PlotMode(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_PlotMode) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_PlotMode_read = img8[img_cur]->get_plot_mode();
	}
	else if (img16.size() > 0) {
		*attr_PlotMode_read = img16[img_cur]->get_plot_mode();
	}	
	else
		*attr_PlotMode_read = cam_param[cam_idx].plot_mode;
	in_mutex->unlock();

	attr.set_value(attr_PlotMode_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_PlotMode
}
//--------------------------------------------------------
/**
 *	Write attribute PlotMode related method
 *	Description: Set plotting mode.(Display Options)\n0x0: plot nothing\n0x1:plot position axis and roi region\n0x2:plot spot region\n0x3: plot both
 *
 *	Data type:	Tango::DevShort
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_PlotMode(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_PlotMode(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevShort	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_PlotMode) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_PlotMode(): " << w_val << endl;
	
	cam_param[cam_idx].plot_mode = w_val;

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_PlotMode
}
//--------------------------------------------------------
/**
 *	Read attribute IconSize related method
 *	Description: Horizontal size of the decimated image used to find automatically the roi.\n(Shadow Parameters)
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_IconSize(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_IconSize(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_IconSize) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_IconSize_read = img8[img_cur]->get_imageicon_width();
	}
	else if (img16.size() > 0) {
		*attr_IconSize_read = img16[img_cur]->get_imageicon_width();
	}
	else
		*attr_IconSize_read = 0;
	in_mutex->unlock();

	attr.set_value(attr_IconSize_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_IconSize
}
//--------------------------------------------------------
/**
 *	Write attribute IconSize related method
 *	Description: Horizontal size of the decimated image used to find automatically the roi.\n(Shadow Parameters)
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_IconSize(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_IconSize(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevLong	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_IconSize) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_IconSize(): " << w_val << endl;
	
	cam_param[cam_idx].gridsize = (int32_t) w_val;

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_IconSize
}
//--------------------------------------------------------
/**
 *	Read attribute LowPassOrder related method
 *	Description: Low pass fiter order used to magnify the main spot on the camera.\n(Shadow Parameters)
 *
 *	Data type:	Tango::DevShort
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_LowPassOrder(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_LowPassOrder(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_LowPassOrder) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_LowPassOrder_read = img8[img_cur]->get_lp_order();
	}
	else if (img16.size() > 0) {
		*attr_LowPassOrder_read = img16[img_cur]->get_lp_order();
	}
	else
		*attr_LowPassOrder_read = 0;
	in_mutex->unlock();

	attr.set_value(attr_LowPassOrder_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_LowPassOrder
}
//--------------------------------------------------------
/**
 *	Write attribute LowPassOrder related method
 *	Description: Low pass fiter order used to magnify the main spot on the camera.\n(Shadow Parameters)
 *
 *	Data type:	Tango::DevShort
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_LowPassOrder(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_LowPassOrder(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevShort	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_LowPassOrder) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_LowPassOrder(): " << w_val << endl;
	
	cam_param[cam_idx].lp_order = (int32_t)  w_val;

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_LowPassOrder
}
//--------------------------------------------------------
/**
 *	Read attribute Rotation related method
 *	Description: Image rotation in degrees. (Display Options)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_Rotation(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_Rotation(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_Rotation) ENABLED START -----*/
	*attr_Rotation_read = cam_param[cam_idx].rotation;
	attr.set_value(attr_Rotation_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_Rotation
}
//--------------------------------------------------------
/**
 *	Write attribute Rotation related method
 *	Description: Image rotation in degrees. (Display Options)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_Rotation(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_Rotation(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevDouble	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_Rotation) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_Rotation(): " << w_val << endl;
	
	in_mutex->lock();
	if (w_val != cam_param[cam_idx].rotation) {
		cam_param[cam_idx].rotation = w_val;
		/* 8 bit */
		if (img8.size() > 0) {
			/* reset permutation vector */
			for (int32_t i = 0; i < img8[0]->get_height()*img8[0]->get_width(); i++) 
				cam_param[cam_idx].img_permutation[i] = i;
			if (cam_param[cam_idx].mirror)
				img8[0]->permutation_mirrorh(cam_param[cam_idx].img_permutation);
			if (cam_param[cam_idx].rotation != 0)
				img8[0]->permutation_rotation(cam_param[cam_idx].rotation,cam_param[cam_idx].img_permutation);
			for (uint32_t i = 0; i < img8.size(); i++) {
				img8[i]->reset();
			}
		}
		else if (img16.size() > 0) {
			/* reset permutation vector */
			for (int32_t i = 0; i < img16[0]->get_height()*img16[0]->get_width(); i++) 
				cam_param[cam_idx].img_permutation[i] = i;
			if (cam_param[cam_idx].mirror)
				img16[0]->permutation_mirrorh(cam_param[cam_idx].img_permutation);
			if (cam_param[cam_idx].rotation != 0)
				img16[0]->permutation_rotation(cam_param[cam_idx].rotation,cam_param[cam_idx].img_permutation);
			for (uint32_t i = 0; i < img16.size(); i++) {
				img16[i]->reset();
			}
		}
	}

	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_Rotation
}
//--------------------------------------------------------
/**
 *	Read attribute Mirror related method
 *	Description: Mirror the image (true) on the horizontal plane. (Display Options)
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_Mirror(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_Mirror(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_Mirror) ENABLED START -----*/
	bool attr_bool;

	in_mutex->lock();
	if (img8.size() > 0) {
		attr_bool = img8[img_cur]->get_mirror();
	}
	else if (img16.size() > 0) {
		attr_bool = img16[img_cur]->get_mirror();
	}
	else
		attr_bool = false;
	in_mutex->unlock();

	if (attr_bool)
		*attr_Mirror_read = true;
	else
		*attr_Mirror_read = false;

	attr.set_value(attr_Mirror_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_Mirror
}
//--------------------------------------------------------
/**
 *	Write attribute Mirror related method
 *	Description: Mirror the image (true) on the horizontal plane. (Display Options)
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_Mirror(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_Mirror(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevBoolean	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_Mirror) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_Mirror(): " << w_val << endl;
	
	in_mutex->lock();
	if (w_val != cam_param[cam_idx].mirror) {
		if (w_val) 
			cam_param[cam_idx].mirror = true;
		else 
			cam_param[cam_idx].mirror = false;

		if (img8.size() > 0) {
			/* reset permutation vector */
			for (int32_t i = 0; i < img8[0]->get_height()*img8[0]->get_width(); i++) 
				cam_param[cam_idx].img_permutation[i] = i;

			if (cam_param[cam_idx].mirror)
				img8[0]->permutation_mirrorh(cam_param[cam_idx].img_permutation);
			if (cam_param[cam_idx].rotation != 0)
				img8[0]->permutation_rotation(cam_param[cam_idx].rotation,cam_param[cam_idx].img_permutation);

			for (uint32_t i = 0; i < img8.size(); i++) {
				img8[i]->reset();
			}

		}
		else if (img16.size() > 0) {
			/* reset permutation vector */
			for (int32_t i = 0; i < img16[0]->get_height()*img16[0]->get_width(); i++) 
				cam_param[cam_idx].img_permutation[i] = i;

			if (cam_param[cam_idx].mirror)
				img16[0]->permutation_mirrorh(cam_param[cam_idx].img_permutation);
			if (cam_param[cam_idx].rotation != 0)
				img16[0]->permutation_rotation(cam_param[cam_idx].rotation,cam_param[cam_idx].img_permutation);

			for (uint32_t i = 0; i < img16.size(); i++) {
				img16[i]->reset();
			}

		}
	}

	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_Mirror
}
//--------------------------------------------------------
/**
 *	Read attribute ImageDepth related method
 *	Description: Image depth. Valide values are 8 o 16.(Camera Setup)
 *
 *	Data type:	Tango::DevShort
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_ImageDepth(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_ImageDepth(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_ImageDepth) ENABLED START -----*/
	if ((img8.size() == 0) && (img16.size() == 0))  {
		*attr_ImageDepth_read = 0;
	}
	attr.set_value(attr_ImageDepth_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_ImageDepth
}
//--------------------------------------------------------
/**
 *	Write attribute ImageDepth related method
 *	Description: Image depth. Valide values are 8 o 16.(Camera Setup)
 *
 *	Data type:	Tango::DevShort
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_ImageDepth(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_ImageDepth(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevShort	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_ImageDepth) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_ImageDepth(): " << w_val << endl;


	if (((img8.size() > 0) || (img16.size() > 0)) && (acqloop != NULL))  {
		if ((w_val == 8) || (w_val == 16)) {
			stop();
			/* wait the loop stopping */
			int32_t cnt = 0;
			while (link_flag) {
				usleep(500000);
				if (cnt++ > 20)
					break;
			}
			/*
			if ((img8.size() == 0) && (img16.size() == 0)) {
				cam_param[cam_idx].img_depth = w_val;
				acqloop = new acqthread::acqthread(this);
				acqloop->start();
			}
			*/
			cam_param[cam_idx].img_depth = w_val;
			start((char *)cam_param[cam_idx].name.c_str());
		}
	}

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_ImageDepth
}
//--------------------------------------------------------
/**
 *	Read attribute BackgroundSubtraction related method
 *	Description: Enable (true) background subtraction. (Processing Setup)
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_BackgroundSubtraction(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_BackgroundSubtraction(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_BackgroundSubtraction) ENABLED START -----*/
	bool attr_bool;

	in_mutex->lock();
	if (img8.size() > 0) {
		attr_bool = cam_param[cam_idx].background_subtraction;
	}
	else if (img16.size() > 0) {
		attr_bool = cam_param[cam_idx].background_subtraction;
	}
	else
		attr_bool = false;
	in_mutex->unlock();

	if (attr_bool)
		*attr_BackgroundSubtraction_read = true;
	else
		*attr_BackgroundSubtraction_read = false;

	attr.set_value(attr_BackgroundSubtraction_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_BackgroundSubtraction
}
//--------------------------------------------------------
/**
 *	Write attribute BackgroundSubtraction related method
 *	Description: Enable (true) background subtraction. (Processing Setup)
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_BackgroundSubtraction(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_BackgroundSubtraction(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevBoolean	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_BackgroundSubtraction) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_BackgroundSubtraction(): " << w_val << endl;

	if (w_val) {
		cam_param[cam_idx].background_subtraction = true;
	}
	else {
		cam_param[cam_idx].background_subtraction = false;
	}

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_BackgroundSubtraction
}
//--------------------------------------------------------
/**
 *	Read attribute EnableProcessing related method
 *	Description: Enable (true) image processing.(Processing Setup)
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_EnableProcessing(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_EnableProcessing(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_EnableProcessing) ENABLED START -----*/
	bool attr_bool;

	in_mutex->lock();
	if (img8.size() > 0) {
		attr_bool = img8[img_cur]->get_enable_process();
	}
	else if (img16.size() > 0) {
		attr_bool = img16[img_cur]->get_enable_process();
	}
	else
		attr_bool = 0;
	in_mutex->unlock();

	if (attr_bool)
		*attr_EnableProcessing_read = true;
	else
		*attr_EnableProcessing_read = false;

	attr.set_value(attr_EnableProcessing_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_EnableProcessing
}
//--------------------------------------------------------
/**
 *	Write attribute EnableProcessing related method
 *	Description: Enable (true) image processing.(Processing Setup)
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_EnableProcessing(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_EnableProcessing(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevBoolean	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_EnableProcessing) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_EnableProcessing(): " << w_val << endl;
	
	if (w_val) {
		cam_param[cam_idx].process_enable = true;
	}
	else {
		cam_param[cam_idx].process_enable = false;
	}

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_EnableProcessing
}
//--------------------------------------------------------
/**
 *	Read attribute FastProcessing related method
 *	Description: Enable fast processing.\nImage processing will be done automatically on a \ndecimate (binning) image to reduce computation time. (Processing Setup)
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_FastProcessing(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_FastProcessing(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_FastProcessing) ENABLED START -----*/
	bool attr_bool;

	in_mutex->lock();
	if (img8.size() > 0) {
		attr_bool = img8[img_cur]->get_enable_fast_calc();
	}
	else if (img16.size() > 0) {
		attr_bool = img16[img_cur]->get_enable_fast_calc();
	}
	else
		attr_bool = false;
	in_mutex->unlock();

	if (attr_bool)
		*attr_FastProcessing_read = true;
	else
		*attr_FastProcessing_read = false;

	attr.set_value(attr_FastProcessing_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_FastProcessing
}
//--------------------------------------------------------
/**
 *	Write attribute FastProcessing related method
 *	Description: Enable fast processing.\nImage processing will be done automatically on a \ndecimate (binning) image to reduce computation time. (Processing Setup)
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_FastProcessing(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_FastProcessing(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevBoolean	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_FastProcessing) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_FastProcessing(): " << w_val << endl;
	
	if (w_val) {
		cam_param[cam_idx].fast_calc = true;
	}
	else {
		cam_param[cam_idx].fast_calc = false;
	}

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_FastProcessing
}
//--------------------------------------------------------
/**
 *	Read attribute Saturation related method
 *	Description: Ratio between the number of the pixels that reached the\nmaximum value and the number of the pixels of the spot.(Measure)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_Saturation(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_Saturation(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_Saturation) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_Saturation_read = img8[img_cur]->saturation;
	}
	else if (img16.size() > 0) {
		*attr_Saturation_read = img16[img_cur]->saturation;
	}
	else
		*attr_Saturation_read = 0;
	in_mutex->unlock();

	attr.set_value(attr_Saturation_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_Saturation
}
//--------------------------------------------------------
/**
 *	Read attribute PlotLevel related method
 *	Description: Color of the plots (0-255 8bit) (0-4095 16bit).(Display Options)
 *
 *	Data type:	Tango::DevShort
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_PlotLevel(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_PlotLevel(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_PlotLevel) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_PlotLevel_read = img8[img_cur]->get_plot_level();
	}
	else if (img16.size() > 0) {
		*attr_PlotLevel_read = img16[img_cur]->get_plot_level();
	}
	else
		*attr_PlotLevel_read = cam_param[cam_idx].plot_level;
	in_mutex->unlock();

	attr.set_value(attr_PlotLevel_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_PlotLevel
}
//--------------------------------------------------------
/**
 *	Write attribute PlotLevel related method
 *	Description: Color of the plots (0-255 8bit) (0-4095 16bit).(Display Options)
 *
 *	Data type:	Tango::DevShort
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_PlotLevel(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_PlotLevel(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevShort	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_PlotLevel) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_PlotLevel(): " << w_val << endl;

	cam_param[cam_idx].plot_level = (short) w_val;

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_PlotLevel
}
//--------------------------------------------------------
/**
 *	Read attribute Pause related method
 *	Description: Pause the image acquisition but don't give up the connection with the camera. (Camera Control)
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_Pause(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_Pause(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_Pause) ENABLED START -----*/
	*attr_Pause_read = pause_flag;
	attr.set_value(attr_Pause_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_Pause
}
//--------------------------------------------------------
/**
 *	Write attribute Pause related method
 *	Description: Pause the image acquisition but don't give up the connection with the camera. (Camera Control)
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_Pause(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_Pause(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevBoolean	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_Pause) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_Pause(): " << w_val << endl;
	
	if (link_flag) {
		if (w_val) {
			pause_flag = true;
		}
		else {
			pause_flag = false;
		}
	}

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_Pause
}
//--------------------------------------------------------
/**
 *	Read attribute Label related method
 *	Description: Label of the camera currently acquired. It corresponds to the first element of the row\nof the table which contains the camera initialization paramenters.(Camera Control)\n
 *
 *	Data type:	Tango::DevString
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_Label(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_Label(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_Label) ENABLED START -----*/
	if (link_flag) {
		strncpy(*attr_Label_read, cam_param[cam_idx].name.c_str(), GIGECAM_DESC);
	}
	else {
		*attr_Label_read[0] = 0;;
	}
	attr.set_value(attr_Label_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_Label
}
//--------------------------------------------------------
/**
 *	Read attribute HorPosRelative related method
 *	Description: Horizontal relative position of the spot int the image. (Measure)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_HorPosRelative(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_HorPosRelative(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_HorPosRelative) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		if (img8[img_cur]->ref_hor_pos_offset != 0)
			*attr_HorPosRelative_read = -((double) img8[img_cur]->hor_pos-img8[img_cur]->ref_hor_pos_offset + 2 * img8[0]->hor_pos_offset)*img8[img_cur]->hor_sign;
		else
			*attr_HorPosRelative_read = 0;
	}
	else if (img16.size() > 0) {
		if (img16[img_cur]->ref_hor_pos_offset != 0)
			*attr_HorPosRelative_read = -((double) img16[img_cur]->hor_pos-img16[img_cur]->ref_hor_pos_offset +  2 * img16[0]->hor_pos_offset)*img8[img_cur]->hor_sign;
		else
			*attr_HorPosRelative_read = 0;
	}
	else
		*attr_HorPosRelative_read = 0;
	in_mutex->unlock();

	attr.set_value(attr_HorPosRelative_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_HorPosRelative
}
//--------------------------------------------------------
/**
 *	Read attribute VerPosRelative related method
 *	Description: Vertical relative position of the spot int the image. (Measure)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_VerPosRelative(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_VerPosRelative(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_VerPosRelative) ENABLED START -----*/
	in_mutex->lock();


	if (img8.size() > 0) {
		if (img8[img_cur]->ref_ver_pos_offset != 0)
			*attr_VerPosRelative_read = -((double) img8[img_cur]->ver_pos-img8[img_cur]->ref_ver_pos_offset) * img8[img_cur]->ver_sign;
		else
			*attr_VerPosRelative_read = 0;
	}
	else if (img16.size() > 0) {
		if (img16[img_cur]->ref_ver_pos_offset != 0)
			*attr_VerPosRelative_read = -((double) img16[img_cur]->ver_pos-img16[img_cur]->ref_ver_pos_offset) * img8[img_cur]->ver_sign;
		else
			*attr_VerPosRelative_read = 0;
	}
	else
		*attr_VerPosRelative_read = 0;
	in_mutex->unlock();

	attr.set_value(attr_VerPosRelative_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_VerPosRelative
}
//--------------------------------------------------------
/**
 *	Read attribute Phase related method
 *	Description: Inclination in degrees of the image.(Measure)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_Phase(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_Phase(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_Phase) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_Phase_read = (double) img8[img_cur]->phase;
	}
	else if (img16.size() > 0) {
		*attr_Phase_read = (double) img16[img_cur]->phase;
	}
	else
		*attr_Phase_read = 0;
	in_mutex->unlock();

	attr.set_value(attr_Phase_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_Phase
}
//--------------------------------------------------------
/**
 *	Read attribute ScaleHor related method
 *	Description: Set the number of pixels per mm in the horizontal plane
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_ScaleHor(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_ScaleHor(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_ScaleHor) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_ScaleHor_read = (double) img8[img_cur]->hor_calibration  / (double) cam_param[cam_idx].binning;
	}
	else if (img16.size() > 0) {
		*attr_ScaleHor_read = (double) img16[img_cur]->hor_calibration / (double) cam_param[cam_idx].binning;
	}
	else {
		*attr_ScaleHor_read = cam_param[cam_idx].hor_calibration;
	}
	in_mutex->unlock();

	attr.set_value(attr_ScaleHor_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_ScaleHor
}
//--------------------------------------------------------
/**
 *	Write attribute ScaleHor related method
 *	Description: Set the number of pixels per mm in the horizontal plane
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_ScaleHor(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_ScaleHor(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevDouble	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_ScaleHor) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_ScaleHor(): " << w_val << endl;

	if (cam_param[cam_idx].binning != 1) {
		Tango::Except::throw_exception(
			(const char *) "Binning must be set to 1",
			(const char *) "Failed to set horizontal scale factor",
			(const char *) "GigeCam::write_ScaleHor()",Tango::ERR);
	}

	in_mutex->lock();

	cam_param[cam_idx].hor_calibration = w_val;
	if (img8.size() > 0) {
		for (uint32_t j = 0; j < img8.size(); j++) {
			img8[j]->hor_calibration = w_val;
		}
	}
	else if (img16.size() > 0) {
		for (uint32_t j = 0; j < img16.size(); j++) {
			img16[j]->hor_calibration = w_val;
		}
	}

	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_ScaleHor
}
//--------------------------------------------------------
/**
 *	Read attribute ScaleVer related method
 *	Description: Set the number of pixels per mm in the vertical plane
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_ScaleVer(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_ScaleVer(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_ScaleVer) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_ScaleVer_read = (double) img8[img_cur]->ver_calibration / (double) cam_param[cam_idx].binning;
	}
	else if (img16.size() > 0) {
		*attr_ScaleVer_read = (double) img16[img_cur]->ver_calibration  / (double) cam_param[cam_idx].binning;
	}
	else {
		*attr_ScaleVer_read = cam_param[cam_idx].ver_calibration;
	}
	in_mutex->unlock();

	attr.set_value(attr_ScaleVer_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_ScaleVer
}
//--------------------------------------------------------
/**
 *	Write attribute ScaleVer related method
 *	Description: Set the number of pixels per mm in the vertical plane
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_ScaleVer(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_ScaleVer(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevDouble	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_ScaleVer) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_ScaleVer(): " << w_val << endl;

	if (cam_param[cam_idx].binning != 1) {
		Tango::Except::throw_exception(
			(const char *) "Binning must be set to 1",
			(const char *) "Failed to set vertical scale factor",
			(const char *) "GigeCam::write_ScaleVer()",Tango::ERR);
	}

	in_mutex->lock();
	cam_param[cam_idx].ver_calibration = w_val;
	for (uint32_t j = 0; j < img8.size(); j++) {
		img8[j]->ver_calibration = w_val;
	}
	for (uint32_t j = 0; j < img16.size(); j++) {
		img16[j]->ver_calibration = w_val;
	}
	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_ScaleVer
}
//--------------------------------------------------------
/**
 *	Read attribute OffsetHor related method
 *	Description: Horizontal mechanical offset in mm
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_OffsetHor(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_OffsetHor(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_OffsetHor) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_OffsetHor_read = (double) img8[img_cur]->hor_pos_offset*cam_param[cam_idx].hor_sign;
	}
	else if (img16.size() > 0) {
		*attr_OffsetHor_read = (double) img16[img_cur]->hor_pos_offset*cam_param[cam_idx].hor_sign;
	}
	else {
		*attr_OffsetHor_read = cam_param[cam_idx].hor_pos_offset;
	}

	in_mutex->unlock();

	attr.set_value(attr_OffsetHor_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_OffsetHor
}
//--------------------------------------------------------
/**
 *	Write attribute OffsetHor related method
 *	Description: Horizontal mechanical offset in mm
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_OffsetHor(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_OffsetHor(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevDouble	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_OffsetHor) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_OffsetHor(): " << w_val << endl;

	w_val *= cam_param[cam_idx].hor_sign;

	in_mutex->lock();
	cam_param[cam_idx].delta_ref_hor_pos_offset -= (cam_param[cam_idx].hor_pos_offset*cam_param[cam_idx].hor_sign - w_val);
	cam_param[cam_idx].hor_pos_offset = w_val*cam_param[cam_idx].hor_sign;

	if (img8.size() > 0) {
		for (uint32_t j = 0; j < img8.size(); j++) {
			if (img8[j]->ref_hor_pos_offset != 0) {
				img8[j]->ref_hor_pos_offset -= (img8[j]->hor_pos_offset - w_val);
			}
			img8[j]->hor_pos_offset = w_val;
		}
	}
	else if (img16.size() > 0) {
		for (uint32_t j = 0; j < img16.size(); j++) {
			if (img16[j]->ref_hor_pos_offset != 0) {
				img16[j]->ref_hor_pos_offset -= (img16[j]->hor_pos_offset - w_val);
			}
			img16[j]->hor_pos_offset = w_val;
		}
	}
	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_OffsetHor
}
//--------------------------------------------------------
/**
 *	Read attribute OffsetVer related method
 *	Description: Vertical mechanical offset in mm
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_OffsetVer(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_OffsetVer(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_OffsetVer) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_OffsetVer_read = (double) img8[img_cur]->ver_pos_offset;
	}
	else if (img16.size() > 0) {
		*attr_OffsetVer_read = (double) img16[img_cur]->ver_pos_offset;
	}
	else {
		*attr_OffsetVer_read = cam_param[cam_idx].ver_pos_offset;
	}
	in_mutex->unlock();

	attr.set_value(attr_OffsetVer_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_OffsetVer
}
//--------------------------------------------------------
/**
 *	Write attribute OffsetVer related method
 *	Description: Vertical mechanical offset in mm
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_OffsetVer(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_OffsetVer(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevDouble	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_OffsetVer) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_OffsetVer(): " << w_val << endl;

	in_mutex->lock();
	cam_param[cam_idx].delta_ref_ver_pos_offset -= (cam_param[cam_idx].ver_pos_offset - w_val);
	cam_param[cam_idx].ver_pos_offset = w_val;

	if (img8.size() > 0) {
		for (uint32_t j = 0; j < img8.size(); j++) {
			if (img8[j]->ref_ver_pos_offset != 0) {
				img8[j]->ref_ver_pos_offset +=  (img8[j]->ver_pos_offset - w_val)*img8[j]->ver_sign;
			}
			img8[j]->ver_pos_offset = w_val;	
		}
	}
	else if (img16.size() > 0) {
		for (uint32_t j = 0; j < img16.size(); j++) {
			if (img16[j]->ref_ver_pos_offset != 0) {
				img16[j]->ref_ver_pos_offset += (img16[j]->ver_pos_offset - w_val)*img8[j]->ver_sign;
			}
			img16[j]->ver_pos_offset = w_val;	
		}
	}
	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_OffsetVer
}
//--------------------------------------------------------
/**
 *	Read attribute PlotOffsetAxis related method
 *	Description: Plot reference position axis. Reference position coordinates are (OffsetHor,OffsetVer)
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_PlotOffsetAxis(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_PlotOffsetAxis(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_PlotOffsetAxis) ENABLED START -----*/
	bool attr_bool;

	in_mutex->lock();
	if (img8.size() > 0) {
		attr_bool = img8[img_cur]->get_plot_offset_axis();
	}
	else if (img16.size() > 0) {
		attr_bool = img16[img_cur]->get_plot_offset_axis();
	}
	else
		attr_bool = false;
	in_mutex->unlock();

	if (attr_bool)
		*attr_PlotOffsetAxis_read = true;
	else
		*attr_PlotOffsetAxis_read = false;

	attr.set_value(attr_PlotOffsetAxis_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_PlotOffsetAxis
}
//--------------------------------------------------------
/**
 *	Write attribute PlotOffsetAxis related method
 *	Description: Plot reference position axis. Reference position coordinates are (OffsetHor,OffsetVer)
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_PlotOffsetAxis(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_PlotOffsetAxis(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevBoolean	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_PlotOffsetAxis) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_PlotOffsetAxis(): " << w_val << endl;
	
	if (w_val) {
		cam_param[cam_idx].plot_offset_axis = true;
	}
	else {
		cam_param[cam_idx].plot_offset_axis = false;
	}

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_PlotOffsetAxis
}
//--------------------------------------------------------
/**
 *	Read attribute Intensity related method
 *	Description: Sum of pixel values inside the roi
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_Intensity(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_Intensity(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_Intensity) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_Intensity_read = img8[img_cur]->intensity;
	}
	else if (img16.size() > 0) {
		*attr_Intensity_read = img16[img_cur]->intensity;
	}
	else
		*attr_Intensity_read = 0;
	in_mutex->unlock();

	attr.set_value(attr_Intensity_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_Intensity
}
//--------------------------------------------------------
/**
 *	Read attribute SignHor related method
 *	Description: Horizontal versus of the axis (1,-1)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_SignHor(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_SignHor(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_SignHor) ENABLED START -----*/
	
	*attr_SignHor_read = cam_param[cam_idx].hor_sign;
	attr.set_value(attr_SignHor_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_SignHor
}
//--------------------------------------------------------
/**
 *	Read attribute SignVer related method
 *	Description: Vertical versus of the axis (1,-1)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_SignVer(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_SignVer(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_SignVer) ENABLED START -----*/

	*attr_SignVer_read = cam_param[cam_idx].ver_sign;
	attr.set_value(attr_SignVer_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_SignVer
}
//--------------------------------------------------------
/**
 *	Read attribute ProcessMode related method
 *	Description: Processing mode:\n0=raw rms\n1=gaussian fit\n2=asymmetric gaussian fit\n3=confiteor
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_ProcessMode(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_ProcessMode(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_ProcessMode) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_ProcessMode_read = img8[img_cur]->get_algorithm();
	}
	else if (img16.size() > 0) {
		*attr_ProcessMode_read = img16[img_cur]->get_algorithm();
	}
	else
		*attr_ProcessMode_read = 0;
	in_mutex->unlock();

	attr.set_value(attr_ProcessMode_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_ProcessMode
}
//--------------------------------------------------------
/**
 *	Write attribute ProcessMode related method
 *	Description: Processing mode:\n0=raw rms\n1=gaussian fit\n2=asymmetric gaussian fit\n3=confiteor
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_ProcessMode(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_ProcessMode(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevLong	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_ProcessMode) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_ProcessMode(): " << w_val << endl;

	cam_param[cam_idx].process_mode = w_val;

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_ProcessMode
}
//--------------------------------------------------------
/**
 *	Read attribute DebugThread related method
 *	Description: 
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_DebugThread(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_DebugThread(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_DebugThread) ENABLED START -----*/
	attr.set_value(attr_DebugThread_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_DebugThread
}
//--------------------------------------------------------
/**
 *	Read attribute BlackLevel related method
 *	Description: 
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_BlackLevel(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_BlackLevel(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_BlackLevel) ENABLED START -----*/

	//	Set the attribute value
	if ((img8.size() == 0) && (img16.size() == 0))  
		*attr_BlackLevel_read = 0;

	attr.set_value(attr_BlackLevel_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_BlackLevel
}
//--------------------------------------------------------
/**
 *	Write attribute BlackLevel related method
 *	Description: 
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_BlackLevel(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_BlackLevel(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevDouble	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_BlackLevel) ENABLED START -----*/
	if (((img8.size() > 0) || (img16.size() > 0)) && (acqloop != NULL))  {
		acqloop->CameraSetBlackLevel(w_val);
	}

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_BlackLevel
}
//--------------------------------------------------------
/**
 *	Read attribute ImageBufferSize related method
 *	Description: Set image circular buffer size
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_ImageBufferSize(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_ImageBufferSize(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_ImageBufferSize) ENABLED START -----*/

	//	Set the attribute value
	*attr_ImageBufferSize_read = imageBufferSize;
	attr.set_value(attr_ImageBufferSize_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_ImageBufferSize
}
//--------------------------------------------------------
/**
 *	Write attribute ImageBufferSize related method
 *	Description: Set image circular buffer size
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_ImageBufferSize(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_ImageBufferSize(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevLong	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_ImageBufferSize) ENABLED START -----*/
	

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_ImageBufferSize
}
//--------------------------------------------------------
/**
 *	Read attribute ImageAcquisitionCounter related method
 *	Description: 
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_ImageAcquisitionCounter(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_ImageAcquisitionCounter(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_ImageAcquisitionCounter) ENABLED START -----*/

	//	Set the attribute value
	*attr_ImageAcquisitionCounter_read = attr_ImageAcquisitionCounter_write;
	attr.set_value(attr_ImageAcquisitionCounter_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_ImageAcquisitionCounter
}
//--------------------------------------------------------
/**
 *	Write attribute ImageAcquisitionCounter related method
 *	Description: 
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_ImageAcquisitionCounter(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_ImageAcquisitionCounter(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevLong	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_ImageAcquisitionCounter) ENABLED START -----*/
	attr_ImageAcquisitionCounter_write = w_val;
	

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_ImageAcquisitionCounter
}
//--------------------------------------------------------
/**
 *	Read attribute ExposureAutoEnable related method
 *	Description: 
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_ExposureAutoEnable(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_ExposureAutoEnable(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_ExposureAutoEnable) ENABLED START -----*/

	//	Set the attribute value
	*attr_ExposureAutoEnable_read = cam_param[cam_idx].exposure_auto;
	attr.set_value(attr_ExposureAutoEnable_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_ExposureAutoEnable
}
//--------------------------------------------------------
/**
 *	Write attribute ExposureAutoEnable related method
 *	Description: 
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_ExposureAutoEnable(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_ExposureAutoEnable(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevBoolean	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_ExposureAutoEnable) ENABLED START -----*/
	if (w_val != cam_param[cam_idx].exposure_auto) {
		if (w_val == true)  {
			/* disable gain loop */
			cam_param[cam_idx].gain_auto = false;
		}
		cam_param[cam_idx].exposure_auto_cnt = 0;
		cam_param[cam_idx].exposure_auto = w_val;
	}
	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_ExposureAutoEnable
}
//--------------------------------------------------------
/**
 *	Read attribute ExposureAutoMin related method
 *	Description: 
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_ExposureAutoMin(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_ExposureAutoMin(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_ExposureAutoMin) ENABLED START -----*/

	//	Set the attribute value
	*attr_ExposureAutoMin_read = cam_param[cam_idx].exposure_auto_min;
	attr.set_value(attr_ExposureAutoMin_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_ExposureAutoMin
}
//--------------------------------------------------------
/**
 *	Write attribute ExposureAutoMin related method
 *	Description: 
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_ExposureAutoMin(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_ExposureAutoMin(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevLong	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_ExposureAutoMin) ENABLED START -----*/
	if (w_val < cam_param[cam_idx].gain_auto_min) {
		Tango::Except::throw_exception (
			(const char *)"Failed to set exposure auto min",
			(const char *)"Exposure higher than exposure auto max",
			(const char *)"GigeCam::write_ExposureAutoMin()",Tango::ERR);
	}
	cam_param[cam_idx].exposure_auto_min = w_val;
	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_ExposureAutoMin
}
//--------------------------------------------------------
/**
 *	Read attribute ExposureAutoMax related method
 *	Description: 
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_ExposureAutoMax(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_ExposureAutoMax(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_ExposureAutoMax) ENABLED START -----*/

	//	Set the attribute value
	*attr_ExposureAutoMax_read = cam_param[cam_idx].exposure_auto_max;
	attr.set_value(attr_ExposureAutoMax_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_ExposureAutoMax
}
//--------------------------------------------------------
/**
 *	Write attribute ExposureAutoMax related method
 *	Description: 
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_ExposureAutoMax(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_ExposureAutoMax(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevLong	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_ExposureAutoMax) ENABLED START -----*/
	if (w_val < cam_param[cam_idx].exposure_auto_min) {
		Tango::Except::throw_exception (
			(const char *)"Failed to set exposure auto max",
			(const char *)"Exposure lower than exposure auto min",
			(const char *)"GigeCam::write_ExposureAutoMax()",Tango::ERR);
	}
	cam_param[cam_idx].exposure_auto_max = w_val;

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_ExposureAutoMax
}
//--------------------------------------------------------
/**
 *	Read attribute GainAutoEnable related method
 *	Description: 
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_GainAutoEnable(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_GainAutoEnable(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_GainAutoEnable) ENABLED START -----*/

	//	Set the attribute value
	*attr_GainAutoEnable_read = cam_param[cam_idx].gain_auto;
	attr.set_value(attr_GainAutoEnable_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_GainAutoEnable
}
//--------------------------------------------------------
/**
 *	Write attribute GainAutoEnable related method
 *	Description: 
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_GainAutoEnable(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_GainAutoEnable(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevBoolean	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_GainAutoEnable) ENABLED START -----*/
	if (w_val != cam_param[cam_idx].gain_auto) {
		if (w_val == true)  {
			/* disable exposure loop */
			cam_param[cam_idx].exposure_auto = false;
		}
		cam_param[cam_idx].gain_auto_cnt = 0;
		cam_param[cam_idx].gain_auto = w_val;
	}
	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_GainAutoEnable
}
//--------------------------------------------------------
/**
 *	Read attribute GainAutoMin related method
 *	Description: 
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_GainAutoMin(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_GainAutoMin(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_GainAutoMin) ENABLED START -----*/

	//	Set the attribute value
	*attr_GainAutoMin_read = cam_param[cam_idx].gain_auto_min;
	attr.set_value(attr_GainAutoMin_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_GainAutoMin
}
//--------------------------------------------------------
/**
 *	Write attribute GainAutoMin related method
 *	Description: 
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_GainAutoMin(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_GainAutoMin(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevDouble	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_GainAutoMin) ENABLED START -----*/
	if (w_val > cam_param[cam_idx].gain_auto_max) {
		Tango::Except::throw_exception (
			(const char *)"Failed to set gain auto min",
			(const char *)"Gain higher than gain auto max",
			(const char *)"GigeCam::write_GainAutoMin()",Tango::ERR);
	}
	cam_param[cam_idx].gain_auto_min = w_val;
	cam_param[cam_idx].gain_auto_raw_min = 190;
	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_GainAutoMin
}
//--------------------------------------------------------
/**
 *	Read attribute GainAutoMax related method
 *	Description: 
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_GainAutoMax(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_GainAutoMax(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_GainAutoMax) ENABLED START -----*/

	//	Set the attribute value
	*attr_GainAutoMax_read = cam_param[cam_idx].gain_auto_max;
	attr.set_value(attr_GainAutoMax_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_GainAutoMax
}
//--------------------------------------------------------
/**
 *	Write attribute GainAutoMax related method
 *	Description: 
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_GainAutoMax(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_GainAutoMax(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevDouble	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_GainAutoMax) ENABLED START -----*/
	if (w_val < cam_param[cam_idx].gain_auto_min) {
		Tango::Except::throw_exception (
			(const char *)"Failed to set gain auto max",
			(const char *)"Gain lower than gain auto min",
			(const char *)"GigeCam::write_GainAutoMax()",Tango::ERR);
	}
	cam_param[cam_idx].gain_auto_raw_max = 1023;
	cam_param[cam_idx].gain_auto_max = w_val;
	
	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_GainAutoMax
}
//--------------------------------------------------------
/**
 *	Read attribute AutoTargetValue related method
 *	Description: 
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_AutoTargetValue(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_AutoTargetValue(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_AutoTargetValue) ENABLED START -----*/

	//	Set the attribute value
	*attr_AutoTargetValue_read = cam_param[cam_idx].auto_target_value;
	attr.set_value(attr_AutoTargetValue_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_AutoTargetValue
}
//--------------------------------------------------------
/**
 *	Write attribute AutoTargetValue related method
 *	Description: 
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_AutoTargetValue(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_AutoTargetValue(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevLong	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_AutoTargetValue) ENABLED START -----*/
	if (cam_param[cam_idx].auto_feedback_target_thres > w_val) {
		Tango::Except::throw_exception (
			(const char *)"Failed to set target value",
			(const char *)"Target value lower then threshold",
			(const char *)"write_AutoFeedbackTargetThres()",Tango::ERR);		
	}
	cam_param[cam_idx].auto_target_value = w_val;

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_AutoTargetValue
}
//--------------------------------------------------------
/**
 *	Read attribute AutoFeedbackGain related method
 *	Description: 
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_AutoFeedbackGain(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_AutoFeedbackGain(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_AutoFeedbackGain) ENABLED START -----*/

	//	Set the attribute value
	*attr_AutoFeedbackGain_read = cam_param[cam_idx].auto_feedback_gain;
	attr.set_value(attr_AutoFeedbackGain_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_AutoFeedbackGain
}
//--------------------------------------------------------
/**
 *	Write attribute AutoFeedbackGain related method
 *	Description: 
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_AutoFeedbackGain(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_AutoFeedbackGain(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevDouble	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_AutoFeedbackGain) ENABLED START -----*/
	cam_param[cam_idx].auto_feedback_gain = w_val;
	

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_AutoFeedbackGain
}
//--------------------------------------------------------
/**
 *	Read attribute AutoFeedbackDws related method
 *	Description: 
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_AutoFeedbackDws(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_AutoFeedbackDws(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_AutoFeedbackDws) ENABLED START -----*/

	//	Set the attribute value
	*attr_AutoFeedbackDws_read = cam_param[cam_idx].auto_feedback_dws;
	attr.set_value(attr_AutoFeedbackDws_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_AutoFeedbackDws
}
//--------------------------------------------------------
/**
 *	Write attribute AutoFeedbackDws related method
 *	Description: 
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_AutoFeedbackDws(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_AutoFeedbackDws(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevLong	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_AutoFeedbackDws) ENABLED START -----*/
	cam_param[cam_idx].auto_feedback_dws = w_val;

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_AutoFeedbackDws
}
//--------------------------------------------------------
/**
 *	Read attribute AutoFeedbackTargetThres related method
 *	Description: 
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_AutoFeedbackTargetThres(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_AutoFeedbackTargetThres(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_AutoFeedbackTargetThres) ENABLED START -----*/

	//	Set the attribute value
	*attr_AutoFeedbackTargetThres_read = cam_param[cam_idx].auto_feedback_target_thres;
	attr.set_value(attr_AutoFeedbackTargetThres_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_AutoFeedbackTargetThres
}
//--------------------------------------------------------
/**
 *	Write attribute AutoFeedbackTargetThres related method
 *	Description: 
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_AutoFeedbackTargetThres(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_AutoFeedbackTargetThres(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevLong	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_AutoFeedbackTargetThres) ENABLED START -----*/
	if (w_val > cam_param[cam_idx].auto_target_value) {
		Tango::Except::throw_exception (
			(const char *)"Failed to set threshold",
			(const char *)"Minimum threshold higher than target value",
			(const char *)"write_AutoFeedbackTargetThres()",Tango::ERR);		
	}
	cam_param[cam_idx].auto_feedback_target_thres = w_val;
	
	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_AutoFeedbackTargetThres
}
//--------------------------------------------------------
/**
 *	Read attribute AutoFeedbackDeadBand related method
 *	Description: 
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_AutoFeedbackDeadBand(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_AutoFeedbackDeadBand(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_AutoFeedbackDeadBand) ENABLED START -----*/

	//	Set the attribute value
	*attr_AutoFeedbackDeadBand_read = cam_param[cam_idx].auto_feedback_deadband;
	attr.set_value(attr_AutoFeedbackDeadBand_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_AutoFeedbackDeadBand
}
//--------------------------------------------------------
/**
 *	Write attribute AutoFeedbackDeadBand related method
 *	Description: 
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_AutoFeedbackDeadBand(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_AutoFeedbackDeadBand(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevLong	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_AutoFeedbackDeadBand) ENABLED START -----*/
	cam_param[cam_idx].auto_feedback_deadband = w_val;
	

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_AutoFeedbackDeadBand
}
//--------------------------------------------------------
/**
 *	Read attribute FitError related method
 *	Description: Fit error
 *               0=no error
 *               1=hor fit err
 *               2= ver fit err
 *               3=hor/ver fit err
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_FitError(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_FitError(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_FitError) ENABLED START -----*/

	//	Set the attribute value
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_FitError_read = img8[img_cur]->fit_error;
	}
	else if (img16.size() > 0) {
		*attr_FitError_read = img16[img_cur]->fit_error;
	}
	else
		*attr_FitError_read = 0;
	in_mutex->unlock();
	attr.set_value(attr_FitError_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_FitError
}
//--------------------------------------------------------
/**
 *	Read attribute BunchNumber related method
 *	Description: Absolute acquisition image counter.(Connection Diagnostics)
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_BunchNumber(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_BunchNumber(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_BunchNumber) ENABLED START -----*/

	//	Set the attribute value
	in_mutex->lock();
	if (img8.size() > 0) {
		*attr_BunchNumber_read = img8[img_cur]->acquisition_counter;
	}
	else if (img16.size() > 0) {
		*attr_BunchNumber_read = img16[img_cur]->acquisition_counter;
	}
	else
		*attr_BunchNumber_read = 0;
	in_mutex->unlock();
	attr.set_value(attr_BunchNumber_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_BunchNumber
}
//--------------------------------------------------------
/**
 *	Read attribute MeanIntensityThreshold related method
 *	Description: 
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_MeanIntensityThreshold(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_MeanIntensityThreshold(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_MeanIntensityThreshold) ENABLED START -----*/

	//	Set the attribute value
	attr.set_value(attr_MeanIntensityThreshold_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_MeanIntensityThreshold
}
//--------------------------------------------------------
/**
 *	Write attribute MeanIntensityThreshold related method
 *	Description: 
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_MeanIntensityThreshold(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_MeanIntensityThreshold(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevLong	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_MeanIntensityThreshold) ENABLED START -----*/
	*attr_MeanIntensityThreshold_read = w_val; 
	

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_MeanIntensityThreshold
}
//--------------------------------------------------------
/**
 *	Read attribute Binning related method
 *	Description: 
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_Binning(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_Binning(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_Binning) ENABLED START -----*/

	//	Set the attribute value
	*attr_Binning_read = cam_param[cam_idx].binning;
	attr.set_value(attr_Binning_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_Binning
}
//--------------------------------------------------------
/**
 *	Write attribute Binning related method
 *	Description: 
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_Binning(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_Binning(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevLong	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_Binning) ENABLED START -----*/

	INFO_STREAM << "Gigecam::write_Binning(): " << w_val << endl;

	if (((img8.size() > 0) || (img16.size() > 0)) && (w_val !=  cam_param[cam_idx].binning)) {
		cam_param[cam_idx].hw_roi = false;
		stop();
		/* wait the loop stopping */
		int32_t cnt = 0;
		while (link_flag) {
			usleep(500000);
			if (cnt++ > 20)
				break;
		}
		cam_param[cam_idx].binning = w_val;
		start((char *)cam_param[cam_idx].name.c_str());
	}	

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_Binning
}
//--------------------------------------------------------
/**
 *	Read attribute DISEnable related method
 *	Description: 
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_DISEnable(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_DISEnable(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_DISEnable) ENABLED START -----*/

	//	Set the attribute value
	*attr_DISEnable_read = cam_param[cam_idx].dis_enable;
	attr.set_value(attr_DISEnable_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_DISEnable
}
//--------------------------------------------------------
/**
 *	Write attribute DISEnable related method
 *	Description: 
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_DISEnable(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_DISEnable(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevBoolean	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_DISEnable) ENABLED START -----*/

	in_mutex->lock();
	bool	attr_bool;
	if (img8.size() > 0) {
		attr_bool = img8[img_cur]->get_enable_manual_roi();
	}
	else if (img16.size() > 0) {
		attr_bool= img16[img_cur]->get_enable_manual_roi();
	}	
	else
		attr_bool = false;
	in_mutex->unlock();
	
	if (attr_bool == false) {
		Tango::Except::throw_exception (
			(const char *)"Manual roi ha to be defined",
			(const char *)"Failed to enable DIS",
			(const char *)"Gigecam::write_DISEnable()",Tango::ERR);
	}


	in_mutex->lock();
	if (w_val == true) {
		if (img8.size() > 0) {
			img8[img_cur]->get_raw_pos(&cam_param[cam_idx].ref_raw_hor_pos, &cam_param[cam_idx].ref_raw_ver_pos);
		}
		else if (img16.size() > 0) {
			img16[img_cur]->get_raw_pos(&cam_param[cam_idx].ref_raw_hor_pos, &cam_param[cam_idx].ref_raw_ver_pos);
		}
	}
	in_mutex->unlock();

	cam_param[cam_idx].dis_enable = w_val;

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_DISEnable
}
//--------------------------------------------------------
/**
 *	Read attribute DisableTriggerTimeout related method
 *	Description: 
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_DisableTriggerTimeout(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_DisableTriggerTimeout(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_DisableTriggerTimeout) ENABLED START -----*/

	//	Set the attribute value
	attr.set_value(attr_DisableTriggerTimeout_read);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_DisableTriggerTimeout
}
//--------------------------------------------------------
/**
 *	Write attribute DisableTriggerTimeout related method
 *	Description: 
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_DisableTriggerTimeout(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_DisableTriggerTimeout(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevBoolean	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_DisableTriggerTimeout) ENABLED START -----*/
	*attr_DisableTriggerTimeout_read = w_val;
	

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_DisableTriggerTimeout
}
//--------------------------------------------------------
/**
 *	Read attribute ImageSumSamples related method
 *	Description: number of images summed in a ``moving`` sum image
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_ImageSumSamples(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_ImageSumSamples(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_ImageSumSamples) ENABLED START -----*/
	//	Set the attribute value
	attr.set_value(attr_ImageSumSamples_read);
	
	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_ImageSumSamples
}
//--------------------------------------------------------
/**
 *	Write attribute ImageSumSamples related method
 *	Description: number of images summed in a ``moving`` sum image
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_ImageSumSamples(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_ImageSumSamples(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevLong	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_ImageSumSamples) ENABLED START -----*/
	if (w_val > imageBufferSize) {
		Tango::Except::throw_exception (
			(const char *)"Value greater then ImageBufferSize property",
			(const char *)"Failed to set image sum sample value",
			(const char *)"Gigecam::write_ImageSumSamples()",Tango::ERR);
	}	
	
	in_mutex->lock();
	if (img8.size() > 0) {
		memset(attr_ImageSum_read, 0, img8[img_cur]->get_width()*img8[img_cur]->get_height()*4);
	}
	else if (img16.size() > 0) {
		memset(attr_ImageSum_read, 0, img16[img_cur]->get_width()*img16[img_cur]->get_height()*4);
	}
	in_mutex->unlock();

	imagesum_counter = 0;
	*attr_ImageSumSamples_read = w_val;
	
	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_ImageSumSamples
}
//--------------------------------------------------------
/**
 *	Read attribute Model related method
 *	Description: 
 *
 *	Data type:	Tango::DevString
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_Model(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_Model(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_Model) ENABLED START -----*/
	//	Set the attribute value
	attr.set_value(attr_Model_read);
	
	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_Model
}
//--------------------------------------------------------
/**
 *	Read attribute FeedbackMode related method
 *	Description: true: when MeanIntensity is below MeanIntensityThreshold, the camera state is turned to FAULT
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_FeedbackMode(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_FeedbackMode(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_FeedbackMode) ENABLED START -----*/
	//	Set the attribute value
	attr.set_value(attr_FeedbackMode_read);
	
	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_FeedbackMode
}
//--------------------------------------------------------
/**
 *	Write attribute FeedbackMode related method
 *	Description: true: when MeanIntensity is below MeanIntensityThreshold, the camera state is turned to FAULT
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::write_FeedbackMode(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_FeedbackMode(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve write value
	Tango::DevBoolean	w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_FeedbackMode) ENABLED START -----*/
	*attr_FeedbackMode_read = w_val;
	
	if (w_val == false)
		*attr_FeedbackError_read = false;

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_FeedbackMode
}
//--------------------------------------------------------
/**
 *	Read attribute MeanIntensity related method
 *	Description: 
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_MeanIntensity(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_MeanIntensity(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_MeanIntensity) ENABLED START -----*/
	//	Set the attribute value
	attr.set_value(attr_MeanIntensity_read);
	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_MeanIntensity
}
//--------------------------------------------------------
/**
 *	Read attribute FeedbackError related method
 *	Description: 
 *
 *	Data type:	Tango::DevBoolean
 *	Attr type:	Scalar
 */
//--------------------------------------------------------
void GigeCam::read_FeedbackError(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_FeedbackError(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_FeedbackError) ENABLED START -----*/
	//	Set the attribute value

	if (*attr_FeedbackMode_read == false)
		*attr_FeedbackError_read = false;

	attr.set_value(attr_FeedbackError_read);
	
	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_FeedbackError
}
//--------------------------------------------------------
/**
 *	Read attribute SearchBackgroundParam related method
 *	Description: param[0]: number of background levels\nparam[1]: derivative percentage threshold
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Spectrum max = 2
 */
//--------------------------------------------------------
void GigeCam::read_SearchBackgroundParam(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_SearchBackgroundParam(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_SearchBackgroundParam) ENABLED START -----*/
	
	attr_SearchBackgroundParam_read[0] = cam_param[cam_idx].search_background_param[0];
	attr_SearchBackgroundParam_read[1] = cam_param[cam_idx].search_background_param[1];	
	attr.set_value(attr_SearchBackgroundParam_read, 2, 0, false);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_SearchBackgroundParam
}
//--------------------------------------------------------
/**
 *	Write attribute SearchBackgroundParam related method
 *	Description: param[0]: number of background levels\nparam[1]: derivative percentage threshold
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Spectrum max = 2
 */
//--------------------------------------------------------
void GigeCam::write_SearchBackgroundParam(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_SearchBackgroundParam(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve number of write values
	int	w_length = attr.get_write_value_length();

	//	Retrieve pointer on write values (Do not delete !)
	const Tango::DevDouble	*w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_SearchBackgroundParam) ENABLED START -----*/
	const double *back_param;
	attr.get_write_value(back_param);

	int size = attr.get_write_value_length();

	if (size != 2) {
		Tango::Except::throw_exception (
			(const char *)"Parameters must be 2",
			(const char *)"Invalid number of parameters",
			(const char *)"Gigecam::write_SearchBackgroundParam()",Tango::ERR);
	}

	if ((back_param[0] == 0) && (back_param[1] == 0)) {
		/* do nothing */
	}
	else if ((back_param[0] < 1) || (back_param[0] > IMAGEPROC_MAX_LEVELS)) {
		Tango::Except::throw_exception (
			(const char *)"Invalid number of background levels",
			(const char *)"Failed to set search background parameters",
			(const char *)"Gigecam::write_SearchBackgroundParam()",Tango::ERR);
	}

	if ((back_param[0] == 0) && (back_param[1] == 0)) {
		/* do nothing */
	}
	else if ((back_param[1] < 0.01) || (back_param[1] > 0.9)) {
		Tango::Except::throw_exception (
			(const char *)"Invalid derivative parameter",
			(const char *)"Failed to set search background parameters",
			(const char *)"Gigecam::write_SearchBackgroundParam()",Tango::ERR);
	}

	for (int32_t i = 0; i < 2; i++)
		cam_param[cam_idx].search_background_param[i] = back_param[i];

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_SearchBackgroundParam
}
//--------------------------------------------------------
/**
 *	Read attribute HorProfile related method
 *	Description: Horizontal profile of the image.(Measure)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Spectrum max = 3000
 */
//--------------------------------------------------------
void GigeCam::read_HorProfile(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_HorProfile(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_HorProfile) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		attr_HorProfile_read = img8[img_cur]->hor_profile;
		attr.set_value(attr_HorProfile_read, img8[img_cur]->get_width(), 0, false);
	}
	else if (img16.size() > 0) {
		attr_HorProfile_read = img16[img_cur]->hor_profile;
		attr.set_value(attr_HorProfile_read, img16[img_cur]->get_width(), 0, false);
	}
	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_HorProfile
}
//--------------------------------------------------------
/**
 *	Read attribute VerProfile related method
 *	Description: Vertical profile of the image.(Measure)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Spectrum max = 3000
 */
//--------------------------------------------------------
void GigeCam::read_VerProfile(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_VerProfile(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_VerProfile) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		attr_VerProfile_read = img8[img_cur]->ver_profile;
		attr.set_value(attr_VerProfile_read, img8[img_cur]->get_height(), 0, false);
	}
	else if (img16.size() > 0) {
		attr_VerProfile_read = img16[img_cur]->ver_profile;
		attr.set_value(attr_VerProfile_read, img16[img_cur]->get_height(), 0, false);
	}
	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_VerProfile
}
//--------------------------------------------------------
/**
 *	Read attribute ErrorCounters related method
 *	Description: Error counters:\ncounter[0]: acquisition error\ncounter[1]: packets lost\ncounter[2]: packets canceled\n(Connection Diagnostics)
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Spectrum max = 10
 */
//--------------------------------------------------------
void GigeCam::read_ErrorCounters(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_ErrorCounters(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_ErrorCounters) ENABLED START -----*/

	for (int32_t i = 0; i < GIGECAM_ERR_CNT_SIZE; i++) {
		attr_ErrorCounters_read[i] = error_counters[i];
	}

	attr.set_value(attr_ErrorCounters_read, GIGECAM_ERR_CNT_SIZE, 0, false);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_ErrorCounters
}
//--------------------------------------------------------
/**
 *	Read attribute TimeStamp related method
 *	Description: Current timestamp (unix time) of the current image
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Spectrum max = 2
 */
//--------------------------------------------------------
void GigeCam::read_TimeStamp(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_TimeStamp(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_TimeStamp) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		attr_TimeStamp_read[0] = img8[img_cur]->acquisition_time.tv_sec;
		attr_TimeStamp_read[1] = img8[img_cur]->acquisition_time.tv_usec;
	}
	else if (img16.size() > 0) {
		attr_TimeStamp_read[0] = img16[img_cur]->acquisition_time.tv_sec;
		attr_TimeStamp_read[1] = img16[img_cur]->acquisition_time.tv_usec;
	}
	else {
		attr_TimeStamp_read[0] = 0;
		attr_TimeStamp_read[1] = 0;
	}
	in_mutex->unlock();

	attr.set_value(attr_TimeStamp_read, 2, 0, false);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_TimeStamp
}
//--------------------------------------------------------
/**
 *	Read attribute AutoRoiParam related method
 *	Description: Coordinates of the automatic roi (x1,y1,x2,y2)
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Spectrum max = 4
 */
//--------------------------------------------------------
void GigeCam::read_AutoRoiParam(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_AutoRoiParam(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_AutoRoiParam) ENABLED START -----*/
	in_mutex->lock();
	attr_AutoRoiParam_read = 0;
	if (img8.size() > 0) {
		attr_AutoRoiParam_read = (Tango::DevLong *)img8[img_cur]->get_auto_roi();
		attr.set_value(attr_AutoRoiParam_read, 4, 0, false);
	}
	else if (img16.size() > 0) {
		attr_AutoRoiParam_read = (Tango::DevLong *)img16[img_cur]->get_auto_roi();
		attr.set_value(attr_AutoRoiParam_read, 4, 0, false);
	} 
	in_mutex->unlock();	

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_AutoRoiParam
}
//--------------------------------------------------------
/**
 *	Read attribute RoiParam related method
 *	Description: Coordinates of the manual roi(x1,y1,x2,y2). (Display Options)
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Spectrum max = 4
 */
//--------------------------------------------------------
void GigeCam::read_RoiParam(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_RoiParam(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_RoiParam) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		attr_RoiParam_read = (Tango::DevLong *)img8[img_cur]->get_manual_roi();
		attr.set_value(attr_RoiParam_read, 4, 0, false);
	}
	else if (img16.size() > 0) {
		attr_RoiParam_read = (Tango::DevLong *)img16[img_cur]->get_manual_roi();
		attr.set_value(attr_RoiParam_read, 4, 0, false);
	}
	in_mutex->unlock();	

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_RoiParam
}
//--------------------------------------------------------
/**
 *	Write attribute RoiParam related method
 *	Description: Coordinates of the manual roi(x1,y1,x2,y2). (Display Options)
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Spectrum max = 4
 */
//--------------------------------------------------------
void GigeCam::write_RoiParam(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_RoiParam(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve number of write values
	int	w_length = attr.get_write_value_length();

	//	Retrieve pointer on write values (Do not delete !)
	const Tango::DevLong	*w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_RoiParam) ENABLED START -----*/
	INFO_STREAM << "Gigecam::write_RoiParam(): " << endl;

#ifdef CONFIG_64
	const int32_t *roi_array;
	attr.get_write_value(roi_array);
#else
	const long *roi_array;
	attr.get_write_value(roi_array);	
#endif
	
	int size = attr.get_write_value_length();

	if (size != 4) {
		Tango::Except::throw_exception (
			(const char *)"Parameters must be 4",
			(const char *)"Invalid number of parameters",
			(const char *)"Gigecam::write_ROI()",Tango::ERR);
	}

	int32_t roi_param_p[4];
	for (int i = 0; i < 4; i++)
		roi_param_p[i] = roi_array[i];

	set_roi_param(roi_param_p);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_RoiParam
}
//--------------------------------------------------------
/**
 *	Read attribute HorFitProfile related method
 *	Description: Horizontal profile of the image.(Measure)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Spectrum max = 3000
 */
//--------------------------------------------------------
void GigeCam::read_HorFitProfile(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_HorFitProfile(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_HorFitProfile) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		attr_HorFitProfile_read = img8[img_cur]->hor_fit_profile;
		attr.set_value(attr_HorFitProfile_read, img8[img_cur]->get_width(), 0, false);
	}
	else if (img16.size() > 0) {
		attr_HorFitProfile_read = img16[img_cur]->hor_fit_profile;
		attr.set_value(attr_HorFitProfile_read, img16[img_cur]->get_width(), 0, false);
	}
	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_HorFitProfile
}
//--------------------------------------------------------
/**
 *	Read attribute VerFitProfile related method
 *	Description: Horizontal profile of the image.(Measure)
 *
 *	Data type:	Tango::DevDouble
 *	Attr type:	Spectrum max = 3000
 */
//--------------------------------------------------------
void GigeCam::read_VerFitProfile(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_VerFitProfile(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_VerFitProfile) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		attr_VerFitProfile_read = img8[img_cur]->ver_fit_profile;
		attr.set_value(attr_VerFitProfile_read, img8[img_cur]->get_height(), 0, false);
	}
	else if (img16.size() > 0) {
		attr_VerFitProfile_read = img16[img_cur]->ver_fit_profile;
		attr.set_value(attr_VerFitProfile_read, img16[img_cur]->get_height(), 0, false);
	}
	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_VerFitProfile
}
//--------------------------------------------------------
/**
 *	Read attribute HwRoiParam related method
 *	Description: Coordinates of the hardware roi(x1,y1,x2,y2). (Display Options)
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Spectrum max = 4
 */
//--------------------------------------------------------
void GigeCam::read_HwRoiParam(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_HwRoiParam(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_HwRoiParam) ENABLED START -----*/

	//	Set the attribute value

	if (cam_param[cam_idx].hw_roi) {
		for (int i = 0; i < 4; i++) {
			attr_HwRoiParam_read[i] = cam_param[cam_idx].hw_roi_param[i];
		}
		attr.set_value(attr_HwRoiParam_read, 4);
	}
	else {
		Tango::Except::throw_exception (
			(const char *)"Hw roi disabled",
			(const char *)"Failed to retreive hw roi coordinates",
			(const char *)"GigeCam::read_HwRoiParam()");				
	}

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_HwRoiParam
}
//--------------------------------------------------------
/**
 *	Write attribute HwRoiParam related method
 *	Description: Coordinates of the hardware roi(x1,y1,x2,y2). (Display Options)
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Spectrum max = 4
 */
//--------------------------------------------------------
void GigeCam::write_HwRoiParam(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "GigeCam::write_HwRoiParam(Tango::WAttribute &attr) entering... " << endl;
	//	Retrieve number of write values
	int	w_length = attr.get_write_value_length();

	//	Retrieve pointer on write values (Do not delete !)
	const Tango::DevLong	*w_val;
	attr.get_write_value(w_val);
	/*----- PROTECTED REGION ID(GigeCam::write_HwRoiParam) ENABLED START -----*/

	

	/*----- PROTECTED REGION END -----*/	//	GigeCam::write_HwRoiParam
}
//--------------------------------------------------------
/**
 *	Read attribute Image8 related method
 *	Description: Image (8bit depth) of the camera.
 *
 *	Data type:	Tango::DevUChar
 *	Attr type:	Image max = 3000 x 3000
 */
//--------------------------------------------------------
void GigeCam::read_Image8(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_Image8(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_Image8) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
		attr_Image8_read = img8[img_cur]->image;
		attr.set_value(attr_Image8_read, img8[img_cur]->get_width(), img8[img_cur]->get_height(), false);
	}
	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_Image8
}
//--------------------------------------------------------
/**
 *	Read attribute Image8Icon related method
 *	Description: Decimated version of the image used to find the roi region
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Image max = 400 x 400
 */
//--------------------------------------------------------
void GigeCam::read_Image8Icon(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_Image8Icon(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_Image8Icon) ENABLED START -----*/
	in_mutex->lock();
	if (img8.size() > 0) {
#ifdef CONFIG_64
		attr_Image8Icon_read = img8[img_cur]->imageicon;
#else
		attr_Image8Icon_read = (long *)img8[img_cur]->imageicon;
#endif
		attr.set_value(attr_Image8Icon_read, img8[img_cur]->get_imageicon_width(),
		 img8[img_cur]->get_imageicon_height(), false);
	}
	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_Image8Icon
}
//--------------------------------------------------------
/**
 *	Read attribute Image16 related method
 *	Description: Image (18bit depth) of the camera
 *
 *	Data type:	Tango::DevUShort
 *	Attr type:	Image max = 3000 x 3000
 */
//--------------------------------------------------------
void GigeCam::read_Image16(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_Image16(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_Image16) ENABLED START -----*/
	in_mutex->lock();
	if (img16.size() > 0) {
		attr_Image16_read = img16[img_cur]->image;
		attr.set_value(attr_Image16_read, img16[img_cur]->get_width(), img16[img_cur]->get_height(), false);
	}
	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_Image16
}
//--------------------------------------------------------
/**
 *	Read attribute Image16Icon related method
 *	Description: Decimated version of the image used to find the roi region
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Image max = 400 x 400
 */
//--------------------------------------------------------
void GigeCam::read_Image16Icon(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_Image16Icon(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_Image16Icon) ENABLED START -----*/
	in_mutex->lock();
	if (img16.size() > 0) {
#ifdef CONFIG_64
		attr_Image16Icon_read = img16[img_cur]->imageicon;
#else
		attr_Image16Icon_read = (long *)img16[img_cur]->imageicon;
#endif
		attr.set_value(attr_Image16Icon_read, img16[img_cur]->get_imageicon_width(),
		 img16[img_cur]->get_imageicon_height(), false);
	}
	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_Image16Icon
}
//--------------------------------------------------------
/**
 *	Read attribute Image8Counter related method
 *	Description: Image (8bit depth) of the camera.
 *
 *	Data type:	Tango::DevUChar
 *	Attr type:	Image max = 3000 x 3000
 */
//--------------------------------------------------------
void GigeCam::read_Image8Counter(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_Image8Counter(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_Image8Counter) ENABLED START -----*/

	//	Set the attribute value
	in_mutex->lock();
	int32_t param_in[2], mode = BUF_GET_BUNCH_MODE, param_out[4];
	param_in[0] = attr_ImageAcquisitionCounter_write;
	param_in[1] = attr_ImageAcquisitionCounter_write;
	/* keep the same code of the get_image command inout */
	/* param_out executes only one "for" loop just for one cycle */ 
	if (get_indexes(mode, 3, param_in, param_out) == true) {
		if (img8.size() > 0) {
			for (int32_t i = param_out[0]; i < param_out[1]; i++) {
				attr_Image8Counter_read = img8[i]->image;
				attr.set_value(attr_Image8Counter_read, img8[i]->get_width(),
					img8[i]->get_height(), false);
			}
			for (int32_t i = param_out[2]; i < param_out[3]; i++) {
				attr_Image8Counter_read = img8[i]->image;
				attr.set_value(attr_Image8Counter_read, img8[i]->get_width(),
					img8[i]->get_height(), false);
				break;
			}
		}
	}
	else {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Invalida data indexes",
			(const char *) "GigeCam::read_Image8Counter()",Tango::ERR);

	}
	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_Image8Counter
}
//--------------------------------------------------------
/**
 *	Read attribute Image16Counter related method
 *	Description: Image (18bit depth) of the camera
 *
 *	Data type:	Tango::DevUShort
 *	Attr type:	Image max = 3000 x 3000
 */
//--------------------------------------------------------
void GigeCam::read_Image16Counter(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_Image16Counter(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_Image16Counter) ENABLED START -----*/

	//	Set the attribute value
	in_mutex->lock();
	int32_t param_in[2], mode = BUF_GET_BUNCH_MODE, param_out[4];
	param_in[0] = attr_ImageAcquisitionCounter_write;
	param_in[1] = attr_ImageAcquisitionCounter_write;
	/* keep the same code of the get_image command inout */
	/* param_out executes only one "for" loop just for one cycle */ 
	if (get_indexes(mode, 3, param_in, param_out) == true) {
		if (img16.size() > 0) {
			for (int32_t i = param_out[0]; i < param_out[1]; i++) {
				attr_Image16Counter_read = img16[i]->image;
				attr.set_value(attr_Image16Counter_read, img16[i]->get_width(),
					img8[i]->get_height(), false);
			}
			for (int32_t i = param_out[2]; i < param_out[3]; i++) {
				attr_Image16Counter_read = img16[i]->image;
				attr.set_value(attr_Image16Counter_read, img16[i]->get_width(),
					img16[i]->get_height(), false);
				break;
			}
		}
	}
	else {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Invalida data indexes",
			(const char *) "GigeCam::read_Image8Counter()",Tango::ERR);

	}
	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_Image16Counter
}
//--------------------------------------------------------
/**
 *	Read attribute ImageSum related method
 *	Description: Moving sum image
 *
 *	Data type:	Tango::DevLong
 *	Attr type:	Image max = 3000 x 3000
 */
//--------------------------------------------------------
void GigeCam::read_ImageSum(Tango::Attribute &attr)
{
	DEBUG_STREAM << "GigeCam::read_ImageSum(Tango::Attribute &attr) entering... " << endl;
	/*----- PROTECTED REGION ID(GigeCam::read_ImageSum) ENABLED START -----*/
	//	Set the attribute value

	if (*attr_ImageSumSamples_read <= 0) {
		Tango::Except::throw_exception(
			(const char *) "Number of image samples set to 0",
			(const char *) "Failed to get sum image",
			(const char *) "GigeCam::read_ImageSum()",Tango::ERR);		
	}

	in_mutex->lock();
	if (img8.size() > 0) {
		attr.set_value(attr_ImageSum_read, img8[img_cur]->get_width(), img8[img_cur]->get_height(), false);
	}
	else if (img16.size() > 0) {
		attr.set_value(attr_ImageSum_read, img16[img_cur]->get_width(), img16[img_cur]->get_height(), false);
	}
	in_mutex->unlock();
	
	/*----- PROTECTED REGION END -----*/	//	GigeCam::read_ImageSum
}

//--------------------------------------------------------
/**
 *	Method      : GigeCam::add_dynamic_attributes()
 *	Description : Create the dynamic attributes if any
 *                for specified device.
 */
//--------------------------------------------------------
void GigeCam::add_dynamic_attributes()
{
	/*----- PROTECTED REGION ID(GigeCam::add_dynamic_attributes) ENABLED START -----*/
	
	//	Add your own code to create and add dynamic attributes if any
	
	/*----- PROTECTED REGION END -----*/	//	GigeCam::add_dynamic_attributes
}

//--------------------------------------------------------
/**
 *	Command Start related method
 *	Description: Connect to a camera and start image acquisition. (Camera Controls)
 *
 *	@param argin camera label or ip number
 */
//--------------------------------------------------------
void GigeCam::start(Tango::DevString argin)
{
	DEBUG_STREAM << "GigeCam::Start()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::start) ENABLED START -----*/

	//	Add your own code

	int32_t ip_idx, name_idx, tmp_idx = 0;

	{ string msg_log("command start executing");info_log(msg_log); }

	if (num_cameras == 1) {
		tmp_idx = 1;
	}
	else {
		ip_idx = camera_ip_index[argin];
		name_idx = camera_name_index[argin];
	
		if ((ip_idx == 0) && (name_idx == 0))	{
			Tango::Except::throw_exception(
				(const char *) "Can't connect to camera",
				(const char *) "Not find camera label",
				(const char *) "GigeCam::start()",Tango::ERR);
		}
		else if (name_idx >  0) {
			tmp_idx = name_idx;
		}
		else if (ip_idx > 0) {
			tmp_idx = ip_idx;
		}
	}

	/* wait the loop stopping */
	if (link_flag) {
		Tango::Except::throw_exception(
			(const char *) "Still acquiring data",
			(const char *) "Stop acquisition before starting it",
			(const char *) "GigeCam::start()",Tango::ERR);
	}

	/* if no camera actually running */
	in_mutex->lock();
	if ((img8.size() == 0) && (img16.size() == 0) && (link_flag == false)) {
		cam_idx = tmp_idx-1;
		run_flag = true;
		acqloop = new acqthread(this);
		acqloop->start();
		int count = 0;
		/* wait link flag becoming true */
		while (link_flag == false) {
			usleep(50000);
			count++;
			if (count >= 20)
				break;
		}
		if (link_flag == false) {
			in_mutex->unlock();
			Tango::Except::throw_exception(
				(const char *) "Fault in starting acquisition loop",
				(const char *) "Acquisition loop starting timeout",
				(const char *) "GigeCam::start()",Tango::ERR);
		}
	}
	in_mutex->unlock();

	{ string msg_log("command start done"); info_log(msg_log); }

	/*----- PROTECTED REGION END -----*/	//	GigeCam::start
}
//--------------------------------------------------------
/**
 *	Command Stop related method
 *	Description: Stop the acquisition on the camera and disconnect from it. (Camera Controls)
 *
 */
//--------------------------------------------------------
void GigeCam::stop()
{
	DEBUG_STREAM << "GigeCam::Stop()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::stop) ENABLED START -----*/

	//	Add your own code
	{
	string msg_log("command stop executing");
	info_log(msg_log);
	}

	if (((img8.size() > 0) || (img16.size() > 0)) && (acqloop != NULL)) {
		acqloop->CameraStop();
	}
	run_flag = false;
	if (link_flag == false) {
		set_state(Tango::OFF);
		set_status("acquisition stopped");
	}

	{
	string msg_log("command stop done");
	info_log(msg_log);
	}

	/*----- PROTECTED REGION END -----*/	//	GigeCam::stop
}
//--------------------------------------------------------
/**
 *	Command Reset related method
 *	Description: Restart connection on the current selected camera. (Camera Controls)
 *
 */
//--------------------------------------------------------
void GigeCam::reset()
{
	DEBUG_STREAM << "GigeCam::Reset()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::reset) ENABLED START -----*/

	//	Add your own code
	//	Add your own code to control device here

	/*----- PROTECTED REGION END -----*/	//	GigeCam::reset
}
//--------------------------------------------------------
/**
 *	Command ListCamera related method
 *	Description: List cameras that could be connected to this device. (Camera Controls)
 *
 *	@returns camera labels
 */
//--------------------------------------------------------
Tango::DevVarStringArray *GigeCam::list_camera()
{
	Tango::DevVarStringArray *argout;
	DEBUG_STREAM << "GigeCam::ListCamera()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::list_camera) ENABLED START -----*/

	//	Add your own code
	//	POGO has generated a method core with argout allocation.
	//	If you would like to use a static reference without copying,
	//	See "TANGO Device Server Programmer's Manual"
	//		(chapter : Writing a TANGO DS / Exchanging data)
	//------------------------------------------------------------
	argout = new Tango::DevVarStringArray();
	DEBUG_STREAM << "GigeCam::list_camera(): entering... !" << endl;

	//	Add your own code to control device here
	argout->length(num_cameras);
	
	for (uint32_t i = 0; i < argout->length(); i++)
		(*argout)[i] = CORBA::string_dup(cam_param[i].name.c_str());

	/*----- PROTECTED REGION END -----*/	//	GigeCam::list_camera
	return argout;
}
//--------------------------------------------------------
/**
 *	Command AcquireBackground related method
 *	Description: Acquire background image. (Processing Setup)
 *
 */
//--------------------------------------------------------
void GigeCam::acquire_background()
{
	DEBUG_STREAM << "GigeCam::AcquireBackground()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::acquire_background) ENABLED START -----*/

	//	Add your own code
	in_mutex->lock();
	if ((img8.size() > 0) || (img16.size() > 0)) {
		trigger_background = true;
	}
	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::acquire_background
}
//--------------------------------------------------------
/**
 *	Command GetImage8 related method
 *	Description: 
 *
 *	@param argin mode (0,1,2), more parameters
 *	@returns 
 */
//--------------------------------------------------------
Tango::DevVarCharArray *GigeCam::get_image8(const Tango::DevVarLongArray *argin)
{
	Tango::DevVarCharArray *argout;
	DEBUG_STREAM << "GigeCam::GetImage8()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::get_image8) ENABLED START -----*/

	//	Add your own code
	//	POGO has generated a method core with argout allocation.
	//	If you would like to use a static reference without copying,
	//	See "TANGO Device Server Programmer's Manual"
	//		(chapter : Writing a TANGO DS / Exchanging data)
	//------------------------------------------------------------

	DEBUG_STREAM << "GigeCam::get_image8(): entering... !" << endl;

	//	Add your own code to control device here
	in_mutex->lock();

	argout = 0;

	int32_t data_size;
	int32_t param_out[4], i;

	if ((img16.size() > 0) || (img8.size() <=0)) {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Invalid data format",
			(const char *) "GigeCam::get_image8()",Tango::ERR);
	}

	/* get buffer indexes */
	if (get_indexes((*argin)[0], argin->length(), (int32_t *) (&(*argin)[1]), param_out) == true) {
		int32_t img_size = img8[img_cur]->get_height()*img8[img_cur]->get_width();
		data_size = ((param_out[3]-param_out[2]) + (param_out[1]-param_out[0]))* img_size;

		argout  = new Tango::DevVarCharArray();
		argout->length(data_size);

		if (img8.size() > 0) {
			int32_t offset = 0;
			for (i = param_out[0]; i < param_out[1]; i++) {
				memcpy((unsigned char *) (&(*argout)[0]) + offset, img8[i]->image, img_size * sizeof(char));
				offset += img_size;
			}
			for (i = param_out[2]; i < param_out[3]; i++) {
				memcpy((unsigned char *) (&(*argout)[0]) + offset, img8[i]->image, img_size * sizeof(char));
				offset += img_size;
			}
		}

	}
	else {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Invalida data indexes",
			(const char *) "GigeCam::get_image8()",Tango::ERR);
	}

	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::get_image8
	return argout;
}
//--------------------------------------------------------
/**
 *	Command GetImage16 related method
 *	Description: 
 *
 *	@param argin mode (0,1,2), more parameters
 *	@returns 
 */
//--------------------------------------------------------
Tango::DevVarUShortArray *GigeCam::get_image16(const Tango::DevVarLongArray *argin)
{
	Tango::DevVarUShortArray *argout;
	DEBUG_STREAM << "GigeCam::GetImage16()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::get_image16) ENABLED START -----*/

	//	Add your own code
	//	POGO has generated a method core with argout allocation.
	//	If you would like to use a static reference without copying,
	//	See "TANGO Device Server Programmer's Manual"
	//		(chapter : Writing a TANGO DS / Exchanging data)
	//------------------------------------------------------------

	DEBUG_STREAM << "GigeCam::get_image16(): entering... !" << endl;

	//	Add your own code to control device here
	in_mutex->lock();

	argout = 0;

	int32_t data_size;
	int32_t param_out[4], i;

	if ((img8.size() > 0) || (img16.size() <=0)) {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Invalid data format",
			(const char *) "GigeCam::get_image16()",Tango::ERR);
	}

	/* get buffer indexes */
	if (get_indexes((*argin)[0], argin->length(), (int32_t *) (&(*argin)[1]), param_out) == true) {
		int32_t img_size = img16[img_cur]->get_height()*img16[img_cur]->get_width();
		data_size = ((param_out[3]-param_out[2]) + (param_out[1]-param_out[0]))* img_size;
		
		argout  = new Tango::DevVarUShortArray();
		argout->length(data_size);

		if (img16.size() > 0) {
			int32_t offset = 0;
			for (i = param_out[0]; i < param_out[1]; i++) {
				memcpy((unsigned short *) (&(*argout)[0]) + offset, img16[i]->image, img_size * sizeof(short));
				offset += img_size;
			}
			for (i = param_out[2]; i < param_out[3]; i++) {
				memcpy((unsigned short *) (&(*argout)[0]) + offset, img16[i]->image, img_size * sizeof(short));
				offset += img_size;
			}
		}
	}
	else {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Invalida data indexes",
			(const char *) "GigeCam::get_image16()",Tango::ERR);
	}

	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::get_image16
	return argout;
}
//--------------------------------------------------------
/**
 *	Command GetHorPos related method
 *	Description: 
 *
 *	@param argin mode (0,1,2), more parameters
 *	@returns 
 */
//--------------------------------------------------------
Tango::DevVarDoubleArray *GigeCam::get_hor_pos(const Tango::DevVarLongArray *argin)
{
	Tango::DevVarDoubleArray *argout;
	DEBUG_STREAM << "GigeCam::GetHorPos()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::get_hor_pos) ENABLED START -----*/

	//	Add your own code
	//	POGO has generated a method core with argout allocation.
	//	If you would like to use a static reference without copying,
	//	See "TANGO Device Server Programmer's Manual"
	//		(chapter : Writing a TANGO DS / Exchanging data)
	//------------------------------------------------------------

	DEBUG_STREAM << "GigeCam::get_hor_pos(): entering... !" << endl;

	//	Add your own code to control device here

	in_mutex->lock();

	argout = 0;

	long data_size, buf_idx[2];
	unsigned char unique;
	int32_t buf_limit = 0;

	if (buf_hor_pos == 0) {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Buffer not initialized",
			(const char *) "GigeCam::get_hor_pos()",Tango::ERR);
	}

	try {
		check_get_data_param(argin, buf_hor_pos, buf_idx, &data_size, &unique);
	}	
	catch(Tango::DevFailed &e) {
		in_mutex->unlock();
		Tango::Except::re_throw_exception(e,
			"Read buffer error", "Read buffer error",
			__FUNCTION__, Tango::ERR);			
	}

	argout  = new Tango::DevVarDoubleArray();

	if ((BUF_GET_LAST_MODE == (*argin)[0]) && (data_size < (*argin)[1])) {
		buf_limit = (*argin)[1]-data_size;
		argout->length((*argin)[1]);
		for (int32_t i = 0; i < buf_limit; i++)
			(*argout)[i] = 0;
	}
	else
		argout->length(data_size);
	
	read_data_calib(unique, buf_idx, &(*argout)[buf_limit], buf_hor_pos, 0, cam_param[cam_idx].hor_sign, 0);

	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::get_hor_pos
	return argout;
}
//--------------------------------------------------------
/**
 *	Command GetVerPos related method
 *	Description: 
 *
 *	@param argin mode (0,1,2), more parameters
 *	@returns 
 */
//--------------------------------------------------------
Tango::DevVarDoubleArray *GigeCam::get_ver_pos(const Tango::DevVarLongArray *argin)
{
	Tango::DevVarDoubleArray *argout;
	DEBUG_STREAM << "GigeCam::GetVerPos()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::get_ver_pos) ENABLED START -----*/

	//	Add your own code
	//	POGO has generated a method core with argout allocation.
	//	If you would like to use a static reference without copying,
	//	See "TANGO Device Server Programmer's Manual"
	//		(chapter : Writing a TANGO DS / Exchanging data)
	//------------------------------------------------------------

	DEBUG_STREAM << "GigeCam::get_ver_pos(): entering... !" << endl;

	//	Add your own code to control device here

	in_mutex->lock();

	argout = 0;

	long data_size, buf_idx[2];
	unsigned char unique;
	int32_t buf_limit = 0;

	if (buf_ver_pos == 0) {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Buffer not initialized",
			(const char *) "GigeCam::get_ver_pos()",Tango::ERR);
	}

	try {
		check_get_data_param(argin, buf_ver_pos, buf_idx, &data_size, &unique);
	}
	catch(Tango::DevFailed &e) {
		in_mutex->unlock();
		Tango::Except::re_throw_exception(e,
			"Read buffer error", "Read buffer error",
			__FUNCTION__, Tango::ERR);		
	}

	argout  = new Tango::DevVarDoubleArray();

	if ((BUF_GET_LAST_MODE == (*argin)[0]) && (data_size < (*argin)[1])) {
		buf_limit = (*argin)[1]-data_size;
		argout->length((*argin)[1]);
		for (int32_t i = 0; i < buf_limit; i++)
			(*argout)[i] = 0;
	}
	else
		argout->length(data_size);

	read_data_calib(unique, buf_idx, &(*argout)[buf_limit], buf_ver_pos, 0, cam_param[cam_idx].ver_sign, 0);

	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::get_ver_pos
	return argout;
}
//--------------------------------------------------------
/**
 *	Command GetHorSigma related method
 *	Description: 
 *
 *	@param argin mode (0,1,2), more parameters
 *	@returns 
 */
//--------------------------------------------------------
Tango::DevVarDoubleArray *GigeCam::get_hor_sigma(const Tango::DevVarLongArray *argin)
{
	Tango::DevVarDoubleArray *argout;
	DEBUG_STREAM << "GigeCam::GetHorSigma()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::get_hor_sigma) ENABLED START -----*/

	//	Add your own code
	//	POGO has generated a method core with argout allocation.
	//	If you would like to use a static reference without copying,
	//	See "TANGO Device Server Programmer's Manual"
	//		(chapter : Writing a TANGO DS / Exchanging data)
	//------------------------------------------------------------

	DEBUG_STREAM << "GigeCam::get_hor_sigma(): entering... !" << endl;

	//	Add your own code to control device here

	in_mutex->lock();

	argout = 0;

	long data_size, buf_idx[2];
	unsigned char unique;
	int32_t buf_limit = 0;

	if (buf_hor_sigma == 0) {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Buffer not initialized",
			(const char *) "GigeCam::get_hor_sigma()",Tango::ERR);
	}

	try {
		check_get_data_param(argin, buf_hor_sigma, buf_idx, &data_size, &unique);
	}
	catch(Tango::DevFailed &e) {
		in_mutex->unlock();
		Tango::Except::re_throw_exception(e,
			"Read buffer error", "Read buffer error",
			__FUNCTION__, Tango::ERR);		
	}

	argout  = new Tango::DevVarDoubleArray();

	if ((BUF_GET_LAST_MODE == (*argin)[0]) && (data_size < (*argin)[1])) {
		buf_limit = (*argin)[1]-data_size;
		argout->length((*argin)[1]);
		for (int32_t i = 0; i < buf_limit; i++)
			(*argout)[i] = 0;
	}
	else
		argout->length(data_size);

	read_data(unique, buf_idx, &(*argout)[buf_limit], buf_hor_sigma);

	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::get_hor_sigma
	return argout;
}
//--------------------------------------------------------
/**
 *	Command GetVerSigma related method
 *	Description: 
 *
 *	@param argin mode (0,1,2), more parameters
 *	@returns 
 */
//--------------------------------------------------------
Tango::DevVarDoubleArray *GigeCam::get_ver_sigma(const Tango::DevVarLongArray *argin)
{
	Tango::DevVarDoubleArray *argout;
	DEBUG_STREAM << "GigeCam::GetVerSigma()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::get_ver_sigma) ENABLED START -----*/

	//	Add your own code
	//	POGO has generated a method core with argout allocation.
	//	If you would like to use a static reference without copying,
	//	See "TANGO Device Server Programmer's Manual"
	//		(chapter : Writing a TANGO DS / Exchanging data)
	//------------------------------------------------------------

	DEBUG_STREAM << "GigeCam::get_ver_sigma(): entering... !" << endl;

	//	Add your own code to control device here

	in_mutex->lock();

	argout = 0;

	long data_size, buf_idx[2];
	unsigned char unique;
	int32_t buf_limit = 0;

	if (buf_ver_sigma == 0) {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Buffer not initialized",
			(const char *) "GigeCam::get_ver_sigma()",Tango::ERR);
	}

	try {
		check_get_data_param(argin, buf_ver_sigma, buf_idx, &data_size,&unique);
	}
	catch(Tango::DevFailed &e) {
		in_mutex->unlock();
		Tango::Except::re_throw_exception(e,
			"Read buffer error", "Read buffer error",
			__FUNCTION__, Tango::ERR);		
	}

	argout  = new Tango::DevVarDoubleArray();

	if ((BUF_GET_LAST_MODE == (*argin)[0]) && (data_size < (*argin)[1])) {
		buf_limit = (*argin)[1]-data_size;
		argout->length((*argin)[1]);
		for (int32_t i = 0; i < buf_limit; i++)
			(*argout)[i] = 0;
	}
	else
		argout->length(data_size);

	read_data(unique,buf_idx, &(*argout)[buf_limit], buf_ver_sigma);

	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::get_ver_sigma
	return argout;
}
//--------------------------------------------------------
/**
 *	Command GetArea related method
 *	Description: 
 *
 *	@param argin mode (0,1,2), more parameters
 *	@returns 
 */
//--------------------------------------------------------
Tango::DevVarDoubleArray *GigeCam::get_area(const Tango::DevVarLongArray *argin)
{
	Tango::DevVarDoubleArray *argout;
	DEBUG_STREAM << "GigeCam::GetArea()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::get_area) ENABLED START -----*/

	//	Add your own code
	//	POGO has generated a method core with argout allocation.
	//	If you would like to use a static reference without copying,
	//	See "TANGO Device Server Programmer's Manual"
	//		(chapter : Writing a TANGO DS / Exchanging data)
	//------------------------------------------------------------

	DEBUG_STREAM << "GigeCam::get_area(): entering... !" << endl;

	//	Add your own code to control device here

	in_mutex->lock();

	argout = 0;

	long data_size, buf_idx[2];
	unsigned char unique;
	int32_t buf_limit = 0;

	if (buf_area == 0) {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Buffer not initialized",
			(const char *) "GigeCam::get_area()",Tango::ERR);
	}

	try {
		check_get_data_param(argin, buf_area, buf_idx, &data_size, &unique);
	}
	catch(Tango::DevFailed &e) {
	in_mutex->unlock();
		Tango::Except::re_throw_exception(e,
			"Read buffer error", "Read buffer error",
			__FUNCTION__, Tango::ERR);		
	}

	argout  = new Tango::DevVarDoubleArray();

	if ((BUF_GET_LAST_MODE == (*argin)[0]) && (data_size < (*argin)[1])) {
		buf_limit = (*argin)[1]-data_size;
		argout->length((*argin)[1]);
		for (int32_t i = 0; i < buf_limit; i++)
			(*argout)[i] = 0;
	}
	else
		argout->length(data_size);

	read_data(unique, buf_idx, &(*argout)[buf_limit], buf_area);

	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::get_area
	return argout;
}
//--------------------------------------------------------
/**
 *	Command GetAcquisitionCounter related method
 *	Description: 
 *
 *	@param argin mode (0,1,2), more parameters
 *	@returns 
 */
//--------------------------------------------------------
Tango::DevVarLongArray *GigeCam::get_acquisition_counter(const Tango::DevVarLongArray *argin)
{
	Tango::DevVarLongArray *argout;
	DEBUG_STREAM << "GigeCam::GetAcquisitionCounter()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::get_acquisition_counter) ENABLED START -----*/

	//	Add your own code
	//	POGO has generated a method core with argout allocation.
	//	If you would like to use a static reference without copying,
	//	See "TANGO Device Server Programmer's Manual"
	//		(chapter : Writing a TANGO DS / Exchanging data)
	//------------------------------------------------------------

	DEBUG_STREAM << "GigeCam::get_acquisition_counter(): entering... !" << endl;

	//	Add your own code to control device here
	in_mutex->lock();

	argout = 0;

	long data_size, buf_idx[2];
	unsigned char unique;

	if (buf_acquisition_counter == 0) {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Buffer not initialized",
			(const char *) "GigeCam::get_acquisition_counter()",Tango::ERR);
	}

	try {
		check_get_data_param(argin, buf_acquisition_counter, buf_idx, &data_size, &unique);
	}
	catch(Tango::DevFailed &e) {
	in_mutex->unlock();	
		Tango::Except::re_throw_exception(e,
			"Read buffer error", "Read buffer error",
			__FUNCTION__, Tango::ERR);		
	}

	argout  = new Tango::DevVarLongArray();
	argout->length(data_size);
	read_data(unique, buf_idx, &(*argout)[0], buf_acquisition_counter);

	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::get_acquisition_counter
	return argout;
}
//--------------------------------------------------------
/**
 *	Command GetSaturation related method
 *	Description: Get saturation values from the circular buffers
 *
 *	@param argin 
 *	@returns 
 */
//--------------------------------------------------------
Tango::DevVarDoubleArray *GigeCam::get_saturation(const Tango::DevVarLongArray *argin)
{
	Tango::DevVarDoubleArray *argout;
	DEBUG_STREAM << "GigeCam::GetSaturation()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::get_saturation) ENABLED START -----*/

	//	Add your own code
	//	POGO has generated a method core with argout allocation.
	//	If you would like to use a static reference without copying,
	//	See "TANGO Device Server Programmer's Manual"
	//		(chapter : Writing a TANGO DS / Exchanging data)
	//------------------------------------------------------------

	DEBUG_STREAM << "GigeCam::get_saturation(): entering... !" << endl;

	//	Add your own code to control device here	

	in_mutex->lock();

	argout = 0;

	long data_size, buf_idx[2];
	unsigned char unique;
	int32_t buf_limit = 0;

	if (buf_saturation == 0) {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Buffer not initialized",
			(const char *) "GigeCam::get_saturation()",Tango::ERR);
	}

	try {
		check_get_data_param(argin, buf_saturation, buf_idx, &data_size, &unique);
	}
	catch(Tango::DevFailed &e) {
		in_mutex->unlock();
		Tango::Except::re_throw_exception(e,
			"Read buffer error", "Read buffer error",
			__FUNCTION__, Tango::ERR);		
	}

	argout  = new Tango::DevVarDoubleArray();

	if ((BUF_GET_LAST_MODE == (*argin)[0]) && (data_size < (*argin)[1])) {
		buf_limit = (*argin)[1]-data_size;
		argout->length((*argin)[1]);
		for (int32_t i = 0; i < buf_limit; i++)
			(*argout)[i] = 0;
	}
	else
		argout->length(data_size);

	read_data(unique, buf_idx, &(*argout)[buf_limit], buf_saturation);

	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::get_saturation
	return argout;
}
//--------------------------------------------------------
/**
 *	Command SetReference related method
 *	Description: 
 *
 */
//--------------------------------------------------------
void GigeCam::set_reference()
{
	DEBUG_STREAM << "GigeCam::SetReference()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::set_reference) ENABLED START -----*/

	//	Add your own code
	take_reference = true;

	cam_param[cam_idx].ref_hor_pos_offset=0;
	cam_param[cam_idx].ref_ver_pos_offset=0;

	if (img8.size() > 0) {
		cam_param[cam_idx].ref_hor_pos_offset =img8[img_cur]->hor_pos + 2 * img8[img_cur]->hor_pos_offset;
		cam_param[cam_idx].ref_ver_pos_offset = img8[img_cur]->ver_pos;
		cam_param[cam_idx].delta_ref_hor_pos_offset = 0;
		cam_param[cam_idx].delta_ref_ver_pos_offset = 0;
	}
	else if (img16.size() > 0) {
		cam_param[cam_idx].ref_hor_pos_offset = img16[img_cur]->hor_pos + 2 * img16[img_cur]->hor_pos_offset;
		cam_param[cam_idx].ref_ver_pos_offset = img16[img_cur]->ver_pos;
		cam_param[cam_idx].delta_ref_hor_pos_offset = 0;
		cam_param[cam_idx].delta_ref_ver_pos_offset = 0;
	}

	if ((cam_param[cam_idx].ref_hor_pos_offset != 0) && (cam_param[cam_idx].ref_ver_pos_offset != 0)) {
		Tango::Database *db = new Tango::Database(host_rw,port_rw);
		try {
			Tango::DbDatum config("RefPosOffset");
			Tango::DbData db_data_put;
			vector<string>	val;
			char conf_str[300];
			memset(conf_str,0,300);
			sprintf(conf_str,"%.4f",cam_param[cam_idx].ref_hor_pos_offset);
			val.push_back(conf_str);
			memset(conf_str,0,300);
			sprintf(conf_str,"%.4f",cam_param[cam_idx].ref_ver_pos_offset);
			val.push_back(conf_str);
			config << val;
			db_data_put.push_back(config);
			db->put_device_property(get_name(), db_data_put);
		}
		catch (Tango::DevFailed &e) {
			delete db;
			Tango::Except::throw_exception (
				(const char *)"Failed to set reference into database",
				(const char *)"Can't write into database",
				(const char *)"GigeCam::modify_configuration()");				
		}
	}

	/*----- PROTECTED REGION END -----*/	//	GigeCam::set_reference
}
//--------------------------------------------------------
/**
 *	Command GetPhase related method
 *	Description: 
 *
 *	@param argin mode (0,1,2), more parameters
 *	@returns 
 */
//--------------------------------------------------------
Tango::DevVarDoubleArray *GigeCam::get_phase(const Tango::DevVarLongArray *argin)
{
	Tango::DevVarDoubleArray *argout;
	DEBUG_STREAM << "GigeCam::GetPhase()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::get_phase) ENABLED START -----*/

	//	Add your own code
	//	POGO has generated a method core with argout allocation.
	//	If you would like to use a static reference without copying,
	//	See "TANGO Device Server Programmer's Manual"
	//		(chapter : Writing a TANGO DS / Exchanging data)
	//------------------------------------------------------------

	DEBUG_STREAM << "GigeCam::get_phase(): entering... !" << endl;

	//	Add your own code to control device here

	in_mutex->lock();

	argout = 0;

	long data_size, buf_idx[2];
	unsigned char unique;
	int32_t buf_limit = 0;

	if (buf_phase == 0) {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Buffer not initialized",
			(const char *) "GigeCam::get_phase()",Tango::ERR);
	}

	try {
		check_get_data_param(argin, buf_phase, buf_idx, &data_size, &unique);
	}
	catch(Tango::DevFailed &e) {
		in_mutex->unlock();
		Tango::Except::re_throw_exception(e,
			"Read buffer error", "Read buffer error",
			__FUNCTION__, Tango::ERR);		
	}

	argout  = new Tango::DevVarDoubleArray();

	if ((BUF_GET_LAST_MODE == (*argin)[0]) && (data_size < (*argin)[1])) {
		buf_limit = (*argin)[1]-data_size;
		argout->length((*argin)[1]);
		for (int32_t i = 0; i < buf_limit; i++)
			(*argout)[i] = 0;
	}
	else
		argout->length(data_size);
	
	in_mutex->unlock();

	read_data(unique, buf_idx, &(*argout)[buf_limit], buf_phase);

	/*----- PROTECTED REGION END -----*/	//	GigeCam::get_phase
	return argout;
}
//--------------------------------------------------------
/**
 *	Command SaveScale related method
 *	Description: Save pixel size calibration
 *
 */
//--------------------------------------------------------
void GigeCam::save_scale()
{
	DEBUG_STREAM << "GigeCam::SaveScale()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::save_scale) ENABLED START -----*/

	//	Add your own code
	modify_configuration(false,true); /* modify only scale */

	/*----- PROTECTED REGION END -----*/	//	GigeCam::save_scale
}
//--------------------------------------------------------
/**
 *	Command RestoreScale related method
 *	Description: Restore pixel calibration
 *
 */
//--------------------------------------------------------
void GigeCam::restore_scale()
{
	DEBUG_STREAM << "GigeCam::RestoreScale()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::restore_scale) ENABLED START -----*/

	//	Add your own code
	INFO_STREAM << "Gigecam::restore_scale" << endl;

	in_mutex->lock();
	for (uint32_t j = 0; j < img8.size(); j++) {
		img8[j]->hor_calibration = cam_param[cam_idx].hor_calibration;
		img8[j]->ver_calibration = cam_param[cam_idx].ver_calibration;
		img8[j]->hor_pos_offset = cam_param[cam_idx].hor_pos_offset;
		img8[j]->ver_pos_offset = cam_param[cam_idx].ver_pos_offset;
	}
	for (uint32_t j = 0; j < img16.size(); j++) {
		img16[j]->hor_calibration = cam_param[cam_idx].hor_calibration;
		img16[j]->ver_calibration = cam_param[cam_idx].ver_calibration;
		img16[j]->hor_pos_offset = cam_param[cam_idx].hor_pos_offset;
		img16[j]->ver_pos_offset = cam_param[cam_idx].ver_pos_offset;
	}
	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::restore_scale
}
//--------------------------------------------------------
/**
 *	Command SetMeasurePoint related method
 *	Description: Set measure point in the image
 *
 *	@param argin horizontal pixel position, vertical pixel position
 */
//--------------------------------------------------------
void GigeCam::set_measure_point(const Tango::DevVarLongArray *argin)
{
	DEBUG_STREAM << "GigeCam::SetMeasurePoint()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::set_measure_point) ENABLED START -----*/

	//	Add your own code
	if ((argin->length() & 0x1) || (argin->length() > IMAGEPROC_MAX_REF*2)) {
		Tango::Except::throw_exception (
			(const char *)"Invalid number of points",
			(const char *)"Failed to set meauser points",
			(const char *)"GigeCam::set_measure_point()");
	}

	INFO_STREAM << "GigeCam::set_measure_points()" << endl;

	in_mutex->lock();
	if (img8.size() > 0) {
		for (uint32_t i = 0; i < (argin->length() >> 1); i++) {
			for (uint32_t j = 0; j < img8.size(); j++) {
				img8[j]->set_reference((*argin)[i*2],(*argin)[i*2+1]);
			}
		}
	}

	if (img16.size() > 0) {
		for (uint32_t i = 0; i < (argin->length() >> 1); i++) {
			for (uint32_t j = 0; j < img16.size(); j++) {
				img16[j]->set_reference((*argin)[i*2],(*argin)[i*2+1]);
			}
		}
	}
	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::set_measure_point
}
//--------------------------------------------------------
/**
 *	Command ClearMeasurePoints related method
 *	Description: Clear all reference points in the image
 *
 */
//--------------------------------------------------------
void GigeCam::clear_measure_points()
{
	DEBUG_STREAM << "GigeCam::ClearMeasurePoints()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::clear_measure_points) ENABLED START -----*/

	//	Add your own code
	INFO_STREAM << "GigeCam::set_measure_points()" << endl;

	in_mutex->lock();
	if (img8.size() > 0) {
		for (int32_t i = 0; i < IMAGEPROC_MAX_REF; i++) {
			for (uint32_t j = 0; j < img8.size(); j++) {
				img8[j]->set_reference(0,0);
				img8[j]->ref_hor_pos_offset = 0;
				img8[j]->ref_ver_pos_offset = 0;
			}
		}
	}

	if (img16.size() > 0) {
		for (int32_t i = 0; i < IMAGEPROC_MAX_REF; i++) {
			for (uint32_t j = 0; j < img16.size(); j++) {
				img16[j]->set_reference(0,0);
				img16[j]->ref_hor_pos_offset = 0;
				img16[j]->ref_ver_pos_offset = 0;
			}
		}
	}
	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::clear_measure_points
}
//--------------------------------------------------------
/**
 *	Command GetIntensity related method
 *	Description: Get intensity values from the circular buffers
 *
 *	@param argin 
 *	@returns 
 */
//--------------------------------------------------------
Tango::DevVarDoubleArray *GigeCam::get_intensity(const Tango::DevVarLongArray *argin)
{
	Tango::DevVarDoubleArray *argout;
	DEBUG_STREAM << "GigeCam::GetIntensity()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::get_intensity) ENABLED START -----*/

	//	Add your own code
	//	POGO has generated a method core with argout allocation.
	//	If you would like to use a static reference without copying,
	//	See "TANGO Device Server Programmer's Manual"
	//		(chapter : Writing a TANGO DS / Exchanging data)
	//------------------------------------------------------------

	DEBUG_STREAM << "GigeCam::get_intensity(): entering... !" << endl;

	//	Add your own code to control device here

	in_mutex->lock();

	argout = 0;

	long data_size, buf_idx[2];
	unsigned char unique;
	int32_t buf_limit = 0;

	if (buf_intensity == 0) {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Buffer not initialized",
			(const char *) "GigeCam::get_intensity()",Tango::ERR);
	}

	try {
		check_get_data_param(argin, buf_intensity, buf_idx, &data_size, &unique);
	}
	catch(Tango::DevFailed &e) {
		in_mutex->unlock();
		Tango::Except::re_throw_exception(e,
			"Read buffer error", "Read buffer error",
			__FUNCTION__, Tango::ERR);		
	}

	argout  = new Tango::DevVarDoubleArray();

	if ((BUF_GET_LAST_MODE == (*argin)[0]) && (data_size < (*argin)[1])) {
		buf_limit = (*argin)[1]-data_size;
		argout->length((*argin)[1]);
		for (int32_t i = 0; i < buf_limit; i++)
			(*argout)[i] = 0;
	}
	else
		argout->length(data_size);

	read_data(unique, buf_idx, &(*argout)[buf_limit], buf_intensity);

	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::get_intensity
	return argout;
}
//--------------------------------------------------------
/**
 *	Command GetHorProfile related method
 *	Description: 
 *
 *	@param argin 
 *	@returns 
 */
//--------------------------------------------------------
Tango::DevVarDoubleArray *GigeCam::get_hor_profile(const Tango::DevVarLongArray *argin)
{
	Tango::DevVarDoubleArray *argout;
	DEBUG_STREAM << "GigeCam::GetHorProfile()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::get_hor_profile) ENABLED START -----*/

	//	Add your own code

	in_mutex->lock();

	argout = 0;

	int32_t data_size, data_size_offset = 0;
	int32_t param_out[4], i;

	/* get buffer indexes */
	if (get_indexes((*argin)[0], argin->length(), (int32_t *) (&(*argin)[1]), param_out) == true) {

		if (img8.size() > 0) {
			int32_t vec_size = img8[img_cur]->get_width();
			data_size = ((param_out[3]-param_out[2]) + (param_out[1]-param_out[0])) * vec_size;
			if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
				data_size_offset = data_size;
				data_size += ((param_out[3]-param_out[2]) + (param_out[1]-param_out[0]));
			}
			argout  = new Tango::DevVarDoubleArray();
			argout->length(data_size);
			int32_t offset = 0,  cnt = 0;
			for (i = param_out[0]; i < param_out[1]; i++) {
				memcpy((double *) (&(*argout)[0]) + offset, img8[i]->hor_profile, vec_size * sizeof(double));
				offset += vec_size;
				if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
					(*argout)[data_size_offset+cnt++] = img8[i]->acquisition_counter;
				}
			}
			for (i = param_out[2]; i < param_out[3]; i++) {
				memcpy((double *) (&(*argout)[0]) + offset, img8[i]->hor_profile, vec_size * sizeof(double));
				offset += vec_size;
				if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
					(*argout)[data_size_offset+cnt++] = img8[i]->acquisition_counter;
				}
			}
		}
		else if (img16.size() > 0) {
			int32_t vec_size = img16[img_cur]->get_width();
			data_size = ((param_out[3]-param_out[2]) + (param_out[1]-param_out[0])) * vec_size ;
			if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
				data_size_offset = data_size;
				data_size += ((param_out[3]-param_out[2]) + (param_out[1]-param_out[0]));
			}
			argout  = new Tango::DevVarDoubleArray();
			argout->length(data_size);
			int32_t offset = 0, cnt = 0;
			for (i = param_out[0]; i < param_out[1]; i++) {
				memcpy((double *) (&(*argout)[0]) + offset, img16[i]->hor_profile, vec_size * sizeof(double));
				offset += vec_size;
				if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
					(*argout)[data_size_offset+cnt++] = img16[i]->acquisition_counter;
				}
			}
			for (i = param_out[2]; i < param_out[3]; i++) {
				memcpy((double *) (&(*argout)[0]) + offset, img16[i]->hor_profile, vec_size * sizeof(double));
				offset += vec_size;
				if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
					(*argout)[data_size_offset+cnt++] = img16[i]->acquisition_counter;
				}
			}
		}
		else {
			in_mutex->unlock();
			Tango::Except::throw_exception(
				(const char *) "Failed to retreive data",
				(const char *) "Data not available",
				(const char *) "GigeCam::GetHorProfile()",Tango::ERR);
		}

	}
	else {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Invalida data indexes",
			(const char *) "GigeCam::GetHorProfile()",Tango::ERR);
	}

	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::get_hor_profile
	return argout;
}
//--------------------------------------------------------
/**
 *	Command GetVerProfile related method
 *	Description: 
 *
 *	@param argin 
 *	@returns 
 */
//--------------------------------------------------------
Tango::DevVarDoubleArray *GigeCam::get_ver_profile(const Tango::DevVarLongArray *argin)
{
	Tango::DevVarDoubleArray *argout;
	DEBUG_STREAM << "GigeCam::GetVerProfile()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::get_ver_profile) ENABLED START -----*/

	//	Add your own code
	in_mutex->lock();

	argout = 0;

	int32_t data_size, data_size_offset = 0;
	int32_t param_out[4], i;

	/* get buffer indexes */
	if (get_indexes((*argin)[0], argin->length(), (int32_t *) (&(*argin)[1]), param_out) == true) {

		if (img8.size() > 0) {
			int32_t vec_size = img8[img_cur]->get_height();
			data_size = ((param_out[3]-param_out[2]) + (param_out[1]-param_out[0])) * vec_size ;
			if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
				data_size_offset = data_size;
				data_size += ((param_out[3]-param_out[2]) + (param_out[1]-param_out[0]));
			}
			argout  = new Tango::DevVarDoubleArray();
			argout->length(data_size);
			int32_t offset = 0, cnt = 0;
			for (i = param_out[0]; i < param_out[1]; i++) {
				memcpy((double *) (&(*argout)[0]) + offset, img8[i]->ver_profile, vec_size * sizeof(double));
				offset += vec_size;
				if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
					(*argout)[data_size_offset+cnt++] = img8[i]->acquisition_counter;
				}
			}
			for (i = param_out[2]; i < param_out[3]; i++) {
				memcpy((double *) (&(*argout)[0]) + offset, img8[i]->ver_profile, vec_size * sizeof(double));
				offset += vec_size;
				if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
					(*argout)[data_size_offset+cnt++] = img8[i]->acquisition_counter;
				}
			}
		}
		else if (img16.size() > 0) {
			int32_t vec_size = img16[img_cur]->get_height();
			data_size = ((param_out[3]-param_out[2]) + (param_out[1]-param_out[0])) * vec_size ;
			argout  = new Tango::DevVarDoubleArray();
			argout->length(data_size);
			if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
				data_size_offset = data_size;
				data_size += ((param_out[3]-param_out[2]) + (param_out[1]-param_out[0]));
			}
			int32_t offset = 0, cnt = 0;
			for (i = param_out[0]; i < param_out[1]; i++) {
				memcpy((double *) (&(*argout)[0]) + offset, img16[i]->ver_profile, vec_size * sizeof(double));
				offset += vec_size;
				if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
					(*argout)[data_size_offset+cnt++] = img16[i]->acquisition_counter;
				}
			}
			for (i = param_out[2]; i < param_out[3]; i++) {
				memcpy((double *) (&(*argout)[0]) + offset, img16[i]->ver_profile, vec_size * sizeof(double));
				offset += vec_size;
				if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
					(*argout)[data_size_offset+cnt++] = img16[i]->acquisition_counter;
				}
			}
		}
		else {
			in_mutex->unlock();
			Tango::Except::throw_exception(
				(const char *) "Failed to retreive data",
				(const char *) "Data not available",
				(const char *) "GigeCam::GetVerProfile()",Tango::ERR);
		}

	}
	else {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Invalida data indexes",
			(const char *) "GigeCam::GetVerProfile()",Tango::ERR);
	}

	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::get_ver_profile
	return argout;
}
//--------------------------------------------------------
/**
 *	Command GetHorFitProfile related method
 *	Description: 
 *
 *	@param argin 
 *	@returns 
 */
//--------------------------------------------------------
Tango::DevVarDoubleArray *GigeCam::get_hor_fit_profile(const Tango::DevVarLongArray *argin)
{
	Tango::DevVarDoubleArray *argout;
	DEBUG_STREAM << "GigeCam::GetHorFitProfile()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::get_hor_fit_profile) ENABLED START -----*/

	//	Add your own code
	in_mutex->lock();

	argout = 0;

	int32_t data_size, data_size_offset = 0;
	int32_t param_out[4], i;

	/* get buffer indexes */
	if (get_indexes((*argin)[0], argin->length(), (int32_t *) (&(*argin)[1]), param_out) == true) {

		if (img8.size() > 0) {
			int32_t vec_size = img8[img_cur]->get_width();
			data_size = ((param_out[3]-param_out[2]) + (param_out[1]-param_out[0])) * vec_size ;
			if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
				data_size_offset = data_size;
				data_size += ((param_out[3]-param_out[2]) + (param_out[1]-param_out[0]));
			}
			argout  = new Tango::DevVarDoubleArray();
			argout->length(data_size);
			int32_t offset = 0, cnt = 0;
			for (i = param_out[0]; i < param_out[1]; i++) {
				memcpy((double *) (&(*argout)[0]) + offset, img8[i]->hor_fit_profile, vec_size * sizeof(double));
				offset += vec_size;
				if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
					(*argout)[data_size_offset+cnt++] = img8[i]->acquisition_counter;
				}
			}
			for (i = param_out[2]; i < param_out[3]; i++) {
				memcpy((double *) (&(*argout)[0]) + offset, img8[i]->hor_fit_profile, vec_size * sizeof(double));
				offset += vec_size; 
				if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
					(*argout)[data_size_offset+cnt++] = img8[i]->acquisition_counter;
				}
			}
		}
		else if (img16.size() > 0) {
			int32_t vec_size = img16[img_cur]->get_width();
			data_size = ((param_out[3]-param_out[2]) + (param_out[1]-param_out[0])) * vec_size ;
			if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
				data_size_offset = data_size;
				data_size += ((param_out[3]-param_out[2]) + (param_out[1]-param_out[0]));
			}
			argout  = new Tango::DevVarDoubleArray();
			argout->length(data_size);
			int32_t offset = 0, cnt = 0;
			for (i = param_out[0]; i < param_out[1]; i++) {
				memcpy((double *) (&(*argout)[0]) + offset, img16[i]->hor_fit_profile, vec_size * sizeof(double));
				offset += vec_size;
				if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
					(*argout)[data_size_offset+cnt++] = img16[i]->acquisition_counter;
				}
			}
			for (i = param_out[2]; i < param_out[3]; i++) {
				memcpy((double *) (&(*argout)[0]) + offset, img16[i]->hor_fit_profile, vec_size * sizeof(double));
				offset += vec_size;
				if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
					(*argout)[data_size_offset+cnt++] = img16[i]->acquisition_counter;
				}
			}
		}
		else {
			in_mutex->unlock();
			Tango::Except::throw_exception(
				(const char *) "Failed to retreive data",
				(const char *) "Data not available",
				(const char *) "GigeCam::GetHorFitProfile()",Tango::ERR);
		}

	}
	else {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Invalida data indexes",
			(const char *) "GigeCam::GetHorFitProfile()",Tango::ERR);
	}

	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::get_hor_fit_profile
	return argout;
}
//--------------------------------------------------------
/**
 *	Command GetVerFitProfile related method
 *	Description: 
 *
 *	@param argin 
 *	@returns 
 */
//--------------------------------------------------------
Tango::DevVarDoubleArray *GigeCam::get_ver_fit_profile(const Tango::DevVarLongArray *argin)
{
	Tango::DevVarDoubleArray *argout;
	DEBUG_STREAM << "GigeCam::GetVerFitProfile()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::get_ver_fit_profile) ENABLED START -----*/

	//	Add your own code
	in_mutex->lock();

	argout = 0;

	int32_t data_size, data_size_offset = 0;
	int32_t param_out[4], i;

	/* get buffer indexes */
	if (get_indexes((*argin)[0], argin->length(), (int32_t *) (&(*argin)[1]), param_out) == true) {

		if (img8.size() > 0) {
			int32_t vec_size = img8[img_cur]->get_height();
			data_size = ((param_out[3]-param_out[2]) + (param_out[1]-param_out[0])) * vec_size ;
			if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
				data_size_offset = data_size;
				data_size += ((param_out[3]-param_out[2]) + (param_out[1]-param_out[0]));
			}
			argout  = new Tango::DevVarDoubleArray();
			argout->length(data_size);
			int32_t offset = 0, cnt = 0;
			for (i = param_out[0]; i < param_out[1]; i++) {
				memcpy((double *) (&(*argout)[0]) + offset, img8[i]->ver_fit_profile, vec_size * sizeof(double));
				offset += vec_size;
				if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
					(*argout)[data_size_offset+cnt++] = img8[i]->acquisition_counter;
				}
			}
			for (i = param_out[2]; i < param_out[3]; i++) {
				memcpy((double *) (&(*argout)[0]) + offset, img8[i]->ver_fit_profile, vec_size * sizeof(double));
				offset += vec_size; 
				if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
					(*argout)[data_size_offset+cnt++] = img8[i]->acquisition_counter;
				}
			}
		}
		else if (img16.size() > 0) {
			int32_t vec_size = img16[img_cur]->get_height();
			data_size = ((param_out[3]-param_out[2]) + (param_out[1]-param_out[0])) * vec_size ;
			if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
				data_size_offset = data_size;
				data_size += ((param_out[3]-param_out[2]) + (param_out[1]-param_out[0]));
			}
			argout  = new Tango::DevVarDoubleArray();
			argout->length(data_size);
			int32_t offset = 0, cnt = 0;
			for (i = param_out[0]; i < param_out[1]; i++) {
				memcpy((double *) (&(*argout)[0]) + offset, img16[i]->ver_fit_profile, vec_size * sizeof(double));
				offset += vec_size;
				if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
					(*argout)[data_size_offset+cnt++] = img16[i]->acquisition_counter;
				}
			}
			for (i = param_out[2]; i < param_out[3]; i++) {
				memcpy((double *) (&(*argout)[0]) + offset, img16[i]->ver_fit_profile, vec_size * sizeof(double));
				offset += vec_size;
				if (((*argin)[0] == BUF_GET_LAST_BUNCH_MODE) || ((*argin)[0] == BUF_GET_BUNCH_BUNCH_MODE)) {
					(*argout)[data_size_offset+cnt++] = img16[i]->acquisition_counter;
				}
			}
		}
		else {
			in_mutex->unlock();
			Tango::Except::throw_exception(
				(const char *) "Failed to retreive data",
				(const char *) "Data not available",
				(const char *) "GigeCam::GetVerFitProfile()",Tango::ERR);
		}

	}
	else {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Invalida data indexes",
			(const char *) "GigeCam::GetVerFitProfile()",Tango::ERR);
	}

	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::get_ver_fit_profile
	return argout;
}
//--------------------------------------------------------
/**
 *	Command SaveParam related method
 *	Description: Save all ccd / processing parameters except scale
 *
 */
//--------------------------------------------------------
void GigeCam::save_param()
{
	DEBUG_STREAM << "GigeCam::SaveParam()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::save_param) ENABLED START -----*/

	//	Add your own code
	modify_configuration(true,false);
	/*----- PROTECTED REGION END -----*/	//	GigeCam::save_param
}
//--------------------------------------------------------
/**
 *	Command ClearReference related method
 *	Description: 
 *
 */
//--------------------------------------------------------
void GigeCam::clear_reference()
{
	DEBUG_STREAM << "GigeCam::ClearReference()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::clear_reference) ENABLED START -----*/

	//	Add your own code

	/*----- PROTECTED REGION END -----*/	//	GigeCam::clear_reference
}
//--------------------------------------------------------
/**
 *	Command ExposureAutoOnce related method
 *	Description: 
 *
 */
//--------------------------------------------------------
void GigeCam::exposure_auto_once()
{
	DEBUG_STREAM << "GigeCam::ExposureAutoOnce()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::exposure_auto_once) ENABLED START -----*/

	//	Add your own code
	cam_param[cam_idx].gain_auto = false;
	cam_param[cam_idx].gain_auto_once = false;	
	cam_param[cam_idx].exposure_auto_once = true;
	cam_param[cam_idx].exposure_auto_cnt = 0;
	cam_param[cam_idx].exposure_once_cnt = 0;
	/*----- PROTECTED REGION END -----*/	//	GigeCam::exposure_auto_once
}
//--------------------------------------------------------
/**
 *	Command GainAutoOnce related method
 *	Description: 
 *
 */
//--------------------------------------------------------
void GigeCam::gain_auto_once()
{
	DEBUG_STREAM << "GigeCam::GainAutoOnce()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::gain_auto_once) ENABLED START -----*/

	//	Add your own code
	cam_param[cam_idx].exposure_auto = false;
	cam_param[cam_idx].exposure_auto_once = false;	
	cam_param[cam_idx].gain_auto_once = true;	
	cam_param[cam_idx].gain_auto_cnt = 0;
	cam_param[cam_idx].gain_once_cnt = 0;
	/*----- PROTECTED REGION END -----*/	//	GigeCam::gain_auto_once
}
//--------------------------------------------------------
/**
 *	Command GetFitError related method
 *	Description: 0=no error
 *               1=hor fit err
 *               2=ver fit err
 *               3=hor/ver fit er
 *
 *	@param argin 
 *	@returns 
 */
//--------------------------------------------------------
Tango::DevVarLongArray *GigeCam::get_fit_error(const Tango::DevVarLongArray *argin)
{
	Tango::DevVarLongArray *argout;
	DEBUG_STREAM << "GigeCam::GetFitError()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::get_fit_error) ENABLED START -----*/

	//	Add your own code
	//	Add your own code
	//	POGO has generated a method core with argout allocation.
	//	If you would like to use a static reference without copying,
	//	See "TANGO Device Server Programmer's Manual"
	//		(chapter : Writing a TANGO DS / Exchanging data)
	//------------------------------------------------------------

	in_mutex->lock();
	
	argout = 0;

	long data_size, buf_idx[2];
	unsigned char unique;
	int32_t buf_limit = 0;

	if (buf_fit_error == 0) {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Buffer not initialized",
			(const char *) "GigeCam::get_fit_error()",Tango::ERR);
	}

	try {
		check_get_data_param(argin, buf_fit_error, buf_idx, &data_size, &unique);
	}
	catch(Tango::DevFailed &e) {
		in_mutex->unlock();
		Tango::Except::re_throw_exception(e,
			"Read buffer error", "Read buffer error",
			__FUNCTION__, Tango::ERR);		
	}

	argout  = new Tango::DevVarLongArray();

	if ((BUF_GET_LAST_MODE == (*argin)[0]) && (data_size < (*argin)[1])) {
		buf_limit = (*argin)[1]-data_size;
		argout->length((*argin)[1]);
		for (int32_t i = 0; i < buf_limit; i++)
			(*argout)[i] = 0;
	}
	else
		argout->length(data_size);
	
	read_data(unique, buf_idx, &(*argout)[buf_limit], buf_fit_error);

	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::get_fit_error
	return argout;
}
//--------------------------------------------------------
/**
 *	Command GetBunchNumber related method
 *	Description: 
 *
 *	@param argin mode (0,1,2), more parameters
 *	@returns 
 */
//--------------------------------------------------------
Tango::DevVarLongArray *GigeCam::get_bunch_number(const Tango::DevVarLongArray *argin)
{
	Tango::DevVarLongArray *argout;
	DEBUG_STREAM << "GigeCam::GetBunchNumber()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::get_bunch_number) ENABLED START -----*/

	//	Add your own code
	//	Add your own code
	//	POGO has generated a method core with argout allocation.
	//	If you would like to use a static reference without copying,
	//	See "TANGO Device Server Programmer's Manual"
	//		(chapter : Writing a TANGO DS / Exchanging data)
	//------------------------------------------------------------

	DEBUG_STREAM << "GigeCam::get_acquisition_counter(): entering... !" << endl;
	//	Add your own code to control device here

	in_mutex->lock();

	argout = 0;

	long data_size, buf_idx[2];
	unsigned char unique;

	if (buf_acquisition_counter == 0) {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Buffer not initialized",
			(const char *) "GigeCam::get_bunch_number()",Tango::ERR);
	}

	try {
		check_get_data_param(argin, buf_acquisition_counter, buf_idx, &data_size, &unique);
	}
	catch(Tango::DevFailed &e) {
		in_mutex->unlock();	
		Tango::Except::re_throw_exception(e,
			"Read buffer error", "Read buffer error",
			__FUNCTION__, Tango::ERR);		
	}

	argout  = new Tango::DevVarLongArray();
	argout->length(data_size);
	read_data(unique, buf_idx, &(*argout)[0], buf_acquisition_counter);

	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::get_bunch_number
	return argout;
}
//--------------------------------------------------------
/**
 *	Command GetMaxVal related method
 *	Description: 
 *
 *	@param argin mode (0,1,2), more parameters
 *	@returns 
 */
//--------------------------------------------------------
Tango::DevVarLongArray *GigeCam::get_max_val(const Tango::DevVarLongArray *argin)
{
	Tango::DevVarLongArray *argout;
	DEBUG_STREAM << "GigeCam::GetMaxVal()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::get_max_val) ENABLED START -----*/

	//	Add your own code
	//	Add your own code
	//	POGO has generated a method core with argout allocation.
	//	If you would like to use a static reference without copying,
	//	See "TANGO Device Server Programmer's Manual"
	//		(chapter : Writing a TANGO DS / Exchanging data)
	//------------------------------------------------------------

	DEBUG_STREAM << "GigeCam::get_acquisition_counter(): entering... !" << endl;
	//	Add your own code to control device here

	in_mutex->lock();

	argout = 0;

	long data_size, buf_idx[2];
	unsigned char unique;

	if (buf_max_val == 0) {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Buffer not initialized",
			(const char *) "GigeCam::get_max_val()",Tango::ERR);
	}

	try {
		check_get_data_param(argin, buf_max_val, buf_idx, &data_size, &unique);
	}
	catch(Tango::DevFailed &e) {
		in_mutex->unlock();
		Tango::Except::re_throw_exception(e,
			"Read buffer error", "Read buffer error",
			__FUNCTION__, Tango::ERR);			
	}

	argout  = new Tango::DevVarLongArray();
	argout->length(data_size);
	read_data(unique, buf_idx, &(*argout)[0], buf_max_val);

	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::get_max_val
	return argout;
}
//--------------------------------------------------------
/**
 *	Command GetGain related method
 *	Description: 
 *
 *	@param argin 
 *	@returns 
 */
//--------------------------------------------------------
Tango::DevVarDoubleArray *GigeCam::get_gain(const Tango::DevVarLongArray *argin)
{
	Tango::DevVarDoubleArray *argout;
	DEBUG_STREAM << "GigeCam::GetGain()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::get_gain) ENABLED START -----*/

	//	Add your own code
	in_mutex->lock();

	argout = 0;

	long data_size, buf_idx[2];
	unsigned char unique;

	if (buf_gain == 0) {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Buffer not initialized",
			(const char *) "GigeCam::get_gain()",Tango::ERR);
	}

	try {
		check_get_data_param(argin, buf_gain, buf_idx, &data_size, &unique);
	}
	catch(Tango::DevFailed &e) {
		in_mutex->unlock();
		Tango::Except::re_throw_exception(e,
			"Read buffer error", "Read buffer error",
			__FUNCTION__, Tango::ERR);			
	}

	argout  = new Tango::DevVarDoubleArray();
	argout->length(data_size);
	read_data(unique, buf_idx, &(*argout)[0], buf_gain);

	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::get_gain
	return argout;
}
//--------------------------------------------------------
/**
 *	Command GetExposure related method
 *	Description: 
 *
 *	@param argin 
 *	@returns 
 */
//--------------------------------------------------------
Tango::DevVarLongArray *GigeCam::get_exposure(const Tango::DevVarLongArray *argin)
{
	Tango::DevVarLongArray *argout;
	DEBUG_STREAM << "GigeCam::GetExposure()  - " << device_name << endl;
	/*----- PROTECTED REGION ID(GigeCam::get_exposure) ENABLED START -----*/

	//	Add your own code

	in_mutex->lock();

	argout = 0;

	long data_size, buf_idx[2];
	unsigned char unique;

	if (buf_exposure == 0) {
		in_mutex->unlock();
		Tango::Except::throw_exception(
			(const char *) "Failed to retreive data",
			(const char *) "Buffer not initialized",
			(const char *) "GigeCam::get_exposure()",Tango::ERR);
	}

	try {
		check_get_data_param(argin, buf_exposure, buf_idx, &data_size, &unique);
	}
	catch(Tango::DevFailed &e) {
		in_mutex->unlock();
		Tango::Except::re_throw_exception(e,
			"Read buffer error", "Read buffer error",
			__FUNCTION__, Tango::ERR);		
	}

	argout  = new Tango::DevVarLongArray();
	argout->length(data_size);
	read_data(unique, buf_idx, &(*argout)[0], buf_exposure);

	in_mutex->unlock();

	/*----- PROTECTED REGION END -----*/	//	GigeCam::get_exposure
	return argout;
}
//--------------------------------------------------------
/**
 *	Method      : GigeCam::add_dynamic_commands()
 *	Description : Create the dynamic commands if any
 *                for specified device.
 */
//--------------------------------------------------------
void GigeCam::add_dynamic_commands()
{
	/*----- PROTECTED REGION ID(GigeCam::add_dynamic_commands) ENABLED START -----*/
	
	//	Add your own code to create and add dynamic commands if any
	
	/*----- PROTECTED REGION END -----*/	//	GigeCam::add_dynamic_commands
}

/*----- PROTECTED REGION ID(GigeCam::namespace_ending) ENABLED START -----*/

	//	Additional Methods
/**
 *	method:	GigeCam::get_configuration
 * Get the camera parameters
 *
*/
bool GigeCam::get_configuration(void)
{
	
	long cam_cnt = 0, conf_size;

	/* local configuration size */
	num_cameras = configuration.size();
	global_flag_prop = false;

	/* if no camera locally defined, go to global property */
	if (num_cameras == 0) {
		Tango::DbData db_data;
		try {
			Tango::Database* db = new Tango::Database();
			db_data.push_back(Tango::DbDatum("Configuration"));
			db->get_property("Gigecam",db_data);
			if (db_data[0].is_empty()) { 
				cout << "ERROR gigecam getting properties" << endl;
				return false;
			}
			else {
				db_data[0] >> configuration;
				global_flag_prop = true;
			}
		}
		catch (CORBA::Exception &e) {
			cout << "ERROR gigecam getting properties" << endl;
			return false;
		}
	}

	cam_param = new cam_param_t[num_cameras];
	if (cam_param == 0)
		return false;

	for (vector<string>::const_iterator it = configuration.begin(); it < configuration.end(); it++) {
		std::string token;
		std::istringstream iss(*it);
		conf_size = 0;
		while ( getline(iss, token, ':') ) {
			switch (conf_size) {
				case 0: cam_param[cam_cnt].name = token; 
					camera_name_index[token.c_str()] = cam_cnt+1; 
					printf("configuration: camera name = %s\n",cam_param[cam_cnt].name.c_str());break;
				case 1: cam_param[cam_cnt].ipaddress = token;
					camera_ip_index[token.c_str()] = cam_cnt+1; 
					printf("configuration: ip address = %s\n",cam_param[cam_cnt].ipaddress.c_str());break; 
				case 2: cam_param[cam_cnt].img_depth = atol(token.c_str());
					printf("configuration: image depth = %d\n", (int) cam_param[cam_cnt].img_depth);break;
				case 3: cam_param[cam_cnt].exposure = atol(token.c_str());
					printf("configuration: exposure = %d us.\n", (int) cam_param[cam_cnt].exposure);break;
				case 4: cam_param[cam_cnt].gain = atof(token.c_str());
					printf("configuration: gain = %.1f dB\n",cam_param[cam_cnt].gain);break;
				case 5: cam_param[cam_cnt].mirror = (bool) atol(token.c_str());
					printf("configuration: mirror = ");
					if (cam_param[cam_cnt].mirror) 
						printf("true\n");
					else
						printf("false\n");
					break;
				case 6: cam_param[cam_cnt].rotation = atof(token.c_str());
					printf("configuration: rotation = %.1f deg.\n",cam_param[cam_cnt].rotation);break;
				case 7: cam_param[cam_cnt].trigger = (bool) atol(token.c_str());
					printf("configuration: trigger = ");
					if (cam_param[cam_cnt].trigger) 
						printf("external\n");
					else
						printf("internal\n");
					break;
				case 8: cam_param[cam_cnt].hor_calibration = atof(token.c_str());
					printf("configuration: hor calib. = %.3f mm\n",cam_param[cam_cnt].hor_calibration);break;
				case 9: cam_param[cam_cnt].ver_calibration = atof(token.c_str());
					printf("configuration: ver calib. = %.3f mm\n",cam_param[cam_cnt].ver_calibration);break;
				case 10: cam_param[cam_cnt].hor_pos_offset = atof(token.c_str());
					printf("configuration: hor offset = %.3f mm\n",cam_param[cam_cnt].hor_pos_offset);break;
				case 11: cam_param[cam_cnt].ver_pos_offset = atof(token.c_str());
					printf("configuration: ver offset = %.3f mm\n",cam_param[cam_cnt].ver_pos_offset);break;
				case 12: cam_param[cam_cnt].hor_sign = atof(token.c_str());
					printf("configuration: hor sign = %0f\n",cam_param[cam_cnt].hor_sign);break;
				case 13: cam_param[cam_cnt].ver_sign = atof(token.c_str());
					printf("configuration: ver sign = %0f\n",cam_param[cam_cnt].ver_sign);break;
			}
			if ((++conf_size) > 13)
				break;
  	}

		cam_param[cam_cnt].img_permutation = 0;

		cam_param[cam_cnt].background_subtraction = false;
		cam_param[cam_cnt].process_enable = imageProcessing;

		cam_param[cam_cnt].auto_roi = autoRoi;
		cam_param[cam_cnt].sw_roi = swRoi;

		cam_param[cam_cnt].gridsize = (long) autoRoiParam[0];
		cam_param[cam_cnt].lp_order = (long) autoRoiParam[1];

		cam_param[cam_cnt].plot_mode = plotMode[0];
		cam_param[cam_cnt].plot_level = plotMode[1];

		cam_param[cam_cnt].manual_roi_param[0] = roiParam[0];
		cam_param[cam_cnt].manual_roi_param[1] = roiParam[1];
		cam_param[cam_cnt].manual_roi_param[2] = roiParam[2];
		cam_param[cam_cnt].manual_roi_param[3] = roiParam[3];

		cam_param[cam_cnt].process_mode = processMode;

		cam_param[cam_cnt].fast_calc = fastCalculation;

		cam_param[cam_cnt].search_background_param[0] = searchBackgroundParam[0]; /* background levels */
		cam_param[cam_cnt].search_background_param[1] = searchBackgroundParam[1]; /* derivative coeff */

		cam_param[cam_cnt].roi_threshold = roiThreshold; /* derivative coeff */

		cam_param[cam_cnt].configured = false;

		cam_param[cam_cnt].ref_hor_pos_offset = 0;
		cam_param[cam_cnt].ref_ver_pos_offset = 0;
		
		cam_param[cam_cnt].delta_ref_hor_pos_offset = 0;
		cam_param[cam_cnt].delta_ref_ver_pos_offset = 0;

		cam_param[cam_cnt].plot_offset_axis = false;
		
		cam_param[cam_cnt].hw_roi = hwRoi;

		cam_param[cam_cnt].ref_hor_pos_offset = refPosOffset[0];
		cam_param[cam_cnt].ref_ver_pos_offset = refPosOffset[1];

		cam_param[cam_cnt].gain_auto = gainAutoParam[0];
		cam_param[cam_cnt].gain_auto_min = gainAutoParam[1];
		cam_param[cam_cnt].gain_auto_max = gainAutoParam[2];

		cam_param[cam_cnt].exposure_auto = exposureAutoParam[0];
		cam_param[cam_cnt].exposure_auto_min = exposureAutoParam[1];
		cam_param[cam_cnt].exposure_auto_max = exposureAutoParam[2];

		cam_param[cam_cnt].hw_roi_param[0] = hwRoiParam[0];
		cam_param[cam_cnt].hw_roi_param[1] = hwRoiParam[1];
		cam_param[cam_cnt].hw_roi_param[2] = hwRoiParam[2];
		cam_param[cam_cnt].hw_roi_param[3] = hwRoiParam[3];

		cam_param[cam_cnt].auto_target_value = autoTargetValue;

		cam_param[cam_cnt].dis_enable = false;

		cam_param[cam_cnt].auto_feedback_gain = autoFeedbackGain;
		cam_param[cam_cnt].auto_feedback_dws = autoFeedbackDws;
		cam_param[cam_cnt].auto_feedback_deadband = autoFeedbackDeadband;
		cam_param[cam_cnt].auto_feedback_target_thres = autoFeedbackTargetThres;
		cam_param[cam_cnt].binning = binning;

		//cam_param[cam_cnt].hor_pos_offset *= cam_param[cam_cnt].hor_sign;
		//cam_param[cam_cnt].ver_pos_offset *= cam_param[cam_cnt].ver_sign;		
		

		cam_cnt++;

	}

	return true;

}
//+------------------------------------------------------------------
/**
 *	method:	GigeCam::get_indexes
 * Get indexes from the image buffer
 *
*/
bool GigeCam::get_indexes(int32_t mode, int32_t nargin, int32_t *param_in, int32_t *param_out)
{
	int32_t i,  image_now = img_cur;

	/* get the last samples */
	if ((nargin == 2) && ((mode == BUF_GET_LAST_MODE) || (mode == BUF_GET_LAST_BUNCH_MODE)) && (param_in[0] < imageBufferSize)) {
		if (image_now >= param_in[0]) {
			param_out[0] = image_now - param_in[0] + 1;
			param_out[1] = param_out[0] + param_in[0];
			param_out[2] = param_out[3] = 0;
		}
		else {
			param_out[0] = imageBufferSize - (param_in[0] - image_now) + 1;
			param_out[1] = imageBufferSize;
			param_out[2] = 0;
			param_out[3] = image_now + 1;
		}
	}
	/* get the time between acquisition counters */
	else if ((nargin == 3) && ((mode == BUF_GET_BUNCH_MODE) || (mode == BUF_GET_BUNCH_BUNCH_MODE))) {
		if (img8.size() > 0) {
			if (param_in[1] >= (int)img8[image_now]->acquisition_counter) {
				return false;
			}
		}
		else if (img16.size() > 0) {
			if (param_in[1] >= (int)img16[image_now]->acquisition_counter) {
				return false;
			}
		}
		else {
			return false;
		}

		if ((param_in[1]-param_in[0]) > (imageBufferSize)) {
			return false;
		}

		int32_t idx_start = -1, idx_end = -1;
		param_in[1] += 1;
		if (img8.size() > 0) {
			for (i = 0; i < imageBufferSize; i++) {
				if (((int)img8[i]->acquisition_counter == param_in[0]) && (idx_start == -1))
					idx_start = i;
				if (((int)img8[i]->acquisition_counter ==  param_in[1]) && (idx_end == -1))
					idx_end = i;
				if ((idx_start != -1) && (idx_end != -1))
					break;
			}
		}
		else if (img16.size() > 0) {
			for (i = 0; i < imageBufferSize; i++) {
				if (((int)img16[i]->acquisition_counter == param_in[0]) && (idx_start == -1))
					idx_start = i;
				if (((int)img16[i]->acquisition_counter ==  param_in[1]) && (idx_end == -1))
					idx_end = i;
				if ((idx_start != -1) && (idx_end != -1))
					break;
			}
		}
		else {
			return false;
		}

		if ((idx_start == -1) || (idx_end == -1)) {
			return false;
		}
		else if (idx_start <= idx_end) {
			param_out[0] = idx_start; param_out[1] = idx_end;
			param_out[2] = 0; param_out[3] = 0;
		}
		else {
			param_out[0] = idx_start; param_out[1] = imageBufferSize;
			param_out[2] = 0; param_out[3] = idx_end;
		}
		/*
		printf("bs=%d be=%d, in[0]=%d in[1]=%d out[0]=%d out[1]=%d out[2]=%d out[3]=%d\n",
			img8[idx_start]->acquisition_counter,img8[idx_end]->acquisition_counter,
			param_in[0],param_in[1],param_out[0],param_out[1],
			param_out[2],param_out[3]);
		*/
	}
	/* get the samples between timestamps */
	/*
	else if ((nargin == 5) && (mode == BUF_GET_TIME_MODE)) {
		unsigned long long time_start,time_end, time_buf_pre, time_buf_post;
		int32_t idx_start = -1, idx_end = -1;
		time_start = (((unsigned long long)param_in[0]) << 32) + (unsigned long long)param_in[1];
		time_end =   (((unsigned long long)param_in[2]) << 32) + (unsigned long long)param_in[3];
		for (i = 0; i < imageBufferSize-1; i++) {
			time_buf_pre =  ((unsigned long long)img8[i]->acquisition_time.tv_sec << 32) + 
				(unsigned long long)img8[i]->acquisition_time.tv_usec;
			time_buf_post = ((unsigned long long)img8[i+1]->acquisition_time.tv_sec << 32) + 
				(unsigned long long)img8[i+1]->acquisition_time.tv_usec;
			if ((time_buf_pre <= time_start) && (time_buf_post >= time_start) && (idx_start == -1))
				idx_start = i;
			if ((time_buf_pre <= time_end) && (time_buf_post >= time_end) && (idx_end == -1))
				idx_end = i;
			if ((idx_start != -1) && (idx_end != -1))
				break;
		}
		if (idx_start < idx_end) {
			param_out[0] = idx_start; param_out[1] = idx_end;
			param_out[2] = 0; param_out[3] = 0;
		}
		else {
			param_out[0] = idx_start; param_out[1] = imageBufferSize;
			param_out[2] = 0; param_out[3] = idx_end;
		}
	}
	*/
	else
		return false;

	return true;

}

//+------------------------------------------------------------------
/**
 *	method:	GigeCam::init_buffers
 *
 *	description:	method to execute "init_buffers"
 *
 * @return	
 *
 */
//+------------------------------------------------------------------
bool GigeCam::init_buffers(GigeCam *gige)
{
	{string msg_log("init_buffers: initializing circular buffers");info_log(msg_log);}

	in_mutex->lock();

	gige->buf_phase = 0;
	init_rt_buffer(dataBufferSize, typeDouble, &gige->buf_phase);
	if (gige->buf_phase == 0) {
		in_mutex->unlock();return false;
	}
	set_mode(BUF_RT_CIRCULAR, gige->buf_phase);
	set_fill_hole(BUF_RT_NO_FILL, gige->buf_phase);

	gige->buf_area = 0;
	init_rt_buffer(dataBufferSize, typeDouble, &gige->buf_area);
	if (gige->buf_area == 0) {
		in_mutex->unlock();return false;
	}
	set_mode(BUF_RT_CIRCULAR, gige->buf_area);
	set_fill_hole(BUF_RT_NO_FILL, gige->buf_area);

	gige->buf_hor_pos = 0;
	init_rt_buffer(dataBufferSize, typeDouble, &gige->buf_hor_pos);
	if (gige->buf_hor_pos == 0) {
		in_mutex->unlock();return false;
	}
	set_mode(BUF_RT_CIRCULAR, gige->buf_hor_pos);
	set_fill_hole(BUF_RT_NO_FILL, gige->buf_hor_pos);

	gige->buf_hor_sigma = 0;
	init_rt_buffer(dataBufferSize, typeDouble, &gige->buf_hor_sigma);
	if (gige->buf_hor_sigma == 0) {
		in_mutex->unlock();return false;
	}
	set_mode(BUF_RT_CIRCULAR, gige->buf_hor_sigma);
	set_fill_hole(BUF_RT_NO_FILL, gige->buf_hor_sigma);

	gige->buf_saturation = 0;
	init_rt_buffer(dataBufferSize, typeDouble, &gige->buf_saturation);
	if (gige->buf_saturation == 0) {
		in_mutex->unlock();return false;
	}
	set_mode(BUF_RT_CIRCULAR, gige->buf_saturation);
	set_fill_hole(BUF_RT_NO_FILL, gige->buf_saturation);

	gige->buf_ver_pos = 0;
	init_rt_buffer(dataBufferSize, typeDouble, &gige->buf_ver_pos);
	if (gige->buf_ver_pos == 0) {
		in_mutex->unlock();return false;
	}
	set_mode(BUF_RT_CIRCULAR, gige->buf_ver_pos);
	set_fill_hole(BUF_RT_NO_FILL, gige->buf_ver_pos);

	gige->buf_ver_sigma = 0;
	init_rt_buffer(dataBufferSize, typeDouble, &gige->buf_ver_sigma);
	if (gige->buf_ver_sigma == 0) {
		in_mutex->unlock();return false;
	}
	set_mode(BUF_RT_CIRCULAR, gige->buf_ver_sigma);
	set_fill_hole(BUF_RT_NO_FILL, gige->buf_ver_sigma);

	gige->buf_intensity = 0;
	init_rt_buffer(dataBufferSize, typeDouble, &gige->buf_intensity);
	if (gige->buf_intensity == 0) {
		in_mutex->unlock();return false;
	}
	set_mode(BUF_RT_CIRCULAR, gige->buf_intensity);
	set_fill_hole(BUF_RT_NO_FILL, gige->buf_intensity);

	gige->buf_fit_error = 0;
	init_rt_buffer(dataBufferSize, typeLong, &gige->buf_fit_error);
	if (gige->buf_fit_error == 0) {
		in_mutex->unlock();return false;
	}
	set_mode(BUF_RT_CIRCULAR, gige->buf_fit_error);
	set_fill_hole(BUF_RT_NO_FILL, gige->buf_fit_error);

	gige->buf_acquisition_counter = 0;
	init_rt_buffer(dataBufferSize, typeLong, &gige->buf_acquisition_counter);
	if (gige->buf_acquisition_counter == 0) {
		in_mutex->unlock();return false;
	}
	set_mode(BUF_RT_CIRCULAR, gige->buf_acquisition_counter);
	set_fill_hole(BUF_RT_NO_FILL, gige->buf_acquisition_counter);

	gige->buf_max_val = 0;
	init_rt_buffer(dataBufferSize, typeLong, &gige->buf_max_val);
	if (gige->buf_max_val == 0) {
		in_mutex->unlock();return false;
	}
	set_mode(BUF_RT_CIRCULAR, gige->buf_max_val);
	set_fill_hole(BUF_RT_NO_FILL, gige->buf_max_val);

	gige->buf_gain = 0;
	init_rt_buffer(dataBufferSize, typeDouble, &gige->buf_gain);
	if (gige->buf_gain == 0) {
		in_mutex->unlock();return false;
	}
	set_mode(BUF_RT_CIRCULAR, gige->buf_gain);
	set_fill_hole(BUF_RT_NO_FILL, gige->buf_gain);

	gige->buf_exposure = 0;
	init_rt_buffer(dataBufferSize, typeLong, &gige->buf_exposure);
	if (gige->buf_exposure == 0) {
		in_mutex->unlock();return false;
	}
	set_mode(BUF_RT_CIRCULAR, gige->buf_exposure);
	set_fill_hole(BUF_RT_NO_FILL, gige->buf_exposure);

	in_mutex->unlock();

	{string msg_log("init_buffers: circular buffers initialized");info_log(msg_log);}

	return true;

}
//+------------------------------------------------------------------
/**
 *	method:	GigeCam::delete_buffers
 *
 *	description:	method to execute "delete_buffers"
 *
 * @return	
 *
 */
//+------------------------------------------------------------------
void GigeCam::delete_buffers(GigeCam *gige)
{

	in_mutex->lock();

	if (gige->buf_phase) {
		delete_rt_buffer(gige->buf_phase); gige->buf_phase = 0;
	}
	if (gige->buf_area) {
		delete_rt_buffer(gige->buf_area); gige->buf_area = 0;
	}
	if (gige->buf_hor_pos) {
		delete_rt_buffer(gige->buf_hor_pos); gige->buf_hor_pos = 0;
	}
	if (gige->buf_hor_sigma) {
		delete_rt_buffer(gige->buf_hor_sigma);  gige->buf_hor_sigma = 0;
	}
	if (gige->buf_saturation) {
		delete_rt_buffer(gige->buf_saturation); gige->buf_saturation = 0;
	}
	if (gige->buf_ver_pos) {
		delete_rt_buffer(gige->buf_ver_pos); gige->buf_ver_pos = 0;
	}
	if (gige->buf_ver_sigma) {
		delete_rt_buffer(gige->buf_ver_sigma); gige->buf_ver_sigma = 0;
	}
	if (gige->buf_intensity) {
		delete_rt_buffer(gige->buf_intensity); gige->buf_intensity = 0;
	}
	if (gige->buf_fit_error) {
		delete_rt_buffer(gige->buf_fit_error); gige->buf_fit_error = 0;	
	}
	if (gige->buf_acquisition_counter) {
		delete_rt_buffer(gige->buf_acquisition_counter); gige->buf_acquisition_counter = 0;
	}
	if (gige->buf_max_val) {
		delete_rt_buffer(gige->buf_max_val); gige->buf_max_val = 0;
	}
	if (gige->buf_gain) {
		delete_rt_buffer(gige->buf_gain); gige->buf_gain = 0;
	}
	if (gige->buf_exposure) {
		delete_rt_buffer(gige->buf_exposure); gige->buf_exposure = 0;
	}

	in_mutex->unlock();

}
//+------------------------------------------------------------------
/**
 *	method:	GigeCam::modify_configuration
 * Modify configuration parameters (horizontal, vertical calibration)
 *
*/
void GigeCam::modify_configuration(bool ccd_param, bool calib_param)
{
	
	int32_t cur_cam_cnt = 0, conf_size, cur_cameras;
	vector<string>	cur_configuration, new_configuration;
	cam_param_t *cur_cam_param;
	Tango::DbData db_data;

	try {
		db_data.push_back(Tango::DbDatum("Configuration"));
		/* global configuration */
		if (global_flag_prop) {
			Tango::Database* db = new Tango::Database();
			db->get_property("Gigecam",db_data);
		}
		/* local configuration */
		else {
			get_db_device()->get_property(db_data); 
		}
	}
	catch (CORBA::Exception &e) {
		Tango::Except::throw_exception (
			(const char *)"Failed to update configuration",
			(const char *)"Can't read configuration",
			(const char *)"GigeCam::modify_configuration()");
	}
	/* read data */
	if (db_data[0].is_empty()) { 
		Tango::Except::throw_exception (
			(const char *)"Can't read global configuration",
			(const char *)"Configuration is empty",
			(const char *)"GigeCam::modify_configuration()");
	}
	else {
		db_data[0] >> cur_configuration;
	}

	/* local configuration size */
	cur_cameras = cur_configuration.size();

	cur_cam_param = new cam_param_t[cur_cameras];
	if (cur_cam_param == 0) {
		Tango::Except::throw_exception (
			(const char *)"Can't locate memory",
			(const char *)"Configuration failed",
			(const char *)"GigeCam::modify_configuration()");
	}


	for (vector<string>::const_iterator it = cur_configuration.begin(); it < cur_configuration.end(); it++) {
		std::string token;
		std::istringstream iss(*it);
		conf_size = 0;
		while ( getline(iss, token, ':') ) {
			switch (conf_size) {
				case 0: cur_cam_param[cur_cam_cnt].name = token; 
					camera_name_index[token.c_str()] = cur_cam_cnt+1; break;
				case 1: cur_cam_param[cur_cam_cnt].ipaddress = token;
					camera_ip_index[token.c_str()] = cur_cam_cnt+1; break; 
				case 2: cur_cam_param[cur_cam_cnt].img_depth = atol(token.c_str());break;
				case 3: cur_cam_param[cur_cam_cnt].exposure = atol(token.c_str());break; 
				case 4: cur_cam_param[cur_cam_cnt].gain = (double) atof(token.c_str());break;
				case 5: cur_cam_param[cur_cam_cnt].mirror = (bool) atol(token.c_str());break;
				case 6: cur_cam_param[cur_cam_cnt].rotation = atof(token.c_str());break;
				case 7: cur_cam_param[cur_cam_cnt].trigger = (bool) atol(token.c_str());break;
				case 8: cur_cam_param[cur_cam_cnt].hor_calibration = (double) atof(token.c_str());break;
				case 9: cur_cam_param[cur_cam_cnt].ver_calibration = (double) atof(token.c_str());break;
				case 10: cur_cam_param[cur_cam_cnt].hor_pos_offset = (double) atof(token.c_str());break;
				case 11: cur_cam_param[cur_cam_cnt].ver_pos_offset = (double) atof(token.c_str());break;
				case 12: cur_cam_param[cur_cam_cnt].hor_sign = (double) atof(token.c_str());break;
				case 13: cur_cam_param[cur_cam_cnt].ver_sign = (double) atof(token.c_str());break;
			}
			if ((++conf_size) > 13)
				break;
  	}

		if (cur_cam_param[cur_cam_cnt].name == cam_param[cam_idx].name) {
	
			if (!global_flag_prop) {
				if ((img8.size() > 0) || (img16.size() > 0))  {
					if (ccd_param) {
						cur_cam_param[cur_cam_cnt].exposure = *attr_Exposure_read;
						cur_cam_param[cur_cam_cnt].gain = *attr_Gain_read;
						cur_cam_param[cur_cam_cnt].trigger = (bool) *attr_Trigger_read;
						cur_cam_param[cur_cam_cnt].img_depth = *attr_ImageDepth_read;
					}
				}
			}

			if (calib_param) {
				if (img8.size() > 0) {
					cur_cam_param[cur_cam_cnt].hor_calibration = img8[img_cur]->hor_calibration;
					cur_cam_param[cur_cam_cnt].ver_calibration = img8[img_cur]->ver_calibration;
					cur_cam_param[cur_cam_cnt].hor_pos_offset = img8[img_cur]->hor_pos_offset;
					cur_cam_param[cur_cam_cnt].ver_pos_offset = img8[img_cur]->ver_pos_offset;
					cur_cam_param[cur_cam_cnt].mirror = img8[img_cur]->get_mirror();
				}
				if (img16.size() > 0) {
					cur_cam_param[cur_cam_cnt].hor_calibration = img16[img_cur]->hor_calibration;
					cur_cam_param[cur_cam_cnt].ver_calibration = img16[img_cur]->ver_calibration;
					cur_cam_param[cur_cam_cnt].hor_pos_offset = img16[img_cur]->hor_pos_offset;
					cur_cam_param[cur_cam_cnt].ver_pos_offset = img16[img_cur]->ver_pos_offset;
					cur_cam_param[cur_cam_cnt].mirror = img16[img_cur]->get_mirror();
				}
				cam_param[cur_cam_cnt].hor_calibration = cur_cam_param[cur_cam_cnt].hor_calibration;
				cam_param[cur_cam_cnt].ver_calibration = cur_cam_param[cur_cam_cnt].ver_calibration;
				cam_param[cur_cam_cnt].hor_pos_offset = cur_cam_param[cur_cam_cnt].hor_pos_offset * cur_cam_param[cur_cam_cnt].hor_sign;
				cam_param[cur_cam_cnt].ver_pos_offset = cur_cam_param[cur_cam_cnt].ver_pos_offset;
				cur_cam_param[cam_idx].rotation = cam_param[cur_cam_cnt].rotation;
			}

		}
		cur_cam_cnt++;
	}

	char conf_str[300];
	for (int32_t i = 0; i < cur_cam_cnt; i++) {
		memset(conf_str,0,300);
		sprintf(conf_str,"%s:%s:%d:%d:%.1f:%d:%.2f:%d:%.2f:%.2f:%.3f:%.3f:%.0f:%.0f",
			cur_cam_param[i].name.c_str(),
			cur_cam_param[i].ipaddress.c_str(),
			cur_cam_param[i].img_depth, 
			cur_cam_param[i].exposure,
			cur_cam_param[i].gain,
			(int)cur_cam_param[i].mirror,
			cur_cam_param[i].rotation,
			(int)cur_cam_param[i].trigger, 
			cam_param[i].hor_calibration,
			cam_param[i].ver_calibration, 
			cam_param[i].hor_pos_offset,
			cam_param[i].ver_pos_offset,
			(double)cur_cam_param[i].hor_sign,
			(double)cur_cam_param[i].ver_sign);
		new_configuration.push_back(conf_str);
	}

	/* now write into database */

	Tango::Database *db = new Tango::Database(host_rw,port_rw);
	try {
		Tango::DbDatum config("Configuration");
		Tango::DbData db_data_put;
		config << new_configuration;
		db_data_put.push_back(config);
		if (global_flag_prop) {
			db->put_property("Gigecam",db_data_put);
		}
		else {
			db->put_device_property(get_name(), db_data_put);
		}
	}
	catch (Tango::DevFailed &e) {
		delete db;
		Tango::Except::throw_exception (
			(const char *)"Failed to update configuration",
			(const char *)"Can't write into database",
			(const char *)"GigeCam::modify_configuration()");				
	}


	if ((!global_flag_prop) && ccd_param) {
		cur_cam_cnt = 0;

		/* fast calculation 0=disable, 1=enable */
		try {
			Tango::DbDatum config("FastCalculation");
			Tango::DbData db_data_put;
			memset(conf_str,0,300);
			if (cam_param[cur_cam_cnt].fast_calc)
				sprintf(conf_str,"true");
			else
				sprintf(conf_str,"false");
			string val(conf_str); config << val;
			db_data_put.push_back(config);
			db->put_device_property(get_name(), db_data_put);
		}
		catch (Tango::DevFailed &e) {
			delete db;
			Tango::Except::throw_exception (
				(const char *)"Failed to update configuration",
				(const char *)"Can't write into database",
				(const char *)"GigeCam::modify_configuration()");				
		}

		/* 0=disable automatic roi, 1=enable automatic roi */
		try {
			Tango::DbDatum config("AutoRoi");
			Tango::DbData db_data_put;
			memset(conf_str,0,300);
			if (cam_param[cur_cam_cnt].auto_roi)
				sprintf(conf_str,"true");
			else
				sprintf(conf_str,"false");
			string val(conf_str); config << val;
			db_data_put.push_back(config);
			db->put_device_property(get_name(), db_data_put);
		}
		catch (Tango::DevFailed &e) {
			delete db;
			Tango::Except::throw_exception (
				(const char *)"Failed to update configuration",
				(const char *)"Can't write into database",
				(const char *)"GigeCam::modify_configuration()");				
		}

		/* 0=disable manual roi, 1=enable manual roi */
		try {
			Tango::DbDatum config("SwRoi");
			Tango::DbData db_data_put;
			memset(conf_str,0,300);
			if (cam_param[cur_cam_cnt].sw_roi)
				sprintf(conf_str,"true");
			else
				sprintf(conf_str,"false");
			string val(conf_str); config << val;
			db_data_put.push_back(config);
			db->put_device_property(get_name(), db_data_put);
		}
		catch (Tango::DevFailed &e) {
			delete db;
			Tango::Except::throw_exception (
				(const char *)"Failed to update configuration",
				(const char *)"Can't write into database",
				(const char *)"GigeCam::modify_configuration()");				
		}

		/* 0=disable hw roi, 1=enable hw roi */
		try {
			Tango::DbDatum config("HwRoi");
			Tango::DbData db_data_put;
			memset(conf_str,0,300);
			if (cam_param[cur_cam_cnt].hw_roi)
				sprintf(conf_str,"true");
			else
				sprintf(conf_str,"false");
			string val(conf_str); config << val;
			db_data_put.push_back(config);
			db->put_device_property(get_name(), db_data_put);
		}
		catch (Tango::DevFailed &e) {
			delete db;
			Tango::Except::throw_exception (
				(const char *)"Failed to update configuration",
				(const char *)"Can't write into database",
				(const char *)"GigeCam::modify_configuration()");				
		}

		/* 0=rms 2D, 1=gaussian fit, 2=asym gaussian fit, 3=confiteor */
		try {
			Tango::DbDatum config("ProcessMode");
			Tango::DbData db_data_put;
			memset(conf_str,0,300); sprintf(conf_str,"%d",(int)cam_param[cam_idx].process_mode);
			string val(conf_str); config << val;
			db_data_put.push_back(config);
			db->put_device_property(get_name(), db_data_put);
		}
		catch (Tango::DevFailed &e) {
			delete db;
			Tango::Except::throw_exception (
				(const char *)"Failed to update configuration",
				(const char *)"Can't write into database",
				(const char *)"GigeCam::modify_configuration()");				
		}

		/* 0=rms 2D, 1=gaussian fit, 2=asym gaussian fit, 3=confiteor */
		try {
			Tango::DbDatum config("RoiThreshold");
			Tango::DbData db_data_put;
			memset(conf_str,0,300); sprintf(conf_str,"%.3f",cam_param[cam_idx].roi_threshold);
			string val(conf_str); config << val;
			db_data_put.push_back(config);
			db->put_device_property(get_name(), db_data_put);
		}
		catch (Tango::DevFailed &e) {
			delete db;
			Tango::Except::throw_exception (
				(const char *)"Failed to update configuration",
				(const char *)"Can't write into database",
				(const char *)"GigeCam::modify_configuration()");				
		}

		/* search background param */
		try {
			Tango::DbDatum config("SearchBackgroundParam");
			Tango::DbData db_data_put;
			vector<string>	val;
			for (int i = 0; i < 2; i++) {
				memset(conf_str,0,300);
				sprintf(conf_str,"%.2f",cam_param[cam_idx].search_background_param[i]);
				val.push_back(conf_str);
			}
			config << val;
			db_data_put.push_back(config);
			db->put_device_property(get_name(), db_data_put);
		}
		catch (Tango::DevFailed &e) {
			delete db;
			Tango::Except::throw_exception (
				(const char *)"Failed to update configuration",
				(const char *)"Can't write into database",
				(const char *)"GigeCam::modify_configuration()");				
		}

		/* manual ROI x1,y1,x2,y2 */
		try {
			Tango::DbDatum config("RoiParam");
			Tango::DbData db_data_put;
			vector<string>	val;
			for (int i = 0; i < 4; i++) {
				memset(conf_str,0,300);
				sprintf(conf_str,"%d",(int)cam_param[cam_idx].manual_roi_param[i]);
				val.push_back(conf_str);
			}
			config << val;
			db_data_put.push_back(config);
			db->put_device_property(get_name(), db_data_put);
		}
		catch (Tango::DevFailed &e) {
			delete db;
			Tango::Except::throw_exception (
				(const char *)"Failed to update configuration",
				(const char *)"Can't write into database",
				(const char *)"GigeCam::modify_configuration()");				
		}

		// reference target position
		try {
			Tango::DbDatum config("RefPosOffset");
			Tango::DbData db_data_put;
			vector<string>	val;
			memset(conf_str,0,300);
			sprintf(conf_str,"%.4f",cam_param[cam_idx].ref_hor_pos_offset);
			val.push_back(conf_str);
			memset(conf_str,0,300);
			sprintf(conf_str,"%.4f",cam_param[cam_idx].ref_ver_pos_offset);
			val.push_back(conf_str);
			config << val;
			db_data_put.push_back(config);
			db->put_device_property(get_name(), db_data_put);
		}
		catch (Tango::DevFailed &e) {
			delete db;
			Tango::Except::throw_exception (
				(const char *)"Failed to update configuration",
				(const char *)"Can't write into database",
				(const char *)"GigeCam::modify_configuration()");				
		}

		// automatic target value
		try {
			Tango::DbDatum config("AutoTargetValue");
			Tango::DbData db_data_put;
			memset(conf_str,0,300);
			sprintf(conf_str,"%d",(int)cam_param[cam_idx].auto_target_value);
			string val(conf_str);
			config << val;
			db_data_put.push_back(config);
			db->put_device_property(get_name(), db_data_put);
		}
		catch (Tango::DevFailed &e) {
			delete db;
			Tango::Except::throw_exception (
				(const char *)"Failed to update configuration",
				(const char *)"Can't write into database",
				(const char *)"GigeCam::modify_configuration()");				
		}

		/* auto feedback gain */
		try {
			Tango::DbDatum config("AutoFeedbackGain");
			Tango::DbData db_data_put;
			memset(conf_str,0,300);
			sprintf(conf_str,"%.3f",cam_param[cam_idx].auto_feedback_gain);
			string val(conf_str);
			config << val;
			db_data_put.push_back(config);
			db->put_device_property(get_name(), db_data_put);
		}
		catch (Tango::DevFailed &e) {
			delete db;
			Tango::Except::throw_exception (
				(const char *)"Failed to update configuration",
				(const char *)"Can't write into database",
				(const char *)"GigeCam::modify_configuration()");				
		}

		try {
			Tango::DbDatum config("AutoFeedbackDws");
			Tango::DbData db_data_put;
			memset(conf_str,0,300);
			sprintf(conf_str,"%d",cam_param[cam_idx].auto_feedback_dws);
			string val(conf_str);
			config << val;
			db_data_put.push_back(config);
			db->put_device_property(get_name(), db_data_put);
		}
		catch (Tango::DevFailed &e) {
			delete db;
			Tango::Except::throw_exception (
				(const char *)"Failed to update configuration",
				(const char *)"Can't write into database",
				(const char *)"GigeCam::modify_configuration()");				
		}

		try {
			Tango::DbDatum config("AutoFeedbackDeadband");
			Tango::DbData db_data_put;
			memset(conf_str,0,300);
			sprintf(conf_str,"%d",cam_param[cam_idx].auto_feedback_deadband);
			string val(conf_str);
			config << val;
			db_data_put.push_back(config);
			db->put_device_property(get_name(), db_data_put);
		}
		catch (Tango::DevFailed &e) {
			delete db;
			Tango::Except::throw_exception (
				(const char *)"Failed to update configuration",
				(const char *)"Can't write into database",
				(const char *)"GigeCam::modify_configuration()");				
		}

		try {
			Tango::DbDatum config("AutoFeedbackTargetThres");
			Tango::DbData db_data_put;
			memset(conf_str,0,300);
			sprintf(conf_str,"%d",cam_param[cam_idx].auto_feedback_target_thres);
			string val(conf_str);
			config << val;
			db_data_put.push_back(config);
			db->put_device_property(get_name(), db_data_put);
		}
		catch (Tango::DevFailed &e) {
			delete db;
			Tango::Except::throw_exception (
				(const char *)"Failed to update configuration",
				(const char *)"Can't write into database",
				(const char *)"GigeCam::modify_configuration()");				
		}

		try {
			Tango::DbDatum config("Binning");
			Tango::DbData db_data_put;
			memset(conf_str,0,300);
			sprintf(conf_str,"%d",cam_param[cam_idx].binning);
			string val(conf_str);
			config << val;
			db_data_put.push_back(config);
			db->put_device_property(get_name(), db_data_put);
		}
		catch (Tango::DevFailed &e) {
			delete db;
			Tango::Except::throw_exception (
				(const char *)"Failed to update configuration",
				(const char *)"Can't write into database",
				(const char *)"GigeCam::modify_configuration()");				
		}

		/* manual ROI x1,y1,x2,y2 */
		try {
			Tango::DbDatum config("HwRoiParam");
			Tango::DbData db_data_put;
			vector<string>	val;
			for (int i = 0; i < 4; i++) {
				memset(conf_str,0,300);
				sprintf(conf_str,"%d",(int)cam_param[cam_idx].hw_roi_param[i]);
				val.push_back(conf_str);
			}
			config << val;
			db_data_put.push_back(config);
			db->put_device_property(get_name(), db_data_put);
		}
		catch (Tango::DevFailed &e) {
			delete db;
			Tango::Except::throw_exception (
				(const char *)"Failed to update configuration",
				(const char *)"Can't write into database",
				(const char *)"GigeCam::modify_configuration()");				
		}

		/* auto roi param: grid size, lp order, roi threshold */
		try {
			Tango::DbDatum config("AutoRoiParam");
			Tango::DbData db_data_put;
			vector<string>	val;
			memset(conf_str,0,300);
			sprintf(conf_str,"%d",(int)cam_param[cam_idx].gridsize);val.push_back(conf_str);
			memset(conf_str,0,300);
			sprintf(conf_str,"%d",(int)cam_param[cam_idx].lp_order);val.push_back(conf_str);
			memset(conf_str,0,300);
			sprintf(conf_str,"%.1f",cam_param[cam_idx].roi_threshold);val.push_back(conf_str);
			config << val;
			db_data_put.push_back(config);
			db->put_device_property(get_name(), db_data_put);
		}
		catch (Tango::DevFailed &e) {
			delete db;
			Tango::Except::throw_exception (
				(const char *)"Failed to update configuration",
				(const char *)"Can't write into database",
				(const char *)"GigeCam::modify_configuration()");				
		}

		// automatic gain auto param
		try {
			Tango::DbDatum config("GainAutoParam");
			Tango::DbData db_data_put;
			vector<string>	val;
			memset(conf_str,0,300);
			sprintf(conf_str,"%d",(int)cam_param[cam_idx].gain_auto);val.push_back(conf_str);
			memset(conf_str,0,300);
			sprintf(conf_str,"%.1f",cam_param[cam_idx].gain_auto_min);val.push_back(conf_str);
			memset(conf_str,0,300);
			sprintf(conf_str,"%.1f",cam_param[cam_idx].gain_auto_max);val.push_back(conf_str);
			config << val;
			db_data_put.push_back(config);
			db->put_device_property(get_name(), db_data_put);
		}
		catch (Tango::DevFailed &e) {
			delete db;
			Tango::Except::throw_exception (
				(const char *)"Failed to update configuration",
				(const char *)"Can't write into database",
				(const char *)"GigeCam::modify_configuration()");				
		}

		// automatic exposure auto param
		try {
			Tango::DbDatum config("ExposureAutoParam");
			Tango::DbData db_data_put;
			vector<string>	val;
			memset(conf_str,0,300);
			sprintf(conf_str,"%d",cam_param[cam_idx].exposure_auto);val.push_back(conf_str);
			memset(conf_str,0,300);
			sprintf(conf_str,"%d",cam_param[cam_idx].exposure_auto_min);val.push_back(conf_str);
			memset(conf_str,0,300);
			sprintf(conf_str,"%d",cam_param[cam_idx].exposure_auto_max);val.push_back(conf_str);
			config << val;
			db_data_put.push_back(config);
			db->put_device_property(get_name(), db_data_put);
		}
		catch (Tango::DevFailed &e) {
			delete db;
			Tango::Except::throw_exception (
				(const char *)"Failed to update configuration",
				(const char *)"Can't write into database",
				(const char *)"GigeCam::modify_configuration()");				
		}

	}
	
	delete db; 

}


/**
 *	method:	GigeCam::set_roi_param
 * Modify configuration parameters (horizontal, vertical calibration)
 *
*/
void GigeCam::set_roi_param(int32_t *roi_array_p)
{

	// multiple of 16
	if (strcmp(cam_param[cam_idx].model,"Basler acA1300-75gm") == 0) {
			roi_array_p[1] += 8;
			roi_array_p[1] += 8;
			roi_array_p[3] &= ~0xf;
			roi_array_p[3] &= ~0xf;
			roi_array_p[3] -= 1;
			roi_array_p[0] += 8;
			roi_array_p[0] += 8;
			roi_array_p[0] &= ~0xf;
			roi_array_p[2] &= ~0xf;
			roi_array_p[2] -= 1;
	}
	else if (strcmp(cam_param[cam_idx].model,"Basler acA1440-73gm") == 0) {
		if (roi_array_p[0] & 0x1)
			roi_array_p[0] +=1;
		if (roi_array_p[1] & 0x1)
			roi_array_p[1] +=1;
		if (roi_array_p[2] & 0x1)
			roi_array_p[2] -=1;
		if (roi_array_p[3] & 0x1)
			roi_array_p[3] -=1;
	}

	// divided by two without rest
	else if ((strcmp(cam_param[cam_idx].model,"Basler avA1600-50gm") == 0) || 
		(strcmp(cam_param[cam_idx].model,"Basler avA2300-25gm") == 0)) {
		if (!((roi_array_p[3]-roi_array_p[1]) & 0x1)) {
			roi_array_p[1] -= 1;
			if (roi_array_p[1] < 0) {
				roi_array_p[1] = 0;
				roi_array_p[3] -= 1;
			}
		}
		if (!((roi_array_p[2]-roi_array_p[0]) & 0x1)) {
			roi_array_p[0] -= 1;
			if (roi_array_p[0] < 0) {
				roi_array_p[0] = 0;
				roi_array_p[2] -= 1;
			}
		}
	}
	else if ((strcmp(cam_param[cam_idx].model,"Basler acA1920-50gm") == 0) || 
		(strcmp(cam_param[cam_idx].model,"Basler acA2040-35gm") == 0)) {
		// offset X,Y multiple of 2
		// horizontal
		if (cam_param[cam_idx].mirror) {
			if ((cam_param[cam_idx].max_width - (roi_array_p[2] + 1)) & 0x1) {
				roi_array_p[2] -= 1;
				if (roi_array_p[2] < 0)
					roi_array_p[2] = 0;
			}
		}
		else {
			if (roi_array_p[0] & 0x1) {
				roi_array_p[0] -= 1;
			}
		}
		// vertical
		if (roi_array_p[1] & 0x1) {
			roi_array_p[1] -= 1;
		}
	}
	else if (strcmp(cam_param[cam_idx].model,"Basler acA780-75gm") == 0) {

		// offset X,Y multiple of 2 
		// horizontal
		if (cam_param[cam_idx].mirror) {
			if ((cam_param[cam_idx].max_width - (roi_array_p[2] + 1)) & 0x1) {
				roi_array_p[2] -= 1;
				if (roi_array_p[2] < 0)
					roi_array_p[2] = 0;
			}
		}
		else {
			if (roi_array_p[0] & 0x1) {
				roi_array_p[0] -= 1;
			}
		}
		// vertical
		if (roi_array_p[1] & 0x1) {
			roi_array_p[1] -= 1;
		}

		// hor size multiple of 4
		if ((roi_array_p[2] - roi_array_p[0] + 1) & 0x3) {
			if (cam_param[cam_idx].mirror) {
				roi_array_p[0] += (4 - ((roi_array_p[2] - roi_array_p[0] + 1) & 0x3));
			}
			else {
				roi_array_p[2] += (4 - ((roi_array_p[2] - roi_array_p[0] + 1) & 0x3));
			}
		}
		// ver size multiple of 4
		if ((roi_array_p[3] - roi_array_p[1] + 1) & 0x3) {
			roi_array_p[3] += (4 - ((roi_array_p[3] - roi_array_p[1] + 1) & 0x3));
		}

	}


	for (int32_t i = 0; i < 4; i++)
		cam_param[cam_idx].manual_roi_param[i] = roi_array_p[i];

	cam_param[cam_idx].dis_enable = false; /* disable digital image stabilization */

}



void GigeCam::info_log(string str)
{
	/*INFO_STREAM << str << endl;*/
	struct timeval timenow;
	gettimeofday(&timenow,NULL);
	cout << ctime(&(timenow.tv_sec)) << __FUNCTION__ << ": " << str << endl;	
}

timespec GigeCam::difftime(timespec start, timespec end)
{
	timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}

double GigeCam::timespec2ms(timespec ts)
{
	return (double)ts.tv_sec+(double)ts.tv_nsec/1000000000;
}

	/*----- PROTECTED REGION END -----*/	//	GigeCam::namespace_ending
} //	namespace

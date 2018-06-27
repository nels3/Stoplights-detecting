#include "ids.h"

void IDS_PARAMETERS::initialize_camera(HIDS* hCam) {
	INT nRet = is_InitCamera(hCam, NULL);
	if (nRet == IS_SUCCESS) {
		std::cout << "Camera initialized!" << std::endl;
	}

	// Setting the pixels clock
	UINT nPixelClockDefault = 21;
	nRet = is_PixelClock(*hCam, IS_PIXELCLOCK_CMD_SET, (void*)&PixelClock, sizeof(PixelClock));

	if (nRet == IS_SUCCESS) {
		std::cout << "Camera pixel clock succesfully set!" << std::endl;
	}
	else if (nRet == IS_NOT_SUPPORTED) {
		std::cout << "Camera pixel clock setting is not supported!" << std::endl;
	}

	// Set the color mode
	INT colorMode = IS_CM_RGB8_PACKED;
	nRet = is_SetColorMode(*hCam, colorMode);

	if (nRet == IS_SUCCESS) {
		std::cout << "Camera color mode succesfully set!" << std::endl;
	}

	INT displayMode = IS_SET_DM_DIB;
	nRet = is_SetDisplayMode(*hCam, displayMode);

	is_Exposure(*hCam, IS_EXPOSURE_CMD_SET_EXPOSURE, (void*)&Exposure, sizeof(Exposure));

	is_SetFrameRate(*hCam, FPS, &NEWFPS);

	// Set gain boost
	nRet = is_SetGainBoost(*hCam, IS_SET_GAINBOOST_ON);
	if (nRet == IS_SUCCESS) {
		std::cout << "Enabled Setting Gain!" << std::endl;
	}
	
	is_SetHWGainFactor(*hCam, IS_SET_MASTER_GAIN_FACTOR, Master_GAIN_Factor);
	is_SetHWGainFactor(*hCam, IS_SET_GREEN_GAIN_FACTOR, Green_GAIN_Factor);
	is_SetHWGainFactor(*hCam, IS_SET_BLUE_GAIN_FACTOR, Blue_GAIN_Factor);
	is_SetHWGainFactor(*hCam, IS_SET_RED_GAIN_FACTOR, Red_GAIN_Factor);

}


// Capture a frame from IDS
void IDS_PARAMETERS::get_frame(HIDS* hCam, int width, int height,cv::Mat& mat) {
	char* pMem = NULL;
	int memID = 0;
	is_AllocImageMem(*hCam, width,height, 24, &pMem, &memID);

	is_SetImageMem(*hCam, pMem, memID);
	is_FreezeVideo(*hCam, IS_WAIT);

	VOID* pMem_b;
	int retInt = is_GetImageMem(*hCam, &pMem_b);
	if (retInt != IS_SUCCESS) {
		std::cout << "Image data could not be read from memory!" << std::endl;
	}
	memcpy(mat.ptr(), pMem_b, width*height*3);

}

void IDS_PARAMETERS::change_params(HIDS* hCam) {
	PixelClock = (UINT)pixelclock_slider;
	is_PixelClock(*hCam, IS_PIXELCLOCK_CMD_SET, (void*)&PixelClock, sizeof(PixelClock));
	Exposure = (DOUBLE)(exposure_slider/10);
	is_Exposure(*hCam, IS_EXPOSURE_CMD_SET_EXPOSURE, (void*)&Exposure, sizeof(Exposure));
	FPS = (DOUBLE)fps_slider;
	is_SetFrameRate(*hCam, FPS, &NEWFPS);
	is_SetHWGainFactor(*hCam, IS_SET_MASTER_GAIN_FACTOR, Master_GAIN_Factor);
	is_SetHWGainFactor(*hCam, IS_SET_GREEN_GAIN_FACTOR, Green_GAIN_Factor);
	is_SetHWGainFactor(*hCam, IS_SET_BLUE_GAIN_FACTOR, Blue_GAIN_Factor);
	is_SetHWGainFactor(*hCam, IS_SET_RED_GAIN_FACTOR, Red_GAIN_Factor);
}

void IDS_PARAMETERS::setting_auto_params(HIDS* hCam) {
	double enable = 1;
	double disable = 0;
	is_SetAutoParameter(*hCam, IS_SET_ENABLE_AUTO_GAIN, &enable, 0);
	is_SetAutoParameter(*hCam, IS_SET_ENABLE_AUTO_WHITEBALANCE, &enable, 0);
	is_SetAutoParameter(*hCam, IS_SET_ENABLE_AUTO_FRAMERATE, &disable, 0);
	is_SetAutoParameter(*hCam, IS_SET_ENABLE_AUTO_SHUTTER, &disable, 0);
	is_SetAutoParameter(*hCam, IS_SET_ENABLE_AUTO_SENSOR_GAIN, &enable, 0);
	is_SetAutoParameter(*hCam, IS_SET_ENABLE_AUTO_SENSOR_WHITEBALANCE, &enable, 0);
	is_SetAutoParameter(*hCam, IS_SET_ENABLE_AUTO_SENSOR_SHUTTER, &disable, 0);
}
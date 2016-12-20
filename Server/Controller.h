#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "stdafx.h"
#include "Sever.h"
#include "vision.h"
#include "trilateration.h"
#include "LokiLog\LokiLog\LokiLog.h"

struct ThreadParamController
{
	HWND *hwnd, *hwndCalibrationButton, *hwndMarkDistance;;
	HINSTANCE *hInstance;

	HRC *hrc;
	KinectSever *sever;
	DataBuffer *buffer;
	Trilateration *calc;
	Vision *vision;
	
	CameraSpacePoint *pSensor;


	DOUBLE *fAngle;
	INT nStatePos, nResPos; //position of two message window

	INT *nCalibrationInterface, *nCalibrationState;
	
	INT nClient;

	LokiLog *lResult;
	//log file class

};

class Controller
{
public:
	Controller(LPCWSTR &sFileName, HWND *hwnd, HINSTANCE *hInstance);
	~Controller();

	void StartController();


	static void ClientState(INT nClientIndex, BOOL bConnect, INT nCalibration, CameraSpacePoint *pCurrentSensor, CameraSpacePoint *pSensor, INT nDelay, HWND *hwnd);
	//print message in state area

	BOOL ButtonClick_Calibration(DOUBLE fDistance);

	INT CalibrationStep();

	void CalibrationInterfaceStepUp();

	static void ClientResult(HWND *hwnd, INT nJointIndex, CameraSpacePoint *pJoint);
	//print message in result area

	static void ShowSeverTime(ThreadParamController *param);
	
private:
	INT nClient;
	INT nPortList[MAX_SENSOR];
	HWND *hwnd, hwndCalibrationButton,hwndMarkDistance;
	HINSTANCE *hInstance;
	Vision *vision;

	HRC Clock;
	//hrc

	KinectSever *sKinect;
	//sever

	DataBuffer *bKinect;
	//buffer of Trilateration data

	Trilateration tKinect;
	//calc class

	INT nCalibrationState;
	//0 for none, 1 for p1 got , 2 for completed

	INT nCalibrationInterface;
	//step of create the interface

	CameraSpacePoint pSensor[MAX_SENSOR], pTmpSensor[2][MAX_SENSOR];

	DOUBLE fAngle[MAX_SENSOR];
	//angle of coordinate system between mark and sensors

	ThreadParamController param;
	
	
	static void ThreadRefresh(ThreadParamController *param);

	static INT PickAvailData(CameraSpacePoint *pStored, INT *nShade, DOUBLE *fSpeed, const INT nClient);




};

#endif
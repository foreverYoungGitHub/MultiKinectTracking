#ifndef VISION_H
#define VISION_H

#include "stdafx.h"

class Vision
{
public:
	Vision(RECT rDisplay, HWND *hwnd, DOUBLE fRatio);
	~Vision();

	void DrawHuman(CameraSpacePoint *pJoint);


private:
	RECT rDisplay;
	HDC hdc;
	HWND *hwnd;
	DOUBLE fRatio;
	HPEN pDraw;
	
	CameraSpacePoint pStandard;
	//use as the zero


	void DrawBody();
	void DrawLeg();
	void DrawArm();

	void DrawCircle(CameraSpacePoint pCircle);

	INT CoordinateTrans(DOUBLE fInput, BOOL bisX);

	void DrawLine(CameraSpacePoint p1, CameraSpacePoint p2);
	
};



#endif
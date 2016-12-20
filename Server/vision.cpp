#include "vision.h"

Vision::Vision(RECT rDisplay, HWND *hwnd, DOUBLE fRatio)
{
	this->hwnd = hwnd;
	this->rDisplay = rDisplay;
	this->fRatio = fRatio;
	
	
}

Vision::~Vision()
{

}

void Vision::DrawHuman(CameraSpacePoint *pJoint)
{
	PAINTSTRUCT ps;
	InvalidateRect(*hwnd, &rDisplay, TRUE);
	hdc = BeginPaint(*hwnd, &ps);
	HBRUSH bBackGround = CreateSolidBrush(RGB(255, 255, 255));
	FillRect(hdc, &rDisplay, bBackGround);


	SelectObject(hdc, pDraw);

	pDraw = CreatePen(PS_SOLID, PEN_SIZE, RGB(0, 0, 0));
	//init
	pStandard = pJoint[JointType_SpineBase]; // 0 is [JointType_SpineBase]

	DrawLine(pJoint[JointType_Head], pJoint[JointType_Neck]);
	DrawLine(pJoint[JointType_Neck], pJoint[JointType_SpineShoulder]);
	DrawLine(pJoint[JointType_SpineShoulder], pJoint[JointType_SpineMid]);
	DrawLine(pJoint[JointType_SpineMid], pJoint[JointType_SpineBase]);
	DrawLine(pJoint[JointType_SpineShoulder], pJoint[JointType_ShoulderRight]);
	DrawLine(pJoint[JointType_SpineShoulder], pJoint[JointType_ShoulderLeft]);
	DrawLine(pJoint[JointType_SpineBase], pJoint[JointType_HipRight]);
	DrawLine(pJoint[JointType_SpineBase], pJoint[JointType_HipLeft]);

	// Right Arm    
	DrawLine(pJoint[JointType_ShoulderRight], pJoint[JointType_ElbowRight]);
	DrawLine(pJoint[JointType_ElbowRight], pJoint[JointType_WristRight]);
	DrawLine(pJoint[JointType_WristRight], pJoint[JointType_HandRight]);
	DrawLine(pJoint[JointType_HandRight], pJoint[JointType_HandTipRight]);
	DrawLine(pJoint[JointType_WristRight], pJoint[JointType_ThumbRight]);

	// Left Arm
	DrawLine(pJoint[JointType_ShoulderLeft], pJoint[JointType_ElbowLeft]);
	DrawLine(pJoint[JointType_ElbowLeft], pJoint[JointType_WristLeft]);
	DrawLine(pJoint[JointType_WristLeft], pJoint[JointType_HandLeft]);
	DrawLine(pJoint[JointType_HandLeft], pJoint[JointType_HandTipLeft]);
	DrawLine(pJoint[JointType_WristLeft], pJoint[JointType_ThumbLeft]);

	// Right Leg
	DrawLine(pJoint[JointType_HipRight], pJoint[JointType_KneeRight]);
	DrawLine(pJoint[JointType_KneeRight], pJoint[JointType_AnkleRight]);
	DrawLine(pJoint[JointType_AnkleRight], pJoint[JointType_FootRight]);

	// Left Leg
	DrawLine(pJoint[JointType_HipLeft], pJoint[JointType_KneeLeft]);
	DrawLine(pJoint[JointType_KneeLeft], pJoint[JointType_AnkleLeft]);
	DrawLine(pJoint[JointType_AnkleLeft], pJoint[JointType_FootLeft]);

	DeleteObject(pDraw);

	EndPaint(*hwnd, &ps);

}


INT Vision::CoordinateTrans(DOUBLE fInput, BOOL bisX)
{
	if (bisX)
		return((INT)(fInput*fRatio + DISPLAY_WIDTH / 2 + rDisplay.left));
	else
		return((INT)(-fInput*fRatio + DISPLAY_HEIGHT / 2 + rDisplay.top));
}

void Vision::DrawLine(CameraSpacePoint p1, CameraSpacePoint p2)
{
	//DrawCircle(p1);
	MoveToEx(hdc,
		CoordinateTrans(p1.Y - pStandard.Y, 1),
		CoordinateTrans(p1.X - pStandard.X, 0),
		NULL);
	LineTo(hdc,
		CoordinateTrans(p2.Y - pStandard.Y, 1),
		CoordinateTrans(p2.X - pStandard.X, 0));
}

void Vision::DrawCircle(CameraSpacePoint pCircle)
{
	INT nCircleRadius = (INT)((1.0 - pCircle.Z / pStandard.Z)*DEFAULT_CIRCLE_RADIUS);
	INT nX = CoordinateTrans(pCircle.Y, 1);
	INT nY = CoordinateTrans(pCircle.X, 0);
	Ellipse(hdc,
		nX - nCircleRadius,
		nY - nCircleRadius,
		nX + nCircleRadius,
		nY + nCircleRadius);
}
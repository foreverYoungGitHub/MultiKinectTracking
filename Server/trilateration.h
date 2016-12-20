#ifndef TRILATERATION_H
#define TRILATERATION_H

#include "stdafx.h"

typedef FLOAT LiteMatrix[3][3];

class Trilateration
{
public:

	Trilateration();
	~Trilateration();

	void InitiliazeSolute(INT nSensor, CameraSpacePoint *pDetect, CameraSpacePoint *pAnswer);
	// made the initial solution for the nonlinear process
	
	INT NonlinearSolute(INT nSensor, CameraSpacePoint *pSensor, CameraSpacePoint *pDetect, CameraSpacePoint *pAnswer);
	// nonlinear process

	void CoordinateTrans(CameraSpacePoint *pAns, const CameraSpacePoint pCurrent, const DOUBLE fAngle, const CameraSpacePoint pShift = { 0, 0, 0 });
	// do the coordinate system transformation for different sensors

	void InitialCoordinateTrans(DOUBLE *fAngle, DOUBLE fDistance, const CameraSpacePoint p1, const CameraSpacePoint p2);
	//get the pRotate


	void InitialCoordinateTrans_ver2(DOUBLE *fAngle, DOUBLE fDistance, const DOUBLE fDiff);

	
private:



	INT MatrixInverse(LiteMatrix *mInput, LiteMatrix *mOutput);
	FLOAT EuclidDis(CameraSpacePoint *p1, CameraSpacePoint *p2);
	FLOAT Error(INT nSensor, CameraSpacePoint *pSensor, FLOAT *fRadius, CameraSpacePoint *pAnswer);
	CameraSpacePoint MatrixMulti(LiteMatrix *mInput, CameraSpacePoint *pInput);
	void PointDec(CameraSpacePoint *p1, CameraSpacePoint *p2);

};
#endif

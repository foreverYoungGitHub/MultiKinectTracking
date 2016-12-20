#include "trilateration.h"

Trilateration::Trilateration()
{

}

Trilateration::~Trilateration()
{

}

INT Trilateration::MatrixInverse(LiteMatrix *mInput, LiteMatrix *mOutput)
{
	FLOAT fDelta = (*mInput)[0][0] * (*mInput)[1][1] * (*mInput)[2][2] + (*mInput)[0][2] * (*mInput)[1][0] * (*mInput)[2][1] + (*mInput)[2][0] * (*mInput)[0][1] * (*mInput)[1][2] - 
				   (*mInput)[0][0] * (*mInput)[1][2] * (*mInput)[2][1] - (*mInput)[0][2] * (*mInput)[1][1] * (*mInput)[2][0] - (*mInput)[2][2] * (*mInput)[0][1] * (*mInput)[1][0];
	if (abs(fDelta) < ZERO)
		return(0);
	(*mOutput)[0][0] = ((*mInput)[1][1] * (*mInput)[2][2] - (*mInput)[1][2] * (*mInput)[2][1]) / fDelta;
	(*mOutput)[1][0] = (-(*mInput)[1][0] * (*mInput)[2][2] + (*mInput)[1][2] * (*mInput)[2][0]) / fDelta;
	(*mOutput)[2][0] = ((*mInput)[1][0] * (*mInput)[2][1] - (*mInput)[1][1] * (*mInput)[2][0]) / fDelta;
	(*mOutput)[0][1] = (-(*mInput)[0][1] * (*mInput)[2][2] + (*mInput)[0][2] * (*mInput)[2][1]) / fDelta;
	(*mOutput)[1][1] = ((*mInput)[0][0] * (*mInput)[2][2] - (*mInput)[0][2] * (*mInput)[2][0]) / fDelta;
	(*mOutput)[2][1] = (-(*mInput)[0][0] * (*mInput)[2][1] + (*mInput)[0][1] * (*mInput)[2][0]) / fDelta;
	(*mOutput)[0][2] = ((*mInput)[0][1] * (*mInput)[1][2] - (*mInput)[0][2] * (*mInput)[1][1]) / fDelta;
	(*mOutput)[1][2] = (-(*mInput)[0][0] * (*mInput)[1][2] + (*mInput)[0][2] * (*mInput)[1][0]) / fDelta;
	(*mOutput)[2][2] = ((*mInput)[0][0] * (*mInput)[1][1] - (*mInput)[0][1] * (*mInput)[1][0]) / fDelta;
	return(1);
}

void Trilateration::InitiliazeSolute(INT nSensor, CameraSpacePoint *pDetect, CameraSpacePoint *pAnswer)
{
	pAnswer->X = 0;
	pAnswer->Y = 0;
	pAnswer->Z = 0;
	for (INT i1 = 0; i1 < nSensor; i1++)
	{
		pAnswer->X += (pDetect + i1)->X;
		pAnswer->Y += (pDetect + i1)->Y;
		pAnswer->Z += (pDetect + i1)->Z;
	}
	pAnswer->X /= nSensor;
	pAnswer->Y /= nSensor;
	pAnswer->Z /= nSensor;
}

FLOAT Trilateration::EuclidDis(CameraSpacePoint *p1, CameraSpacePoint *p2)
{
	CameraSpacePoint pTmp = *p1;
	PointDec(&pTmp, p2);
	return(sqrt(pow(pTmp.X,2)+
				pow(pTmp.Y,2)+
				pow(pTmp.Z,2)));
}

FLOAT Trilateration::Error(INT nSensor, CameraSpacePoint *pSensor, FLOAT *fRadius, CameraSpacePoint *pAnswer)
{
	FLOAT fError = 0;
	for (INT i1 = 0; i1 < nSensor; i1++)
	{
		fError += pow(EuclidDis((pSensor+i1), pAnswer) - *(fRadius+i1), 2);
	}
	return(fError);
}

void Trilateration::PointDec(CameraSpacePoint *p1, CameraSpacePoint *p2)
{
	p1->X -= p2->X;
	p1->Y -= p2->Y;
	p1->Z -= p2->Z;
}

CameraSpacePoint Trilateration::MatrixMulti(LiteMatrix *mInput, CameraSpacePoint *mPoint)
{
	CameraSpacePoint pTmp;
	pTmp.X = (*mInput)[0][0] * mPoint->X + (*mInput)[0][1] * mPoint->Y + (*mInput)[0][2] * mPoint->Z;
	pTmp.Y = (*mInput)[1][0] * mPoint->X + (*mInput)[1][1] * mPoint->Y + (*mInput)[1][2] * mPoint->Z;
	pTmp.Z = (*mInput)[2][0] * mPoint->X + (*mInput)[2][1] * mPoint->Y + (*mInput)[2][2] * mPoint->Z;
	return(pTmp);
}

INT Trilateration::NonlinearSolute(INT nSensor, CameraSpacePoint *pSensor, CameraSpacePoint *pDetect, CameraSpacePoint *pAnswer)
{
	INT nIteration = 0;
	FLOAT fRadius[MAX_SENSOR];
	for (INT i1 = 0; i1 < nSensor; i1++)
	{
		fRadius[i1] = EuclidDis(pSensor + i1, pDetect + i1);
	}
	FLOAT fLastError = 0;
	FLOAT fCurrentError = Error(nSensor, pSensor, &fRadius[0], pAnswer);
	LiteMatrix mJTJ;
	CameraSpacePoint pJTF;
	FLOAT fSingleError;
	do
	{
		memset(mJTJ, 0, sizeof(mJTJ));
		pJTF.X = 0;
		pJTF.Y = 0;
		pJTF.Z = 0;

		for (INT i1 = 0; i1<nSensor; i1++)
		{
			CameraSpacePoint *pTmp = pSensor + i1;
			fSingleError = EuclidDis(pAnswer, pTmp);
			mJTJ[0][0] += pow(pAnswer->X - pTmp->X, 2) / pow(fSingleError, 2);
			mJTJ[0][1] += (pAnswer->X - pTmp->X) * (pAnswer->Y - pTmp->Y) / pow(fSingleError, 2);
			mJTJ[0][2] += (pAnswer->X - pTmp->X) * (pAnswer->Z - pTmp->Z) / pow(fSingleError, 2);
			mJTJ[1][1] += pow(pAnswer->Y - pTmp->Y, 2) / pow(fSingleError, 2);
			mJTJ[1][2] += (pAnswer->Y - pTmp->Y) * (pAnswer->Z - pTmp->Z) / pow(fSingleError, 2);
			mJTJ[2][2] += pow(pAnswer->Z - pTmp->Z, 2) / pow(fSingleError, 2);
			pJTF.X += (pAnswer->X - pTmp->X)*(fSingleError - fRadius[i1]) / fSingleError;
			pJTF.Y += (pAnswer->Y - pTmp->Y)*(fSingleError - fRadius[i1]) / fSingleError;
			pJTF.Z += (pAnswer->Z - pTmp->Z)*(fSingleError - fRadius[i1]) / fSingleError;
		}
		mJTJ[1][0] = mJTJ[0][1];
		mJTJ[2][0] = mJTJ[0][2];
		mJTJ[2][1] = mJTJ[1][2];

		LiteMatrix mInverseResult;
		if (!MatrixInverse(&mJTJ, &mInverseResult))
			return(0);
		CameraSpacePoint pTmp = MatrixMulti(&mInverseResult, &pJTF);
		PointDec(pAnswer, &pTmp);
		fLastError = fCurrentError;
		fCurrentError = Error(nSensor, pSensor, &fRadius[0], pAnswer);

	//	std::cout << fCurrentError << std::endl;

	} while (abs(fLastError - fCurrentError)>ZERO && nIteration++ < MAX_ITERATION);
	return(1);
}


void Trilateration::CoordinateTrans(CameraSpacePoint *pAns, const CameraSpacePoint pCurrent, const DOUBLE fAngle, const CameraSpacePoint pShift)
{
	pAns->X = pCurrent.X + pShift.X;
	pAns->Y = pCurrent.Y*cos(fAngle) - pCurrent.Z*sin(fAngle) + pShift.Y;
	pAns->Z = pCurrent.Y*sin(fAngle) + pCurrent.Z*cos(fAngle) + pShift.Z;
}

void Trilateration::InitialCoordinateTrans(DOUBLE *fAngle, DOUBLE fDistance, const CameraSpacePoint p1, const CameraSpacePoint p2)
{
	DOUBLE fDeltaY, fDeltaZ, fNormalizePara;
	fDeltaY = p2.Y - p1.Y;
	fDeltaZ = p2.Z - p1.Z;
	fNormalizePara = sqrt(pow(fDistance, 2) / (pow(fDeltaY, 2) + pow(fDeltaZ, 2)));

	*fAngle = acos(fDeltaY*fNormalizePara / fDistance);
}

void Trilateration::InitialCoordinateTrans_ver2(DOUBLE *fAngle, DOUBLE fDistance, const DOUBLE fDiff)
{

	*fAngle = asin(fDiff / fDistance);
}

/*
int main(){
	Trilateration tri;
	INT nSensor = 3;
	CameraSpacePoint pSensor[3];
	pSensor[0].X = 0;
	pSensor[0].Y = 0;
	pSensor[0].Z = 0;
	pSensor[1].X = 0;
	pSensor[1].Y = 100;
	pSensor[1].Z = 100;
	pSensor[2].X = -50;
	pSensor[2].Y = 50;
	pSensor[2].Z = -50;
	CameraSpacePoint pDetect[3];
	pDetect[0].X = -0.5;
	pDetect[0].Y = 0.9;
	pDetect[0].Z = 3;
	pDetect[1].X = 1;
	pDetect[1].Y = 1;
	pDetect[1].Z = 0.5;
	pDetect[2].X = 0.5;
	pDetect[2].Y = 0.2;
	pDetect[2].Z = 0.3;
	CameraSpacePoint pAnswer;
	tri.InitiliazeSolute(3, &pDetect[0], &pAnswer);
	tri.NonlinearSolute(3, &pSensor[0], &pDetect[0], &pAnswer);
	std::cout << pAnswer.X << " " << pAnswer.Y << " " << pAnswer.Z << std::endl;
	system("pause");

}

*/
#include "Sever.h"

KinectSever::KinectSever(INT nClient, INT *nPortList, HRC *hrc, DataBuffer *bSever)
{
	this->nClient = nClient;
	for (INT i1 = 0; i1 < nClient; i1++)
	{
		this->nPortList[i1] = *(nPortList + i1);
	}
	memset(nDelay, 0, sizeof(nDelay));

	INT err;
	err = WSAStartup(MAKEWORD(2, 2), &wsaData);

	Clock = hrc;

	nConnected = 0;

	nCalibration = 0;

	mSensorPos = TRUE;

	this->bSever = bSever;


	for (INT i1 = 0; i1 < nClient; i1++)
	{
		nDelay[i1] = -1;
	}
}

KinectSever::~KinectSever()
{
	for (INT i1 = 0; i1 < nClient; i1++)
	{
		DataSend(&sClient[i1], NULL, 0, HALT_MESSAGE );
		param[i1].bStopThread = TRUE;
		closesocket(sSever[i1]);
		closesocket(sClient[i1]);
	}
}

INT KinectSever::DataSend(SOCKET *sClient, void *data, INT nLength, INT nType)
{
	SeverBuffer bMessage;
	bMessage.nType = nType;
	memcpy(bMessage.bData, data, nLength);
	INT nSendLen = send(*sClient, (CHAR*)&bMessage, sizeof(bMessage), 0);
	return(nSendLen);
}

void KinectSever::InitAddr(SOCKADDR_IN *addrInput, ULONG addr, ADDRESS_FAMILY family, USHORT port)
{
	addrInput->sin_addr.S_un.S_addr = addr;
	addrInput->sin_family = family;
	addrInput->sin_port = port;
}

void KinectSever::InitSever()
{
	std::thread threadWaiting[MAX_SENSOR];

	for (INT i1 = 0; i1 < nClient; i1++)
	{
		InitAddr(&addrSever[i1], htonl(INADDR_ANY), AF_INET, htons(nPortList[i1]));

		param[i1].addrSever = &addrSever[i1];
		param[i1].bSever = bSever;
		param[i1].sClient = &sClient[i1];
		param[i1].sSever = &sSever[i1];
		param[i1].nClientIndex = i1;
		param[i1].bStopThread = FALSE;
		param[i1].hrc = Clock; 
		param[i1].nCurrentDelayStamp = 0;
		param[i1].nDelay = &nDelay[i1];
		param[i1].nConnected = &nConnected;
		param[i1].pCurrentSensor = &pCurrentSensor[i1];
		param[i1].nCalibration = &nCalibration;
		param[i1].mSensorPos = &mSensorPos;

		threadWaiting[i1] = std::thread(ThreadWaitClient, &param[i1]);
		threadWaiting[i1].detach();
	}
}


void KinectSever::ThreadWaitClient(ThreadParam *param)
{
	INT nAddrClientLength = sizeof(SOCKADDR);


	*(param->sSever) = socket(AF_INET, SOCK_STREAM, 0);
	bind(*(param->sSever), (SOCKADDR*)(param->addrSever), sizeof(*(param->addrSever)));
	listen(*(param->sSever), 1);
	*(param->sClient) = accept(*(param->sSever), (SOCKADDR*)(&param->addrClient), &nAddrClientLength);
	*(param->nConnected) |= 1<<param->nClientIndex;
	//param->mthread.unlock();

	std::thread threadRecv,threadTiming;
	
    threadTiming = std::thread(ThreadTiming, param);
	threadTiming.detach();


	threadRecv = std::thread(ThreadDataReceive, param);
	threadRecv.detach();
	
}


void KinectSever::ThreadTiming(ThreadParam *param)
{

	TIMETICK tThreadStamp;
	TimingMessage mSend;
	mSend.nIndex = 0;
	mSend.nSendTime = 0;
	while (!param->bStopThread)
	{	
		param->hrc->TimeStampStart(&tThreadStamp);
		mSend.nSendTime = param->hrc->HighResolutionTime();
		mSend.nIndex++;
		DataSend(param->sClient, &mSend, sizeof(mSend), TIMING_MESSAGE);
		param->hrc->HighResolutionSleep(1000, &tThreadStamp);

	}

}

void KinectSever::ThreadDataReceive(ThreadParam *param)
{
	INT nRecvLen;
	SeverBuffer bMessage;

	CHAR cKey[30];
	WCHAR cwKey[30];
	sprintf_s(cKey, "log_recv%d.txt", param->nClientIndex, 30);
	MultiByteToWideChar(CP_ACP, 0, cKey, strlen(cKey) + 1, cwKey, sizeof(cwKey) / sizeof(cwKey[0]));
	HANDLE flogRecv;
	flogRecv = CreateFile(cwKey, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
	DWORD nWrite;
	//make log file

	while (!param->bStopThread)
	{

		nRecvLen = recv(*(param->sClient), (CHAR*)&bMessage, sizeof(bMessage), 0);
		if (nRecvLen > 0)
		{

			CHAR bFile[1024];
			INT ntTime = param->hrc->HighResolutionTime();
			sprintf_s(bFile, 1024, "%d %d\n", ntTime, bMessage.nType);
			WriteFile(flogRecv, bFile, strlen(bFile), &nWrite, NULL);
			//make log file

			switch (bMessage.nType)
			{
			case (TIMING_MESSAGE) :
			{
				TimingMessageProc(param, bMessage.bData);
				break;
			}
			case (TRILATERATION_MESSAGE) :
			{
				TrilaterationMessageProc(param, bMessage.bData);
				break;
			}
			case (CALIBRATION_MESSAGE) :
			{
				CalibrationMessageProc(param, bMessage.bData);
				break;
			}
			case (HALT_MESSAGE) :
			{
				param->bStopThread = TRUE;
				break;
			}
			default:
			{
				break;
			}
			}
		}
	}
}

void  KinectSever::TimingMessageProc(ThreadParam *param, CHAR *buffer)
{
	TimingMessage *mRecv = (TimingMessage*)buffer;
	if (mRecv->nIndex < param->nCurrentDelayStamp)
		return;
	param->nCurrentDelayStamp = mRecv->nIndex + 1;
	LLONG nTimeRecv = param->hrc->HighResolutionTime();
	*(param->nDelay) = (nTimeRecv - mRecv->nSendTime - mRecv->nProcTime) / 2;
}

void KinectSever::TrilaterationMessageProc(ThreadParam *param, CHAR *buffer)
{
	TrilaterationMessage *mRecv = (TrilaterationMessage*)buffer;
	mRecv->tRecv += param->nDelay[param->nClientIndex];
	param->bSever->InsertData(param->bSever, param->nClientIndex, mRecv);
}

LLONG KinectSever::GetDelay(INT nClientIndex)
{
	return nDelay[nClientIndex];
}

INT KinectSever::ClientsConnected()
{
	return (nConnected);
}

BOOL KinectSever::Calibration()
{
	return(nCalibration);
}

void KinectSever::CalibrationMessageProc(ThreadParam *param, CHAR *buffer)
{
	CalibrationMessage *mRecv = (CalibrationMessage*)buffer;

	if (*param->mSensorPos)
	{
		param->pCurrentSensor->X = mRecv->pTarget.X;
		param->pCurrentSensor->Y = mRecv->pTarget.Y;
		param->pCurrentSensor->Z = mRecv->pTarget.Z;
		param->fDiff = mRecv->fHighDepth - mRecv->fLowDepth;


		*(param->nCalibration) |= (1 << param->nClientIndex);
	}
}


CameraSpacePoint *KinectSever::GetSenorPos()
{
	return(pCurrentSensor);
}


void KinectSever::CalibrationProcess(CameraSpacePoint *pSensor, DOUBLE *fDiff)
{
	mSensorPos = FALSE;
	for (INT i1 = 0; i1 < nClient; i1++)
	{
		*(pSensor + i1) = pCurrentSensor[i1];
		*(fDiff + i1) = fDiff[i1];
	}
	mSensorPos = TRUE;
}

void KinectSever::EndCalibration()
{
	for (INT i1 = 0; i1 < nClient; i1++)
	{
		DataSend(&sClient[i1], NULL, 0, SENT_END_CALIBRATION_MESSAGE);
	}
}
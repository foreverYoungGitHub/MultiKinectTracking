#ifndef DATABUFFER_H
#define DATABUFFER_H

#include "stdafx.h"

typedef struct TrilaterationMessage
{
	LLONG tRecv;
	CameraSpacePoint pBody[BODY_LENGTH];
	INT nShade[BODY_LENGTH];
	DOUBLE fSpeed[BODY_LENGTH];
};


typedef struct TrilaterationNode
{
	TrilaterationMessage data;
	TrilaterationNode *next, *last;
};



typedef struct TimingMessage
{
	INT nIndex;
	LLONG nSendTime, nProcTime;
};

typedef struct CalibrationMessage
{
	CameraSpacePoint pTarget;
	DOUBLE fLowDepth, fHighDepth;
};

class DataBuffer
{

public:
	DataBuffer(INT nClient);
	~DataBuffer();

	void InsertData_Run(INT nClientIndex, TrilaterationMessage *mRecv);
	static void InsertData(DataBuffer *bCurrent, INT nClientIndex, TrilaterationMessage *mRecv);
	//insert node to the tail of chain

	static TrilaterationNode *PickData(INT nClientIndex, DataBuffer *bCurrent);
	TrilaterationNode *PickData_Run(INT nClientIndex);

	BOOL RefreshData_Run();
	static BOOL RefreshData(DataBuffer *bCurrent);


private:
	INT nClient;

	std::mutex *mData[MAX_SENSOR];
	//lock for the buffer

	TrilaterationNode *Head[MAX_SENSOR], *Tail[MAX_SENSOR];
	//the head and tail of the chain


};


#endif
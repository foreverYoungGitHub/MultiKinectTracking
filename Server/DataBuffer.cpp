#include "DataBuffer.h"

DataBuffer::DataBuffer(INT nClient)
{
	this->nClient = nClient;
	for (INT i1 = 0; i1 < nClient; i1++)
	{
		Head[i1] = NULL;
		Tail[i1] = NULL;
		mData[i1] = new std::mutex();
	}
}

DataBuffer::~DataBuffer()
{
	for (INT i1 = 0; i1 < nClient; i1++)
	{
		while (Head[i1] != NULL)
		{
			TrilaterationNode *nTemp = PickData_Run(i1);
			delete(nTemp);
		}
	}
}

void DataBuffer::InsertData(DataBuffer *bCurrent, INT nClientIndex, TrilaterationMessage *mRecv)
{
	bCurrent->InsertData_Run(nClientIndex, mRecv);
}

void DataBuffer::InsertData_Run(INT nClientIndex, TrilaterationMessage *mRecv)
{
	if (nClientIndex == 1)
	{
		nClientIndex = 1;
	}
	mData[nClientIndex]->lock();
	TrilaterationNode *nTemp = new TrilaterationNode;
	memcpy(&nTemp->data, mRecv, sizeof(TrilaterationMessage));
	nTemp->next = NULL;
	nTemp->last = Tail[nClientIndex];
	if (Head[nClientIndex] != NULL)
	{
		if (Tail[nClientIndex]->data.tRecv < nTemp->data.tRecv)
			Tail[nClientIndex]->next = nTemp;
	}
	else
	{
		Head[nClientIndex] = nTemp;
		Tail[nClientIndex] = nTemp;
	}
	mData[nClientIndex]->unlock();
}


TrilaterationNode *DataBuffer::PickData(INT nClientIndex, DataBuffer *bCurrent)
{
	return(bCurrent->PickData_Run(nClientIndex));
}

TrilaterationNode *DataBuffer::PickData_Run(INT nClientIndex)
{
	mData[nClientIndex]->lock();
	TrilaterationNode *nTemp = Head[nClientIndex];
	Head[nClientIndex] = nTemp->next;
	mData[nClientIndex]->unlock();
	return(nTemp);
}

BOOL DataBuffer::RefreshData(DataBuffer *bCurrent)
{
	return(bCurrent->RefreshData_Run());
}

BOOL DataBuffer::RefreshData_Run()
{
	LLONG nLatest = 0;
	for (INT i1 = 0; i1 < nClient; i1++)
	{
		if (Head[i1] == NULL) 
			continue;
		if (nLatest < Head[i1]->data.tRecv)
		{
			nLatest = Head[i1]->data.tRecv;
		}
	}
	for (INT i1 = 0; i1 < nClient; i1++)
	{
		while (Head[i1] != NULL && Head[i1]->data.tRecv + KINECT_FRAME_FREQ / 2 < nLatest)
		{
			TrilaterationNode *nTemp = PickData_Run(i1);
			delete(nTemp);
		}
	}
	for (INT i1 = 0; i1 < nClient; i1++)
		if (Head[i1] == NULL)
			return(FALSE);
	return(TRUE);
}
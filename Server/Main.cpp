#include "stdafx.h"|
#include "Controller.h"

#pragma comment( lib, "ws2_32.lib"  )

LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);

void ThreadProc();

Controller *controller;

HWND hwndCalibrationButton, hwndMarkDistance;
//HINSTANCE hi;

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, PSTR szCmdLine, int iCmdShow){
	static TCHAR szAppName[] = TEXT("MultiKinectTracking_Sever");
	HWND hwnd;
	MSG msg;
	WNDCLASS wndclass;
	wndclass.style = CS_HREDRAW | CS_VREDRAW;
	wndclass.cbClsExtra = 0;
	wndclass.cbWndExtra = 0;
	wndclass.hInstance = hInstance;
	wndclass.hIcon = LoadIcon(NULL, IDI_APPLICATION);
	wndclass.hCursor = LoadCursor(NULL, IDC_ARROW);
	wndclass.hbrBackground = (HBRUSH)GetStockObject(WHITE_BRUSH);
	wndclass.lpszMenuName = NULL;
	wndclass.lpfnWndProc = WndProc;
	wndclass.lpszClassName = szAppName;
	if (!RegisterClass(&wndclass)){
		MessageBox(NULL, TEXT("Register ERROR!"), szAppName, MB_ICONINFORMATION);
		return(0);
	}
	hwnd = CreateWindow(szAppName,
		TEXT("MultiKinectTracking_Sever"),
		WS_OVERLAPPEDWINDOW & ~WS_MAXIMIZEBOX & ~WS_THICKFRAME,
		CW_USEDEFAULT,
		CW_USEDEFAULT,
		WINDOW_WIDTH,
		WINDOW_HEIGHT,
		NULL,
		NULL,
		hInstance,
		NULL);
	ShowWindow(hwnd, iCmdShow);
	UpdateWindow(hwnd);

	//init values
		
	//end init

	LPCWSTR sFileName = TEXT(".\\config.inf");
	controller = new Controller(sFileName, &hwnd,&hInstance);
	controller->StartController();

	SetTimer(hwnd, TIMER_WINDOW_REFRESH, WINDOW_REFRESH_INTERVAL, NULL);

//	hi = hInstance;
	
	while (GetMessage(&msg, NULL, 0, 0)){
		TranslateMessage(&msg);
		DispatchMessage(&msg);
	}
	return(msg.wParam);

}

void PaintWindow(HWND hwnd)
{
	HDC hdc;
	PAINTSTRUCT ps;
	RECT rect;
	InvalidateRect(hwnd, NULL, TRUE);
	hdc = BeginPaint(hwnd, &ps);
	GetClientRect(hwnd, &rect);
	
	//paint start

	LPCWSTR strsubTittle1 = TEXT("Clients State");
	LPCWSTR strsubTittle2 = TEXT("Results");

	TextOut(hdc, 50, 5, strsubTittle1, wcslen(strsubTittle1));
	TextOut(hdc, 475, 5, strsubTittle2, wcslen(strsubTittle2));

	MoveToEx(hdc, 300, 0, NULL);
	LineTo(hdc, 300, WINDOW_HEIGHT);

	MoveToEx(hdc, 0, 300, NULL);
	LineTo(hdc, 300, 300);

	MoveToEx(hdc, 0, 0, NULL);
	LineTo(hdc, WINDOW_WIDTH, 0);
	
	
	
	//paint end

	EndPaint(hwnd, &ps);
};



LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	HINSTANCE hInstance = GetModuleHandle(NULL);
	switch (message)
	{
	case WM_CREATE:
	{
		return(0);
	}
	case WM_PAINT:
	{
		PaintWindow(hwnd);
		return(0);
	}
	case SC_MAXIMIZE:
	{
		PaintWindow(hwnd);
		return(0);
	}
	case WM_TIMER:
	{
		switch (wParam)
		{
		case(TIMER_WINDOW_REFRESH) :
		{
			switch (controller->CalibrationStep())
			{
			case(1) :
			{
				hwndCalibrationButton = CreateWindow(TEXT("Button"), TEXT("Calibration Step 1"),
					WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 20, 500, 180, 50,
					hwnd, (HMENU)IDB_CALIBRATION, hInstance, NULL);
				controller->CalibrationInterfaceStepUp();
				break;
			}
			}
			break;
		}
		default:
		{
			break;
		}
								  
		}
		return(0);
	}
	case WM_DESTROY:
	{
		controller->~Controller();
		PostQuitMessage(0);
		return(0);
	}
	case WM_COMMAND:
	{
		switch (LOWORD(wParam))
		{
		case (IDB_CALIBRATION) :
		{
			INT nCalibrationStep = controller->CalibrationStep();
			switch (nCalibrationStep)
			{
			case(2) :
			{
				if (controller->ButtonClick_Calibration(0))
				{
					controller->CalibrationInterfaceStepUp();
					hwndMarkDistance = CreateWindowEx(WS_EX_CLIENTEDGE, TEXT("Edit"), TEXT(""),
						WS_CHILD | WS_VISIBLE, 20, 450, 180, 25,
						hwnd, (HMENU)IDE_MARKDISTANCE, hInstance, NULL);
					SendMessage(hwndCalibrationButton, WM_SETTEXT, (WPARAM)NULL, (LPARAM)L"Calibration Step 2");
				//	SendMessage(hwndCalibrationButton, WM_CLOSE, NULL, NULL);     //ver2
				}
				break;
			}
			case(3) :
			{
				WCHAR wsMarkDistance[256];
				CHAR sMarkDistance[256];
				DOUBLE fMarkDistance;
				GetWindowText(hwndMarkDistance, wsMarkDistance, 256);
				INT wsLen = wcslen(wsMarkDistance);
				WideCharToMultiByte(CP_ACP, 0, (LPCWSTR)wsMarkDistance, wsLen, (LPSTR)sMarkDistance, wsLen, NULL, NULL);
				fMarkDistance = atof(sMarkDistance);
				if (controller->ButtonClick_Calibration(fMarkDistance))
				{
					controller->CalibrationInterfaceStepUp();
					SendMessage(hwndCalibrationButton, WM_CLOSE, NULL, NULL);
					SendMessage(hwndMarkDistance, WM_CLOSE, NULL, NULL);
				}
				break;
			}
			default:
			{
				break;
			}
			}
		}
		default:
		{
			break;
		}
		}
	}
	}
	return DefWindowProc(hwnd, message, wParam, lParam);
}
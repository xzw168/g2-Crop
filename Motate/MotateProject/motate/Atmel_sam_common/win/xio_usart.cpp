

#include <Windows.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <process.h> 
#include <conio.h>


enum xioCodes {
	XIO_OK = 0,				// OK - ALWAYS ZERO
	XIO_ERR,				// generic error return (errors start here)
	XIO_EAGAIN,				// function would block here (must be called again)
	XIO_NOOP,				// function had no-operation
	XIO_COMPLETE,			// operation complete
	XIO_TERMINATE,			// operation terminated (gracefully)
	XIO_RESET,				// operation reset (ungraceful)
	XIO_EOL,				// function returned end-of-line
	XIO_EOF,				// function returned end-of-file
	XIO_FILE_NOT_OPEN,		// file is not open
	XIO_FILE_SIZE_EXCEEDED, // maximum file size exceeded
	XIO_NO_SUCH_DEVICE,		// illegal or unavailable device
	XIO_BUFFER_EMPTY,		// more of a statement of fact than an error code
	XIO_BUFFER_FULL,
	XIO_BUFFER_FULL_FATAL,
	XIO_INITIALIZING,		// system initializing, not ready for use
	XIO_ERROR_16,			// reserved
	XIO_ERROR_17,			// reserved
	XIO_ERROR_18,			// reserved
	XIO_ERROR_19			// NOTE: XIO codes align to here
};

#define LF	0x0A		// ^j - line feed
#define CR	0x0D		// ^m - carriage return

CRITICAL_SECTION CriticalSection;
//#define LOCAL_ECHO

HANDLE hSerial = INVALID_HANDLE_VALUE;
char xio_usart_bufTmp[1024];

static int rxLen = 0;
static int rxFo = 0;

char rxBuf[1024];

void RecvthreadFunction(void *pVoid)
{
	DWORD  dwBytesRead;
	uint8_t data;
	

	for (;;)
	{
		if (hSerial != INVALID_HANDLE_VALUE)
		{
			if (ReadFile(hSerial, &data, 1, &dwBytesRead, NULL) && dwBytesRead == 1){

			}else{
				Sleep(1);
				data = 0;
			}


		}else{
			while (_kbhit() == 0);
			data = _getch();
		}
		if (data == 0)
			continue;
		if (data == CR || data == LF) {
			if (rxLen) {
				rxBuf[rxLen] = 0;
				rxFo = 1;
				while (rxFo) {
					Sleep(0);
				}
			}
			rxLen = 0;
			continue;
		}
		rxBuf[rxLen] = data;
		if (++rxLen > sizeof(rxBuf))
			rxLen = 0;

	}

}



#define MAX_DEVPATH_LENGTH 1024
void winserial_init(char *pPort)
{
	DCB dcb;
	BOOL fSuccess;
	TCHAR devicePath[MAX_DEVPATH_LENGTH];
	COMMTIMEOUTS commTimeout;

	if (pPort != NULL)
	{
		mbstowcs_s(NULL, devicePath, MAX_DEVPATH_LENGTH, pPort, strlen(pPort));//¶à×Ö½Ú±àÂë×Ö·û´®×ª»»Îª¿í×Ö·û±àÂë×Ö·û´®£¬¼´½«char*×ª»»³Éwchar_t*
		hSerial = CreateFile(devicePath, GENERIC_READ | GENERIC_WRITE, 0, NULL,
			OPEN_EXISTING, 0, NULL);
	}
	if (hSerial != INVALID_HANDLE_VALUE)
	{
		//  Initialize the DCB structure.
		SecureZeroMemory(&dcb, sizeof(DCB));
		dcb.DCBlength = sizeof(DCB);
		fSuccess = GetCommState(hSerial, &dcb);
		if (!fSuccess)
		{
			CloseHandle(hSerial);
			hSerial = INVALID_HANDLE_VALUE;
			return;
		}

		GetCommState(hSerial, &dcb);
		dcb.BaudRate = CBR_115200;     //  baud rate
		dcb.ByteSize = 8;             //  data size, xmit and rcv
		dcb.Parity = NOPARITY;      //  parity bit
		dcb.StopBits = ONESTOPBIT;    //  stop bit
		dcb.fBinary = TRUE;
		dcb.fParity = TRUE;

		fSuccess = SetCommState(hSerial, &dcb);
		if (!fSuccess)
		{
			CloseHandle(hSerial);
			hSerial = INVALID_HANDLE_VALUE;
			return;
		}

		GetCommTimeouts(hSerial, &commTimeout);
		commTimeout.ReadIntervalTimeout = 1;
		commTimeout.ReadTotalTimeoutConstant = 1;
		commTimeout.ReadTotalTimeoutMultiplier = 1;
		commTimeout.WriteTotalTimeoutConstant = 1;
		commTimeout.WriteTotalTimeoutMultiplier = 1;
		SetCommTimeouts(hSerial, &commTimeout);
	}
	_beginthread(RecvthreadFunction, 0, NULL);
	//_beginthread(SendthreadFunction, 0, NULL);
}

int xio_usart_gets(char *buf, const int size)
{
	if(rxFo==0)
		return (XIO_EAGAIN);
	memcpy(buf, rxBuf, rxLen);
	buf[rxLen] = 0;
	rxFo = 0;
	return (XIO_OK);
}
int xiom_writeline(const char *buffer)
{
	DWORD dwBytesWritten;
	int len = strlen(buffer);
	WriteFile(hSerial, buffer, len, &dwBytesWritten, NULL);
	return len;
}
int xiom_write(const char *buffer,int len)
{
	DWORD dwBytesWritten;
	WriteFile(hSerial, buffer, len, &dwBytesWritten, NULL);
	return len;
}

void xio_usart_Init(void)
{
	InitializeCriticalSectionAndSpinCount(&CriticalSection, 0x00000400);
	winserial_init("\\\\.\\COM3");
}

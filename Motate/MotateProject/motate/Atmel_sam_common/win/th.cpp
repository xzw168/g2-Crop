
#include <Windows.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <process.h> 
#include <conio.h>
#include <time.h>

#include "MotateTimers.h" 

using Motate::TimerChannel;

typedef TimerChannel<3, 0> dda_timer_type;	// stepper pulse generation in stepper.cpp
typedef TimerChannel<4, 0> exec_timer_type;	// request exec timer in stepper.cpp
typedef TimerChannel<5, 0> fwd_plan_timer_type;	// request exec timer in stepper.cpp
extern   dda_timer_type dda_timer;
extern   exec_timer_type exec_timer;         // 触发下一个+ 1步进段的计算
extern   fwd_plan_timer_type fwd_plan_timer; // 触发下一个块的计划

double QuadPart;

void DDA_Thread(void *pVoid)
{
	LARGE_INTEGER StartingTime, EndingTime, ElapsedMicroseconds;
	uint32_t DDA_PER;
	DDA_PER = QuadPart*(1000000/200000);//us
	for (;;)
	{
		while (dda_timer.irqEn == 0)
			Sleep(0);
		QueryPerformanceCounter(&StartingTime);
		do
		{
			QueryPerformanceCounter(&EndingTime);
			ElapsedMicroseconds.QuadPart = EndingTime.QuadPart - StartingTime.QuadPart;
		} while (ElapsedMicroseconds.QuadPart < DDA_PER);
		dda_timer.interrupt();
	}
}

void EXEC_Thread(void *pVoid)
{

	for (;;)
	{
		while (exec_timer.irqEn == 0)
			Sleep(0);
		exec_timer.irqEn = 0;
		exec_timer.interrupt();
	}
}

void fwd_plan_Thread(void *pVoid)
{

	for (;;)
	{
		while (fwd_plan_timer.irqEn == 0)
			Sleep(0);
		fwd_plan_timer.irqEn = 0;
		fwd_plan_timer.interrupt();
	}
}

void SysTick_Handler(void);
void SysTick_Thread(void *pVoid)
{
	for (;;)
	{
		SysTick_Handler();
		Sleep(1);
	}
}

void xio_tim_Init(void)
{
	LARGE_INTEGER Win32Frequency;
	BOOL res=QueryPerformanceFrequency(&Win32Frequency);//获取每秒多少CPU Performance Tick 
	if (res) {

	}
	QuadPart = Win32Frequency.QuadPart/1000000;

	_beginthread(DDA_Thread, 0, NULL);
	_beginthread(EXEC_Thread, 0, NULL);
	_beginthread(fwd_plan_Thread, 0, NULL);
	_beginthread(SysTick_Thread, 0, NULL);
}

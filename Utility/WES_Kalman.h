
#ifndef WES_Kalman_H
#define WES_Kalman_H

/*-----------------------------------------------------------------------------   
* Copyright (C) 2012 EMLAB529_WANGESHEN. All rights reserved.   
*   
* $Date:        3. july 2012 
* $Revision:    V0.0.1  
*   
* Project:      Digital Filter Library   
* Title:        WES_Kalman.h   
*   
* Description:  Floating-point Kalman filter function header file.   
*   
* Target Processor: Cortex-M4/Cortex-M3/Cortex-M0
*  
* Version 0.0.1  2012/07/3   
*    Initial version   
* ---------------------------------------------------------------------------*/
//#include "stm32f10x.h"
#include "main.h"

#define s32 int32_t
/*-----------------------------------------------------------------------------
 *   Floating-point 
 *---------------------------------------------------------------------------*/
//Kalman filter instance for float-point
typedef struct {
	float x0;   //初始估计值
	float x1;   //中间状态
	float x2;   //中间状态
	float p0;   //初始估计协方差，任何(P != 0)都可以使滤波器收敛
	float p1;   //中间状态
	float p2;   //中间状态
	float K;    //比例系数
	float Q;    //过程激励协方差
	float R;     //观测噪声协方差，R增大，滤波器反应变慢，导致估计方差减小，反之，同理。
	}kalman_instance_f32;

/*-----------------------------------------------------------------------------
 *   Fixed-point 
 *---------------------------------------------------------------------------*/
typedef struct {
	int x0;   //初始估计值
	int x1;   //中间状态
	int x2;   //中间状态
	int p0;   //初始估计协方差，任何(P != 0)都可以使滤波器收敛
	int p1;   //中间状态
	int p2;   //中间状态
	int K;    //比例系数
	int Q;    //过程激励协方差
	int R;     //观测噪声协方差，R增大，滤波器反应变慢，导致估计方差减小，反之，同理。
	}kalman_instance_q31;

/*-----------------------------------------------------------------------------
 *   Application API 
 ----------------------------------------------------------------------------*/
//Init
void KalmanFilterInit_Hx712(int x, int p, int q, int r);

//Kalman Filter for once
s32 KalmanFilter_Hx712(s32 newValue);

void KalmanFilterInit_MT(float x, float p, float q, float r);

float KalmanFilter_MT(float newValue);

#endif

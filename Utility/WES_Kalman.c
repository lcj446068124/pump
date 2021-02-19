/*-----------------------------------------------------------------------------   
* Copyright (C) 2012 EMLAB529_WANGESHEN. All rights reserved.   
*   
* $Date:        3. july 2012 
* $Revision:    V0.0.1  
*   
* Project:      Digital Filter Library   
* Title:        WES_Kalman.c   
*   
* Description:  Floating-point Kalman filter function implementation file.   
*   
* Target Processor: Cortex-M4/Cortex-M3/Cortex-M0
*  
* Version 0.0.1  2012/07/3   
*    Initial version   
* ---------------------------------------------------------------------------*/

#include "WES_Kalman.h"

/*------------------------------- Filter instance ---------------------------*/
kalman_instance_q31 Kalman_ADS1256_J10, Kalman_ADS1256_J13;

s32 J10_LastValue = 0, J13_LastValue = 0;

static char WES_Filter_KALMAN_SetR_q31(kalman_instance_q31 *pS,int gain);
static void WES_Kalman_init_q31(kalman_instance_q31 *pS, int x, int p, int q, int r);
static int WES_Kalman_q31(kalman_instance_q31 *pS, int measurement);



/*-----------------------------------------------------------------------------
 *   Upper kalman function 
-----------------------------------------------------------------------------*/
//Kalman filter Init

void KalmanFilterInit_Hx712(int x, int p, int q, int r)
{
  WES_Kalman_init_q31(&Kalman_ADS1256_J13,x,p,q,r); 
}



//kalman filter implimentation

s32 KalmanFilter_Hx712(s32 newValue)
{
  s32 result;
  result = WES_Kalman_q31(&Kalman_ADS1256_J13,newValue);
  
  if( ((result-J13_LastValue) >= 10000) || ((J13_LastValue-result) >= 10000) )
  {
    WES_Filter_KALMAN_SetR_q31(&Kalman_ADS1256_J13,-200); //R = R-8
  }
  else if( ((result-J13_LastValue) >= 5000) || ((J13_LastValue-result) >= 5000) )
  {
    WES_Filter_KALMAN_SetR_q31(&Kalman_ADS1256_J13,-100);  //R = R-5
  }
  else if( ((result-J13_LastValue) >= 1000) || ((J13_LastValue-result) >= 1000) )
  {
    WES_Filter_KALMAN_SetR_q31(&Kalman_ADS1256_J13,-50);  //R = R-3
  }
  else if( ((result-J13_LastValue) >= 100) || ((J13_LastValue-result) >= 100) )
  {
    WES_Filter_KALMAN_SetR_q31(&Kalman_ADS1256_J13,-30);  //R = R-1
  }
  else if( ((result-J13_LastValue) >= 10) || ((J13_LastValue-result) >= 10) )
  {
    WES_Filter_KALMAN_SetR_q31(&Kalman_ADS1256_J13,-10);  //R = R-1
  }
  else if( ((result-J13_LastValue) >= 3) || ((J13_LastValue-result) >= 3) )
  {
    WES_Filter_KALMAN_SetR_q31(&Kalman_ADS1256_J13,50);  //R = R+5
  }
  else
  {
    WES_Filter_KALMAN_SetR_q31(&Kalman_ADS1256_J13,100);  //R = R+8
  }
  J13_LastValue = result;
  
  return result;
}


/*-----------------------------------------------------------------------------
 *   Lower kalman function 
-----------------------------------------------------------------------------*/

/*-------------------------------- Floating-point ----------------------------*/
/*
//滤波器初始化, x为初始估计值，只有第一次估计时输入，后续自动更新
static void WES_Kalman_init(kalman_instance_f32 *pS, float x, float p, float q, float r)
{
  pS->x0 = x;  //初始估计值
  pS->x1 = 0;
  pS->x2 = 0;
  pS->p0 = p;  //初始估计协方差，任何(P != 0)都可以使滤波器收敛
  pS->p1 = 0;
  pS->p2 = 0;
  pS->K = 0;
  pS->Q = q;   //过程激励协方差
  pS->R = r;   //观测噪声协方差，R增大，滤波器反应变慢，导致估计方差减小，反之，同理。
}
*/

/*--------------------------------- Fixed-point -----------------------------*/

static void WES_Kalman_init_q31(kalman_instance_q31 *pS, int x, int p, int q, int r)
{
  pS->x0 = x;  //初始估计值
  pS->x1 = 0;
  pS->x2 = 0;
  pS->p0 = p;  //初始估计协方差，任何(P != 0)都可以使滤波器收敛
  pS->p1 = 0;
  pS->p2 = 0;
  pS->K = 0;
  pS->Q = q;   //过程激励协方差
  pS->R = r;   //观测噪声协方差，R增大，滤波器反应变慢，导致估计方差减小，反之，同理。
}

static int WES_Kalman_q31(kalman_instance_q31 *pS, int measurement)
{
  //时间方程
  pS->x1 = pS->x0;
  pS->p1 = pS->p0 + pS->Q;

  //更新方程
  pS->K = (pS->p1 * 100) / (pS->p1 + pS->R);  //K取百分制0~100
  pS->x2 = pS->x1 + pS->K * (measurement - pS->x1) / 100;
  pS->p2 = (100 - pS->K) * pS->p1 / 100;

  //k+1 => k
  pS->x0 = pS->x2;
  pS->p0 = pS->p2;

  return pS->x2;
}

//改变 R = R + gain
static char WES_Filter_KALMAN_SetR_q31(kalman_instance_q31 *pS,int gain)
{
   int x0,p0,q,r;
   x0 = pS->x0;
   p0 = pS->p0;
   q = pS->Q; 
   r = pS->R + gain;
   if(r < 0) 
     r = 1;   
   if(r > 1000)  
     r = 1000;
   WES_Kalman_init_q31(pS,x0,p0,q,r); 
   return 1;
}

/*----------------------------------------------------------------------------*/



//kalman_instance_f32 Kalman_MT_J15;
//float J15_LastValue = 0;
//
//static void WES_Kalman_init(kalman_instance_f32 *pS, float x, float p, float q, float r);
//static float WES_Kalman(kalman_instance_f32 *pS, float measurement);
//static char WES_Filter_KALMAN_SetR(kalman_instance_f32 *pS, float gain);
//
//void KalmanFilterInit_MT(float x, float p, float q, float r)
//{
//  WES_Kalman_init(&Kalman_MT_J15,x,p,q,r); 
//}
//
//kalman filter implimentation
//
//float KalmanFilter_MT(float newValue)
//{
//  float result;
//  result = WES_Kalman(&Kalman_MT_J15,newValue);
//  
//  if( (result-J15_LastValue) >= 0.01 || (J15_LastValue-result) >= 0.01)
//  {
//    WES_Filter_KALMAN_SetR(&Kalman_MT_J15, -0.01);  //-0.0005
//  }
//  else if((result-J15_LastValue) >= 0.001 || (J15_LastValue-result) >= 0.001)
//  {
//    WES_Filter_KALMAN_SetR(&Kalman_MT_J15, -0.008); //-0.0001
//  }
//  else 
//  { //增大Kalman系数R = R+0.01
//    WES_Filter_KALMAN_SetR(&Kalman_MT_J15, 0.0005);  //0.0005 
//  }
//  J15_LastValue = result;
//  
//  return result;
//}
//
//static float WES_Kalman(kalman_instance_f32 *pS, float measurement)
//{
//  //时间方程
//  pS->x1 = pS->x0;
//  pS->p1 = pS->p0 + pS->Q;
//
//  //更新方程
//  pS->K = pS->p1 / (pS->p1 + pS->R);
//  pS->x2 = pS->x1 + pS->K * (measurement - pS->x1);
//  pS->p2 = (1-pS->K)*pS->p1;
//
//  //k+1 => k
//  pS->x0 = pS->x2;
//  pS->p0 = pS->p2;
//
//  return pS->x2;
//}
//
////增大R = R+gain
//static char WES_Filter_KALMAN_SetR(kalman_instance_f32 *pS, float gain)
//{
//   float x0,p0,q,r;
//   x0 = pS->x0;
//   p0 = pS->p0;
//   q = pS->Q; 
//   r = pS->R + gain;
//   if(r < 0.0001) r = 0.0001;   //0.001
//   if(r > 1)  r = 1;
//   WES_Kalman_init(pS,x0,p0,q,r);
//   return 1;
//}


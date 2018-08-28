#pragma once
#include<stdio.h>
#include<string.h>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
typedef struct PID {
	float SetPoint;            //设定值
	float Kp;                  //比例系数
	float Ki;                  //积分系数
	float Kd;                  //微分系数
	float LastError;           //最后一次误差数Er[-1]
	float PrevError;           //最后第二次误差数er[-2]
	float SumError;            //误差积分  
}PID;

float keep_high(float Expected,float Actual,float kp,float ki,float kd)
{
	PID pid_high;
	pid_high.Kp = kp;
	pid_high.Ki = ki;
	pid_high.Kd = kd;
	pid_high.LastError = 0;
	pid_high.PrevError = 0;
	pid_high.SumError = 0;
	pid_high.SetPoint = Expected;
	static float init;
	static float err_r;

	float err = (float)(pid_high.SetPoint - Actual);
	if (fabs(err) < 0.01)err = 0;
	if (err > 0.1)err = (float)0.1;
	if (err < -0.1)err = -(float)0.1;
	init += err*pid_high.Ki;
	if (init > 0.1)init = (float)0.1;
	if (init < -0.1)init = -(float)0.1;
	float a = (float)err*pid_high.Kp + init + (err - err_r)*pid_high.Kd;
	err_r = err;
	if (a > 0.3)a = (float)0.3;
	if (a < -0.3)a = -(float)0.3;
	float thr = a + (float)0.585;
	if (thr > 0.72)thr = (float)0.72;
	if (thr < 0.4)thr = (float)0.4;
	return (float)(thr);
}
float keep_height(float Expected, float Actual, float kp, float ki, float kd)
{
	static float lastErr_height;
	static float err_sum_height;
	float err_height = Expected - Actual;
	if (fabs(err_height) < 0.01)err_height = 0;
	err_sum_height = err_sum_height + err_height*ki;
	if (err_sum_height > 0.1)err_sum_height = (float)0.1;
	if (err_sum_height < -0.1)err_sum_height = -(float)0.1;
	float derr_height = (err_height - lastErr_height);
	lastErr_height = err_height;
	float a_height = (kp * err_height + ki * err_sum_height + kd * derr_height);
	if (a_height > 0.02)a_height = (float)0.02;
	if (a_height < -0.02)a_height = -(float)0.02;
	float thr_height = a_height + 0.585;
	if (thr_height > 0.60)thr_height = (float)0.60;
	if (thr_height < 0.2)thr_height = (float)0.2;
	return (float)(thr_height);
}


float keep_horizontal_x(float Expected, float Actual, float kp, float ki, float kd)
{
	static float lastErr_x;
	static float err_sum_x;
	float err_x = Expected - Actual;
	if (fabs(err_x) < 0.001)err_x = 0;
	err_sum_x = err_sum_x +err_x*ki;
	if (err_sum_x > 0.005)err_sum_x = (float)0.005;
	if (err_sum_x < -0.005)err_sum_x = -(float)0.005;
	float derr_x = (err_x - lastErr_x);
	lastErr_x = err_x;
	float a_x= (kp * err_x + ki * err_sum_x + kd * derr_x);
	float thr_x = a_x;
	if (thr_x > 1)thr_x = (float)1;
	if (thr_x < -1)thr_x = (float)-1;
	return (float)(thr_x);
}

float keep_horizontal_y(float Expected, float Actual, float kp, float ki, float kd)
{
	static float lastErr_y;
	static float err_sum_y;
	float err_y = Expected - Actual;
	if (fabs(err_y) < 0.001)err_y = 0;
	err_sum_y = err_sum_y + err_y*ki;
	if (err_sum_y > 0.005)err_sum_y = (float)0.005;
	if (err_sum_y < -0.005)err_sum_y = -(float)0.005;
	float derr_y = (err_y - lastErr_y);
	lastErr_y = err_y;
	float a_y = (kp * err_y + ki * err_sum_y + kd * derr_y);
	float thr_y = a_y;
	if (thr_y > 1)thr_y = (float)1;
	if (thr_y < -1)thr_y = (float)-1;
	return (float)(thr_y);
}

//俯仰
float keep_pitch(float Expected, float Actual, float kp, float ki, float kd)
{
	static float lastErr_pitch;
	static float err_sum_pitch;
	float err_pitch = Expected - Actual;
	if (fabs(err_pitch) < 0.01)err_pitch = 0;
	err_sum_pitch = err_sum_pitch + err_pitch*ki;
	if (err_sum_pitch > 0.1)err_sum_pitch = (float)0.1;
	if (err_sum_pitch < -0.1)err_sum_pitch = -(float)0.1;
	float derr_pitch = (err_pitch - lastErr_pitch);
	lastErr_pitch = err_pitch;
	float a_pitch = (kp * err_pitch + ki * err_sum_pitch + kd * derr_pitch);
	float thr_pitch = a_pitch;
	if (thr_pitch > 0.05)thr_pitch = (float)0.05;
	if (thr_pitch < -0.05)thr_pitch = (float)-0.05;
	return (float)(thr_pitch);
}
//横滚
float keep_roll(float Expected, float Actual, float kp, float ki, float kd)
{
	static float lastErr_roll;
	static float err_sum_roll;
	float err_roll = Expected - Actual;
	if (fabs(err_roll) < 0.01)err_roll = 0;
	err_sum_roll = err_sum_roll + err_roll*ki;
	if (err_sum_roll > 0.1)err_sum_roll = (float)0.1;
	if (err_sum_roll < -0.1)err_sum_roll = -(float)0.1;
	float derr_roll = (err_roll - lastErr_roll);
	lastErr_roll = err_roll;
	float a_roll = (kp * err_roll + ki * err_sum_roll + kd * derr_roll);
	float thr_roll = a_roll;
	if (thr_roll > 0.2)thr_roll = (float)0.2;
	if (thr_roll < -0.2)thr_roll = (float)-0.2;
	return (float)(thr_roll);
}


//偏航
float keep_yaw(float Expected, float Actual, float kp, float ki, float kd)
{
	static float lastErr_yaw;
	static float err_sum_yaw;
	float err_yaw = Expected - Actual;
	if (fabs(err_yaw) < 0.01)err_yaw = 0;
	err_sum_yaw = err_sum_yaw + err_yaw*ki;
	if (err_sum_yaw > 0.1)err_sum_yaw = (float)0.1;
	if (err_sum_yaw < -0.1)err_sum_yaw = -(float)0.1;
	float derr_yaw = (err_yaw - lastErr_yaw);
	lastErr_yaw = err_yaw;
	float a_yaw = (kp * err_yaw + ki * err_sum_yaw + kd * derr_yaw);
	float thr_yaw = a_yaw;
	if (thr_yaw > 0.1)thr_yaw = (float)0.1;
	if (thr_yaw < -0.1)thr_yaw = (float)-0.1;
	return (float)(thr_yaw);
}
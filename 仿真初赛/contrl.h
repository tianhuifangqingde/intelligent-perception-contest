#ifndef _CONTRl_H
#define _CONTRl_H

#include<stdio.h>
#include<string.h>
#include <cmath>

void MOVE(float Front,float Behind,float left,float right,float up,float down,float high_control,float yaw_k,double ECEF_X_strat,
	double ECEF_Y_strat,double ECEF_Z_strat, double ECEF_X, double ECEF_Y,
	double ECEF_Z, msr::airlib::MultirotorRpcLibClient RpcLibClientBase)
{
	if (Front != 0)//向前飞
	{
		RpcLibClientBase.moveByAngleThrottle((float)-0.05, 0, (float)(high_control), 0, (float)0.2);
	}




	if (Behind != 0)//向后飞
	{
		RpcLibClientBase.moveByAngleThrottle((float)0.05, 0, (float)(high_control), 0, (float)0.2);
	}
	if (left != 0)  //向左飞
	{
		RpcLibClientBase.moveByAngleThrottle(0, (float)0.05, (float)(high_control), 0, (float)0.2);
	}
    if (right != 0) //向右
	{
		RpcLibClientBase.moveByAngleThrottle(0, (float)-0.05, (float)(high_control), 0, (float) 0.2);
	}
	if (yaw_k !=0)//偏航
	{
		RpcLibClientBase.moveByAngleThrottle(0, 0, (float)(high_control), (float)yaw_k, (float)0.2);
	}
	if (up!=0)//向上飞
	{
		RpcLibClientBase.moveByAngleThrottle(0, 0, (float)0.7, 0, (float)0.2);
	}
	if (down!=0)//向下飞
	{
		RpcLibClientBase.moveByAngleThrottle(0, 0, (float)0.3, 0, (float)0.2);
	}
}


#endif




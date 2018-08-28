#ifndef _IMU_H
#define _IMU_H


#include <stdio.h>
#include <cmath>
#include "mymath.h"
#define Kp 0.6f                	// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.1f                	// 0.001  integral gain governs rate of convergence of gyroscope biases
#define ANGLE_TO_RADIAN 0.01745329f  
#define IMU_INTEGRAL_LIM  ( 2.0f *ANGLE_TO_RADIAN )
#define NORM_ACC_LPF_HZ 10  		//(Hz)
#define REF_ERR_LPF_HZ  1				//(Hz)
typedef struct
{
	float x;
	float y;
	float z;
}xyz_f_t;

typedef struct
{
	xyz_f_t err;
	xyz_f_t err_tmp;
	xyz_f_t err_lpf;
	xyz_f_t err_Int;
	xyz_f_t g;

}ref_t;

typedef struct Attitude
{
	float pitch;
	float roll;
	float yaw;

}Attitude;

xyz_f_t reference_v;
ref_t 	ref;

float ref_q[4] = { 1,0,0,0 };
float norm_acc, norm_q;
float norm_acc_lpf;
float Roll, Pitch, Yaw;
int8_t fly_ready;

//快速平方根算法
float my_sqrt(float number)
{
	long i;
	float x, y;
	const float f = 1.5F;
	x = number * 0.5F;
	y = number;
	i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);

	y = *(float *)&i;
	y = y * (f - (x * y * y));
	y = y * (f - (x * y * y));
	return number * y;
}
struct Attitude IMUupdate(float half_T, float gx, float gy, float gz, float ax, float ay, float az,float Mag_x,float Mag_y,float Mag_z)
{                //周期            角速度                    角加速度                              姿态角
	Attitude plane;
	plane.pitch =(float)0.0;
	plane.roll = (float)0.0;
	plane.yaw = (float)0.0;
	float ref_err_lpf_hz;
	float yaw_correct;
	float mag_norm_tmp;
	static float mag_norm, mag_norm_xyz, mag_tmp_x, mag_tmp_y, yaw_mag;

	mag_norm = my_sqrt(Mag_x* Mag_x + Mag_y* Mag_y);
	mag_norm_xyz = my_sqrt(Mag_x* Mag_x + Mag_y* Mag_y + Mag_z *Mag_z);

	mag_norm_tmp = LIMIT(0.02f *((float)50 - ABS(my_deathzoom((mag_norm_xyz - 100), 10))), 0.05f, 1) * 20 * (6.28f *half_T);    

	mag_tmp_x += mag_norm_tmp *((float)Mag_x - mag_tmp_x);
	mag_tmp_y += mag_norm_tmp *((float)Mag_y - mag_tmp_y);

	if (Mag_x != 0 && mag_tmp_y != 0)
	{
		yaw_mag = (float)fast_atan2(mag_tmp_y / mag_norm, mag_tmp_x / mag_norm) *57.3f;

	}
	//=============================================================================
	reference_v.x = 2 * (ref_q[1] * ref_q[3] - ref_q[0] * ref_q[2]);
	reference_v.y = 2 * (ref_q[0] * ref_q[1] + ref_q[2] * ref_q[3]);
	reference_v.z = 1 - 2 * (ref_q[1] * ref_q[1] + ref_q[2] * ref_q[2]);//ref_q[0]*ref_q[0] - ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3];



	norm_acc = my_sqrt(ax*ax + ay*ay + az*az);
	norm_acc_lpf += NORM_ACC_LPF_HZ *(6.28f *half_T) *(norm_acc - norm_acc_lpf);  //10hz *3.14 * 2*0.001


	if (ABS(ax)<4400 && ABS(ay)<4400 && ABS(az)<4400)
	{
		ax = ax / norm_acc;//4096.0f;
		ay = ay / norm_acc;//4096.0f;
		az = az / norm_acc;//4096.0f; 

		if (3800 < norm_acc && norm_acc < 4400)
		{
			ref.err_tmp.x = ay*reference_v.z - az*reference_v.y;
			ref.err_tmp.y = az*reference_v.x - ax*reference_v.z;
			//ref.err_tmp.z = ax*reference_v.y - ay*reference_v.x;


			ref_err_lpf_hz = REF_ERR_LPF_HZ *(6.28f *half_T);
			ref.err_lpf.x += ref_err_lpf_hz *(ref.err_tmp.x - ref.err_lpf.x);
			ref.err_lpf.y += ref_err_lpf_hz *(ref.err_tmp.y - ref.err_lpf.y);
			//			 ref.err_lpf.z += ref_err_lpf_hz *( ref.err_tmp.z  - ref.err_lpf.z );

			ref.err.x = ref.err_lpf.x;//
			ref.err.y = ref.err_lpf.y;//
									  //				ref.err.z = ref.err_lpf.z ;
		}
	}
	else
	{
		ref.err.x = 0;
		ref.err.y = 0;
	}

	ref.err_Int.x += ref.err.x *Ki * 2 * half_T;
	ref.err_Int.y += ref.err.y *Ki * 2 * half_T;
	ref.err_Int.z += ref.err.z *Ki * 2 * half_T;

	ref.err_Int.x = LIMIT(ref.err_Int.x, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);
	ref.err_Int.y = LIMIT(ref.err_Int.y, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);
	ref.err_Int.z = LIMIT(ref.err_Int.z, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);

	if (fly_ready)
	{
		yaw_correct = Kp *0.1f *LIMIT(my_deathzoom(To_180_degrees(yaw_mag - Yaw), 10), -20, 20);
	}
	else
	{
		yaw_correct = Kp *1.0f *To_180_degrees(yaw_mag - Yaw);
	}

	if (reference_v.z > 0.7f)
	{
		plane.yaw += 2 * half_T *(-gx *reference_v.x - gy *reference_v.y - gz *reference_v.z + yaw_correct);

	}
	else
	{
		plane.yaw += 2 * half_T *(-gx *reference_v.x - gy *reference_v.y - gz *reference_v.z);
	}
	plane.yaw = To_180_degrees(plane.yaw);

	ref.g.x = gx *ANGLE_TO_RADIAN + (Kp*(ref.err.x + ref.err_Int.x));     //IN RADIAN
	ref.g.y = gy *ANGLE_TO_RADIAN + (Kp*(ref.err.y + ref.err_Int.y));		  //IN RADIAN
	ref.g.z = gz *ANGLE_TO_RADIAN;


	// integrate quaternion rate and normalise
	ref_q[0] = ref_q[0] + (-ref_q[1] * ref.g.x - ref_q[2] * ref.g.y - ref_q[3] * ref.g.z)*half_T;
	ref_q[1] = ref_q[1] + (ref_q[0] * ref.g.x + ref_q[2] * ref.g.z - ref_q[3] * ref.g.y)*half_T;
	ref_q[2] = ref_q[2] + (ref_q[0] * ref.g.y - ref_q[1] * ref.g.z + ref_q[3] * ref.g.x)*half_T;
	ref_q[3] = ref_q[3] + (ref_q[0] * ref.g.z + ref_q[1] * ref.g.y - ref_q[2] * ref.g.x)*half_T;


	norm_q = my_sqrt(ref_q[0] * ref_q[0] + ref_q[1] * ref_q[1] + ref_q[2] * ref_q[2] + ref_q[3] * ref_q[3]);
	ref_q[0] = ref_q[0] / norm_q;
	ref_q[1] = ref_q[1] / norm_q;
	ref_q[2] = ref_q[2] / norm_q;
	ref_q[3] = ref_q[3] / norm_q;


	plane.roll = (float)fast_atan2(2 * (ref_q[0] * ref_q[1] + ref_q[2] * ref_q[3]), 1 - 2 * (ref_q[1] * ref_q[1] + ref_q[2] * ref_q[2])) ;//输出单位弧度
	plane.pitch = asin(2 * (ref_q[1] * ref_q[3] - ref_q[0] * ref_q[2]));
	return plane;
	//rol = (float)fast_atan2(2 * (ref_q[0] * ref_q[1] + ref_q[2] * ref_q[3]), 1 - 2 * (ref_q[1] * ref_q[1] + ref_q[2] * ref_q[2])) *57.3f;
	//pit = asin(2 * (ref_q[1] * ref_q[3] - ref_q[0] * ref_q[2])) *57.3f;
	 

}
#endif



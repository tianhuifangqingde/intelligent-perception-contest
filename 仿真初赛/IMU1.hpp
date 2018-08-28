#ifndef _IMU1_H
#define _IMU1_H


#include <stdio.h>
#include <cmath>

float IMU_Kp = 0.6f;                	// proportional gain governs rate of convergence to accelerometer/magnetometer
float IMU_Ki = 0.1f;              	// 0.001  integral gain governs rate of convergence of gyroscope biases
float ANGLE_TO_RADIAN = 0.01745329f;
float IMU_INTEGRAL_LIM=2.0f *ANGLE_TO_RADIAN;
float NORM_ACC_LPF_HZ=10; 		//(Hz)
float REF_ERR_LPF_HZ =1;			//(Hz)

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
int fly_ready;

 
float LIMIT(float x, float min, float max)
{
	if (x < min)
		return min;
	else if (x > max)
		return max;
	else
		return x;
}

float my_deathzoom(float x, float zoom)
{
	float t;
	if (x>0)
	{
		t = x - zoom;
		if (t<0)
		{
			t = 0;
		}
	}
	else
	{
		t = x + zoom;
		if (t>0)
		{
			t = 0;
		}
	}
	return  t ;
}
float To_180_degrees(float x)
{
	return (x>180 ? (x - 360) : (x<-180 ? (x + 360) : x));
}
void reset_req()
{
	ref_q[0] = 1;
	ref_q[1] = 0;
	ref_q[2] = 0;
	ref_q[3] = 0;
}


struct Attitude IMUupdate(float half_T, float gx, float gy, float gz, float ax, float ay, float az, float Mag_x, float Mag_y, float Mag_z)
{                //周期            角速度                    角加速度                              姿态角
	reset_req();
	Attitude plane;
	plane.pitch = (float)0.0;
	plane.roll = (float)0.0;
	plane.yaw = (float)0.0;
	float ref_err_lpf_hz;
	float yaw_correct;
	float mag_norm_tmp;
	static float mag_norm, mag_norm_xyz, mag_tmp_x, mag_tmp_y, yaw_mag;
	mag_norm = sqrt(Mag_x* Mag_x + Mag_y* Mag_y);
	mag_norm_xyz = sqrt(Mag_x* Mag_x + Mag_y* Mag_y + Mag_z *Mag_z);
	mag_norm_tmp = LIMIT(0.02f *((float)50 - abs(my_deathzoom((mag_norm_xyz - 100), 10))), 0.05f, 1) * 20 * (6.28f *half_T);
	mag_tmp_x += mag_norm_tmp *((float)Mag_x - mag_tmp_x);
	mag_tmp_y += mag_norm_tmp *((float)Mag_y - mag_tmp_y);
	if (Mag_x != 0 && mag_tmp_y != 0)
	{
		yaw_mag = (float)atan2(mag_tmp_y / mag_norm, mag_tmp_x / mag_norm) *57.3f;
	}
	reference_v.x = 2 * (ref_q[1] * ref_q[3] - ref_q[0] * ref_q[2]);
	reference_v.y = 2 * (ref_q[0] * ref_q[1] + ref_q[2] * ref_q[3]);
	reference_v.z = 1 - 2 * (ref_q[1] * ref_q[1] + ref_q[2] * ref_q[2]);//ref_q[0]*ref_q[0] - ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3];

	norm_acc = sqrt(ax*ax + ay*ay + az*az);
	norm_acc_lpf += NORM_ACC_LPF_HZ *(6.28f *half_T) *(norm_acc - norm_acc_lpf);  //10hz *3.14 * 2*0.001

	if (abs(ax)<4400 && abs(ay)<4400 && abs(az)<4400)
	{
		ax = ax / norm_acc;//4096.0f;
		ay = ay / norm_acc;//4096.0f;
		az = az / norm_acc;//4096.0f; 
		if (3800 < norm_acc && norm_acc < 4400)
		{
			ref.err_tmp.x = ay*reference_v.z - az*reference_v.y;
			ref.err_tmp.y = az*reference_v.x - ax*reference_v.z;
			ref_err_lpf_hz = REF_ERR_LPF_HZ *(6.28f *half_T);
			ref.err_lpf.x += ref_err_lpf_hz *(ref.err_tmp.x - ref.err_lpf.x);
			ref.err_lpf.y += ref_err_lpf_hz *(ref.err_tmp.y - ref.err_lpf.y);
			ref.err.x = ref.err_lpf.x;//
			ref.err.y = ref.err_lpf.y;//
		}
	}
	else
	{
		ref.err.x = 0;
		ref.err.y = 0;
	}

	ref.err_Int.x += ref.err.x *IMU_Ki * 2 * half_T;
	ref.err_Int.y += ref.err.y *IMU_Ki * 2 * half_T;
	ref.err_Int.z += ref.err.z *IMU_Ki * 2 * half_T;

	ref.err_Int.x = LIMIT(ref.err_Int.x, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);
	ref.err_Int.y = LIMIT(ref.err_Int.y, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);
	ref.err_Int.z = LIMIT(ref.err_Int.z, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);

	if (fly_ready)
	{
		yaw_correct = IMU_Kp *0.1f *LIMIT(my_deathzoom(To_180_degrees(yaw_mag - Yaw), 10), -20, 20);
	}
	else
	{
		yaw_correct = IMU_Kp *1.0f *To_180_degrees(yaw_mag - Yaw);
	}
	if (reference_v.z > 0.7f)
	{
		plane.yaw += 2 * half_T *(-gx *reference_v.x - gy *reference_v.y - gz *reference_v.z + yaw_correct);
	}
	else
	{
		plane.yaw += 2 * half_T *(-gx *reference_v.x - gy *reference_v.y - gz *reference_v.z);
	}
	plane.yaw = To_180_degrees(plane.yaw)/57.3;
	ref.g.x = gx *ANGLE_TO_RADIAN + (IMU_Kp*(ref.err.x + ref.err_Int.x));     //IN RADIAN
	ref.g.y = gy *ANGLE_TO_RADIAN + (IMU_Kp*(ref.err.y + ref.err_Int.y));		  //IN RADIAN
	ref.g.z = gz *ANGLE_TO_RADIAN;
	// integrate quaternion rate and normalise
	ref_q[0] = ref_q[0] + (-ref_q[1] * ref.g.x - ref_q[2] * ref.g.y - ref_q[3] * ref.g.z)*half_T;
	ref_q[1] = ref_q[1] + (ref_q[0] * ref.g.x + ref_q[2] * ref.g.z - ref_q[3] * ref.g.y)*half_T;
	ref_q[2] = ref_q[2] + (ref_q[0] * ref.g.y - ref_q[1] * ref.g.z + ref_q[3] * ref.g.x)*half_T;
	ref_q[3] = ref_q[3] + (ref_q[0] * ref.g.z + ref_q[1] * ref.g.y - ref_q[2] * ref.g.x)*half_T;
	norm_q = sqrt(ref_q[0] * ref_q[0] + ref_q[1] * ref_q[1] + ref_q[2] * ref_q[2] + ref_q[3] * ref_q[3]);
	ref_q[0] = ref_q[0] / norm_q;
	ref_q[1] = ref_q[1] / norm_q;
	ref_q[2] = ref_q[2] / norm_q;
	ref_q[3] = ref_q[3] / norm_q;
	plane.roll = (float)atan2(2 * (ref_q[0] * ref_q[1] + ref_q[2] * ref_q[3]), 1 - 2 * (ref_q[1] * ref_q[1] + ref_q[2] * ref_q[2]));//输出单位弧度
	plane.pitch = asin(2 * (ref_q[1] * ref_q[3] - ref_q[0] * ref_q[2]));
	return plane;
	//rol = (float)fast_atan2(2 * (ref_q[0] * ref_q[1] + ref_q[2] * ref_q[3]), 1 - 2 * (ref_q[1] * ref_q[1] + ref_q[2] * ref_q[2])) *57.3f;
	//pit = asin(2 * (ref_q[1] * ref_q[3] - ref_q[0] * ref_q[2])) *57.3f;


}















#endif



#pragma once
#ifndef _FILTER_HPP
#define _FILTER_HPP




float planeBarometerData_filter(float Filter_value,float Current_value,float ratio)
{
	return (float)((1 - ratio)*Filter_value + ratio*Current_value);
}

float position_filter_x(float Filter_value, float Current_value, float ratio)
{
	return (float)((1 - ratio)*Filter_value + ratio*Current_value);
}

float position_filter_y(float Filter_value, float Current_value, float ratio)
{
	return (float)((1 - ratio)*Filter_value + ratio*Current_value);
}





#endif
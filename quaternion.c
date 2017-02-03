/*$T indentinput.c GC 1.140 01/16/17 11:20:06 */

/*
 * Sensor Fusion code for estimating orientation of Arduino-based IMU 2011 Joseph
 * Malloch / Input Devices and Music Interaction Laboratory This program is free software;
 * you can redistribute it and/or modify it under the terms of the GNU License.
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE. See the GNU License V2 for more details.
 */
#include "quaternion.h"

/* Constants */
double	PI = 3.14159265359;
double	halfPI = 1.5707963267949;
double	twoPI = 6.283185307179586;

/*
 =======================================================================================================================
    -(quat_t initialization)
 =======================================================================================================================
 */
void quaternion_init(quat_t *quat)
{
	quat->q_w = 1;
	quat->q_x = 0;
	quat->q_y = 0;
	quat->q_z = 0;
}

/*
 =======================================================================================================================
    -(quaternion multiplication)
 =======================================================================================================================
 */
void quaternion_multiply(quat_t *left, quat_t *right, quat_t *out)
{
	/*~~~~~~~~~*/
	double	temp;
	/*~~~~~~~~~*/

	temp = left->q_w * right->q_w - left->q_x * right->q_x - left->q_y * right->q_y - left->q_z * right->q_z;
	out->q_w = temp;

	temp = left->q_x * right->q_w + left->q_w * right->q_x + left->q_y * right->q_z - left->q_z * right->q_y;
	out->q_x = temp;

	temp = left->q_w * right->q_y - left->q_x * right->q_z + left->q_y * right->q_w + left->q_z * right->q_x;
	out->q_y = temp;

	temp = left->q_w * right->q_z + left->q_x * right->q_y - left->q_y * right->q_x + left->q_z * right->q_w;
	out->q_z = temp;
}

/*
 =======================================================================================================================
    -(quaternion inverse)
 =======================================================================================================================
 */
void quaternion_inverse(quat_t *input, quat_t *out)
{
	out->q_w = input->q_w;
	out->q_x = input->q_x;
	out->q_y = input->q_y;
	out->q_z = input->q_z;
}

/*
 =======================================================================================================================
    -(quaternion conjugate)
 =======================================================================================================================
 */
void quaternion_conjugate(quat_t *input, quat_t *out)
{
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	float	squared_quat = input->q_w *
		input->q_w +
		input->q_x *
		input->q_x +
		input->q_y *
		input->q_y +
		input->q_z *
		input->q_z;
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	if(squared_quat == 0.0) squared_quat = 0.0000001;

	out->q_w = input->q_w / squared_quat;
	out->q_x = input->q_x * -1.0 / squared_quat;
	out->q_y = input->q_y * -1.0 / squared_quat;
	out->q_z = input->q_z * -1.0 / squared_quat;
}

/*
 =======================================================================================================================
    -(quaternion slerp)
 =======================================================================================================================
 */
void quaternion_slerp(quat_t *left, quat_t *right, quat_t *out, double weight)
{
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	double	dot = quaternion_dot_product(left, right);
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	if(dot > 0.9995)
	{
		out->q_w = left->q_w + (right->q_w - left->q_w) * weight;
		out->q_x = left->q_x + (right->q_x - left->q_x) * weight;
		out->q_y = left->q_y + (right->q_y - left->q_y) * weight;
		out->q_z = left->q_z + (right->q_z - left->q_z) * weight;
		quaternion_normalize(out);
		return;
	}

	if(dot > 1.0)
		dot = 1.0;
	else if(dot < -1.0)
		dot = -1.0;

	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	double	theta_0 = acos(dot);
	double	theta = (0. < theta_0 && theta_0 < halfPI) ? theta_0 * weight : (theta_0 - PI) * weight;
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	out->q_w = right->q_w - left->q_w * dot;
	out->q_x = right->q_x - left->q_x * dot;
	out->q_y = right->q_y - left->q_y * dot;
	out->q_z = right->q_z - left->q_z * dot;

	quaternion_normalize(o);

	out->q_w = left->q_w * cos(theta) + out->q_w * sin(theta);
	out->q_x = left->q_x * cos(theta) + out->q_x * sin(theta);
	out->q_y = left->q_y * cos(theta) + out->q_y * sin(theta);
	out->q_z = left->q_z * cos(theta) + out->q_z * sin(theta);
}

/*
 =======================================================================================================================
    -(quaternion dot product)
 =======================================================================================================================
 */
double quaternion_dot_product(quat_t *left, quat_t *right)
{
	return(left->q_w * right->q_w + left->q_x * right->q_x + left->q_y * right->q_y + left->q_z * right->q_z);
}

/*
 =======================================================================================================================
    -(quaternion normalization)
 =======================================================================================================================
 */
void quaternion_normalize(quat_t *quat)
{
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	double	squared_quat = sqrt(quat->q_w * quat->q_w + quat->q_x * quat->q_x + quat->q_y * quat->q_y + quat->q_z * quat->q_z);
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	if(squared_quat == 0.0) squared_quat = 0.0000001;

	quat->q_w /= squared_quat;
	quat->q_x /= squared_quat;
	quat->q_y /= squared_quat;
	quat->q_z /= squared_quat;
}

/*
 =======================================================================================================================
    -(quaternion copy)
 =======================================================================================================================
 */
void quaternion_copy(quat_t *original, quat_t *newer)
{
	newer->q_w = original->q_w;
	newer->q_x = original->q_x;
	newer->q_y = original->q_y;
	newer->q_z = original->q_z;
}

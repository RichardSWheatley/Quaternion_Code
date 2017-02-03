/**************************************************************************
 *                                                                         *
 * Sensor Fusion code for estimating orientation of Arduino-based IMU      *
 * 2011 Joseph Malloch / Input Devices and Music Interaction Laboratory    *
 *                                                                         *
 ***************************************************************************
 *                                                                         *
 * This program is free software; you can redistribute it and/or modify    *
 * it under the terms of the GNU License.                                  *
 * This program is distributed in the hope that it will be useful,         *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 * GNU License V2 for more details.                                        *
 *                                                                         *
 ***************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif

// Data structures
typedef struct _quat
{
    double  q_w;
    double  q_x;
    double  q_y;
    double  q_z;
} t_quat;

typedef struct _axes
{
    double  x;
    double  y;
    double  z;
} t_axes;

// Function prototypes
// *********************************************************
// Quaternion Initialization
//   This initialization is 1, 0, 0, 0
void    quaternion_init(quat_t *quat);

// *********************************************************
// Quaternion Multiplication
//   Quaternion multiplication is non-commutative
//   This multiplication
void    quaternion_multiply(quat_t *left, quat_t *right, quat_t *out);

// *********************************************************
// Quaternion Inverse
void    quaternion_inverse(quat_t *input, quat_t *out);

// *********************************************************
// Quaternion Conjugate
void    quaternion_conjugate(quat_t *input, quat_t *out);

// *********************************************************
// Quaternion Spherical Linear Interpolation
void    quaternion_slerp(quat_t *left, quat_t *right, quat_t *out, double weight);

// *********************************************************
// -(quaternion dot product)--------------------------------
double  quaternion_dot_product(quat_t *left, quat_t *right);

// *********************************************************
// -(quaternion normalization)------------------------------
void    quaternion_normalize(quat_t *quat);

// *********************************************************
// -(quaternion copy)---------------------------------------
void    quaternion_copy(quat_t *original, quat_t *newer);

#ifdef __cplusplus
}
#endif

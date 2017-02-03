#include <math.h>
#include <stdio.h>

class Quaternion{
	private:
		double q0, q1, q2, q3;
	protected:
		~Quaternion();
	public:
		Quaternion();
		Quaternion(double, double, double, double);
		
		// Quaternion multiply by a scalar
		Quaternion multiply(Quaternion q_a, double s);

		// Quaternion multiply by another quaternion
		Quaternion multiply(Quaternion q_a, Quaternion q_b);
		
		// Quaternion squared magnitude
		double length(Quaternion q_a);
		
		// Quaternion Norm
		Quaternion norm(Quaternion q_a);

		// Quaternion Inverse
		Quaternion inverse(Quaternion q_a);
};

Quaternion::Quaternion(void){
	q0 = 1; q1 = 0; q2 = 0; q3 = 0;
}

Quaternion::Quaternion(double a, double b, double c, double d){
	q0 = a; q1 = b; q2 = c; q3 = d;
}

// Quaternion multiply by a scalar
Quaternion Quaternion::multiply(Quaternion q_a, double s){
	return Quaternion(q_a->q0 * s, q_a->q1 * s, q_a->q2 * s, q_a->q3 * s);
}

// Quaternion multiply
Quaternion Quaternion::multiply(Quaternion q_a, Quaternion q_b){
	return Quaternion(q_a->q0*q_b->q0 - q_a->q1*q_b->q1 - q_a->q2*q_b->q2 - q_a->q3*q_b->q3, q_a->q1*q_b->q0 + q_a->q0*q_b->q1 + q_a->q2*q_b->q3 - q_a->q3*q_b->q2, q_a->q0*q_b->q2 - q_a->q1*q_b->q3 + q_a->q2*q_b->q0 + q_a->q3*q_b->q1, q_a->q0*q_b->q3 + q_a->q1*q_b->q2 - q_a->q2*q_b->q1 + q_a->q3*q_b->q0);
}

// Quaternion squared magnitude
double Quaternion::length(Quaternion q_a){ return(q_a->q0*q_a->q0 + q_a->q1*q_a->q1 + q_a->q2*q_a->q2 + q_a->q3*q_a->q3) }

// Quaternion Norm
Quaternion Quaternion::norm(Quaternion q_a){
	double len = length(q_a);
	return Quaternion(q_a->q0 / len, q_a->q1 / len, q_a->q2 / len, q_a->q3 / len);
}

// Quaternion Inverse
Quaternion Quaternion::inverse(Quaternion q_a){
	double len = length(q_a);
	return Quaternion(q_a->q0 / len, -q_a->q1 / len, -q_a->q2 / len, -q_a->q3 / len);
}

class Quaternion_Rotations{
	private:
		Quaternion Quat;
	protected:
		~Quaternion_Rotations();
	public:
		Quaternion_Rotations();

		// Elevation Quaternion from Normalized Accel data
		//   Accel data must be normalized or the
		//   equation asin(-ax) won't work.
		Quaternion Quaternion_Rotations::elevationQuaternion(double ax, double ay, double az);

		// Roll Quaternion from Normalized Accel data
		//   Accel data must be normalized or the
		//   equation asin(-ax) won't work.
		Quaternion Quaternion_Rotations::rollQuaternion(double ax, double ay, double az);

		// Mag Quaternion from Normalized mag data
		Quaternion Quaternion_Rotations::magQuaternion(double mx, double my, double mz);

		// Azimuth Quaternion from Normalized mag data
		Quaternion Quaternion_Rotations::azimuthQuaternion(Quaternion qe, Quaternion qr, Quaternion qm);

		Quaternion Quaternion_Rotations::totalQuaternion(Quaternion q_a, Quaternion q_e, Quaternion q_r);
};

Quaternion Quaternion_Rotations::elevationQuaternion(double ax, double ay, double az){
	double angle_pitch = asin(-ax);
	return Quaternion(cos(angle_pitch * 0.5), 0.0, sin(angle_pitch * 0.5), 0.0)
}

// Roll Quaternion from Normalized Accel data
//   Accel data must be normalized or the
//   equation asin(-ax) won't work.
Quaternion Quaternion_Rotations::rollQuaternion(double ax, double ay, double az){
	double angle_pitch = asin(-ax);
	double angle_roll = acos(-az / cos(angle_pitch));
	return Quaternion(cos(angle_roll * 0.5), sin(angle_roll * 0.5), 0.0, 0.0)
}

// Mag Quaternion from Normalized mag data
Quaternion Quaternion_Rotations::magQuaternion(double mx, double my, double mz){
	return Quaternion(0.0, mx, my, mz)
}

// Azimuth Quaternion from Normalized mag data
Quaternion Quaternion_Rotations::azimuthQuaternion(Quaternion qe, Quaternion qr, Quaternion qm){
	Quaternion qm_rot = multiply(multiply(multiply(multiply(qe, qr), qm), inverse(qr)), inverse(qe));
	double angle_az = atan2(qm_rot->1, qm_rot->q2);
	return Quaternion(cos(angle_az * 0.5), 0.0, 0.0, sin(angle_az * 0.5))
}

Quaternion Quaternion_Rotations::totalQuaternion(Quaternion q_a, Quaternion q_e, Quaternion q_r){
	return(multiply(multiply(q_a, q_e), q_r))
}

#ifndef QUATERNION_CLASS_CPP
#define QUATERNION_CLASS_CPP

#include <math.h>
// Constants
double PI = 3.141592653589793;
double halfPI = 1.570796326794897;
double twoPI = 6.283185307179586;

class _3Vect{
private:
	double w0, w1, w2;

protected:
	// a simple destructor.
	virtual ~_3Vect();
	
	
public:
	// Constructors with and without initialized values
	_3Vect(void){
		w0 = 0; w1 = 0; w2 = 0;
	}

	_3Vect(double a, double b, double c){
		w0 = a; w1 = b; w2 = c;
	}

};


class Quat{
private:
	double q0, q1, q2, q3;

protected:
	// a simple destructor.
	virtual ~Quat();
	
	
public:
	Quat(void){
		q0 = 1; q1 = 0; q2 = 0; q3 = 0;
	}

	Quat(double a, double b, double c, double d){
		q0 = a; q1 = b; q2 = c; q3 = d;
	}

	// Quaternion add
	void add(Quat *q_a, Quat *q_b, Quat *q_add){
		q_add->q0 = q_a->q0 + q_b->q0; 
		q_add->q1 = q_a->q1 + q_b->q1; 
		q_add->q2 = q_a->q2 + q_b->q3; 
		q_add->q3 = q_a->q3 + q_b->q3; 
	}

	// Quaternion multiply a scalar
	void multiply(Quat *q_mult, double value){
		q_mult->q0 *= value;
		q_mult->q1 *= value;
		q_mult->q2 *= value;
		q_mult->q3 *= value;
	}
	
	// Quaternion multiply quaternions
	void multiply(Quat *q_a, Quat *q_b, Quat *q_mult){
		q_mult->q0 = q_a->q0*q_b->q0 - q_a->q1*q_b->q1 - q_a->q2*q_b->q2 - q_a->q3*q_b->q3;
		q_mult->q1 = q_a->q1*q_b->q0 + q_a->q0*q_b->q1 + q_a->q2*q_b->q3 - q_a->q3*q_b->q2;
		q_mult->q2 = q_a->q0*q_b->q2 - q_a->q1*q_b->q3 + q_a->q2*q_b->q0 + q_a->q3*q_b->q1;
		q_mult->q3 = q_a->q0*q_b->q3 + q_a->q1*q_b->q2 - q_a->q2*q_b->q1 + q_a->q3*q_b->q0;
	}
	
	// Squared Magnitiude of a quaterion (4-vector)
	double quat_squared_magnitude(Quat *q_a){
		return(sqrt(q_a->q0*q_a->q0 + q_a->q1*q_a->q1 + q_a->q2*q_a->q2 + q_a->q3*q_a->q3));
}
	
	// Squared Magnitiude of two values
	double quat_squared_magnitude(double a, double b){ return(sqrt(a*a + b*b)); }
	
	// Quaternion norm
	void norm(Quat *q_a){
		double squared_magnitude = quat_squared_magnitude(q_a);
		if (squared_magnitude == 0)
			squared_magnitude = 0.00000001;
	    q_a->q0 /= squared_magnitude;
	    q_a->q1 /= squared_magnitude;
	    q_a->q2 /= squared_magnitude;
	    q_a->q3 /= squared_magnitude;
	}
	
	// Quaternion Inverse
	void inverse(Quat *q_a){
		q_a->q1 = -(q_a->q1);
		q_a->q2 = -(q_a->q2);
		q_a->q3 = -(q_a->q3);
	}

	// Elevation Quaternion from Normalized Accel data
	//   Accel data must be normalized or the
	//   equation asin(-ax) won't work.
	void quat_pitch(double ax, double ay, double az, Quat *q_elevation){
		double angle_pitch = asin(-ax);
		q_elevation->q0 = cos(angle_pitch/2.0);
		q_elevation->q1 = 0.0;
		q_elevation->q2 = sin(angle_pitch/2.0);
		q_elevation->q3 = 0.0;
	}
	
	// Roll Quaternion from Normalized Accel data
	//   Accel data must be normalized or the
	//   equation angle equations won't work.
	void quat_roll(double ax, double ay, double az, Quat *q_roll){
		double angle_pitch = asin(-ax);
		double angle_roll = acos(-az/cos(angle_pitch));
		q_roll->q0 = cos(angle_roll/2.0);
		q_roll->q1 = sin(angle_roll/2.0);
		q_roll->q2 = 0.0;
		q_roll->q3 = 0.0;
	}
	
	// Mag Quaternion from Normalized mag data
	//   Magnetometer data must be normalized
	void quat_mag(double mx, double my, double mz, Quat *q_mag){
		q_mag->q0 = 0.0;
		q_mag->q1 = mx;
		q_mag->q2 = my;
		q_mag->q3 = mz;
	}
	
	// Azimuth Quaternion from Normalized mag data
	void quat_az(Quat *q_e, Quat *q_r, Quat *q_m, Quat *q_azimuth){
		multiply(q_e, q_r, q_azimuth);
		multiply(q_azimuth, q_m, q_azimuth);
		// finish

		double squared_magnitude = quat_squared_magnitude(q_azimuth->q1, q_azimuth->q2);
		double angle_az = atan2(q_azimuth->q1/squared_magnitude, q_azimuth->q2/square_magnitude);
	
		q_azimuth->q0 = cos(angle_az/2.0);
		q_azimuth->q1 = 0.0;
		q_azimuth->q2 = 0.0;
		q_azimuth->q3 = sin(angle_az/2.0);
	}
	
	// Total Quaternion from Normalized mag data
	void quat_total_rot(Quat *q_a, Quat *q_e, Quat *q_r, Quat *q_total){
		Quat q_int = Quat();
		multiply(q_a, q_e, &q_int);
		multiply(&q_int, q_r, q_total);
	}
	
	void quat_vect_multiply(Quat *q, _3Vect *v, Quat *qm){
		qm->q0 = 0.5*(-q->q1 * v->w0 - q->q2 * v->w1 - q->q3 * v->w2);
		qm->q1 = 0.5*( q->q0 * v->w0 - q->q3 * v->w1 + q->q2 * v->w2);
		qm->q2 = 0.5*( q->q3 * v->w0 + q->q0 * v->w1 - q->q1 * v->w2);
		qm->q3 = 0.5*(-q->q2 * v->w0 + q->q1 * v->w1 + q->q0 * v->w2);
	}

};

#endif
#ifndef __QUARTERNION_H__
#define __QUARTERNION_H__

/****************************************
 * Quaternion class
 * By Will Perone
 * Original: 12-09-2003  
 * Revised:  27-09-2003
 *           22-11-2003
 *           10-12-2003
 *           15-01-2004
 *           16-04-2004
 *           29-07-2011   added corrections from website,
 *           22-12-2011   added correction to *= operator, thanks Steve Rogers
 *			 09-05-2012   modification for rlab by shoefer
 *
 * Notes:  
 * if |q|=1 then q is a unit quaternion
 * if q=(0,v) then q is a pure quaternion 
 * if |q|=1 then q conjugate = q inverse
 * if |q|=1 then q= [cos(angle), u*sin(angle)] where u is a unit vector 
 * q and -q represent the same rotation 
 * q*q.conjugate = (q.length_squared, 0) 
 * ln(cos(theta),sin(theta)*v)= ln(e^(theta*v))= (0, theta*v)
 ****************************************/


#include "rMath/rMKL.h"

class Quaternion
{
	double    s; // the real component
	dVector v; // the imaginary components

public:

	Quaternion() {
		s = 0;
		v.resize(3,0.);
	}
	Quaternion(double real, double x, double y, double z): s(real) {
		v.resize(3);
		v[0] = x;
		v[1] = y;
		v[2] = z;
	}

	/**
	 * From dVector - format (real, x, y, z)
	 */
	Quaternion(dVector d) {
		assert(d.size() == 4);
		s = d[0];
		v.resize(3);
		v[0] = d[1];
		v[1] = d[2];
		v[2] = d[3];
	}

	/**
	 * From scalar (real part) and dVector (imaginary part)
	 */
	Quaternion(double real, dVector imag) : s(real) {
		assert(imag.size() == 3);
		v.resize(3);
		v[0] = imag[1];
		v[1] = imag[2];
		v[2] = imag[3];
	}
	/**
	 * From three Euler angles ZYX
	 */
	Quaternion(double theta_z, double theta_y, double theta_x)
	{
		double cos_z_2 = cos(0.5*theta_z);
		double cos_y_2 = cos(0.5*theta_y);
		double cos_x_2 = cos(0.5*theta_x);

		double sin_z_2 = sin(0.5*theta_z);
		double sin_y_2 = sin(0.5*theta_y);
		double sin_x_2 = sin(0.5*theta_x);

		v.resize(3);

		s   = cos_z_2*cos_y_2*cos_x_2 + sin_z_2*sin_y_2*sin_x_2;
		v[0] = cos_z_2*cos_y_2*sin_x_2 - sin_z_2*sin_y_2*cos_x_2;
		v[1] = cos_z_2*sin_y_2*cos_x_2 + sin_z_2*cos_y_2*sin_x_2;
		v[2] = sin_z_2*cos_y_2*cos_x_2 - cos_z_2*sin_y_2*sin_x_2;

	}
			
	//! basic operations
	Quaternion &operator =(const Quaternion &q)		
	{ s= q.s; v= q.v; return *this; }

	const Quaternion operator +(const Quaternion &q) const	
	{ return Quaternion(s+q.s, v+q.v); }

	const Quaternion operator -(const Quaternion &q) const	
	{ return Quaternion(s-q.s, v-q.v); }

	const Quaternion operator *(const Quaternion &q) const	
	{	return Quaternion(s*q.s - v.inner(q.v),
				  v[1]*q.v[2] - v[2]*q.v[1] + s*q.v[0] + v[0]*q.s,
				  v[2]*q.v[0] - v[0]*q.v[2] + s*q.v[1] + v[1]*q.s,
				  v[0]*q.v[1] - v[1]*q.v[0] + s*q.v[2] + v[2]*q.s);
	}

	const Quaternion operator /(const Quaternion &q) const	
	{
		Quaternion p(q); 
		p.invert(); 
		return *this * p;
	}

	const Quaternion operator *(double scale) const
	{ return Quaternion(s*scale,v*scale); }

	const Quaternion operator /(double scale) const
	{ return Quaternion(s/scale,v/scale); }

	const Quaternion operator -() const
	{ return Quaternion(-s, -v); }
	
	const Quaternion &operator +=(const Quaternion &q)		
	{ v+=q.v; s+=q.s; return *this; }

	const Quaternion &operator -=(const Quaternion &q)		
	{ v-=q.v; s-=q.s; return *this; }

	const Quaternion &operator *=(const Quaternion &q)		
	{			
		double x= v[0], y= v[1], z= v[2], sn= s*q.s - v.inner(q.v);
		v[0]= y*q.v[2] - z*q.v[1] + s*q.v[0] + x*q.s;
		v[1]= z*q.v[0] - x*q.v[2] + s*q.v[1] + y*q.s;
		v[2]= x*q.v[1] - y*q.v[0] + s*q.v[2] + z*q.s;
		s= sn;
		return *this;
	}
	
	const Quaternion &operator *= (double scale)			
	{ v*=scale; s*=scale; return *this; }

	const Quaternion &operator /= (double scale)			
	{ v/=scale; s/=scale; return *this; }
	

	//! gets the length of this Quaternion
	double length() const
	{ return (double)sqrt(s*s + v.inner(v)); }

	//! gets the squared length of this Quaternion
	double length_squared() const
	{ return (double)(s*s + v.inner(v)); }

	//! normalizes this Quaternion
	void normalize()
	{ *this/=length(); }

	//! returns the normalized version of this Quaternion
	Quaternion normalized() const
	{ return  *this/length(); }

	//! computes the conjugate of this Quaternion
	void conjugate()
	{ v=-v; }

	//! inverts this Quaternion
	void invert()
	{ conjugate(); *this/=length_squared(); }


	/*
	//! returns the logarithm of a Quaternion = v*a where q = [cos(a),v*sin(a)]
	Quaternion log() const
	{
		double a = (double)acos(s);
		double sina = (double)sin(a);
		Quaternion ret;

		ret.s = 0;
		if (sina > 0)
		{
			ret.v[0] = a*v[0]/sina;
			ret.v[1] = a*v[1]/sina;
			ret.v[2] = a*v[2]/sina;
		} else {
			ret.v[0]= ret.v[1]= ret.v[2]= 0;
		}
		return ret;
	}

	//! returns e^Quaternion = exp(v*a) = [cos(a),vsin(a)]
	Quaternion exp() const
	{
		double a = (double)v.size();
		double sina = (double)sin(a);
		double cosa = (double)cos(a);
		Quaternion ret;

		ret.s = cosa;
		if (a > 0)
		{
			ret.v[0] = sina * v[0] / a;
			ret.v[1] = sina * v[1] / a;
			ret.v[2] = sina * v[2] / a;
		} else {
			ret.v[0] = ret.v[1] = ret.v[2] = 0;
		}
		return ret;
	}

	//! casting to a 4x4 isomorphic matrix for right multiplication with vector
	operator matrix4() const
	{			
		return matrix4(s,  -v[0], -v[1],-v[2],
				v[0],  s,  -v[2], v[1],
				v[1], v[2],    s,-v[0],
				v[2],-v[1],  v[0],   s);
	}
	
	//! casting to 3x3 rotation matrix
	operator matrix3() const
	{
		Assert(length() > 0.9999 && length() < 1.0001, "Quaternion is not normalized");		
		return matrix3(1-2*(v[1]*v[1]+v[2]*v[2]), 2*(v[0]*v[1]-s*v[2]),   2*(v[0]*v[2]+s*v[1]),   
				2*(v[0]*v[1]+s*v[2]),   1-2*(v[0]*v[0]+v[2]*v[2]), 2*(v[1]*v[2]-s*v[0]),   
				2*(v[0]*v[2]-s*v[1]),   2*(v[1]*v[2]+s*v[0]),   1-2*(v[0]*v[0]+v[1]*v[1]));
	}
	*/

	double inner(const Quaternion &q2)
	{ return this->v.inner(q2.v) + this->s*q2.s; }


	//! computes the dot product of 2 Quaternions
	static inline double dot(const Quaternion &q1, const Quaternion &q2) 
	{ return q1.v.inner(q2.v) + q1.s*q2.s; }

	/*
	//! linear Quaternion interpolation
	static Quaternion lerp(const Quaternion &q1, const Quaternion &q2, double t) 
	{ return (q1*(1-t) + q2*t).normalized(); }

	//! spherical linear interpolation
	static Quaternion slerp(const Quaternion &q1, const Quaternion &q2, double t) 
	{
		Quaternion q3;
		double dot = Quaternion::dot(q1, q2);

		//	dot = cos(theta)
		//	if (dot < 0), q1 and q2 are more than 90 degrees apart,
		//	so we can invert one to reduce spinning	
		if (dot < 0)
		{
			dot = -dot;
			q3 = -q2;
		} else q3 = q2;
		
		if (dot < 0.95f)
		{
			double angle = acos(dot);
			return (q1*sin(angle*(1-t)) + q3*sin(angle*t))/sin(angle);
		} else // if the angle is small, use linear interpolation								
			return lerp(q1,q3,t);		
	}

	//! This version of slerp, used by squad, does not check for theta > 90.
	static Quaternion slerpNoInvert(const Quaternion &q1, const Quaternion &q2, double t) 
	{
		double dot = Quaternion::dot(q1, q2);

		if (dot > -0.95f && dot < 0.95f)
		{
			double angle = acos(dot);			
			return (q1*sin(angle*(1-t)) + q2*sin(angle*t))/sin(angle);
		} else  // if the angle is small, use linear interpolation								
			return lerp(q1,q2,t);			
	}

	//! spherical cubic interpolation
	static Quaternion squad(const Quaternion &q1,const Quaternion &q2,const Quaternion &a,const Quaternion &b,double t)
	{
		Quaternion c= slerpNoInvert(q1,q2,t),
			       d= slerpNoInvert(a,b,t);		
		return slerpNoInvert(c,d,2*t*(1-t));
	}

	//! Shoemake-Bezier interpolation using De Castlejau algorithm
	static Quaternion bezier(const Quaternion &q1,const Quaternion &q2,const Quaternion &a,const Quaternion &b,double t)
	{
		// level 1
		Quaternion q11= slerpNoInvert(q1,a,t),
				q12= slerpNoInvert(a,b,t),
				q13= slerpNoInvert(b,q2,t);		
		// level 2 and 3
		return slerpNoInvert(slerpNoInvert(q11,q12,t), slerpNoInvert(q12,q13,t), t);
	}

	//! Given 3 Quaternions, qn-1,qn and qn+1, calculate a control point to be used in spline interpolation
	static Quaternion spline(const Quaternion &qnm1,const Quaternion &qn,const Quaternion &qnp1)
	{
		Quaternion qni(qn.s, -qn.v);	
		return qn * (( (qni*qnm1).log()+(qni*qnp1).log() )/-4).exp();
	}
	*/

	//! converts from a normalized axis - angle pair rotation to a Quaternion
	static inline Quaternion from_axis_angle(const dVector &axis, double angle)
	{ return Quaternion(cos(angle/2), axis*sin(angle/2)); }

	//! returns the axis and angle of this unit Quaternion
	void to_axis_angle(dVector &axis, double &angle) const
	{
		angle = acos(s);

		// pre-compute to save time
		double sin_theta_inv = 1.0/sin(angle);

		// now the vector
		axis.resize(3);
		axis[0] = v[0]*sin_theta_inv;
		axis[1] = v[1]*sin_theta_inv;
		axis[2] = v[2]*sin_theta_inv;

		// multiply by 2
		angle*=2;
	}

	/*
	//! rotates v by this Quaternion (Quaternion must be unit)
	vector3f rotate(const vector3f &v)
	{   
		Quaternion V(0, v);
		Quaternion conjugate(*this);
		conjugate.conjugate();
		return (*this * V * conjugate).v;
	}

	//! returns the euler angles from a rotation Quaternion
	vector3f euler_angles(bool homogenous=true) const
	{
		double sqw = s*s;    
		double sqx = v[0]*v[0];    
		double sqy = v[1]*v[1];    
		double sqz = v[2]*v[2];    

		vector3f euler;
		if (homogenous) {
			euler.x = atan2f(2.f * (v[0]*v[1] + v[2]*s), sqx - sqy - sqz + sqw);    		
			euler.y = asin(-2.f * (v[0]*v[2] - v[1]*s));
			euler.z = atan2f(2.f * (v[1]*v[2] + v[0]*s), -sqx - sqy + sqz + sqw);    
		} else {
			euler.x = atan2f(2.f * (v[2]*v[1] + v[0]*s), 1 - 2*(sqx + sqy));
			euler.y = asin(-2.f * (v[0]*v[2] - v[1]*s));
			euler.z = atan2f(2.f * (v[0]*v[1] + v[2]*s), 1 - 2*(sqy + sqz));
		}
		return euler;
	}
	*/
};

#endif;
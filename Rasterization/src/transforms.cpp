#include "transforms.h"
#define _USE_MATH_DEFINES

#include "CGL/matrix3x3.h"
#include "CGL/vector2D.h"
#include "CGL/vector3D.h"
#include <math.h>

namespace CGL {

Vector2D operator*(const Matrix3x3 &m, const Vector2D &v) {
	Vector3D mv = m * Vector3D(v.x, v.y, 1);
	return Vector2D(mv.x / mv.z, mv.y / mv.z);
}

Matrix3x3 translate(float dx, float dy) {
	// Part 3: Fill this in.
	return Matrix3x3(1, 0, dx,
					0, 1, dy,
					0, 0, 1);
	/*
	Matrix3x3(a, b, c, 
			d, e, f, 
			0, 0, 1)
	after multiplication with (x, y, 1) (3D vector) leads to
	(ax + by + c, dx + ey + f, 1)
	so if a = 1, and b = 0, c = dx, the resultant x value is old_x + dy
	same case for y, where a = 0, b = 1
	
	// right scalar multiplication on the 3D vector by the matrix
	
	converts 2D vector to 3D by adding a unit vector in the z-direction, then multiples the
	transformation matrix, and returns a 2D vector using values from the resultant transformed 3D vector
	(in x y directions normalized by z?)
	*/
}

Matrix3x3 scale(float sx, float sy) {
	// Part 3: Fill this in.
	return Matrix3x3(sx, 0 , 0,
					0, sy, 0,
					0, 0, 1);
	/*
	(ax + by + c, dx + ey + f, 1)
	a = sx, b = 0, c = 0, similarly for y, e= sy, the rest are 0, scalar multiple/coefficient
	(1, 1), sx=2 , sy=3 then = (2, 3, 1)
	*/
}

// The input argument is in degrees counterclockwise
Matrix3x3 rotate(float deg) {
	// Part 3: Fill this in.
	float rad = deg * 3.14159265f / 180.0f; //the cos, sine functions take radian as parameter
	return Matrix3x3(cos(rad), -sin(rad), 0,
					sin(rad), cos(rad), 0,
					0, 0, 1);
	/*
	(ax + by + c, dx + ey + f, 1)
	c = 0, f = 0 since translation is not required, only multiples of x and y will determine the rotation
	the rest is the counter-clockwise matrix in 2D, as the argument is in ccw
	(cos(0) * x - sin(0) * y, sin(0) * x + cos(0) * y, 1)
	*/
		
}

}

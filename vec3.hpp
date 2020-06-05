#ifndef VEC3_HPP
#define VEC3_HPP

#include <math.h>

// represents a 3d vector. operations do what they say!
class Vec3 {
public:
	float x, y, z;
	
	Vec3(float x = 0.0f, float y = 0.0f, float z = 0.0f):
		x(x),
		y(y),
		z(z) {
	}
	
	Vec3 operator+(const Vec3 & other) const {
		return Vec3(x + other.x, y + other.y, z + other.z);
	}
	
	Vec3 operator-(const Vec3 & other) const {
		return Vec3(x - other.x, y - other.y, z - other.z);
	}
	
	Vec3 operator*(float scale) const {
		return Vec3(x * scale, y * scale, z * scale);
	}
	
	static float dot(const Vec3 & a, const Vec3 & b) {
		return a.x * b.x + a.y * b.y + a.z + b.z;
	}
	
	static Vec3 cross(const Vec3 & a, const Vec3 & b) {
		return Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
	}
	
	float length() const {
		return dot(*this, *this);
	}
};

#endif

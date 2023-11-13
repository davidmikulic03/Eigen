#pragma once
#include "Eigen.hpp"

namespace Eigen {
	class Vector3f {
	public:
		Vector3f() 
		{
			this->x = 0.0f;
			this->y = 0.0f;
			this->z = 0.0f;
		}
		Vector3f(float x, float y, float z) {
			this->x = x;
			this->y = y;
			this->z = z;
		}
		Vector3f(float x, float y) {
			this->x = x;
			this->y = y;
			this->z = 0.0f;
		}
		Vector3f(float value) {
			this->x = value;
			this->y = value;
			this->z = value;
		}

	public:
		inline float SqrMagnitude() {
			return x * x + y * y + z * z;
		}
		inline float Magnitude() {
			return sqrt(SqrMagnitude());
		}
		inline Vector3f Normalized() {
			float Mag = Magnitude();

			if (Mag == 0)
				return Zero;
			return Vector3f(x, y, z) / Mag;
		}

	public:
		static inline float Dot(Vector3f a, Vector3f b) {
			return a.x * b.x + a.y * b.y + a.z * b.z;
		}
		static inline Vector3f Cross(Vector3f a, Vector3f b) {
			return Vector3f(
				a.y * b.z - a.z * b.y,
				a.z * b.x - a.x * b.z,
				a.x * b.y - a.y * b.x);
		}

		static inline float Angle(Vector3f From, Vector3f To) {
			float dot = Dot(From, To);
			float cos = dot / (From.Magnitude() * To.Magnitude());

			return acos(cos);
		}

		static inline Vector3f Lerp(Vector3f From, Vector3f To, float t) {
			return From * (1.0f - t) + To * t;
		}
		static inline Vector3f Slerp(Vector3f a, Vector3f b, float t) {
			float angle = Angle(a, b);
			return a * (sin((1.0f - t) * angle) / sin(angle)) + b * (sin(t * angle) / sin(angle));
		}

		static inline Vector3f Point(Vector3f From, Vector3f To) {
			return (To - From).Normalized();
		}

		static inline float SqrDistance(Vector3f From, Vector3f To) {
			return (To - From).SqrMagnitude();
		}
		static inline float Distance(Vector3f From, Vector3f To) {
			return (To - From).Magnitude();
		}

		static inline Vector3f Project(Vector3f From, Vector3f Onto) {
			return Onto * (Dot(From, Onto) / Onto.SqrMagnitude());
		}
		static inline Vector3f ProjectOntoPlane(Vector3f From, Vector3f Normal) {
			return From - Project(From, Normal);
		}
		static inline Vector3f Reflect(Vector3f Vector, Vector3f Normal) {
			return Vector - ProjectOntoPlane(Vector, Normal) * 2;
		}

	public:
		float operator[](unsigned int index) {
			switch (index)
			{
			case 0:
				return x;
				break;
			case 1:
				return y;
				break;
			case 2:
				return z;
				break;
			default:
				return 0;
			}
		}
		bool operator==(Vector3f v) {
			if (x == v.x && y == v.y && z == v.z)
				return true;
			return false;
		}

		inline Vector3f operator+(Vector3f v) {
			return Vector3f(x + v.x, y + v.y, z + v.z);
		}
		inline Vector3f operator+(float f) {
			return Vector3f(x + f, y + f, z + f);
		}
		inline Vector3f operator-(Vector3f v) {
			return Vector3f(x - v.x, y - v.y, z - v.z);
		}
		inline Vector3f operator-(float f) {
			return Vector3f(x - f, y - f, z - f);
		}

		inline Vector3f operator*(float k) {
			return Vector3f(k * x, k * y, k * z);
		}
		inline Vector3f operator/(float q) {
			return Vector3f(x / q, y / q, z / q);
		}

		inline void operator=(float a) {
			x = a; y = a; z = a;
		}
		inline void operator+=(float a) {
			x += a;
			y += a;
			z += a;
		}
		inline void operator-=(float a) {
			x -= a;
			y -= a;
			z -= a;
		}
		inline void operator*=(float k) {
			x *= k;
			y *= k;
			z *= k;
		}
		inline void operator/=(float q) {
			x /= q;
			y /= q;
			z /= q;
		}

		inline Vector3f operator-() {
			return Vector3f(-x, -y, -z);
		}

	//public: //SWIZZLING!!!
	//	const static Vector3f xyz;
	//	const static Vector3f yxz;
	//	const static Vector3f yzx;
	//	const static Vector3f xzy;
	//	const static Vector3f yxz;
	//	const static Vector3f zxy;
	//	const static Vector3f xzy;

	public:
		float x = 0.0f, y = 0.0f, z = 0.0f;

	public:
		static const Vector3f Zero;
		static const Vector3f Forward;
		static const Vector3f Backward;
		static const Vector3f Right;
		static const Vector3f Left;
		static const Vector3f Up;
		static const Vector3f Down;
	};

	//const Vector3f xyz = Vector3f(x, y, z);
	//const Vector3f yxz = Vector3f(y, x, z);
	//const Vector3f yzx = Vector3f(y, z, x);
	//const Vector3f xzy = Vector3f(x, z, y);
	//const Vector3f yxz = Vector3f(y, x, z);
	//const Vector3f zxy = Vector3f(z, x, y);
	//const Vector3f xzy = Vector3f(x, z, y);
	
	const Vector3f Vector3f::Zero = Vector3f(0.0f);
	const Vector3f Vector3f::Forward = Vector3f(0.0f, 1.0f, 0.0f);
	const Vector3f Vector3f::Backward = Vector3f(0.0f, -1.0f, 0.0f);
	const Vector3f Vector3f::Right = Vector3f(1.0f, 0.0f, 0.0f);
	const Vector3f Vector3f::Left = Vector3f(-1.0f, 0.0f, 0.0f);
	const Vector3f Vector3f::Up = Vector3f(0.0f, 0.0f, 1.0f);
	const Vector3f Vector3f::Down = Vector3f(0.0f, 0.0f, -1.0f);
}

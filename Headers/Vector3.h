#pragma once
#include "Eigen.h"

namespace Eigen {
	class Vector3 {
	public:
		Vector3() {
			this->x = 0.0;
			this->y = 0.0;
			this->z = 0.0;
		}
		Vector3(double x, double y, double z) {
			this->x = x;
			this->y = y;
			this->z = z;
		}
		Vector3(double x, double y) {
			this->x = x;
			this->y = y;
			this->z = 0.0;
		}
		Vector3(double value) {
			this->x = value;
			this->y = value;
			this->z = value;
		}

	public:
		inline double SqrMagnitude() {
			return x * x + y * y + z * z;
		}
		inline double Magnitude() {
			return sqrt(SqrMagnitude());
		}
		inline Vector3 Normalized() {
			double Mag = Magnitude();
			if (Mag <= Epsilon)
				return Zero;

			return Vector3(x, y, z) / Mag;
		}

	public:
		static inline double Dot(Vector3 a, Vector3 b) {
			return a.x * b.x + a.y * b.y + a.z * b.z;
		}
		static inline Vector3 Cross(Vector3 a, Vector3 b) {
			return Vector3(
				a.y * b.z - a.z * b.y,
				a.z * b.x - a.x * b.z,
				a.x * b.y - a.y * b.x);
		}

		static inline double Angle(Vector3 From, Vector3 To) {
			double dot = Dot(From, To);
			double cos = dot / (From.Magnitude() * To.Magnitude());

			return acos(cos);
		}

		static inline Vector3 Lerp(Vector3 From, Vector3 To, double t) {
			return From * (1.0 - t) + To * t;
		}
		static inline Vector3 Slerp(Vector3 From, Vector3 To, double t) {
			double angle = Angle(From, To);
			return From * (sin((1.0 - t) * angle) / sin(angle)) + To * (sin(t * angle) / sin(angle));
		}

		static inline Vector3 Point(Vector3 From, Vector3 To) {
			return (To - From).Normalized();
		}

		static inline double SqrDistance(Vector3 From, Vector3 To) {
			return (To - From).SqrMagnitude();
		}
		static inline double Distance(Vector3 From, Vector3 To) {
			return (To - From).Magnitude();
		}

		static inline Vector3 Project(Vector3 From, Vector3 Onto) {
			return Onto * (Dot(From, Onto) / Onto.SqrMagnitude());
		}
		static inline Vector3 ProjectOntoPlane(Vector3 From, Vector3 Normal) {
			return From - Project(From, Normal);
		}
		static inline Vector3 Reflect(Vector3 Vector, Vector3 Normal) {
			return Vector - ProjectOntoPlane(Vector, Normal) * 2;
		}

	public:
		double operator[](unsigned int index) {
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
		bool operator==(Vector3 v) {
			if (x == v.x && y == v.y && z == v.z)
				return true;
			return false;
		}

		inline Vector3 operator+(Vector3 v) {
			return Vector3(x + v.x, y + v.y, z + v.z);
		}
		inline Vector3 operator+(double d) {
			return Vector3(x + d, y + d, z + d);
		}
		inline Vector3 operator-(Vector3 v) {
			return Vector3(x - v.x, y - v.y, z - v.z);
		}
		inline Vector3 operator-(double d) {
			return Vector3(x - d, y - d, z - d);
		}

		inline Vector3 operator*(double k) {
			return Vector3(k * x, k * y, k * z);
		}
		inline Vector3 operator/(double q) {
			return Vector3(x / q, y / q, z / q);
		}

		inline void operator=(double a) {
			x = a; y = a; z = a;
		}
		inline void operator=(Vector3 a) {
			x = a.x; y = a.y; z = a.z;
		}
		inline void operator+=(double a) {
			x += a;
			y += a;
			z += a;
		}
		inline void operator-=(double a) {
			x -= a;
			y -= a;
			z -= a;
		}
		inline void operator*=(double k) {
			x *= k;
			y *= k;
			z *= k;
		}
		inline void operator/=(double q) {
			x /= q;
			y /= q;
			z /= q;
		}

		inline Vector3 operator-() {
			return Vector3(-x, -y, -z);
		}

	public:
		double x = 0.0, y = 0.0, z = 0.0;

	public:
		static const Vector3 Zero;
		static const Vector3 Forward;
		static const Vector3 Backward;
		static const Vector3 Right;
		static const Vector3 Left;
		static const Vector3 Up;
		static const Vector3 Down;
	};
	
	const Vector3 Vector3::Zero = Vector3(0.0);
	const Vector3 Vector3::Forward = Vector3(0.0, 1.0, 0.0);
	const Vector3 Vector3::Backward = Vector3(0.0, -1.0, 0.0);
	const Vector3 Vector3::Right = Vector3(1.0, 0.0, 0.0);
	const Vector3 Vector3::Left = Vector3(-1.0, 0.0, 0.0);
	const Vector3 Vector3::Up = Vector3(0.0, 0.0, 1.0);
	const Vector3 Vector3::Down = Vector3(0.0, 0.0, -1.0);
}
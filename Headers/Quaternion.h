#pragma once
#include "Eigen.h"

namespace Eigen {
	class Quaternion {
	public:
		Quaternion() {
			w = 0.0f;
			x = 0.0f;
			y = 0.0f;
			z = 0.0f;
		}
		Quaternion(double w, double x, double y, double z) {
			this->w = w;
			this->x = x;
			this->y = y;
			this->z = z;
		}
		Quaternion(double r, Vector3 v) {
			w = r;
			x = v.x;
			y = v.y;
			z = v.z;
		}

	public:
		inline Vector3 Vector() {
			return Vector3(x, y, z);
		}

	public:
		inline double SqrMagnitude() {
			return w * w + Vector().SqrMagnitude();
		}
		inline double Magnitude() {
			return sqrt(SqrMagnitude());
		}
		inline Quaternion Inverse() {
			Vector3 v = Vector();
			double SqrMag = SqrMagnitude();
			if (SqrMag -1 <= Epsilon)
				return Conjugate();
			return Quaternion(w / SqrMag, -v / SqrMag);
		}
		inline Quaternion Conjugate() {
			return Quaternion(w, -Vector());
		}

		inline Quaternion Normalized() {
			return *this / Magnitude();
		}
		
	public:
		static inline double Dot(Quaternion q1, Quaternion q2) {
			return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
		}
		static inline double Angle(Quaternion q1, Quaternion q2 = Quaternion::Identity) {
			return 2 * acos(Dot(q1, q2));
		}

		static inline Quaternion AngleAxis(double Angle, Vector3 Axis, bool NormalizeAxis = true) {
			Vector3 NewAxis = NormalizeAxis ? Axis.Normalized() : Axis;

			return Quaternion(cos(Angle / 2), NewAxis.x * sin(Angle / 2), NewAxis.y * sin(Angle / 2), NewAxis.z * sin(Angle / 2));
		}
		static inline Quaternion FromEulerAngles(Vector3 eulerAngles) {
			double cr = cos(eulerAngles.x * 0.5); double sr = sin(eulerAngles.x * 0.5); 
			double cp = cos(eulerAngles.y * 0.5); double sp = sin(eulerAngles.y * 0.5); 
			double cy = cos(eulerAngles.z * 0.5); double sy = sin(eulerAngles.z * 0.5);
			return Quaternion(
				cr * cp * cy + sr * sp * sy, 
				sr * cp * cy - cr * sp * sy, 
				cr * sp * cy + sr * cp * sy, 
				cr * cp * sy - sr * sp * cy);
		}
		static inline Quaternion FromEulerAngles(double x, double y, double z) {
			return FromEulerAngles(Vector3(x, y, z));
		}

		static inline Quaternion Rotate(Quaternion q1, Quaternion q2, bool CorrectMagnitude = true) {
			return CorrectMagnitude ? (q1 * q2).Normalized() : q1 * q2;
		}
		static inline Vector3 Rotate(Vector3 Vector, Quaternion Quaternion, bool CorrectMagnitude = true) {
			Vector3 output = Quaternion * Vector;
			if (CorrectMagnitude)
				return output.Normalized() * Vector.Magnitude();
			return output;
		}
		static inline Vector3 RotateNormalizedVector(Vector3 Vector, Quaternion Quaternion) {
			return (Quaternion * Vector).Normalized();
		}

		static inline Quaternion FromTo(Vector3 a, Vector3 b) {
			double dot = Vector3::Dot(a, b);
			if (abs(dot) - 1 <= Epsilon) 
				return Quaternion::Identity;

			Vector3 v = Vector3::Cross(a, b);
			double r = sqrt(a.SqrMagnitude() * b.SqrMagnitude()) + dot;
			return Quaternion(r, v).Normalized();
		}

	public:
		inline Quaternion operator*(Quaternion q) {
			Vector3 v1 = Vector();
			Vector3 v2 = q.Vector();
			Vector3 Cross = Vector3::Cross(v1, v2);
			return Quaternion(
				w * q.w - Vector3::Dot(v1, v2),
				w * q.x + q.w * x + Cross.x,
				w * q.y + q.w * y + Cross.y,
				w * q.z + q.w * z + Cross.z);
		}
		inline Vector3 operator*(Vector3 v) {
			double qSqrMag = Vector().SqrMagnitude();
			double wSqr = w * w;
			Vector3 qVec = Vector();
			Vector3 cross = Vector3::Cross(qVec, v);
			double dot = Vector3::Dot(qVec, v);

			Vector3 output = Vector3(
				qVec.x * 2.0 * dot + v.x * (wSqr - qSqrMag) + cross.x * 2.0 * w,
				qVec.y * 2.0 * dot + v.y * (wSqr - qSqrMag) + cross.y * 2.0 * w,
				qVec.z * 2.0 * dot + v.z * (wSqr - qSqrMag) + cross.z * 2.0 * w
			);

			return output;
		}
		inline Quaternion operator/(double q) {
			return Quaternion(w / q, x / q, y / q, z / q);
		}

	public:
		double w = 1.0, x = 0.0, y = 0.0, z = 0.0;

	public:
		const static Quaternion Identity;

	};
	const Quaternion Quaternion::Identity = Quaternion(1.0, 0.0, 0.0, 0.0);
}
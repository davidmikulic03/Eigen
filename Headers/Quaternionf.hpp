#pragma once
#include "Eigen.hpp"

namespace Eigen {
	class Quaternionf {
	public:
		Quaternionf() {
			w = 0.0f;
			x = 0.0f;
			y = 0.0f;
			z = 0.0f;
		}
		Quaternionf(float w, float x, float y, float z) {
			this->w = w;
			this->x = x;
			this->y = y;
			this->z = z;
		}
		Quaternionf(float r, Vector3f v) {
			w = r;
			x = v.x;
			y = v.y;
			z = v.z;
		}

	public:
		inline Vector3f Vector() {
			return Vector3f(x, y, z);
		}

	public:
		inline float SqrMagnitude() {
			return w * w + Vector().SqrMagnitude();
		}
		inline Quaternionf Inverse() {
			Vector3f v = Vector();
			float SqrMag = SqrMagnitude();
			if (SqrMag == 1.0f)
				return Conjugate();
			return Quaternionf(w / SqrMag, -v / SqrMag);
		}
		inline Quaternionf Conjugate() {
			return Quaternionf(w, -Vector());
		}
		
	public:
		static inline float Dot(Quaternionf q1, Quaternionf q2) {
			return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
		}
		static inline float Angle(Quaternionf q1, Quaternionf q2 = Quaternionf::Identity) {
			return 2 * acos(Dot(q1, q2));
		}

		static inline Quaternionf AngleAxis(float Angle, Vector3f Axis, bool NormalizeAxis = true) {
			Vector3f NewAxis = NormalizeAxis ? Axis.Normalized() : Axis;

			return Quaternionf(cos(Angle / 2), NewAxis.x * sin(Angle / 2), NewAxis.y * sin(Angle / 2), NewAxis.z * sin(Angle / 2));
		}

		static inline Vector3f Rotate(Vector3f Vector, Quaternionf Quaternion, bool CorrectMagnitude = true) {
			Vector3f output = Quaternion * Vector;
			if (CorrectMagnitude)
				return output.Normalized() * Vector.Magnitude();
			return output;
		}
		static inline Vector3f RotateUnitVector(Vector3f Vector, Quaternionf Quaternion) {
			return (Quaternion * Vector).Normalized();
		}

	public:
		inline Quaternionf operator*(Quaternionf q) {
			Vector3f v1 = Vector();
			Vector3f v2 = q.Vector();
			Vector3f Cross = Vector3f::Cross(v1, v2);
			return Quaternionf(
				w * q.w - Vector3f::Dot(v1, v2),
				w * q.x + q.w * x + Cross.x,
				w * q.y + q.w * y + Cross.y,
				w * q.z + q.w * z + Cross.z);
		}
		inline Vector3f operator*(Vector3f v) {
			float qSqrMag = Vector().SqrMagnitude();
			float wSqr = w * w;
			Vector3f qVec = Vector();
			Vector3f cross = Vector3f::Cross(qVec, v);
			float dot = Vector3f::Dot(qVec, v);

			Vector3f output = Vector3f(
				qVec.x * 2.0f * dot + v.x * (wSqr - qSqrMag) + cross.x * 2.0f * w,
				qVec.y * 2.0f * dot + v.y * (wSqr - qSqrMag) + cross.y * 2.0f * w,
				qVec.z * 2.0f * dot + v.z * (wSqr - qSqrMag) + cross.z * 2.0f * w
			);

			return output;
		}

	public:
		float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;

	public:
		const static Quaternionf Identity;

	};

	const Quaternionf Quaternionf::Identity = Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
}
#pragma once
#include "Eigen.h"

namespace Eigen {
	namespace Physics {
		class Orbit {
		public:
			Orbit(double massOfParent) {
				m_ParentMass = massOfParent;
				semiMajorAxis = AU;
				eccentricity = 0;
				inclination = 0;
				longitudeOfTheAscendingNode = 0;
				argumentOfPeriapsis = 0;
				argumentOfPeriapsis = 0;
				trueAnomaly = 0;
			}
			Orbit(double massOfParent, double semiMajorAxis, double eccentricity, double inclination, double longitudeOfTheAscendingNode, double argumentOfPeriapsis, double trueAnomaly) {
				m_ParentMass = massOfParent;
				this->semiMajorAxis = semiMajorAxis;
				this->eccentricity = eccentricity;
				this->inclination = inclination;
				this->longitudeOfTheAscendingNode = longitudeOfTheAscendingNode;
				this->argumentOfPeriapsis = argumentOfPeriapsis;
				this->trueAnomaly = trueAnomaly;
			}
			Orbit(Orbit& orbit) {
				m_Parent = orbit.GetParent();
				m_ParentMass = orbit.GetParentMass();
				semiMajorAxis = orbit.semiMajorAxis;
				eccentricity = orbit.eccentricity;
				inclination = orbit.inclination;
				longitudeOfTheAscendingNode = orbit.longitudeOfTheAscendingNode;
				argumentOfPeriapsis = orbit.argumentOfPeriapsis;
				trueAnomaly = orbit.trueAnomaly;
			}

		public:
			inline double Period() { return 2 * PI * sqrt((semiMajorAxis * semiMajorAxis * semiMajorAxis) / (GravitationalConstant * m_ParentMass)); }
			inline double MeanAnomaly(double time) { 
				return time * 2 * PI / Period();
			}
			inline double ApproximateEccentricAnomaly(double time, unsigned int solverIterations) {
				double meanAnomaly = MeanAnomaly(time);
				double result = meanAnomaly;
				for (unsigned int i = 0; i < solverIterations; i++) {
					//result = meanAnomaly + eccentricity * sin(result);
					result = result - (result - eccentricity * sin(result) - meanAnomaly) / (1 - eccentricity * cos(result));
				}

				result -= floor(result / (2 * PI)) * 2 * PI;
				return result;
			}
			inline void EvaluateTrueAnomaly(double time, unsigned int solverIterations) {
				double eccentricAnomaly = ApproximateEccentricAnomaly(time, solverIterations);
				double beta = eccentricity / (1.0 + sqrt(1 - eccentricity * eccentricity));
				trueAnomaly = eccentricAnomaly + 2 * atan(beta * sin(eccentricAnomaly) / (1.0 - beta * cos(eccentricAnomaly)));
				trueAnomaly -= floor(trueAnomaly / (2 * PI)) * 2 * PI;
			}

		public:
			inline void Setparent(Orbit& orbit) { m_Parent = &orbit; }
			Orbit* GetParent() { return m_Parent; }

		public:
			inline Quaternion GetLocalRotation() {
				return Quaternion(
					cos(0.5 * inclination) * cos(0.5 * (longitudeOfTheAscendingNode + argumentOfPeriapsis)),
					sin(0.5 * inclination) * cos(0.5 * (longitudeOfTheAscendingNode - argumentOfPeriapsis)),
					sin(0.5 * inclination) * sin(0.5 * (longitudeOfTheAscendingNode - argumentOfPeriapsis)),
					cos(0.5 * inclination) * sin(0.5 * (longitudeOfTheAscendingNode + argumentOfPeriapsis))).Normalized();
			}
			inline Quaternion GetGlobalRotation() {
				if (m_Parent == nullptr)
					return GetLocalRotation();
				else
					return m_Parent->GetGlobalRotation() * GetLocalRotation();
			}
		public:
			static inline Orbit FromStateVectors(double massOfParent, Vector3 localPosition, Vector3 velocity) {
				double mu = GravitationalConstant * massOfParent;

				Vector3 radialVelocity = Vector3::Project(velocity, localPosition.Normalized());
				//double azimuthalSpeed = (velocity - radialVelocity).Magnitude();

				Vector3 angularMomentum = Vector3::Cross(localPosition, velocity);

				double inclination = acos(angularMomentum.z / angularMomentum.Magnitude());

				if (inclination != inclination) 
					inclination = 0;

				Vector3 nodeVector = Vector3::Cross(Vector3::Up, angularMomentum);
				double longitudeOfTheAscendingNode = acos(nodeVector.x / nodeVector.Magnitude());
				if (nodeVector.y < 0) longitudeOfTheAscendingNode = 2 * PI - longitudeOfTheAscendingNode;

				Vector3 eccentricityVector = Vector3::Cross(velocity, angularMomentum) / mu - localPosition.Normalized();
				double eccentricity = eccentricityVector.Magnitude();

				double argumentOfPeriapsis = acos(Vector3::Dot(eccentricityVector, nodeVector) / (eccentricityVector.Magnitude() * nodeVector.Magnitude()));
				if (eccentricityVector.z < 0) argumentOfPeriapsis = 2 * PI - argumentOfPeriapsis;

				double trueAnomaly = acos(Vector3::Dot(eccentricityVector, localPosition) / (eccentricityVector.Magnitude() * localPosition.Magnitude()));
				if (radialVelocity.Magnitude() < 0) trueAnomaly = 2 * PI - trueAnomaly;

				double semiMajorAxis = angularMomentum.SqrMagnitude() / (mu * (1 - eccentricity * eccentricity));

				if (longitudeOfTheAscendingNode != longitudeOfTheAscendingNode) 
					longitudeOfTheAscendingNode = 0;

				if (argumentOfPeriapsis != argumentOfPeriapsis)
					argumentOfPeriapsis = 0;

				return Orbit(massOfParent, semiMajorAxis, eccentricity, inclination, longitudeOfTheAscendingNode, argumentOfPeriapsis, trueAnomaly);
			}
			static inline void ToLocalStateVectors(Orbit orbit, Vector3& positionOut, Vector3& velocityOut) {
				Quaternion localRotation = orbit.GetLocalRotation();
				//Vector3 perifocalX = localRotation * Vector3::Right;
				//Vector3 perifocalY = localRotation * Vector3::Forward;
				//Vector3 perifocalZ = localRotation * Vector3::Up;

				double sinTrueAnomaly = sin(orbit.trueAnomaly);
				double cosTrueAnomaly = cos(orbit.trueAnomaly);

				positionOut = localRotation * 
					Vector3(cos(orbit.trueAnomaly), sinTrueAnomaly) * orbit.semiMajorAxis / (1 + orbit.eccentricity * cosTrueAnomaly);
				velocityOut = localRotation * 
					Vector3(-sinTrueAnomaly, orbit.eccentricity + cosTrueAnomaly) *
					sqrt((GravitationalConstant * orbit.GetParentMass()) / (orbit.semiMajorAxis * (1 - orbit.eccentricity * orbit.eccentricity)));
			}

		public:
			inline bool operator<(Orbit o) { return semiMajorAxis < o.semiMajorAxis; }
			inline bool operator>(Orbit o) { return semiMajorAxis > o.semiMajorAxis; }
			inline bool operator<=(Orbit o) { return semiMajorAxis <= o.semiMajorAxis; }
			inline bool operator>=(Orbit o) { return semiMajorAxis >= o.semiMajorAxis; }
			//inline void operator=(Orbit o) { *this = Orbit(o); }

		public:
			double GetParentMass() { return m_ParentMass; }
			

		public:
			double semiMajorAxis, eccentricity, inclination, longitudeOfTheAscendingNode, argumentOfPeriapsis, trueAnomaly;
		private:
			double m_ParentMass = 0;
			Orbit* m_Parent = nullptr;
		};
	}
}
#include "Eigen.h"
#include <iostream>
#include <exception>
#include <chrono>

int main([[maybe_unused]] int argc, [[maybe_unused]] char* argv) {
	using namespace Eigen;

	Vector3 pos = Vector3(500, 0, 0);
	Vector3 vel = Vector3(0.0, 2, 1);

	Physics::Orbit orbit = Physics::Orbit::FromStateVectors(3e13, pos, vel);
	double dt = 10;
	double time = 0;
	for (unsigned int i = 0; i < 1000; i++) {
		orbit.EvaluateTrueAnomaly(time, 10);
		Physics::Orbit::ToLocalStateVectors(orbit, pos, vel);
		std::cout << pos.Magnitude() << std::endl;
		time += dt;
	}

	std::cout << "Periapsis: " << orbit.Periapsis() << ", Apoapsis: " << orbit.Apoapsis() << ", Eccentricity: " << orbit.eccentricity << ", Orbital Period: " << orbit.Period() << std::endl;

	return 0;
}
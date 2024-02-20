#include "Eigen.h"
#include <iostream>
#include <exception>
#include <chrono>

int main([[maybe_unused]] int argc, [[maybe_unused]] char* argv) {
	using namespace Eigen;

	Vector3 pos = Vector3(10000, 0, 0);
	Vector3 vel = Vector3(5.0, 1000.0, -500.0);

	Physics::Orbit orbit = Physics::Orbit::FromStateVectors(Physics::EarthMass * 0.0001, pos, vel);
	double dt = .1;
	double time = 0;
	for (unsigned int i = 0; i < 1000; i++) {
		orbit.EvaluateTrueAnomaly(time, 10);
		Physics::Orbit::ToLocalStateVectors(orbit, pos, vel);
		std::cout << pos.Magnitude() << std::endl;
		time += dt;
	}

	return 0;
}
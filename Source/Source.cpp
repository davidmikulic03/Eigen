#include "Eigen.hpp"
#include <iostream>
#include <exception>
#include <cmath>
#include <chrono>

int main([[maybe_unused]] int argc, [[maybe_unused]] char* argv) {
	using namespace Eigen;

	std::chrono::high_resolution_clock Clock;


	Vector3f a = Vector3f(2.0f, 7.0f, 1.0f);
	Vector3f b = Vector3f::Forward;
	Quaternionf q1 = Quaternionf::AngleAxis(1.0f * DegToRad, Vector3f(0.0f, 0.0f , 1.0f));
	Vector3f c = a;
	auto Start = Clock.now();

	unsigned int Iterations = 360;

	for (unsigned int i = 0; i < Iterations; i++) {
		
		//c = Quaternionf::Rotate(c, q1);
		c = q1 * c;
	}

	auto End = Clock.now();
	auto Results = std::chrono::duration_cast<std::chrono::microseconds>(End - Start);

	std::cout << "Vector A:\t(" << a.x << ", " << a.y << ", " << a.z << ")" << "\tof magnitude " << a.Magnitude() << std::endl;
	std::cout << "Vector q*A =\t(" << c.x << ", " << c.y << ", " << c.z << ")" << "\tof magnitude " << c.Magnitude() << std::endl;
	//std::cout << "Vector B:\t(" << b.x << ", " << b.y << ", " << b.z << ")" << "\tof magnitude " << b.Magnitude() << std::endl;
	std::cout << "Angle =\t\t" << (Quaternionf::Angle(q1) * RadToDeg) << std::endl;
	std::cout << "\n" << std::endl;

	
	std::cout << "Computed in " << Results << std::endl;

	return 0;
}
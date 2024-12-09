#include <iostream>
#include <exception>
#include <chrono>
#include <exception>

int main([[maybe_unused]] int argc, [[maybe_unused]] char* argv) {
	try {
		

	}
	catch (std::exception e) {
		std::cerr << e.what() << std::endl;
		return -1;
	}

	return 0;
}
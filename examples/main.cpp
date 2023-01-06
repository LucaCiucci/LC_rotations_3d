
#include <iostream>

#include <LC/geometry/rotations/Rotation3D.hpp>

int main() {
	std::cout << "Hello, World!" << std::endl;

	lc::Rotation3d rot;

	rot = lc::Rotation3d::from_euler(std::numbers::pi, 0.0, 0.0);
	std::cout << rot << std::endl;

	return 0;
}
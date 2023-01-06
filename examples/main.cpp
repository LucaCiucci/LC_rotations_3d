
#include <iostream>

#include <LC/geometry/rotations/Rotation3D.hpp>

using namespace lc;

int main() {
	std::cout << "Hello, World!" << std::endl;

	Rotation3d rot;

	rot = Rotation3d::from_euler(std::numbers::pi / 2, 0.0, 0.0);
	std::cout << rot << std::endl;
	std::cout << rot.to_quaternion() << std::endl;

	const Vector3d vec(1.0, 0.0, 0.0);

	std::cout << rot.rotate(vec) << std::endl;
	std::cout << rot.to_Mat3x3() << std::endl;


	return 0;
}
#pragma once

#include <corecrt_math_defines.h>

template <typename T>
bool CheckVectorHasValue(std::vector<T> vectorSeraching, T searchFor) {
	for (T tempVec : vectorSeraching) {
		if (tempVec == searchFor)
			return true;
	}
	return false;
}

inline Quaternion* toEuler(float x, float y, float z, float angle) {
	float s = std::sin(angle);
	float c = std::cos(angle);
	float t = 1 - c;
	float eulerY;
	float eulerX;
	float eulerZ;
	if ((x * y * t + z * s) > 0.998) { // north pole singularity detected
		eulerY = 2 * atan2(x * std::sin(angle / 2), std::cos(angle / 2));
		eulerX = M_PI / 2;
		eulerZ = 0;
		return nullptr;
	}
	if ((x * y * t + z * s) < -0.998) { // south pole singularity detected
		eulerY = -2 * atan2(x * std::sin(angle / 2), std::cos(angle / 2));
		eulerX = M_PI / 2;
		eulerZ = 0;
		return nullptr;
	}
	eulerY = std::atan2(y * s - x * z * t, 1 - (y * y + z * z) * t);
	std::cout << eulerY << '\n';
	eulerX = std::asin(x * y * t + z * s);
	std::cout << eulerX << '\n';
	eulerZ = std::atan2(x * s - y * z * t, 1 - (x * x + z * z) * t);
	std::cout << eulerZ << '\n' << '\n';

	Quaternion quaternionRotation = Quaternion::EulerAnglesToQuaternion(eulerX, eulerY, eulerZ);

	return &quaternionRotation;
}
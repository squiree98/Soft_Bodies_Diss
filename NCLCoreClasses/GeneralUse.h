#pragma once

template <typename T>
bool CheckVectorHasValue(std::vector<T> vectorSeraching, T searchFor) {
	for (T tempVec : vectorSeraching) {
		if (tempVec == searchFor)
			return true;
	}
	return false;
}
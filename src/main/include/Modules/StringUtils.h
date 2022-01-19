#pragma once

#include <string>

// ==========================================================================

class StringUtils {
public:
	static std::string LeftTrim(std::string s);
	static std::string RightTrim(std::string s);
	static std::string Trim(std::string s);
};

// ==========================================================================

template <typename T> std::string Join(const T &v, const std::string &delim) {
	std::string s;
	for (const auto &i : v) {
		if (&i != &v[0]) {
			s += delim;
		}
		s += i;
	}
	return s;
}

// ==========================================================================

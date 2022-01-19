#include "Modules/StringUtils.h"
#include <algorithm>
#include <cctype>
#include <functional>

// ==========================================================================

std::string StringUtils::LeftTrim(std::string s) {
	s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
	return s;
}

// ==========================================================================

std::string StringUtils::RightTrim(std::string s) {
	s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(),	s.end());
	return s;
}

// ==========================================================================

std::string StringUtils::Trim(std::string s) {
	return LeftTrim(RightTrim(s));
}

// ==========================================================================

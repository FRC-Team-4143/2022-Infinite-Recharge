#include "Modules/NumberListParser.h"

// ==========================================================================

NumberListParser::NumberListParser()
: _rxNumberList(), _haveNumberListRx(false) {
}

// ==========================================================================

NumberListParser::~NumberListParser() {
}

// ==========================================================================

bool NumberListParser::IsValid(std::string s) {
	return std::regex_match(s, _GetNumberListRx());
}

// ==========================================================================

std::vector<double> NumberListParser::Parse(std::string s) {
	std::vector<double> result;

	auto rx = _GetNumberListRx();
	std::smatch m;
	while (std::regex_match(s.cbegin(), s.cend(), m, rx)) {
		auto parameter = m.str(1);
		auto remainder = m.str(2);
		result.push_back(std::stof(parameter));
		s = remainder;
	}
	return result;
}

// ==========================================================================

std::regex NumberListParser::_GetNumberListRx() {
	if (!_haveNumberListRx) {
		std::string s;
		s += R"(\s*)"; // optional whitespace
		s += "(";      // open capture group
		s += "[-+]?";  // optional sign
		s += "(?:";
		s += R"(\d+(?:\.\d*)?)"; // accept formats: ##, ##., and ##.##
		s += "|";
		s += R"(\.\d+)"; // accept formats: .##
		s += ")";
		s += ")";      // close capture group
		s += R"(\s*)"; // optional whitespace

		s += "(?:";     // open group
		s += R"(,\s*)"; // comma, optional whitespace

		s += "("; // open capture group

		s += "[-+]?"; // optional sign
		s += "(?:";
		s += R"(\d+(?:\.\d*)?)"; // accept formats: ##, ##., and ##.##
		s += "|";
		s += R"(\.\d+)"; // accept formats: .##
		s += ")";
		s += R"(\s*)"; // optional whitespace

		s += "(?:";     // open group
		s += R"(,\s*)"; // comma, optional whitespace

		s += "[-+]?"; // optional sign
		s += "(?:";
		s += R"(\d+(?:\.\d*)?)"; // accept formats: ##, ##., and ##.##
		s += "|";
		s += R"(\.\d+)"; // accept formats: .##
		s += ")";
		s += R"(\s*)"; // optional whitespace

		s += ")*"; // close group, zero or more instances
		s += ")";  // close capture group
		s += ")?"; // close group, zero or one instance
		_rxNumberList.assign(s);
		_haveNumberListRx = true;
	}
	return _rxNumberList;
}

// ==========================================================================

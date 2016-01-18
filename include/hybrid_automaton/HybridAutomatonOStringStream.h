#ifndef HYBRID_AUTOMATON_O_STRING_STREAM_H_
#define HYBRID_AUTOMATON_O_STRING_STREAM_H_

// FIXME remove
#include <iostream>
#include <Eigen/Dense>

namespace ha {

/**
* @brief This is a helper class to override the << operator for some data types.
* i.e. we use it here to have a compactly formatted output for Eigen:: types without newlines
*/
class ha_ostringstream
	{
	public:
		ha_ostringstream()
		{
			// set "classic" locale (floating point instead of floating comma: http://stackoverflow.com/questions/571359/how-do-i-set-the-proper-initial-locale-for-a-c-program-on-windows)
			this->_oss.imbue(std::locale("C"));
		}
		
		ha_ostringstream(const std::string& val)
			: _oss(val)
		{
			// set "classic" locale (floating point instead of floating comma: http://stackoverflow.com/questions/571359/how-do-i-set-the-proper-initial-locale-for-a-c-program-on-windows)
			this->_oss.imbue(std::locale("C"));
		}

		~ha_ostringstream()
		{
		}

		template <typename T>
		ha_ostringstream& operator<<(const T& pX)
		{
			this->_oss << pX;

			return *this;
		}

		template <typename T>
		ha_ostringstream& operator>>(T& pX)
		{
			this->_oss >> pX;

			return *this;
		}

		std::string str()
		{
			return this->_oss.str();
		}

	public:
		std::stringstream _oss;    
	};

	template <>
	ha_ostringstream& ha_ostringstream::operator<<(const ::Eigen::MatrixXd& pX);

	template <>
    ha_ostringstream& ha_ostringstream::operator>>(Eigen::MatrixXd& matrix);
}

#endif

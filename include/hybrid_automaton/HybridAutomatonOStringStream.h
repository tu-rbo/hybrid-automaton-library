#ifndef HYBRID_AUTOMATON_O_STRING_STREAM_H_
#define HYBRID_AUTOMATON_O_STRING_STREAM_H_

// FIXME remove
#include <iostream>
#include <Eigen/Dense>

namespace ha {

class ha_ostringstream
	{
	public:
		ha_ostringstream()
		{
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

		std::string str()
		{
			return this->_oss.str();
		}

	private:
		std::ostringstream _oss;    
	};

	template <>
	ha_ostringstream& ha_ostringstream::operator<<(const ::Eigen::MatrixXd& pX);
}

#endif
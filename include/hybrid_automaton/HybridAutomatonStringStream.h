/*
 * Copyright 2015-2017, Robotics and Biology Lab, TU Berlin
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in the 
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef HYBRID_AUTOMATON_STRING_STREAM_H_
#define HYBRID_AUTOMATON_STRING_STREAM_H_

// FIXME remove
#include <iostream>
#include <Eigen/Dense>

namespace ha {

/**
* @brief This is a helper class to override the << operator for some data types.
* i.e. we use it here to have a compactly formatted output for Eigen:: types without newlines
*/
class ha_stringstream
	{
	public:
		ha_stringstream()
		{
			// set "classic" locale (floating point instead of floating comma: http://stackoverflow.com/questions/571359/how-do-i-set-the-proper-initial-locale-for-a-c-program-on-windows)
			this->_ss.imbue(std::locale("C"));
		}
		
		ha_stringstream(const std::string& val)
			: _ss(val)
		{
			// set "classic" locale (floating point instead of floating comma: http://stackoverflow.com/questions/571359/how-do-i-set-the-proper-initial-locale-for-a-c-program-on-windows)
			this->_ss.imbue(std::locale("C"));
		}

		~ha_stringstream()
		{
		}

		template <typename T>
		ha_stringstream& operator<<(const T& pX)
		{
			this->_ss << pX;

			return *this;
		}

		template <typename T>
		ha_stringstream& operator>>(T& pX)
		{
			this->_ss >> pX;

			return *this;
		}

		std::string str()
		{
			return this->_ss.str();
		}

	private:
		std::stringstream _ss;    
	};

	template <>
	ha_stringstream& ha_stringstream::operator<<(const ::Eigen::MatrixXd& pX);

	template <>
    ha_stringstream& ha_stringstream::operator>>(Eigen::MatrixXd& matrix);
}

#endif

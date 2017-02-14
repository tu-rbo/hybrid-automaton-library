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
#include "hybrid_automaton/Sensor.h"

namespace ha{

	Sensor::Sensor()
	{
	}

	Sensor::~Sensor()
	{
	}

	/**
	* Copy constructor
	*/
	Sensor::Sensor(const Sensor& ss):
		_initial_sensor_value(ss._initial_sensor_value)
	{
	}

	const std::string Sensor::getType() const
	{
		return this->_type;
	}

	void Sensor::setType(const std::string& new_type)
	{
		this->_type = new_type;
	}

	void Sensor::setSystem(const System::ConstPtr& system)
	{
		this->_system = system;
	}

	void Sensor::initialize(const double& t) 
	{
		this->_initial_sensor_value = this->getCurrentValue();
	}

	void Sensor::terminate() 
	{
	}

	void Sensor::step(const double& t) 
	{
	}

	::Eigen::MatrixXd Sensor::getInitialValue() const
	{
		return this->_initial_sensor_value;
	}

	::Eigen::MatrixXd Sensor::getRelativeCurrentValue() const
	{
		return (this->getCurrentValue() - this->_initial_sensor_value);
	}

}
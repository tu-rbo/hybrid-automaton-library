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
#include "hybrid_automaton/HybridAutomatonStringStream.h"


namespace ha{
	template <>
	ha_stringstream& ha_stringstream::operator<<(const ::Eigen::MatrixXd& pX)
	{
		this->_ss << "[" << pX.rows() << "," << pX.cols() << "]" ;
		for(int rows_it = 0; rows_it < pX.rows() ; ++rows_it)
		{
			for(int cols_it = 0; cols_it < pX.cols() ; ++cols_it)
			{
				this->_ss << pX(rows_it, cols_it);
				if(cols_it != pX.cols()-1)
				{
					this->_ss << ",";
				}
			}
			if(rows_it != pX.rows()-1)
			{
				this->_ss << ";";
			}
		}

		return *this;
	}

	template <>
	ha_stringstream& ha_stringstream::operator>>(Eigen::MatrixXd& matrix)
	{
		int num_rows = 0;
		int num_cols = 0;
		std::string deparsing_string;

		// First we read the char '['
		std::getline(this->_ss, deparsing_string, '[');

		// Then we read the number of rows
		std::getline(this->_ss, deparsing_string, ',');
		ha_stringstream iss_num_rows(deparsing_string);
		iss_num_rows >> num_rows;
		//std::cout << "Num of rows: " << num_rows << std::endl;

		// Then we read the number of cols
		std::getline(this->_ss, deparsing_string, ']');
		ha_stringstream iss_num_cols(deparsing_string);
		iss_num_cols >> num_cols;
		//std::cout << "Num of cols: " << num_cols << std::endl;

		// Resize the output matrix
		matrix.resize(num_rows,num_cols);

		// Read matrix values from the string
		double matrix_element = -1.0;
		for(int i = 0; i<num_rows; ++i)
		{
			std::getline(this->_ss, deparsing_string, ';');
			ha_stringstream iss_row(deparsing_string);
			for(int j=0; j<num_cols; ++j)
			{
				std::getline(iss_row._ss, deparsing_string, ',');
				ha_stringstream iss_element(deparsing_string);
				iss_element >> matrix_element;
				matrix(i,j) = matrix_element;
			}
		}

		return *this;
	}
}

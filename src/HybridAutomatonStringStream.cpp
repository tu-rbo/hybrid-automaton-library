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

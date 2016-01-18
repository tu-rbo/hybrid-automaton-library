#include "hybrid_automaton/HybridAutomatonOStringStream.h"


namespace ha{
	template <>
	ha_ostringstream& ha_ostringstream::operator<<(const ::Eigen::MatrixXd& pX)
	{
		this->_oss << "[" << pX.rows() << "," << pX.cols() << "]" ;
		for(int rows_it = 0; rows_it < pX.rows() ; ++rows_it)
		{
			for(int cols_it = 0; cols_it < pX.cols() ; ++cols_it)
			{
				this->_oss << pX(rows_it, cols_it);
				if(cols_it != pX.cols()-1)
				{
					this->_oss << ",";
				}
			}
			if(rows_it != pX.rows()-1)
			{
				this->_oss << ";";
			}
		}

		return *this;
	}

	template <>
	ha_ostringstream& ha_ostringstream::operator>>(Eigen::MatrixXd& matrix)
	{
		int num_rows = 0;
		int num_cols = 0;
		std::string deparsing_string;

		// First we read the char '['
		std::getline(this->_oss, deparsing_string, '[');

		// Then we read the number of rows
		std::getline(this->_oss, deparsing_string, ',');
		ha_ostringstream iss_num_rows(deparsing_string);
		iss_num_rows >> num_rows;
		//std::cout << "Num of rows: " << num_rows << std::endl;

		// Then we read the number of cols
		std::getline(this->_oss, deparsing_string, ']');
		ha_ostringstream iss_num_cols(deparsing_string);
		iss_num_cols >> num_cols;
		//std::cout << "Num of cols: " << num_cols << std::endl;

		// Resize the output matrix
		matrix.resize(num_rows,num_cols);

		// Read matrix values from the string
		double matrix_element = -1.0;
		for(int i = 0; i<num_rows; ++i)
		{
			std::getline(this->_oss, deparsing_string, ';');
			ha_ostringstream iss_row(deparsing_string);
			for(int j=0; j<num_cols; ++j)
			{
				std::getline(iss_row._oss, deparsing_string, ',');
				ha_ostringstream iss_element(deparsing_string);
				iss_element >> matrix_element;
				matrix(i,j) = matrix_element;
			}
		}

		return *this;
	}
}

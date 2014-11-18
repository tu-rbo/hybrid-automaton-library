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

		//std::cout << "special call" << std::endl;

		return *this;
	}
}
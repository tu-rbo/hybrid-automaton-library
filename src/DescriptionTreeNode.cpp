#include "hybrid_automaton/DescriptionTreeNode.h"

#include <algorithm>

namespace ha {

	std::istringstream& operator>>(std::istringstream& iss, Eigen::MatrixXd& matrix)
		{
			int num_rows = 0;
			int num_cols = 0;
			std::string deparsing_string;

			// First we read the char '['
			getline(iss,deparsing_string,'[');

			// Then we read the number of rows
			getline(iss,deparsing_string,',');
			std::istringstream iss_num_rows(deparsing_string);
			iss_num_rows >> num_rows;
			//std::cout << "Num of rows: " << num_rows << std::endl;

			// Then we read the number of cols
			getline(iss,deparsing_string,']');
			std::istringstream iss_num_cols(deparsing_string);
			iss_num_cols >> num_cols;
			//std::cout << "Num of cols: " << num_cols << std::endl;

			// Resize the output matrix
			matrix.resize(num_rows,num_cols);

			// Read matrix values from the string
			double matrix_element = -1.0;
			for(int i = 0; i<num_rows; ++i)
			{
				getline(iss,deparsing_string,';');
				std::istringstream iss_row(deparsing_string);
				for(int j=0; j<num_cols; ++j)
				{
					getline(iss_row, deparsing_string, ',');
					std::istringstream iss_element(deparsing_string);
					iss_element >> matrix_element;
					matrix(i,j) = matrix_element;
				}
			}

			return iss;
		};

	DescriptionTreeNode::DescriptionTreeNode()
	{

	}

	DescriptionTreeNode::~DescriptionTreeNode()
	{

	}

	DescriptionTreeNode::DescriptionTreeNode(const DescriptionTreeNode& dtn)
	{

	}
}

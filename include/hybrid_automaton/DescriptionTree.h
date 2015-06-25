#ifndef HYBRID_AUTOMATON_DESCRIPTION_TREE_H_
#define HYBRID_AUTOMATON_DESCRIPTION_TREE_H_

#include <string>
#include <assert.h>

#include "hybrid_automaton/DescriptionTreeNode.h"

namespace ha {

/**
 * @brief A DescriptionTree is an interface for a tree-based description of a HybridAutomaton.
 * This inteface could be implemented by xml, yaml or similar languages.
 */
	class DescriptionTree {

	protected:
	
	private:  

	public:		
		typedef boost::shared_ptr<DescriptionTree> Ptr;
		typedef boost::shared_ptr<const DescriptionTree> ConstPtr;

        /**
         * @brief Return the first tree element
         */
		virtual DescriptionTreeNode::Ptr getRootNode() = 0;

        /**
         * @brief Set the first tree element
         */
		virtual void setRootNode(const DescriptionTreeNode::Ptr& root_node) = 0;

        /**
         * @brief Factory function to create a tree node
         */
		virtual DescriptionTreeNode::Ptr createNode(const std::string& type) const  = 0;
	};

	typedef boost::shared_ptr<DescriptionTree> DescriptionTreePtr;
	typedef boost::shared_ptr<const DescriptionTree> DescriptionTreeConstPtr;
}

#endif

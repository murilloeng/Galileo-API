#pragma once

//std
#include <cstdio>

namespace fea
{
	namespace mesh
	{
		namespace nodes
		{
			class Node;
			enum class dof : unsigned;
		}
	}
	namespace analysis
	{
		namespace solvers
		{
			class Solver;
		}
	}
}

namespace fea
{
	namespace analysis
	{
		namespace solvers
		{
			class Watch_Dof
			{
				friend class Solver;

			protected:
				//constructors
				Watch_Dof(Solver*);

				//destructor
				virtual ~Watch_Dof(void);

				//serialization
				void load(FILE*);
				void save(FILE*) const;

			public:
				//data
				mesh::nodes::Node* node(unsigned);
				mesh::nodes::Node* node(void) const;

				mesh::nodes::dof dof(void) const;
				mesh::nodes::dof dof(mesh::nodes::dof);

				//index
				unsigned index_dof(void) const;
				unsigned index_node(void) const;

				//analysis
				void prepare(void);

			protected:
				//data
				unsigned m_node;
				unsigned m_dof_index;

				Solver* m_solver;
				mesh::nodes::dof m_dof;
			};
		}
	}
}
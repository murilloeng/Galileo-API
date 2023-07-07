#pragma once

//std
#include <vector>

namespace fea
{
	namespace mesh
	{
		namespace nodes
		{
			enum class dof : unsigned;
		}
	}
	namespace boundary
	{
		class Boundary;
		namespace loads
		{
			class Node;
			class Element;
			enum class type : unsigned;
		}
	}
	namespace analysis
	{
		class Assembler;
	}
}

namespace fea
{
	namespace boundary
	{
		namespace loads
		{
			class Load_Case
			{
				friend class boundary::Boundary;
				friend class analysis::Assembler;

			protected:
				//constructors
				Load_Case(void);

				//destructor
				virtual ~Load_Case(void);

				//serialization
				void load(FILE*);
				void save(FILE*) const;

			public:
				//data
				const char* label(void) const;
				const char* label(const char*);

				static Boundary* boundary(void);

				loads::Node* load_node(unsigned) const;
				loads::Element* load_element(unsigned) const;

				//lists
				const std::vector<loads::Node*>& loads_nodes(void) const;
				const std::vector<loads::Element*>& loads_elements(void) const;

				//add
				void add_load_node(unsigned, mesh::nodes::dof, double = 1, unsigned = UINT_MAX);
				void add_load_element(loads::type, std::vector<unsigned> = {}, double = 1, unsigned = UINT_MAX);

				//remove
				void remove_load_node(unsigned);
				void remove_load_element(unsigned);

				//index
				unsigned index(void) const;

			protected:
				//analysis
				bool check(void) const;
				void prepare(void);

				//draw
				void draw(void) const;

				//data
				char m_label[200];
				static Boundary* m_boundary;
				std::vector<loads::Node*> m_loads_nodes;
				std::vector<loads::Element*> m_loads_elements;
			};
		}
	}
}
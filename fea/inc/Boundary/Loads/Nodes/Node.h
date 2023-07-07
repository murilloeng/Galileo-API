#pragma once

//fea
#include "Boundary/Loads/Load.h"

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
	namespace boundary
	{
		class Boundary;
		namespace loads
		{
			class Load_Case;
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
			class Node : public Load
			{
				friend class Load_Case;
				friend class boundary::Boundary;
				friend class analysis::Assembler;

			protected:
				//constructors
				Node(void);

				//destructor
				virtual ~Node(void) override;

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
				//data
				mesh::nodes::dof dof(void) const;
				mesh::nodes::dof dof(mesh::nodes::dof);

				mesh::nodes::Node* node(void) const;
				mesh::nodes::Node* node(unsigned);

				//index
				unsigned index(void) const;
				unsigned index_node(void) const;

			protected:
				//analysis
				virtual void add_dof(void) const;
				virtual void prepare(void) override;
				virtual bool check(void) const override;

				//draw
				virtual void draw(void) const override;
				virtual void draw_rotation_x(void) const;
				virtual void draw_rotation_y(void) const;
				virtual void draw_rotation_z(void) const;
				virtual void draw_translation_x(void) const;
				virtual void draw_translation_y(void) const;
				virtual void draw_translation_z(void) const;

				//data
				unsigned m_node;
				unsigned m_dof_index;
				mesh::nodes::dof m_dof;
			};
		}
	}
}
#pragma once

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
		class Initial
		{
			friend class boundary::Boundary;
			friend class analysis::Assembler;

		protected:
			//constructors
			Initial(void);

			//destructor
			virtual ~Initial(void);

			//serialization
			void load(FILE*);
			void save(FILE*) const;

		public:
			//data
			static Boundary* boundary(void);

			mesh::nodes::dof dof(void) const;
			mesh::nodes::dof dof(mesh::nodes::dof);
			mesh::nodes::Node* node(void) const;
			mesh::nodes::Node* node(unsigned);

			double state(void) const;
			double state(double);
			double velocity(void) const;
			double velocity(double);

			//index
			unsigned index(void) const;
			unsigned index_node(void) const;

		protected:
			//analysis
			bool check(void) const;
			void prepare(void);
			void add_dof(void) const;

			//apply
			void apply(void) const;

			//data
			static Boundary* m_boundary;

			unsigned m_node;
			unsigned m_dof_index;
			mesh::nodes::dof m_dof;

			double m_state;
			double m_velocity;
		};
	}
}
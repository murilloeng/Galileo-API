#pragma once

//std
#include <list>
#include <vector>
#include <functional>

namespace fea
{
	namespace mesh
	{
		namespace nodes
		{
			enum class dof : unsigned;
		}
		namespace joints
		{
			class Joint;
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
		class Dependency
		{
			friend class boundary::Boundary;
			friend class analysis::Assembler;
			friend class mesh::joints::Joint;

		protected:
			//constructors
			Dependency(void);

			//destructor
			virtual ~Dependency(void);

			//serialization
			void load(FILE*);
			void save(FILE*) const;

		public:
			//data
			static Boundary* boundary(void);

			mesh::nodes::dof slave_dof(void) const;
			mesh::nodes::dof slave_dof(mesh::nodes::dof);

			mesh::nodes::dof master_dof(unsigned) const;
			mesh::nodes::dof master_dof(unsigned, mesh::nodes::dof);

			mesh::nodes::Node* slave_node(void) const;
			mesh::nodes::Node* slave_node(unsigned);

			mesh::nodes::Node* master_node(unsigned) const;
			mesh::nodes::Node* master_node(unsigned, unsigned);

			std::function<double(double*)> state(void) const;
			std::function<double(double*)> state(std::function<double(const double*)>);

			std::function<double(double*, unsigned)> gradient(void) const;
			std::function<double(double*, unsigned)> gradient(std::function<double(const double*, unsigned)>);

			std::function<double(double*, unsigned, unsigned)> hessian(void) const;
			std::function<double(double*, unsigned, unsigned)> hessian(std::function<double(const double*, unsigned, unsigned)>);

			//index
			unsigned index(void) const;
			unsigned index_slave_dof(void) const;
			unsigned index_slave_node(void) const;
			unsigned index_master_dof(unsigned) const;
			unsigned index_master_node(unsigned) const;

			//sizes
			unsigned masters(void) const;

			//add
			void add_master(unsigned, mesh::nodes::dof);

			//remove
			void remove_master(unsigned);

		protected:
			//state
			double* master_state(double*) const;
			double* master_velocity(double*) const;
			double* master_acceleration	(double*) const;

			//analysis
			bool check(void) const;
			void prepare(void);
			void add_dof(void) const;

			//assembler
			void apply(double*, double*, double*) const;

			//data
			static Boundary* m_boundary;

			unsigned m_slave_node;
			unsigned m_slave_dof_index;
			mesh::nodes::dof m_slave_dof;

			std::vector<unsigned> m_master_nodes;
			std::vector<unsigned> m_master_dofs_index;
			std::vector<mesh::nodes::dof> m_master_dofs;

			std::function<double(const double*)> m_state;
			std::function<double(const double*, unsigned)> m_gradient;
			std::function<double(const double*, unsigned, unsigned)> m_hessian;

			static std::list<unsigned> m_update_nodes;
		};
	}
}
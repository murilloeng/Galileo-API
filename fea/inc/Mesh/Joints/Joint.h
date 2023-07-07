#pragma once

//std
#include <vector>

namespace fea
{
	namespace mesh
	{
		class Mesh;
		namespace nodes
		{
			class Node;
			enum class dof : unsigned;
		}
		namespace joints
		{
			enum class type : unsigned;
			enum class state : unsigned;
		}
	}
	namespace analysis
	{
		class Assembler;
	}
	namespace boundary
	{
		class Boundary;
		class Dependency;
	}
}

namespace fea
{
	namespace mesh
	{
		namespace joints
		{
			class Joint
			{
				friend class mesh::Mesh;
				friend class boundary::Boundary;
				friend class analysis::Assembler;

			protected:
				//constructors
				Joint(void);

				//destructor
				virtual ~Joint(void);

				//serialization
				virtual void load(FILE*);
				virtual void save(FILE*) const;

				virtual bool load_state(void);
				virtual bool save_state(void) const;

				virtual bool load_energy(void);
				virtual bool save_energy(void) const;

				//create
				static void create(Joint*&, const Joint*);
				static void create(Joint*&, joints::type, const Joint* = nullptr);

			public:
				//data
				static Mesh* mesh(void);

				virtual nodes::Node* node(unsigned) const;
				virtual nodes::Node* node(unsigned, unsigned);

				//types
				virtual joints::type type(void) const = 0;
				virtual unsigned nodes_max(void) const = 0;
				virtual unsigned state_set(void) const = 0;
				virtual unsigned dof_set(unsigned) const = 0;

				//name
				virtual const char* name(void) const;
				static const char* name(joints::type);

				//state
				static const char* state_name(state);

				//lists
				const std::vector<unsigned>& nodes(void) const;

				//index
				virtual unsigned index(void) const;
				virtual unsigned index_node(unsigned) const;

				//add
				virtual void add_node(unsigned);

				//remove
				virtual void remove_node(unsigned);

			protected:
				//analysis
				virtual void add_dof(void) const;

				virtual void apply(void) = 0;

				virtual bool check(void) const;
				virtual void prepare(void);
				virtual void configure(void);

				virtual void update(void);
				virtual void restore(void);

				virtual void record(void);
				virtual void finish(void) const;

				virtual boundary::Dependency* dependency(unsigned, mesh::nodes::dof, unsigned, mesh::nodes::dof);
				virtual boundary::Dependency* dependency(unsigned, mesh::nodes::dof, std::vector<unsigned>, std::vector<mesh::nodes::dof>);

			public:
				//formulation
				virtual double kinetic_energy(void) const;
				virtual double internal_energy(void) const;

				virtual double* kinetic_force(double*) const;
				virtual double* inertial_force(double*) const;
				virtual double* internal_force(double*) const = 0;

				virtual double* inertia(double*) const = 0;
				virtual double* damping(double*) const = 0;
				virtual double* stiffness(double*) const = 0;

			protected:
				//draw
				virtual void draw(void) const = 0;
				virtual void draw_number(unsigned) const;

				//data
				double* m_state;
				double* m_energy;
				static Mesh* m_mesh;
				std::vector<unsigned> m_nodes;
				std::vector<unsigned> m_dof_index;
				std::vector<boundary::Dependency*> m_dependencies;
			};
		}
	}
}
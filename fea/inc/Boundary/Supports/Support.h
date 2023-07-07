#pragma once

//std
#include <list>

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
		namespace time
		{
			class Time;
		}
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
		class Support
		{
			friend class boundary::Boundary;
			friend class analysis::Assembler;

		protected:
			//constructor
			Support(void);

			//destructor
			virtual ~Support(void);

			//serialization
			virtual void load(FILE*);
			virtual void save(FILE*) const;

			virtual bool load_results(void);
			virtual bool save_results(void) const;

		public:
			//data
			bool fixed(bool);
			bool fixed(void) const;

			double inertia(double);
			double damping(double);
			double stiffness(double);

			double inertia(void) const;
			double damping(void) const;
			double stiffness(void) const;

			static Boundary* boundary(void);

			mesh::nodes::Node* node(unsigned);
			mesh::nodes::Node* node(void) const;

			time::Time* state(unsigned);
			time::Time* velocity(unsigned);
			time::Time* acceleration(unsigned);

			time::Time* state(void) const;
			time::Time* velocity(void) const;
			time::Time* acceleration(void) const;

			mesh::nodes::dof dof(void) const;
			mesh::nodes::dof dof(mesh::nodes::dof);

			//index
			unsigned index(void) const;
			unsigned index_node(void) const;
			unsigned index_state(void) const;
			unsigned index_velocity(void) const;
			unsigned index_acceleration(void) const;

			//data
			double state(double) const;
			double velocity(double) const;
			double acceleration(double) const;

		protected:
			//assembler
			void apply_values(void) const;

			//analysis
			void record(void);
			void prepare(void);
			bool check(void) const;
			void finish(void) const;
			void add_dof(void) const;

			//draw
			virtual void draw(void) const;
			virtual void draw_rotation_x(const double*, const double*) const;
			virtual void draw_rotation_y(const double*, const double*) const;
			virtual void draw_rotation_z(const double*, const double*) const;
			virtual void draw_temperature(const double*, const double*) const;
			virtual void draw_translation_x(const double*, const double*) const;
			virtual void draw_translation_y(const double*, const double*) const;
			virtual void draw_translation_z(const double*, const double*) const;

			//data
			bool m_fixed;
			unsigned m_node;
			unsigned m_dof_index;
			mesh::nodes::dof m_dof;

			double m_inertia;
			double m_damping;
			double m_stiffness;

			double* m_reaction;

			unsigned m_state;
			unsigned m_velocity;
			unsigned m_acceleration;

			static Boundary* m_boundary;
			static std::list<unsigned> m_update_nodes;
		};
	}
}
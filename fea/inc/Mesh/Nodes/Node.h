#pragma once

#include <string>
#include <vector>

namespace mat
{
	class vec3;
}
namespace fea
{
	namespace mesh
	{
		class Mesh;
		namespace nodes
		{
			enum class dof : unsigned;
		}
		namespace elements
		{
			class Element;
		}
		namespace joints
		{
			class Joint;
		}
	}
	namespace boundary
	{
		class Initial;
		class Support;
		class Constraint;
		class Dependency;
		namespace loads
		{
			class Node;
		}
	}
	namespace analysis
	{
		class Analysis;
		class Assembler;
		namespace solvers
		{
			class Solver;
		}
	}
}

namespace fea
{
	namespace mesh
	{
		namespace nodes
		{
			class Node
			{
				friend class mesh::Mesh;
				friend class mesh::joints::Joint;
				friend class mesh::elements::Element;

				friend class boundary::Initial;
				friend class boundary::Support;
				friend class boundary::Constraint;
				friend class boundary::Dependency;
				friend class boundary::loads::Node;

				friend class analysis::Analysis;
				friend class analysis::Assembler;
				friend class analysis::solvers::Solver;

			protected:
				//constructors
				Node(void);

				//destructor
				virtual ~Node(void);

				//serialization
				virtual void load(FILE*);
				virtual void save(FILE*) const;

				virtual bool load_results(void);
				virtual bool save_results(void) const;

			public:
				//data
				static Mesh* mesh(void);

				virtual double coordinate(unsigned) const;
				virtual const double* coordinates(void) const;
				virtual const double* coordinates(const double*);
				virtual const double* coordinates(double, unsigned);
				virtual const double* coordinates(double, double, double);

				//index
				virtual unsigned index(void) const;

				//kinematics
				virtual double* position(double*) const;
				virtual const double* quaternion(bool = true) const;
				virtual const double* rotation(bool = true, unsigned = 0) const;
				virtual const double* translation(bool = true, unsigned = 0) const;

				//transform
				virtual Node& move(const mat::vec3&, bool = false);
				virtual Node& scale(const mat::vec3&, double, bool = false);
				virtual Node& rotate(const mat::vec3&, const mat::vec3&, bool = false);
				virtual Node& mirror(const mat::vec3&, const mat::vec3&, bool = false);

				//state
				virtual double& state(dof);
				virtual double& velocity(dof);
				virtual double& acceleration(dof);

				virtual double state(dof, unsigned) const;
				virtual double velocity(dof, unsigned) const;
				virtual double acceleration(dof, unsigned) const;

				//dof
				static const char* dof_name(dof);
				static const char* load_name(dof);

				virtual unsigned dof_set(void) const;
				virtual unsigned dof_size(void) const;
				virtual unsigned dof_index(dof) const;
				virtual unsigned dof_index(unsigned) const;

			protected:
				//analysis
				virtual void add_dof(dof);
				virtual void add_dof(unsigned);

				virtual bool check(void) const;
				virtual void prepare(unsigned&);
				virtual void finish(void) const;

				virtual void record(void);
				virtual void update(void);
				virtual void restore(void);

				virtual void setup_memory(void);
				virtual void setup_rotation(void);

				virtual void update_rotation(void);

				//apply
				virtual void apply_state(void);
				virtual void apply_velocity(void);
				virtual void apply_acceleration(void);

				//increment
				virtual void increment_state(void);
				virtual void increment_velocity(void);
				virtual void increment_acceleration(void);

				//draw
				virtual void draw(unsigned) const;
				virtual void draw_number(unsigned) const;

				//data
				double* m_quat_old;
				double* m_quat_new;
				double* m_state_old;
				double* m_state_new;
				double* m_state_mem;
				double* m_velocity_old;
				double* m_velocity_new;
				double* m_velocity_mem;
				double* m_acceleration_old;
				double* m_acceleration_new;
				double* m_acceleration_mem;

				unsigned m_dof_set;
				double m_coordinates[3];
				std::vector<unsigned> m_dof_index;

				static Mesh* m_mesh;
			};
		}
	}
}
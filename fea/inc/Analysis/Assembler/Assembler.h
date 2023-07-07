#pragma once

//std
#include <vector>

namespace fea
{
	namespace mesh
	{
		class Mesh;
		namespace elements
		{
			class Element;
		}
	}
	namespace boundary
	{
		class Dependency;
	}
	namespace analysis
	{
		class Analysis;
		namespace solvers
		{
			class Solver;
		}
	}

	namespace analysis
	{
		class Assembler
		{
			friend class mesh::Mesh;
			friend class analysis::Analysis;
			friend class mesh::elements::Element;
			friend class analysis::solvers::Solver;

		protected:
			//constructors
			Assembler(void);

			//destructor
			virtual ~Assembler(void);

			//serialization
			virtual bool load_results(void);
			virtual bool save_results(void) const;

			//dof
			virtual void map_dof(void);
			virtual void add_dof(unsigned, unsigned);

			virtual void sort_dof(void);
			virtual void save_dof(void);
			virtual void apply_dof(void);
			virtual void count_dof(unsigned);
			virtual void count_dof(const std::vector<unsigned>&);

			//memory
			virtual void free(void);
			virtual void allocate(void);

		public:
			//analysis
			virtual void setup(void);
			virtual void prepare(void);
			virtual bool check(void) const;
			virtual void finish(void) const;

			virtual void record(void) const;
			virtual void update(void) const;
			virtual void restore(void) const;

			//dof
			virtual unsigned dof_know(void) const;
			virtual unsigned dof_local(void) const;
			virtual unsigned dof_total(void) const;
			virtual unsigned dof_unknow(void) const;
			virtual unsigned dof_triplet(void) const;
			virtual unsigned dof_nonzero(void) const;
			virtual unsigned dof_dependent(void) const;

			//map
			virtual const int* row_map(void) const;
			virtual const int* col_map(void) const;
			virtual const int* row_triplet(void) const;
			virtual const int* col_triplet(void) const;

			//assembly
			virtual void assembly_state(void) const;
			virtual void assembly_velocity(void) const;
			virtual void assembly_acceleration(void) const;

			virtual void assembly_kinetic_force(void) const;
			virtual void assembly_inertial_force(void) const;
			virtual void assembly_internal_force(void) const;

			virtual void assembly_kinetic_energy(void) const;
			virtual void assembly_internal_energy(void) const;

			virtual void assembly_dead_force(void) const;
			virtual void assembly_external_force(void) const;
			virtual void assembly_external_force(double) const;
			virtual void assembly_reference_force(void) const;

			virtual void assembly_inertia(void) const;
			virtual void assembly_damping(void) const;
			virtual void assembly_stiffness(void) const;

			virtual void assembly_state_increment(void) const;
			virtual void assembly_velocity_increment(void) const;
			virtual void assembly_acceleration_increment(void) const;

			//apply
			virtual void apply_state(void) const;
			virtual void apply_velocity(void) const;
			virtual void apply_acceleration(void) const;

			virtual void apply_initials(void) const;
			virtual void apply_supports(void) const;
			virtual void apply_dependencies(void) const;
			virtual void apply_configurations(void) const;

			//increment
			virtual void increment_state(void) const;
			virtual void increment_velocity(void) const;
			virtual void increment_acceleration(void) const;

			//rotation
			virtual void adjust_moments(double*, double*) const;
			virtual void adjust_moments(double*, const double*, const double*) const;

		protected:
			//assembly
			virtual void assembly_number(double, unsigned, double*, double*) const;
			virtual void assembly_number(double, unsigned, unsigned, double*) const;
			virtual void assembly_vector(const double*, const std::vector<unsigned>&, double*, double*, double = 1) const;
			virtual void assembly_matrix(const double*, const std::vector<unsigned>&, double*, double = 1) const;

			virtual void assembly_dependency(double, const boundary::Dependency*, double*) const;
			virtual void assembly_dependency(double, const boundary::Dependency*, double*, double*) const;
			virtual void assembly_dependency(double, unsigned, const boundary::Dependency*, double*) const;
			virtual void assembly_dependency(double, const boundary::Dependency*, unsigned, double*) const;
			virtual void assembly_dependency(double, const boundary::Dependency*, const boundary::Dependency*, double*) const;

			virtual void assembly_dead_force_nodes(double*, double*) const;
			virtual void assembly_dead_force_elements(double*, double*) const;

			virtual void assembly_kinetic_force_joints(double*, double*) const;
			virtual void assembly_kinetic_force_elements(double*, double*) const;

			virtual void assembly_inertial_force_joints(double*, double*) const;
			virtual void assembly_inertial_force_elements(double*, double*) const;
			virtual void assembly_inertial_force_supports(double*, double*) const;

			virtual void assembly_internal_force_joints(double*, double*) const;
			virtual void assembly_internal_force_elements(double*, double*) const;
			virtual void assembly_internal_force_supports(double*, double*) const;

			virtual void assembly_external_force_nodes(double*, double*, double) const;
			virtual void assembly_external_force_elements(double*, double*, double) const;

			virtual void assembly_reference_force_nodes(double*, double*) const;
			virtual void assembly_reference_force_elements(double*, double*) const;

			virtual void assembly_inertia_joints(double*) const;
			virtual void assembly_inertia_elements(double*) const;
			virtual void assembly_inertia_supports(double*) const;

			virtual void assembly_damping_elements(double*) const;
			virtual void assembly_damping_supports(double*) const;

			virtual void assembly_stiffness_loads(double*) const;
			virtual void assembly_stiffness_joints(double*) const;
			virtual void assembly_stiffness_elements(double*) const;
			virtual void assembly_stiffness_supports(double*) const;

			//data
			static Analysis* m_analysis;

			int* m_row_map;
			int* m_col_map;
			int* m_row_triplet;
			int* m_col_triplet;

			mutable double m_qe;
			mutable double* m_ue;
			mutable double* m_ve;
			mutable double* m_ae;
			mutable double* m_fe;
			mutable double* m_ke;
			mutable double* m_ce;
			mutable double* m_me;
			mutable unsigned* m_de;

			unsigned m_dof_know;
			unsigned m_dof_local;
			unsigned m_dof_total;
			unsigned m_dof_unknow;
			unsigned m_dof_triplet;
			unsigned m_dof_dependent;
		};
	}
}
#pragma once

//std
#include <cstdio>
#include <string>
#include <functional>

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
		class Initial;
		class Support;
	}
	namespace analysis
	{
		class Analysis;
		class Assembler;
		namespace solvers
		{
			class Watch_Dof;
			enum class type : unsigned;
		}
	}
	namespace models
	{
		class Model;
	}
}

namespace fea
{
	namespace analysis
	{
		namespace solvers
		{
			enum class state : unsigned
			{
				u = 1 << 0,
				v = 1 << 1,
				a = 1 << 2
			};
			enum class force : unsigned
			{
				r = 1 << 0,
				R = 1 << 1,
				Fn = 1 << 2,
				Fk = 1 << 3,
				Fi = 1 << 4,
				Fd = 1 << 5,
				Fr = 1 << 6,
				Fe = 1 << 7
			};
			enum class tangent : unsigned
			{
				e = 1 << 0,
				K = 1 << 1,
				C = 1 << 2,
				M = 1 << 3
			};

			class Solver
			{
				friend class boundary::Initial;
				friend class boundary::Support;
				friend class analysis::Analysis;
				friend class analysis::Assembler;

			protected:
				//constructors
				Solver(void);

				//destructor
				virtual ~Solver(void);

				//setup
				virtual void setup(void);

				//create
				static void create(Solver*&, solvers::type);

				//serialization
				virtual void load(FILE*);
				virtual void save(FILE*) const;

				virtual bool load_results(void);
				virtual bool save_results(void) const;

			public:
				//data
				virtual double dof_min(double);
				virtual double dof_min(void) const;

				virtual double dof_max(double);
				virtual double dof_max(void) const;

				virtual unsigned step_max(unsigned);
				virtual unsigned step_max(void) const;

				virtual unsigned load_set(unsigned);
				virtual unsigned load_set(void) const;

				virtual Watch_Dof* watch_dof(void) const;
				virtual Watch_Dof* watch_dof(unsigned, mesh::nodes::dof);

				virtual std::function<void(void)> adapt_increment(void) const;
				virtual std::function<void(void)> adapt_iteration(void) const;
				virtual std::function<void(void)> adapt_increment(std::function<void(void)>);
				virtual std::function<void(void)> adapt_iteration(std::function<void(void)>);

				virtual std::function<void(unsigned)> run_interface(void) const;
				virtual std::function<void(unsigned)> run_interface(std::function<void(unsigned)>);

				//type
				virtual solvers::type type(void) const = 0;

				//name
				virtual const char* name(void) const;
				static const char* name(solvers::type);
				virtual const char* parameter(void) const;

				//sets
				virtual unsigned state_set(void) const = 0;
				virtual unsigned force_set(void) const = 0;
				virtual unsigned tangent_set(void) const = 0;

				//step
				virtual void stop_run(void);
				virtual unsigned step(void) const;

				//dof
				virtual double dof(void) const;

				//data
				static Analysis* analysis(void);

				virtual double time(void) const;
				virtual double time_min(double);
				virtual double time_max(double);
				virtual double time_min(void) const;
				virtual double time_max(void) const;
				virtual double time_increment(void) const;
				virtual double time_increment_max(void) const;
				virtual double time_increment_max(double);

				virtual double load(void) const;
				virtual double load_guess(double);
				virtual double load_guess(void) const;
				virtual double load_predictor(void) const;
				virtual double load_increment(void) const;
				virtual double load_corrector(void) const;

				virtual const double* state(void) const;
				virtual const double* state_increment(void) const;
				virtual const double* state_predictor(void) const;
				virtual const double* state_corrector_residue(void) const;
				virtual const double* state_corrector_tangent(void) const;

				virtual const double* velocity(void) const;
				virtual const double* velocity_increment(void) const;
				virtual const double* velocity_predictor(void) const;

				virtual const double* acceleration(void) const;
				virtual const double* acceleration_increment(void) const;

				virtual const double* residue(void) const;
				virtual const double* reaction(void) const;
				virtual const double* eigen_values(void) const;
				virtual const double* eigen_vectors(void) const;

				virtual const double* kinetic_force(bool = true) const;
				virtual const double* inertial_force(bool = true) const;
				virtual const double* internal_force(bool = true) const;
				virtual const double* external_force(bool = true) const;
				virtual const double* reference_force(bool = true) const;

				virtual const double* inertia(void) const;
				virtual const double* damping(void) const;
				virtual const double* stiffness(void) const;

			protected:
				//memory
				virtual void free(void);
				virtual void allocate(unsigned, unsigned, unsigned);

				virtual void allocate_state(unsigned);
				virtual void allocate_velocity(unsigned);
				virtual void allocate_acceleration(unsigned);
				virtual void allocate_force(unsigned, unsigned);
				virtual void allocate_tangent(unsigned, unsigned);

				//analysis
				virtual void prepare(void);
				virtual bool check(void) const;

				virtual bool solve(void) = 0;

				virtual bool stop(bool) const;

				virtual void record(void) = 0;
				virtual void finish(void) const;
				virtual void symmetric(double*) const;

				//linear
				virtual void lindel(void);
				virtual bool lindec(const double*);
				virtual bool linsub(double*, const double*, const double*);
				virtual bool linsolve(double*, const double*, const double*);

				//data
				bool m_stop_run;
				void *m_sym, *m_num;
				Watch_Dof* m_watch_dof;
				static Analysis* m_analysis;
				unsigned m_step, m_step_max, m_load_set;

				std::function<void(unsigned)> m_run_step;
				std::function<void(void)> m_adapt_increment, m_adapt_iteration;

				double m_dof_min, m_dof_max;
				double m_time_min, m_time_max, m_time_increment_max;
				double m_t, m_dt, m_ue, m_ke, m_l, m_dl, m_dl0, m_dlg, m_ddl;

				double *m_K, *m_C, *m_M, *m_k, *m_m, *m_z, *m_state;
				double *m_u, *m_v, *m_a, *m_e, *m_du, *m_dv, *m_da, *m_dut, *m_ddu, *m_ddur, *m_ddut, *m_dvt;
				double *m_r, *m_R, *m_Fiu, *m_Fik, *m_Fnu, *m_Fnk, *m_Fku, *m_Fkk, *m_Feu, *m_Fek, *m_Fru, *m_Frk, *m_Fdu, *m_Fdk;
			};
		}
	}
}
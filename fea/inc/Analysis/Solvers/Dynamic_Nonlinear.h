#pragma once

//fea
#include "Analysis/Solvers/Time.h"
#include "Analysis/Solvers/Nonlinear.h"

namespace fea
{
	namespace analysis
	{
		namespace solvers
		{
			enum class integration : unsigned
			{
				newmark,
				runge_kutta
			};

			class Dynamic_Nonlinear : public virtual Time, public virtual Nonlinear
			{
				friend class analysis::solvers::Solver;

			protected:
				//constructors
				Dynamic_Nonlinear(void);

				//destructor
				virtual ~Dynamic_Nonlinear(void) override;

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
				//data
				solvers::integration integration(void) const;
				solvers::integration integration(solvers::integration);

				//type
				virtual solvers::type type(void) const override;

				//sets
				virtual unsigned state_set(void) const override;
				virtual unsigned force_set(void) const override;
				virtual unsigned tangent_set(void) const override;

			protected:
				//analysis
				virtual void prepare(void) override;

				virtual bool solve(void) override;

				virtual bool stop(bool) const override;

				virtual void record(void) override;

				virtual bool solve_newmark(void);
				virtual bool solve_runge_kutta(void);

				virtual bool compute_initial(void);
				virtual void compute_residue(void);
				virtual void compute_tangent(void);
				virtual void compute_tangent_1(void);
				virtual bool compute_tangent_2(void);
				virtual bool compute_tangent_3(void);
				virtual bool compute_tangent_4(void);
				virtual void compute_reference(void);
				virtual void compute_predictor(void);
				virtual bool compute_corrector(void);
				virtual void compute_reactions(void);
				virtual bool compute_acceleration(void);

				//data
				double* m_vn;
				solvers::integration m_integration;
			};
		}
	}
}
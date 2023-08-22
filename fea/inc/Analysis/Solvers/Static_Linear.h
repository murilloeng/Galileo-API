#pragma once

//fea
#include "fea/inc/Analysis/Solvers/Solver.h"

namespace fea
{
	namespace analysis
	{
		namespace solvers
		{
			class Static_Linear : public Solver
			{
				friend class Solver;

			protected:
				//constructors
				Static_Linear(void);

				//destructor
				~Static_Linear(void) override;

			public:
				//type
				solvers::type type(void) const override;

				//sets
				unsigned state_set(void) const override;
				unsigned force_set(void) const override;
				unsigned tangent_set(void) const override;

			protected:
				//analysis
				bool solve(void) override;

				void record(void) override;

				void solve_setup(void);
				bool solve_state(void);
				bool solve_apply(void);

				void compute_state(void);
				void compute_reactions(void);
			};
		}
	}
}
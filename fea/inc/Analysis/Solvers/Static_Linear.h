#pragma once

//fea
#include "Analysis/Solvers/Solver.h"

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
				virtual ~Static_Linear(void) override;

			public:
				//type
				virtual solvers::type type(void) const override;

				//sets
				virtual unsigned state_set(void) const override;
				virtual unsigned force_set(void) const override;
				virtual unsigned tangent_set(void) const override;

			protected:
				//analysis
				virtual bool solve(void) override;

				virtual void record(void) override;

				virtual void compute_state(void);
				virtual bool compute_increment(void);
				virtual void compute_reactions(void);
			};
		}
	}
}
#pragma once

//fea
#include "fea/inc/Analysis/Solvers/Eigen.h"

namespace fea
{
	namespace analysis
	{
		namespace solvers
		{
			class Buckling : public Eigen
			{
				friend class Solver;

			protected:
				//constructors
				Buckling(void);

				//destructor
				virtual ~Buckling(void) override;

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
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

				virtual void record(void) override;

				virtual void compute_state(void);
				virtual bool compute_buckling(void);
				virtual void compute_reactions(void);
			};
		}
	}
}
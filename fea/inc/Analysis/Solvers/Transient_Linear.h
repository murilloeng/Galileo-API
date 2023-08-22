#pragma once

//fea
#include "fea/inc/Analysis/Solvers/Time.h"

namespace fea
{
	namespace analysis
	{
		namespace solvers
		{
			class Transient_Linear : public Time
			{
				friend class Solver;

			protected:
				//constructors
				Transient_Linear(void);

				//destructor
				virtual ~Transient_Linear(void) override;

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
			};
		}
	}
}
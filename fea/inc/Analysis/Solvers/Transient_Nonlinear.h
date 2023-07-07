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
			class Transient_Nonlinear : public virtual Time, public virtual Nonlinear
			{
				friend class Solver;

			protected:
				//constructors
				Transient_Nonlinear(void);

				//destructor
				virtual ~Transient_Nonlinear(void) override;

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
				virtual bool solve(void) override;

				virtual bool stop(bool) const override;

				virtual void record(void) override;
			};
		}
	}
}
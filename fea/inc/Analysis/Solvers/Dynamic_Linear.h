#pragma once

//fea
#include "fea/inc/Analysis/Solvers/Time.h"

namespace fea
{
	namespace analysis
	{
		namespace solvers
		{
			class Dynamic_Linear : public Time
			{
			public:
				//constructors
				Dynamic_Linear(void);

				//destructor
				virtual ~Dynamic_Linear(void) override;

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

				virtual bool compute_initial(void);
				virtual void compute_tangent(void);
				virtual void compute_reference(void);
				virtual void compute_predictor(void);
				virtual bool compute_corrector(void);
				virtual void compute_reactions(void);
				virtual bool compute_acceleration(void);
			};
		}
	}
}
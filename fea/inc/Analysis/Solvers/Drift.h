#pragma once

//fea
#include "Analysis/Solvers/Field.h"
#include "Analysis/Solvers/Nonlinear.h"

namespace fea
{
	namespace analysis
	{
		namespace solvers
		{
			class Drift : public Nonlinear
			{
				friend class Solver;

			protected:
				//constructors
				Drift(void);

				//destructor
				virtual ~Drift(void) override;

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

				//data
				virtual double scale(double);
				virtual double scale(void) const;
				virtual double threshold(double);
				virtual double threshold(void) const;
				virtual solvers::field field(void) const;
				virtual solvers::field field(solvers::field);

			protected:
				//analysis
				virtual bool solve(void) override;

				virtual void record(void) override;

				virtual void rand_configuration(void);

				virtual void numeric_force(double*, double*, const double*);
				virtual void numeric_stiffness(double*, double*, const double*);

				virtual void solve_force(double*, double*, double*);
				virtual void solve_stiffness(double*, double*, double*, const int*, const int *);

				//data
				double m_scale;
				double m_error;
				double m_threshold;
				solvers::field m_field;
			};
		}
	}
}
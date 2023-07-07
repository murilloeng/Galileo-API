#pragma once

//std
#include <cmath>
#include <climits>

//fea
#include "Analysis/Solvers/Eigen.h"
#include "Analysis/Solvers/Nonlinear.h"

namespace fea
{
	namespace analysis
	{
		namespace solvers
		{
			enum class type : unsigned;
		}
		namespace strategies
		{
			class Strategy;
			enum class type : unsigned;
		}
	}
}

namespace fea
{
	namespace analysis
	{
		namespace solvers
		{
			struct unload
			{
				bool test = false;
				double load = INFINITY;
				double state = INFINITY;
				unsigned step = UINT_MAX;
			};

			class Static_Nonlinear : public virtual Eigen, public virtual Nonlinear
			{
				friend class Solver;

			protected:
				//constructors
				Static_Nonlinear(void);

				//destructor
				virtual ~Static_Nonlinear(void) override;

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
				//data
				bool load_adjust(bool);
				bool load_adjust(void) const;

				bool frequencies(bool);
				bool frequencies(void) const;

				bool branch_switch(bool);
				bool branch_switch(void) const;

				double load_min(double);
				double load_min(void) const;

				double load_max(double);
				double load_max(void) const;

				double load_factor(void) const;

				double mode_injection(double);
				double mode_injection(void) const;

				double load_increment_min(double);
				double load_increment_min(void) const;

				double load_increment_max(double);
				double load_increment_max(void) const;

				double state_increment_min(double);
				double state_increment_min(void) const;

				double state_increment_max(double);
				double state_increment_max(void) const;

				unsigned bifurcation_track(unsigned);
				unsigned bifurcation_track(void) const;

				solvers::unload& unload(void);

				strategies::Strategy* strategy(void) const;
				strategies::Strategy* strategy(strategies::type);

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

				virtual void solve_setup(void);
				virtual void solve_check(void);
				virtual bool solve_predictor(void);
				virtual bool solve_corrector(void);

				virtual void compute_unload(void);
				virtual bool compute_residue(void);
				virtual bool compute_predictor(void);
				virtual bool compute_corrector(void);
				virtual void compute_reactions(void);
				virtual bool compute_frequencies(void);
				virtual void compute_branch_switch(void);

				//data
				bool m_frequencies;
				bool m_load_adjust;
				bool m_branch_switch;

				double m_load_min;
				double m_load_max;
				double m_load_factor;
				double m_mode_injection;
				double m_load_increment_min;
				double m_load_increment_max;
				double m_state_increment_min;
				double m_state_increment_max;

				unsigned m_bifurcation_count;
				unsigned m_bifurcation_track;

				solvers::unload m_unload;

				strategies::Strategy* m_strategy;
			};
		}
	}
}
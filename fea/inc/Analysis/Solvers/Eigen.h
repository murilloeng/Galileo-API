#pragma once

//fea
#include "Analysis/Solvers/Solver.h"
#include "Analysis/Solvers/Spectre.h"

namespace fea
{
	namespace analysis
	{
		namespace solvers
		{
			class Eigen : public virtual Solver
			{
			protected:
				//constructors
				Eigen(void);

				//destructor
				virtual ~Eigen(void) override;

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
				//data
				double scale(double);
				double scale(void) const;

				unsigned spectre_min(unsigned);
				unsigned spectre_max(unsigned);
				unsigned spectre_min(void) const;
				unsigned spectre_max(void) const;

				solvers::spectre spectre(void) const;
				solvers::spectre spectre(solvers::spectre);

				unsigned modes(void) const;

			protected:
				//solve
				bool eigen_std(void) const;
				bool eigen_gen(void) const;

				//data
				double m_scale;
				unsigned m_mode_crop;
				unsigned m_spectre_min;
				unsigned m_spectre_max;
				solvers::spectre m_spectre;
			};
		}
	}
}
#pragma once

//fea
#include "Analysis/Solvers/Solver.h"
#include "Analysis/Solvers/Euller.h"
#include "Analysis/Solvers/Newmark.h"

namespace fea
{
	namespace analysis
	{
		namespace solvers
		{
			class Time : public virtual Solver
			{
			protected:
				//constructors
				Time(void);

				//destructor
				virtual ~Time(void) override;

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
				//data
				bool time_adjust(void) const;
				bool time_adjust(bool);

				solvers::Euller& euller(void);
				solvers::Newmark& newmark(void);

			protected:
				//analysis
				virtual bool stop(bool) const override;

				virtual void record(void) override;

				//data
				bool m_time_adjust;
				solvers::Euller m_euller;
				solvers::Newmark m_newmark;
			};
		}
	}
}
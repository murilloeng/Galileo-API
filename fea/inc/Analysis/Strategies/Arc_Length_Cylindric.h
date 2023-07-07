#pragma once

//fea
#include "Analysis/Strategies/Strategy.h"

namespace fea
{
	namespace analysis
	{
		namespace strategies
		{
			//declarations
			enum class type : unsigned;

			class Arc_Length_Cylindric : public Strategy
			{
			public:
				//constructors
				Arc_Length_Cylindric(void);

				//destructor
				virtual ~Arc_Length_Cylindric(void) override;

				//type
				virtual strategies::type type(void) const override;

				//analysis
				virtual double predictor(void) const override;
				virtual double corrector(void) const override;
			};
		}
	}
}
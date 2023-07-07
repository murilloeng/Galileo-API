#pragma once

//fea
#include "Mesh/Cells/Line/Line.h"

namespace fea
{
	namespace mesh
	{
		namespace cells
		{
			class Bar : public Line
			{
				friend class Cell;

			protected:
				//constructors
				Bar(unsigned = 1);

				//destructor
				virtual ~Bar(void) override;

			public:
				//type
				virtual cells::type type(void) const override;

				//interpolation
				virtual double* function(double*, const double*) const override;
				virtual double* gradient(double*, const double*) const override;
			};
		}
	}
}
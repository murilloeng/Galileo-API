#pragma once

//fea
#include "Mesh/Cells/Volume/Brick.h"

namespace fea
{
	namespace mesh
	{
		namespace cells
		{
			class Brick27 : public Brick
			{
				friend class Cell;

			protected:
				//constructors
				Brick27(unsigned = 3);

				//destructor
				virtual ~Brick27(void) override;

			public:
				//type
				virtual cells::type type(void) const override;

				//topology
				virtual unsigned vertices(void) const override;

				virtual std::vector<unsigned> edge(unsigned, bool) const override;
				virtual std::vector<unsigned> face(unsigned, bool) const override;

				//interpolation
				virtual double* function(double*, const double*) const override;
				virtual double* gradient(double*, const double*) const override;
			};
		}
	}
}
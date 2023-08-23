#pragma once

//fea
#include "fea/inc/Mesh/Cells/Plane/Quad.h"

namespace fea
{
	namespace mesh
	{
		namespace cells
		{
			class Quad9 : public Quad
			{
				friend class Cell;

			protected:
				//constructors
				Quad9(unsigned = 3);

				//destructor
				virtual ~Quad9(void) override;

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
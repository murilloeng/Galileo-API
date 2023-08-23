#pragma once

//fea
#include "fea/inc/Mesh/Cells/Plane/Tri.h"

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			class Section;
			class Warping;
		}
	}
}

namespace fea
{
	namespace mesh
	{
		namespace cells
		{
			class Tri6 : public Tri
			{
				friend class Cell;
				friend class sections::Section;
				friend class sections::Warping;

			protected:
				//constructors
				Tri6(unsigned = 2);

				//destructor
				virtual ~Tri6(void) override;

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
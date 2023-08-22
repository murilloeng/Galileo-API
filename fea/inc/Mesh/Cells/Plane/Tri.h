#pragma once

//fea
#include "fea/inc/Mesh/Cells/Plane/Plane.h"

namespace fea
{
	namespace mesh
	{
		namespace cells
		{
			class Tri : public Plane
			{
			protected:
				//constructors
				Tri(unsigned);

				//destructor
				virtual ~Tri(void) override;

			public:
				//topology
				virtual unsigned edges(void) const override;

				//parametrization
				virtual void edge(double*, unsigned, double) const override;
				virtual void gradient(double*, unsigned, double) const override;

				//integration
				virtual double point(double*, unsigned) const override;
			};
		}
	}
}
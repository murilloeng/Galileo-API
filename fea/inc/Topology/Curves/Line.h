#pragma once

#include "fea/inc/Topology/Curves/Curve.h"

namespace fea
{
	namespace topology
	{
		namespace curves
		{
			class Line : public Curve
			{
				friend class Curve;

			protected:
				//constructors
				Line(Topology*);

				//destructor
				virtual ~Line(void);

			public:
				//type
				virtual curves::type type(void) const override;

				//data
				virtual unsigned points_min(void) const override;
				virtual unsigned points_max(void) const override;

				//draw
				virtual unsigned draw_mesh(void) const override;

				//geometry
				virtual double length(void) const override;
				virtual double* vertex(double*, double) const override;
				virtual double* tangent(double*, double) const override;
			};
		}
	}
}
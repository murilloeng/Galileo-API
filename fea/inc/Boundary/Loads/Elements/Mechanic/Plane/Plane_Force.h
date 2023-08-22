#pragma once

//fea
#include "fea/inc/Boundary/Loads/Elements/Mechanic/Plane/Plane.h"

namespace fea
{
	namespace boundary
	{
		namespace loads
		{
			class Plane_Force : public Plane
			{
				friend class Element;

			protected:
				//constructors
				Plane_Force(void);

				//destructor
				virtual ~Plane_Force(void) override;

			public:
				//type
				virtual loads::type type(void) const override;

			protected:
				//draw
				virtual void draw(void) const override;
			};
		}
	}
}
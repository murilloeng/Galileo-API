#pragma once

//fea
#include "fea/inc/Boundary/Loads/Elements/Mechanic/Plane/Plane.h"

namespace fea
{
	namespace boundary
	{
		namespace loads
		{
			class Plane_Moment : public Plane
			{
				friend class Element;

			protected:
				//constructors
				Plane_Moment(void);

				//destructor
				virtual ~Plane_Moment(void) override;

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
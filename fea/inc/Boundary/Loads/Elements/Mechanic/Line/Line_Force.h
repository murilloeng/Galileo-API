#pragma once

//fea
#include "fea/inc/Boundary/Loads/Elements/Mechanic/Line/Line.h"

namespace fea
{
	namespace boundary
	{
		namespace loads
		{
			class Line_Force : public Line
			{
				friend class Element;

			protected:
				//constructors
				Line_Force(void);

				//destructor
				virtual ~Line_Force(void) override;

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
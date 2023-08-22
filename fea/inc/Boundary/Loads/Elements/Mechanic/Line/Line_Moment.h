#pragma once

//fea
#include "fea/inc/Boundary/Loads/Elements/Mechanic/Line/Line.h"

namespace fea
{
	namespace boundary
	{
		namespace loads
		{
			class Line_Moment : public Line
			{
				friend class Element;

			protected:
				//constructors
				Line_Moment(void);

				//destructor
				virtual ~Line_Moment(void) override;

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
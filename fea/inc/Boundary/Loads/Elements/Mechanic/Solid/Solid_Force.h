#pragma once

//fea
#include "fea/inc/Boundary/Loads/Elements/Mechanic/Mechanic.h"

namespace fea
{
	namespace boundary
	{
		namespace loads
		{
			class Solid_Force : public Mechanic
			{
				friend class Element;

			protected:
				//constructors
				Solid_Force(void);

				//destructor
				virtual ~Solid_Force(void) override;

			public:
				//type
				virtual loads::type type(void) const override;
			};
		}
	}
}
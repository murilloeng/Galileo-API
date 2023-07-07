#pragma once

//fea
#include "Boundary/Loads/Elements/Mechanic/Mechanic.h"

namespace fea
{
	namespace boundary
	{
		namespace loads
		{
			class Plane : public Mechanic
			{
			protected:
				//constructors
				Plane(void);

				//destructor
				virtual ~Plane(void) override;

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
				//data
				unsigned face(unsigned);
				unsigned face(void) const;

			protected:
				//data
				unsigned m_face;
			};
		}
	}
}
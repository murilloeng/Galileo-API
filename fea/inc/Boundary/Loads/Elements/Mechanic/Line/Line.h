#pragma once

//fea
#include "fea/inc/Boundary/Loads/Elements/Mechanic/Mechanic.h"

namespace fea
{
	namespace boundary
	{
		namespace loads
		{
			class Line : public Mechanic
			{
			protected:
				//constructors
				Line(void);

				//destructor
				virtual ~Line(void) override;

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

				//data
				unsigned edge(unsigned);
				unsigned edge(void) const;

			protected:
				//data
				unsigned m_edge;
			};
		}
	}
}
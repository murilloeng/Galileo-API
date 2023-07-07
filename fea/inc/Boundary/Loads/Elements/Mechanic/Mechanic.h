#pragma once

//fea
#include "Boundary/Loads/Elements/Element.h"

namespace fea
{
	namespace boundary
	{
		namespace loads
		{
			class Mechanic : public Element
			{
			protected:
				//constructors
				Mechanic(void);

				//destructors
				virtual ~Mechanic(void) override;

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
				//data
				double direction(unsigned) const;
				const double* direction(void) const;
				const double* direction(const double*);
				const double* direction(double, double, double);

			protected:
				//draw
				virtual void draw_force(void) const;
				virtual void draw_moment(void) const;

				//data
				double m_direction[3];
			};
		}
	}
}
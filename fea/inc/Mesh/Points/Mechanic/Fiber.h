#pragma once

//std
#include <vector>

//fea
#include "Mesh/Points/Mechanic/Mechanic.h"

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			class Fiber;
		}
		namespace elements
		{
			class Element;
		}
	}
}

namespace fea
{
	namespace mesh
	{
		namespace points
		{
			class Fiber
			{
			public:
				//constructors
				Fiber(void);

				//destructor
				virtual ~Fiber(void);

				//data
				double area(void) const;
				double weight(unsigned) const;
				double jacobian(unsigned) const;
				double shear_center(unsigned) const;
				double warping(unsigned, unsigned) const;
				double position(unsigned, unsigned) const;
				double gradient(unsigned, unsigned, unsigned) const;

				//analysis
				virtual void update(void);
				virtual void restore(void);
				virtual void prepare(const elements::Element*, const sections::Fiber*);

				//data
				std::vector<Mechanic> m_points;
				const sections::Fiber* m_fiber;
			};
		}
	}
}
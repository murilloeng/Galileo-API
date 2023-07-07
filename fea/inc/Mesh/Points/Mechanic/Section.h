#pragma once

//std
#include <vector>

//fea
#include "Mesh/Points/Point.h"
#include "Mesh/Points/Mechanic/Fiber.h"

namespace fea
{
	namespace mesh
	{
		namespace points
		{
			class Section : public Point
			{
			public:
				//constructors
				Section(void);

				//destructor
				virtual ~Section(void) override;

				//type
				virtual points::type type(void) const override;

				//analysis
				virtual void update(void) override;
				virtual void restore(void) override;
				virtual void prepare(const elements::Element*) override;

				//data
				double m_strain[12];
				double m_stress[12];
				double m_stiffness[144];
				std::vector<Fiber> m_fibers;
			};
		}
	}
}
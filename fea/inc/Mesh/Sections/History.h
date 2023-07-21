#pragma once

//std
#include <vector>

//fea
#include "Mesh/Sections/Point.h"

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			class Resultant;
		}
	}
}

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			class History
			{
				friend class Resultant;

			private:
				//constructors
				History(Resultant*);

				//destructor
				virtual ~History(void);

				//serialization
				virtual void load(FILE*);
				virtual void save(FILE*) const;

			public:
				//data
				unsigned step(unsigned);
				unsigned step(void) const;
				unsigned steps(void) const;
				const double* bounds(void) const;
				double bound(unsigned, unsigned) const;
				const Point& point(unsigned, unsigned) const;

			private:
				//analysis
				void bound(void);
				void resize(void);
				void prepare(void);
				void compute(unsigned);

				//data
				unsigned m_step;
				unsigned m_steps;
				double m_bounds[10];
				Resultant* m_resultant;
				std::vector<Point> m_points;
			};
		}
	}
}
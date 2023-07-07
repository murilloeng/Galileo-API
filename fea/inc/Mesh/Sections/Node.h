#pragma once

//std
#include <cstdio>

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			class Node
			{
			public:
				//constructors
				Node(void);
				Node(const double*);
				Node(double, double);

				//destructor
				virtual ~Node(void);

				//serialization
				virtual void load(FILE*);
				virtual void save(FILE*) const;

				//data
				double m_x[2];
				double m_u[3];
			};
		}
	}
}
#pragma once

namespace fea
{
	namespace results
	{
		class Limit_Point
		{
		public:
			//constructors
			Limit_Point(unsigned, unsigned, double, double);

			//destructor
			virtual ~Limit_Point(void);

			//data
			unsigned m_step;
			unsigned m_type;
			double m_coordinates[2];
		};
	}
}
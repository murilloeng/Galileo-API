#pragma once

#include <cstdio>

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			class Rebar
			{
			public:
				//constructors
				Rebar(double = 0, double = 0, double = 0);

				//destructor
				virtual ~Rebar(void);

				//serialization
				virtual void load(FILE*);
				virtual void save(FILE*) const;

				//draw
				virtual void draw(void) const;

				//data
				double diameter(double);
				double position_y(double);
				double position_z(double);

				double diameter(void) const;
				double position_y(void) const;
				double position_z(void) const;

			protected:
				//data
				double m_diameter;
				double m_position_y;
				double m_position_z;
			};
		}
	}
}
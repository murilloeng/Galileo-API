#pragma once

//std
#include <cstdio>

namespace fea
{
	namespace boundary
	{
		namespace time
		{
			class Time;
		}
		namespace loads
		{
			class Load_Case;
		}
	}
}

namespace fea
{
	namespace boundary
	{
		namespace loads
		{
			class Load
			{
				friend class Load_Case;

			protected:
				//constructors
				Load(void);

				//destructor
				virtual ~Load(void);

				//serialization
				virtual void load(FILE*);
				virtual void save(FILE*) const;

			public:
				//data
				double value(double);
				double value(void) const;

				time::Time* time(unsigned);
				time::Time* time(void) const;

				double time_value(double) const;

				Load_Case* load_case(void) const;

				//index
				unsigned index_time(void) const;

			protected:
				//analysis
				virtual void prepare(void) = 0;
				virtual bool check(void) const = 0;

				//draw
				virtual void draw(void) const = 0;

				//data
				double m_value;
				unsigned m_time;
				Load_Case* m_load_case;
			};
		}
	}
}
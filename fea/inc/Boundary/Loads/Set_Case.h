#pragma once

//std
#include <cstdio>

namespace fea
{
	namespace boundary
	{
		namespace loads
		{
			class Load_Set;
		}
	}
}

namespace fea
{
	namespace boundary
	{
		namespace loads
		{
			class Set_Case
			{
				//friends
				friend class Load_Set;

			private:
				//constructors
				Set_Case(void);

				//destructor
				virtual ~Set_Case(void);

				//serialization
				virtual void load(FILE*);
				virtual void save(FILE*) const;

			public:
				//data
				virtual bool fixed(bool);
				virtual bool fixed(void) const;

				virtual double value(double);
				virtual double value(void) const;

				virtual unsigned index(unsigned);
				virtual unsigned index(void) const;

			private:
				//data
				bool m_fixed = false;
				double m_value = 1.0;
				unsigned m_index = 0;
			};
		}
	}
}
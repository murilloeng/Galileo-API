//fea
#include "Boundary/Time/Time.h"

namespace fea
{
	namespace boundary
	{
		namespace time
		{
			class Product : public Time
			{
				friend class Time;

			protected:
				//constructors
				Product(void);

				//destructor
				~Product(void);

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
				//data
				unsigned time(unsigned) const;
				unsigned time(unsigned, unsigned);

				//type
				virtual time::type type(void) const override;

				//value
				virtual double value(double) const override;

			protected:
				//data
				unsigned m_times[2];
			};
		}
	}
}
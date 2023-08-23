//std
#include <functional>

//fea
#include "fea/inc/Boundary/Time/Time.h"

namespace fea
{
	namespace boundary
	{
		namespace time
		{
			class Custom : public Time
			{
				friend class Time;

			protected:
				//constructors
				Custom(void);

				//destructor
				~Custom(void);

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
				//data
				std::function<double(double)> function(void) const;
				std::function<double(double)> function(std::function<double(double)>);

				//type
				virtual time::type type(void) const override;

				//value
				virtual double value(double) const override;

			protected:
				//data
				std::function<double(double)> m_function;
			};
		}
	}
}
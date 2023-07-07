//std
#include <vector>

//fea
#include "Boundary/Time/Time.h"

namespace fea
{
	namespace boundary
	{
		namespace time
		{
			class Polynomial : public Time
			{
				friend class Time;

			protected:
				//constructors
				Polynomial(void);

				//destructor
				~Polynomial(void);

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
				//data
				bool pulse(bool);
				bool pulse(void) const;

				double phase(double);
				double phase(void) const;

				double period(double);
				double period(void) const;

				bool periodic(bool);
				bool periodic(void) const;

				std::vector<double>& terms(void);

				//type
				virtual time::type type(void) const override;

				//value
				virtual double value(double) const override;

			protected:
				//data
				bool m_pulse;
				double m_phase;
				double m_period;
				bool m_periodic;
				std::vector<double> m_terms;
			};
		}
	}
}
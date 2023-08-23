//fea
#include "fea/inc/Boundary/Time/Time.h"

namespace fea
{
	namespace boundary
	{
		namespace time
		{
			class Sine_Wave : public Time
			{
				friend class Time;

			protected:
				//constructors
				Sine_Wave(void);

				//destructor
				~Sine_Wave(void);

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

				double amplitude(double);
				double amplitude(void) const;

				double frequency(double);
				double frequency(void) const;

				double angular_frequency(double);
				double angular_frequency(void) const;

				//type
				virtual time::type type(void) const override;

				//value
				virtual double value(double) const override;

			protected:
				//data
				bool m_pulse;
				double m_phase;
				double m_period;
				double m_amplitude;
			};
		}
	}
}
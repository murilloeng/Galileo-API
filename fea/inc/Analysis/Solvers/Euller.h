#pragma once

namespace fea
{
	namespace analysis
	{
		namespace solvers
		{
			class Euller
			{
			public:
				//constructors
				Euller(void);

				//destructor
				~Euller(void);

				//serialization
				void load(FILE*);
				void save(FILE*) const;

				//data
				double alpha(double);
				double alpha(void) const;

			private:
				//data
				double m_alpha;
			};
		}
	}
}
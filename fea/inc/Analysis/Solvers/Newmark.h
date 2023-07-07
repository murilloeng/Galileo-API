#pragma once

namespace fea
{
	namespace analysis
	{
		namespace solvers
		{
			class Newmark
			{
			public:
				//constructors
				Newmark(void);

				//destructor
				~Newmark(void);

				//serialization
				void load(FILE*);
				void save(FILE*) const;

				//data
				bool hht(bool);
				double beta(double);
				double gamma(double);
				double alpha(double);

				bool hht(void) const;
				double beta(void) const;
				double gamma(void) const;
				double alpha(void) const;

				//analysis
				void parameters(double&, double&, double&) const;

			private:
				//data
				bool m_hht;
				double m_beta;
				double m_gamma;
				double m_alpha;
			};
		}
	}
}
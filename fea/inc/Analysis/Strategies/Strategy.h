#pragma once

namespace fea
{
	namespace analysis
	{
		class Analysis;
		namespace solvers
		{
			class Static_Nonlinear;
		}
		namespace strategies
		{
			enum class type : unsigned;
		}
	}
}

namespace fea
{
	namespace analysis
	{
		namespace strategies
		{
			class Strategy
			{
				friend class analysis::Analysis;
				friend class analysis::solvers::Static_Nonlinear;

			protected:
				//constructors
				Strategy(void);

				//destructor
				virtual ~Strategy(void);

				//create
				static void create(Strategy*&, strategies::type);

			public:
				//type
				virtual strategies::type type(void) const = 0;

				//name
				virtual const char* name(void) const;
				static const char* name(strategies::type);

			protected:
				//analysis
				virtual double predictor(void) const = 0;
				virtual double corrector(void) const = 0;

				//atributes
				static Analysis* m_analysis;
			};
		}
	}
}
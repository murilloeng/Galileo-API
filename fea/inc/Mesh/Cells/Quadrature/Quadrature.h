#pragma once

//std
#include <cstdio>

namespace fea
{
	namespace mesh
	{
		namespace cells
		{
			class Cell;
			enum class rule : unsigned;
		}
	}
}

namespace fea
{
	namespace mesh
	{
		namespace cells
		{
			class Quadrature
			{
				friend class Cell;

			private:
				//constructor
				Quadrature(Cell*);
				Quadrature(Cell*, unsigned);
				Quadrature(Cell*, cells::rule, unsigned);

				//destructor
				virtual ~Quadrature(void);

				//serialization
				virtual void load(FILE*);
				virtual void save(FILE*) const;

			public:
				//data
				virtual Cell* cell(void) const;
				virtual unsigned order(unsigned);
				virtual unsigned order(void) const;
				virtual cells::rule rule(void) const;
				virtual cells::rule rule(cells::rule);

				//name
				static const char* rule_name(cells::rule);
				virtual const char* rule_name(void) const;

				//analysis
				virtual void points(double*, double*) const;
				virtual double point(unsigned, double&) const;

			private:
				//data
				Cell* m_cell;
				unsigned m_order;
				cells::rule m_rule;
			};
		}
	}
}
#pragma once

#include "Mesh/Elements/Mechanic/Mechanic.h"

namespace fea
{
	namespace mesh
	{
		namespace elements
		{
			class Frame : public Mechanic
			{
			protected:
				//constructors
				Frame(void);

				//destructor
				virtual ~Frame(void) override;

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
				//data
				const double* orientation(unsigned);
				const double* orientation(void) const;
				const double* orientation(const double*);
				const double* orientation(double, double, double);

				//types
				virtual points::type point(void) const override;

				//analysis
				virtual void prepare(void) override;
				virtual bool check(void) const override;

			protected:
				//data
				double m_orientation[3];
			};
		}
	}
}
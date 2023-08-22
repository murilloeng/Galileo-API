#pragma once

//fea
#include "fea/inc/Mesh/Sections/Section.h"

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			class Generic : public Section
			{
				friend class Section;

			protected:
				//constructor
				Generic(void);

				//destructor
				virtual ~Generic(void) override;

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
				//type
				virtual sections::type type(void) const override;

				//area
				virtual double area(double);
				virtual double area(unsigned, unsigned, double);

				//inertia
				virtual double inertia(double);
				virtual double inertia(void) const override;
				virtual double inertia(unsigned, unsigned, double);
				virtual double inertia(unsigned, unsigned, unsigned, double);
				virtual double inertia(unsigned, unsigned, unsigned, unsigned, double);

				//warping
				virtual double shear_center(unsigned, double);
				virtual double inertia_warping(unsigned, unsigned, double);

				//analysis
				virtual bool prepare(void) override;
				virtual void update(void) const override;

			private:
				//data
				double m_J;
			};
		}
	}
}
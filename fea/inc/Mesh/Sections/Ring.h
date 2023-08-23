#pragma once

//fea
#include "fea/inc/Mesh/Sections/Section.h"

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			class Ring : public Section
			{
				friend class Section;

			protected:
				//constructors
				Ring(void);

				//destructor
				virtual ~Ring(void) override;

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
				//type
				virtual sections::type type(void) const override;

				//data
				double diameter(double);
				double thickness(double);

				double diameter(void) const;
				double thickness(void) const;

				//analysis
				virtual void update(void) const override;
				virtual bool check_sizes(void) const override;

			protected:
				//data
				double m_diameter;
				double m_thickness;
			};
		}
	}
}
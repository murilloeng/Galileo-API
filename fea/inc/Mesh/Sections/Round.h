#pragma once

//fea
#include "Mesh/Sections/Section.h"

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			class Round : public Section
			{
				friend class Section;

			protected:
				//constructors
				Round(void);

				//destructor
				virtual ~Round(void) override;

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
				//type
				virtual sections::type type(void) const override;

				//data
				double diameter(double);
				double diameter(void) const;

				//analysis
				virtual void update(void) const override;

			protected:
				//data
				double m_diameter;
			};
		}
	}
}
#pragma once

//fea
#include "Mesh/Sections/Section.h"

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			class Rectangle : public Section
			{
				friend class Section;

			protected:
				//constructors
				Rectangle(void);

				//destructor
				virtual ~Rectangle(void) override;

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
				//type
				virtual sections::type type(void) const override;

				//data
				double width(double);
				double height(double);

				double width(void) const;
				double height(void) const;

				//analysis
				virtual void update(void) const override;

			protected:
				//data
				double m_width;
				double m_height;
			};
		}
	}
}
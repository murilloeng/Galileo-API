#pragma once

//fea
#include "Mesh/Sections/Section.h"

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			class Box : public Section
			{
				friend class Section;

			protected:
				//constructor
				Box(void);

				//destructor
				virtual ~Box(void) override;

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
				//type
				virtual sections::type type(void) const override;

				//data
				double width(double);
				double height(double);
				double thickness(double);
				double radius_inner(double);
				double radius_outer(double);

				double width(void) const;
				double height(void) const;
				double thickness(void) const;
				double radius_inner(void) const;
				double radius_outer(void) const;

				//analysis
				virtual void topology_1(void) const;
				virtual void topology_2(void) const;
				virtual void topology_3(void) const;
				virtual void topology_4(void) const;
				virtual void update(void) const override;
				virtual bool check_sizes(void) const override;

			protected:
				//data
				double m_width;
				double m_height;
				double m_thickness;
				double m_radius_inner;
				double m_radius_outer;
			};
		}
	}
}
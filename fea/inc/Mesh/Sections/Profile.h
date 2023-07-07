#pragma once

//fea
#include "Mesh/Sections/Section.h"

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			class Mesh;
		}
	}
}

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			class Profile : public Section
			{
				friend class Mesh;
				friend class Section;

			protected:
				//constructors
				Profile(void);

				//destructor
				virtual ~Profile(void) override;

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
				//data
				virtual double angle(double);
				virtual double radius(double);
				virtual double web_height(double);
				virtual double web_thickness(double);
				virtual double flange_top_width(double);
				virtual double flange_bottom_width(double);
				virtual double flange_top_thickness(double);
				virtual double flange_bottom_thickness(double);

				virtual double angle(void) const;
				virtual double radius(void) const;
				virtual double web_height(void) const;
				virtual double web_thickness(void) const;
				virtual double flange_top_width(void) const;
				virtual double flange_bottom_width(void) const;
				virtual double flange_top_thickness(void) const;
				virtual double flange_bottom_thickness(void) const;

				//walls
				virtual unsigned walls(void) const = 0;
				virtual void wall(unsigned, double*, double*) const = 0;

				//analysis
				virtual bool check_sizes(void) const override;

			protected:
				//data
				double m_angle;
				double m_radius;
				double m_web_height;
				double m_web_thickness;
				double m_flange_top_width;
				double m_flange_bottom_width;
				double m_flange_top_thickness;
				double m_flange_bottom_thickness;
			};
		}
	}
}
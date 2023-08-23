#pragma once

//fea
#include "fea/inc/Mesh/Cells/Cell.h"

namespace fea
{
	namespace mesh
	{
		namespace cells
		{
			class Plane : public Cell
			{
			protected:
				//constructors
				Plane(unsigned);

				//destructor
				virtual ~Plane(void) override;

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
				//data
				double thickness(double);
				double thickness(void) const;

				//topology
				virtual unsigned faces(void) const override;
				virtual unsigned points(void) const override;
				virtual unsigned dimension(void) const override;

				//parametrization
				virtual void face(double*, unsigned, double, double) const override;
				virtual void gradient(double*, unsigned, double, double) const override;

			protected:
				//draw
				virtual void draw(unsigned) const override;

				virtual void draw_back(unsigned) const;
				virtual void draw_front(unsigned) const;
				virtual void draw_sides(unsigned) const;
				virtual void draw_graph(unsigned) const;

				virtual void normal(double*, unsigned) const;

				//data
				double m_thickness;
			};
		}
	}
}
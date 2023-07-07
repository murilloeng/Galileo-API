#pragma once

//fea
#include "Mesh/Cells/Cell.h"

namespace fea
{
	namespace mesh
	{
		class Mesh;
		namespace sections
		{
			class Section;
		}
	}
	namespace models
	{
		class Model;
	}
}

namespace fea
{
	namespace mesh
	{
		namespace cells
		{
			class Line : public Cell
			{
				friend class mesh::Mesh;
				friend class models::Model;

			protected:
				//constructors
				Line(unsigned = 1);

				//destructor
				virtual ~Line(void) override;

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
				//data
				sections::Section* section(unsigned);
				sections::Section* section(void) const;

				//index
				unsigned index_section(void) const;

				//topology
				virtual unsigned faces(void) const override;
				virtual unsigned edges(void) const override;
				virtual unsigned points(void) const override;
				virtual unsigned vertices(void) const override;
				virtual unsigned dimension(void) const override;

				virtual std::vector<unsigned> edge(unsigned, bool) const override;
				virtual std::vector<unsigned> face(unsigned, bool) const override;

				//parametrization
				virtual void edge(double*, unsigned, double) const override;
				virtual void face(double*, unsigned, double, double) const override;

				virtual void gradient(double*, unsigned, double) const override;
				virtual void gradient(double*, unsigned, double, double) const override;

				//integration
				virtual double point(double*, unsigned) const override;

				//mesh
				static void refine(unsigned, unsigned);

			protected:
				//draw
				virtual void draw(unsigned) const override;

				virtual void draw_lines(unsigned) const;
				virtual void draw_walls(unsigned) const;
				virtual void draw_graph(unsigned) const;
				virtual void draw_section(unsigned) const;

				//data
				unsigned m_section;
			};
		}
	}
}
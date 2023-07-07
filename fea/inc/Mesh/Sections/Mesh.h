#pragma once

//std
#include <vector>

//fea
#include "Mesh/Sections/Node.h"
#include "Mesh/Sections/Fiber.h"

namespace fea
{
	namespace draw
	{
		class Palette;
	}
	namespace mesh
	{
		namespace cells
		{
			class Cell;
		}
		namespace sections
		{
			class Draw;
			class Fiber;
			class Section;
			class Generic;
			class Warping;
			class Resultant;
		}
		namespace points
		{
			class Fiber;
			class Section;
		}
	}
	namespace topology
	{
		class Topology;
	}
}

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			class Mesh
			{
				friend class points::Fiber;
				friend class points::Section;
				friend class sections::Fiber;
				friend class sections::Section;
				friend class sections::Generic;
				friend class sections::Warping;
				friend class topology::Topology;
				friend class sections::Resultant;

			private:
				//constructors
				Mesh(const Section*);

				//destructor
				virtual ~Mesh(void);

			public:
				//serialization
				virtual void load(FILE*);
				virtual void save(FILE*) const;

				virtual void load_inertia(FILE*);
				virtual void load_warping(FILE*);
				virtual void save_inertia(FILE*) const;
				virtual void save_warping(FILE*) const;

				//mesh
				virtual void compute(void);

				//draw
				virtual void draw_mesh(void) const;
				virtual void draw_walls(void) const;
				virtual void draw_setup(const double*) const;
				virtual void draw_reset(const double*) const;
				virtual void draw_topology(const double* = nullptr) const;

				//data
				virtual double size(double);
				virtual double size(void) const;

				virtual double box_size(void) const;
				virtual double box_angle(void) const;
				virtual const double* box(void) const;
				virtual const double* box_center(void) const;
				virtual const std::vector<Node>& nodes(void) const;
				virtual const std::vector<Fiber>& fibers(void) const;

				//search
				virtual bool warping(double&, const double*, unsigned) const;
				virtual bool strain(double&, const double*, unsigned, unsigned) const;

			private:
				//draw
				virtual void draw_axis(void) const;
				virtual void draw_axes(void) const;
				virtual void draw_edges(void) const;
				virtual void draw_faces(void) const;
				virtual void draw_points(void) const;
				virtual void draw_rebars(void) const;

				//mesh
				virtual void clear(void);
				virtual void setup(void);
				virtual void bounds(void);
				virtual void center(void);
				virtual void rotate(void);
				virtual void cleanup(void);
				virtual void update(unsigned);
				virtual void remove_node(unsigned);
				virtual void assemble(unsigned, double, double, const double*);

				//data
				Warping* m_warping;
				const Section* m_section;
				const cells::Cell* m_cell;
				std::vector<Node> m_nodes;
				std::vector<Fiber> m_fibers;
				const draw::Palette* m_palette;
				double m_t, m_A, m_size, m_x[2], m_I1[2], m_I2[3], m_I3[4], m_I4[5], m_box[4];
			};
		}
	}
}
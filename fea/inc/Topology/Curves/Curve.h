#pragma once

//std
#include <cstdio>
#include <vector>

namespace fea
{
	namespace mesh
	{
		class Mesh;
		namespace cells
		{
			class Cell;
		}
		namespace materials
		{
			class Material;
		}
		namespace elements
		{
			enum class type : unsigned;
		}
	}
	namespace topology
	{
		class Topology;
		namespace points
		{
			class Point;
		}
		namespace curves
		{
			enum class type : unsigned;
		}
	}
}

namespace fea
{
	namespace topology
	{
		namespace curves
		{
			class Curve
			{
				friend class mesh::Mesh;
				friend class topology::Topology;

			protected:
				//constructors
				Curve(Topology*);

				//destructor
				virtual ~Curve(void);

				//serialization
				virtual void load(FILE*);
				virtual void save(FILE*) const;

				//create
				static void create(Topology*, Curve*&, const Curve*);
				static void create(Topology*, Curve*&, curves::type, const Curve* = nullptr);

			public:
				//data
				virtual Topology* topology(void) const;

				virtual unsigned structured(unsigned);
				virtual unsigned structured(void) const;

				virtual mesh::cells::Cell* cell(unsigned);
				virtual mesh::cells::Cell* cell(void) const;

				virtual mesh::materials::Material* material(unsigned);
				virtual mesh::materials::Material* material(void) const;

				virtual mesh::elements::type element(void) const;
				virtual mesh::elements::type element(mesh::elements::type);

				virtual std::vector<unsigned>& points(void);
				virtual unsigned points_min(void) const = 0;
				virtual unsigned points_max(void) const = 0;
				virtual points::Point* point(unsigned) const;
				virtual points::Point* point(unsigned, unsigned);
				virtual const std::vector<unsigned>& points(void) const;

				virtual std::vector<unsigned> nodes(bool) const;
				virtual const std::vector<unsigned>& nodes(void) const;
				virtual const std::vector<unsigned>& elements(void) const;

				//index
				virtual unsigned index(void) const;
				virtual unsigned index_cell(void) const;
				virtual unsigned index_material(void) const;
				virtual unsigned index_point(unsigned) const;

				//name
				virtual const char* name(void) const;
				static const char* name(curves::type);

				//type
				virtual curves::type type(void) const = 0;

				//draw
				virtual unsigned draw_mesh(void) const = 0;

				//geometry
				virtual double length(void) const = 0;
				virtual double* vertex(double*, double) const = 0;
				virtual double* tangent(double*, double) const = 0;

			protected:
				//draw
				virtual void draw(unsigned) const;
				virtual void draw_number(unsigned) const;

				//mesh
				virtual void mesh(void) const;
				virtual bool active(void) const;

				//data
				Topology* m_topology;
				mesh::elements::type m_element;
				unsigned m_cell, m_material, m_structured;

				std::vector<unsigned> m_nodes;
				std::vector<unsigned> m_points;
				std::vector<unsigned> m_elements;
			};
		}
	}
}
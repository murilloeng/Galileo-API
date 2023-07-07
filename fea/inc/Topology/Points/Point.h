#pragma once

namespace mat
{
	class vec3;
}
namespace fea
{
	namespace topology
	{
		class Topology;
		namespace surfaces
		{
			class Surface;
		}
	}
}

namespace fea
{
	namespace topology
	{
		namespace points
		{
			class Point
			{
				friend class topology::Topology;
				friend class topology::surfaces::Surface;

			protected:
				//constructors
				Point(Topology*);
				Point(Topology*, const double*, double = 0);
				Point(Topology*, double, double, double, double = 0);

				//destructor
				virtual ~Point(void);

				//serialization
				virtual void load(FILE*);
				virtual void save(FILE*) const;

			public:
				//data
				virtual double size(double);
				virtual double size(void) const;
				virtual Topology* topology(void) const;

				virtual double coordinate(unsigned) const;
				virtual const double* coordinates(void) const;
				virtual const double* coordinates(const double*);
				virtual const double* coordinates(double, unsigned);
				virtual const double* coordinates(double, double, double);

				//transform
				virtual Point& move(const mat::vec3&, bool = false);
				virtual Point& scale(const mat::vec3&, double, bool = false);
				virtual Point& rotate(const mat::vec3&, const mat::vec3&, bool = false);
				virtual Point& mirror(const mat::vec3&, const mat::vec3&, bool = false);

				//index
				virtual unsigned& node(void);
				virtual unsigned index(void) const;

			protected:
				//draw
				virtual void draw(unsigned) const;
				virtual void draw_number(unsigned) const;

				//mesh
				virtual void mesh(double) const;

				//data
				unsigned m_node;
				Topology* m_topology;
				double m_size, m_coordinates[3];
			};
		}
	}
}
#pragma once

//std
#include <cstdio>
#include <vector>

namespace mat
{
	class vec3;
}
namespace fea
{
	namespace models
	{
		class Model;
	}
	namespace mesh
	{
		namespace sections
		{
			class Mesh;
			class Section;
		}
	}
	namespace topology
	{
		namespace points
		{
			class Point;
		}
		namespace curves
		{
			class Curve;
			enum class type : unsigned;
		}
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
		class Topology
		{
			friend class models::Model;
			friend class mesh::sections::Mesh;
			friend class mesh::sections::Section;

		protected:
			//constructor
			Topology(mesh::sections::Mesh* = nullptr);

			//destructor
			virtual ~Topology(void);

			//serialization
			virtual void load(FILE*);
			virtual void load_points(FILE*);
			virtual void load_curves(FILE*);
			virtual void load_surfaces(FILE*);

			virtual void save(FILE*) const;
			virtual void save_points(FILE*) const;
			virtual void save_curves(FILE*) const;
			virtual void save_surfaces(FILE*) const;

		public:
			//bound
			virtual void bounds(double*) const;
			virtual double size(const double*) const;

			//model
			virtual models::Model* model(void) const;

			//data
			virtual double size(double);
			virtual double size(void) const;

			virtual unsigned order(unsigned);
			virtual unsigned order(void) const;

			virtual bool recombine(bool);
			virtual bool recombine(void) const;

			virtual bool incomplete(bool);
			virtual bool incomplete(void) const;

			virtual points::Point* point(unsigned) const;
			virtual curves::Curve* curve(unsigned) const;
			virtual surfaces::Surface* surface(unsigned) const;

			//merge
			virtual void merge_nodes(unsigned, unsigned);
			virtual void merge_points(std::vector<unsigned>, std::vector<unsigned>);
			virtual void merge_curves(std::vector<unsigned>, std::vector<unsigned>);
			virtual void merge_query_points(std::vector<unsigned>&, std::vector<unsigned>&);
			virtual void merge_query_curves(std::vector<unsigned>&, std::vector<unsigned>&);

			//lists
			virtual const std::vector<points::Point*>& points(void) const;
			virtual const std::vector<curves::Curve*>& curves(void) const;
			virtual const std::vector<surfaces::Surface*>& surfaces(void) const;

			//add
			virtual points::Point* add_point(const double*, double = 0);
			virtual points::Point* add_point(double, double, double, double = 0);

			virtual curves::Curve* add_curve(const curves::Curve*);
			virtual curves::Curve* add_curve(curves::type, std::vector<unsigned>);

			virtual surfaces::Surface* add_surface(void);

			//remove
			virtual void clear(void);
			virtual void remove_node(unsigned);
			virtual void remove_point(unsigned);
			virtual void remove_curve(unsigned);
			virtual void remove_surface(unsigned);
			virtual void remove_element(unsigned);

			virtual void remove_points(std::vector<unsigned>);
			virtual void remove_curves(std::vector<unsigned>);
			virtual void remove_surfaces(std::vector<unsigned>);

			//transform
			virtual void move_points(const std::vector<unsigned>&, const mat::vec3&, bool = false);
			virtual void scale_points(const std::vector<unsigned>&, const mat::vec3&, double, bool = false);
			virtual void rotate_points(const std::vector<unsigned>&, const mat::vec3&, const mat::vec3&, bool = false);
			virtual void mirror_points(const std::vector<unsigned>&, const mat::vec3&, const mat::vec3&, bool = false);

			virtual void move_curves(const std::vector<unsigned>&, const mat::vec3&, bool = false);
			virtual void scale_curves(const std::vector<unsigned>&, const mat::vec3&, double, bool = false);
			virtual void rotate_curves(const std::vector<unsigned>&, const mat::vec3&, const mat::vec3&, bool = false);
			virtual void mirror_curves(const std::vector<unsigned>&, const mat::vec3&, const mat::vec3&, bool = false);

			virtual void move_surfaces(const std::vector<unsigned>&, const mat::vec3&, bool = false);
			virtual void scale_surfaces(const std::vector<unsigned>&, const mat::vec3&, double, bool = false);
			virtual void rotate_surfaces(const std::vector<unsigned>&, const mat::vec3&, const mat::vec3&, bool = false);
			virtual void mirror_surfaces(const std::vector<unsigned>&, const mat::vec3&, const mat::vec3&, bool = false);

			//mesh
			virtual void mesh(unsigned) const;

		protected:
			//draw
			virtual void draw(void) const;
			virtual void draw_numbers(void) const;

			//transform
			virtual void curves_set(std::vector<unsigned>&, const std::vector<unsigned>&) const;
			virtual void curves_copy(const std::vector<unsigned>&, const std::vector<unsigned>&, unsigned);

			virtual void surfaces_set(std::vector<unsigned>&, const std::vector<unsigned>&) const;
			virtual void surfaces_copy(const std::vector<unsigned>&, const std::vector<unsigned>&, unsigned);

			//mesh
			static unsigned gmsh_cell(unsigned);

			virtual void mesh_setup(unsigned) const;
			virtual void mesh_nodes(unsigned) const;
			virtual void mesh_elements(unsigned) const;

			virtual void section_nodes(void) const;
			virtual void section_elements(void) const;

			//data
			double m_size;
			unsigned m_order;
			bool m_recombine;
			bool m_incomplete;
			mesh::sections::Mesh* m_mesh;
			static models::Model* m_model;
			std::vector<points::Point*> m_points;
			std::vector<curves::Curve*> m_curves;
			std::vector<surfaces::Surface*> m_surfaces;
		};
	}
}
#pragma once

//std
#include <vector>
#include <string>

namespace fea
{
	namespace mesh
	{
		class Mesh;
		namespace nodes
		{
			class Node;
		}
		namespace cells
		{
			class Cell;
		}
		namespace elements
		{
			enum class type : unsigned;
			enum class state : uint64_t;
		}
		namespace materials
		{
			class Material;
		}
		namespace points
		{
			class Point;
			enum class type : unsigned;
		}
	}
	namespace boundary
	{
		namespace loads
		{
			class Element;
		}
	}
	namespace analysis
	{
		class Assembler;
	}
}

namespace fea
{
	namespace mesh
	{
		namespace elements
		{
			class Element
			{
				friend class mesh::Mesh;
				friend class mesh::points::Point;
				friend class analysis::Assembler;
				friend class boundary::loads::Element;

			protected:
				//constructors
				Element(void);

				//destructor
				virtual ~Element(void);

				//serialization
				virtual void load(FILE*);
				virtual void save(FILE*) const;
				
				virtual bool load_state(void);
				virtual bool save_state(void) const;

				virtual bool load_energy(void);
				virtual bool save_energy(void) const;

				//create
				static void create(Element*&, const Element*);
				static void create(Element*&, elements::type, const Element* = nullptr);

			public:
				//data
				static Mesh* mesh(void);

				virtual cells::Cell* cell(unsigned);
				virtual cells::Cell* cell(void) const;

				virtual nodes::Node* node(unsigned) const;
				virtual nodes::Node* node(unsigned, unsigned);

				virtual materials::Material* material(unsigned);
				virtual materials::Material* material(void) const;

				//types
				virtual unsigned cell_set(void) const = 0;
				virtual unsigned load_set(void) const = 0;
				virtual uint64_t state_set(void) const = 0;
				virtual unsigned dof_set(unsigned) const = 0;

				virtual points::type point(void) const = 0;
				virtual elements::type type(void) const = 0;
				virtual std::vector<unsigned> dof(void) const;

				//name
				virtual const char* name(void) const;
				static const char* name(elements::type);

				//state
				static const char* state_name(state);

				//lists
				virtual const std::vector<unsigned>& nodes(void) const;

				//index
				virtual unsigned index(void) const;
				virtual unsigned index_cell(void) const;
				virtual unsigned index_node(unsigned) const;
				virtual unsigned index_material(void) const;

				//add
				virtual void add_node(unsigned);

				//remove
				virtual void remove_node(unsigned);

			protected:
				//analysis
				virtual void add_dof(void) const;

				virtual void apply(void);

				virtual void prepare(void);
				virtual bool check(void) const;
				virtual bool check_plane(void) const;

				virtual void update(void);
				virtual void restore(void);

				virtual void record(void);
				virtual void finish(void) const;

				//data
				double* state(double*, unsigned = 1) const;
				double* velocity(double*, unsigned = 1) const;
				double* acceleration(double*, unsigned = 1) const;

			public:
				//formulation
				virtual double kinetic_energy(void) const;
				virtual double internal_energy(void) const;

				virtual double* kinetic_force(double*) const;
				virtual double* inertial_force(double*) const;
				virtual double* internal_force(double*) const;

				virtual double* reference_force(double*, const boundary::loads::Element*) const;

				virtual double* inertia(double*) const;
				virtual double* damping(double*) const;
				virtual double* stiffness(double*) const;

			protected:
				//draw
				virtual void draw(unsigned) const;
				virtual void draw_number(unsigned) const;

				//data
				double* m_state;
				double* m_energy;
				static Mesh* m_mesh;
				unsigned m_cell, m_material;
				std::vector<unsigned> m_nodes;
				std::vector<unsigned> m_dof_index;
				std::vector<points::Point*> m_points;
			};
		}
	}
}
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
		namespace loops
		{
			class Loop;
		}
	}
}

namespace fea
{
	namespace topology
	{
		namespace surfaces
		{
			class Surface
			{
			protected:
				//constructors
				Surface(Topology*);

				//destructor
				virtual ~Surface(void);

				//serialization
				virtual void load(FILE*);
				virtual void save(FILE*) const;

			public:
				//data
				virtual Topology* topology(void) const;

				virtual bool structured(bool);
				virtual bool structured(void) const;

				virtual mesh::cells::Cell* cell(unsigned);
				virtual mesh::cells::Cell* cell(void) const;

				virtual mesh::materials::Material* material(unsigned);
				virtual mesh::materials::Material* material(void) const;

				virtual mesh::elements::type element(void) const;
				virtual mesh::elements::type element(mesh::elements::type);

				virtual std::vector<unsigned> nodes(bool) const;
				virtual const std::vector<unsigned>& nodes(void) const;
				virtual const std::vector<unsigned>& elements(void) const;

				//loops
				void remove_loop(unsigned);
				loops::Loop* loop(unsigned);
				loops::Loop* add_loop(void);
				virtual std::vector<loops::Loop*>& loops(void);

				//index
				virtual unsigned index(void) const;
				virtual unsigned index_cell(void) const;
				virtual unsigned index_material(void) const;

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

				bool m_structured;
				unsigned m_cell, m_material;

				std::vector<unsigned> m_nodes;
				std::vector<unsigned> m_elements;
				std::vector<loops::Loop*> m_loops;

				//friends
				friend class mesh::Mesh;
				friend class topology::Topology;
			};
		}
	}
}
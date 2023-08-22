#pragma once

//std
#include <vector>
#include <cstdio>

//fea
#include "fea/inc/Mesh/Sections/Rebar.h"
#include "fea/inc/Mesh/Cells/Plane/Tri6.h"

namespace fea
{
	namespace mesh
	{
		class Mesh;
		namespace cells
		{
			class Line;
		}
		namespace points
		{
			class Section;
		}
		namespace sections
		{
			class Draw;
			class Mesh;
			class Fiber;
			class Warping;
			class Resultant;
			class E3Section;
			enum class type : unsigned;
		}
		namespace materials
		{
			class Material;
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
			class Section
			{
				friend class mesh::Mesh;
				friend class mesh::cells::Line;
				friend class mesh::sections::Draw;
				friend class mesh::sections::Mesh;
				friend class mesh::points::Section;
				friend class mesh::sections::Fiber;
				friend class mesh::sections::Warping;
				friend class mesh::sections::Resultant;

			protected:
				//constructors
				Section(void);

				//destructor
				virtual ~Section(void);

				//serialization
				virtual void load(FILE*);
				virtual void save(FILE*) const;

				//create
				static Section* create(Section*&, const Section*);
				static Section* create(Section*&, sections::type, const Section* = nullptr);

			public:
				//type
				virtual sections::type type(void) const = 0;

				//name
				virtual const char* name(void) const;
				static const char* name(sections::type);

				//data
				static mesh::Mesh* mesh(void);
				static const cells::Tri6& cell(void);

				virtual double size(double);
				virtual double size(void) const;

				virtual const char* label(void) const;
				virtual const char* label(const char*);

				virtual std::vector<Rebar>& rebars(void);

				virtual Mesh* mesh_local(void) const;
				virtual Warping* warping(void) const;
				virtual Resultant* resultant(void) const;

				virtual const topology::Topology* topology(void) const;

				//index
				virtual unsigned index(void) const;

				//area
				virtual double area(void) const;
				virtual double area(unsigned, unsigned) const;

				//inertia
				virtual double inertia(void) const;
				virtual double inertia(unsigned, unsigned) const;
				virtual double inertia(unsigned, unsigned, unsigned) const;
				virtual double inertia(unsigned, unsigned, unsigned, unsigned) const;

				//warping
				virtual double shear_center(unsigned) const;
				virtual double inertia_warping(unsigned, unsigned) const;

				//analysis
				virtual bool prepare(void);
				virtual bool check(void) const;
				virtual void update(void) const = 0;
				virtual bool check_sizes(void) const;

			protected:
				//data
				double m_size;
				char m_label[200];
				Mesh* m_mesh_local;
				Warping* m_warping;
				Resultant* m_resultant;
				std::vector<Rebar> m_rebars;
				topology::Topology* m_topology;

				static mesh::Mesh* m_mesh;
				static cells::Tri6 m_cell;
			};
		}
	}
}
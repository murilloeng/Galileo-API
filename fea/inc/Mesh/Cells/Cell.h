#pragma once

//std
#include <vector>
#include <string>

namespace fea
{
	namespace mesh
	{
		class Mesh;
		namespace cells
		{
			class Quadrature;
			enum class type : unsigned;
		}
		namespace elements
		{
			class Element;
		}
	}
}

namespace fea
{
	namespace mesh
	{
		namespace cells
		{
			class Cell
			{
				friend class mesh::Mesh;
				friend class mesh::elements::Element;

			protected:
				//constructors
				Cell(unsigned);

				//destructor
				virtual ~Cell(void);

				//serialization
				virtual void load(FILE*);
				virtual void save(FILE*) const;

				//create
				static void create(Cell*&, const Cell*);
				static void create(Cell*&, cells::type, const Cell* = nullptr);

			public:
				//type
				virtual cells::type type(void) const = 0;

				//index
				unsigned index(void) const;

				//name
				static const char* name(cells::type);
				virtual const char* name(void) const;

				//data
				static Mesh* mesh(void);

				virtual Quadrature* quadrature(void);

				virtual const char* label(void) const;
				virtual const char* label(const char*);

				//topology
				virtual unsigned faces(void) const = 0;
				virtual unsigned edges(void) const = 0;
				virtual unsigned points(void) const = 0;
				virtual unsigned vertices(void) const = 0;
				virtual unsigned dimension(void) const = 0;

				virtual std::vector<unsigned> edge(unsigned, bool) const = 0;
				virtual std::vector<unsigned> face(unsigned, bool) const = 0;

				//parametrization
				virtual void edge(double*, unsigned, double) const = 0;
				virtual void face(double*, unsigned, double, double) const = 0;

				virtual void gradient(double*, unsigned, double) const = 0;
				virtual void gradient(double*, unsigned, double, double) const = 0;

				//geometry
				virtual double edge(const elements::Element*, unsigned) const;
				virtual double face(const elements::Element*, unsigned) const;
				virtual double volume(const elements::Element*) const;

				virtual void position(double*, const double*, const double*) const;
				virtual double jacobian(double*, const double*, const double*) const;

				virtual void position(double*, const elements::Element*, const double*) const;
				virtual double jacobian(double*, const elements::Element*, const double*) const;
				virtual double gradient(double*, double*, const elements::Element*, const double*) const;

				//integration
				virtual double point(double*, unsigned) const = 0;

				//extrapolation
				virtual void extrapolate(double*, const double*) const;

				//interpolation
				virtual double* function(double*, const double*) const = 0;
				virtual double* gradient(double*, const double*) const = 0;

			protected:
				//analysis
				virtual bool check(void) const;

				//plot
				virtual void draw(unsigned) const = 0;

				//data
				char m_label[200];
				static Mesh* m_mesh;
				Quadrature* m_quadrature;
			};
		}
	}
}
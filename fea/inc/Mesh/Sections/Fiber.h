#pragma once

//std
#include <cstdio>

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			class Mesh;
		}
	}
}

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			class Fiber
			{
			public:
				//constructors
				Fiber(const Mesh*);
				Fiber(const Mesh*, const unsigned*);

				//destructor
				virtual ~Fiber(void);

				//serialization
				virtual void load(FILE*);
				virtual void save(FILE*) const;

				//data
				virtual double area(void) const;
				virtual double weight(unsigned) const;
				virtual double jacobian(unsigned) const;
				virtual double warping(unsigned, unsigned) const;
				virtual double position(unsigned, unsigned) const;
				virtual double gradient(unsigned, unsigned, unsigned) const;

				//topology
				virtual bool inside(const double*) const;

				virtual void point_local(double*, const double*) const;
				virtual void point_global(double*, const double*) const;

				virtual double warping(const double*, unsigned) const;
				virtual double strain(const double*, unsigned, unsigned) const;
				virtual double gradient(const double*, unsigned, unsigned) const;

				//stress
				virtual double stress(unsigned, unsigned, unsigned) const;

				//data
				unsigned m_e[6];
				const Mesh* m_mesh;
			};
		}
	}
}
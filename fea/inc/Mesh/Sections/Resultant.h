#pragma once

//std
#include <functional>

//fea
#include "Mesh/Points/Mechanic/Fiber.h"

namespace mat
{
	namespace solvers
	{
		class newton_raphson;
	}
}
namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			class Mesh;
			class Section;
			class History;
		}
		namespace materials
		{
			class Mechanic;
		}
	}
}

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			class Resultant
			{
				friend class Section;
				friend class History;

			private:
				//constructor
				Resultant(Section*);

				//destructor
				virtual ~Resultant(void);

			public:
				//serialization
				virtual bool load(void);
				virtual bool save(void) const;
				virtual char* path(char*) const;

				//data
				void mark(void);
				bool computed(void) const;

				unsigned size(void) const;

				unsigned mode(unsigned);
				unsigned mode(void) const;

				unsigned plane(unsigned);
				unsigned plane(void) const;

				double criteria(double);
				double criteria(void) const;

				unsigned index(unsigned) const;
				unsigned index(unsigned, unsigned);

				History* history(void);

				materials::Mechanic* material(unsigned);
				materials::Mechanic* material(void) const;

				materials::Mechanic* reinforcement(unsigned);
				materials::Mechanic* reinforcement(void) const;

				const double* bounds(void) const;
				const Section* section(void) const;
				const std::vector<double>& data(unsigned) const;
				mat::solvers::newton_raphson* solver(void) const;

				std::function<void(unsigned)> run_interface(void) const;
				std::function<void(unsigned)> run_interface(std::function<void(unsigned)>);

				//analysis
				void bound(void);
				bool compute(void);

			private:
				//analysis
				bool stop(void);
				void clear(void);
				void setup(void);
				void update(void);
				void restore(void);
				void prepare(void);
				bool compute_single(void);
				bool compute_double(void);
				void gradient(double*, unsigned, unsigned);
				void system(double*, double*, const double*);

				//data
				Mesh* m_mesh;
				bool m_computed;
				double m_criteria;
				History* m_history;
				Section* m_section;
				double m_bounds[24];
				unsigned m_material;
				double *m_sst, *m_Kst;
				unsigned m_reinforcement;
				std::vector<double> m_data[2];
				std::vector<points::Fiber> m_fibers;
				unsigned m_mode, m_plane, m_index[2];
				mat::solvers::newton_raphson* m_solver;
				std::function<void(unsigned)> m_run_interface;
			};
		}
	}
}
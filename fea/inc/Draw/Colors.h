#pragma once

//std
#include <cstdio>

namespace fea
{
	namespace draw
	{
		class Colors
		{
		public:
			//constructors
			Colors(void);

			//destructor
			virtual ~Colors(void);

			//serialization
			void load(FILE*);
			void save(FILE*) const;

			//data
			double* back(const double*);
			double* nodes(const double*);
			double* loads(const double*);
			double* paths(const double*);
			double* joints(const double*);
			double* points(const double*);
			double* curves(const double*);
			double* elements(const double*);
			double* supports(const double*);
			double* surfaces(const double*);
			double* selection(const double*);

			const double* back(void) const;
			const double* nodes(void) const;
			const double* loads(void) const;
			const double* paths(void) const;
			const double* joints(void) const;
			const double* points(void) const;
			const double* curves(void) const;
			const double* elements(void) const;
			const double* supports(void) const;
			const double* surfaces(void) const;
			const double* selection(void) const;

			//misc
			double* back_inverse(double*) const;

			//config
			void reset(void);
			void print(void);

		private:
			//data
			double m_back[4];
			double m_nodes[4];
			double m_loads[4];
			double m_paths[4];
			double m_joints[4];
			double m_points[4];
			double m_curves[4];
			double m_elements[4];
			double m_supports[4];
			double m_surfaces[4];
			double m_selection[4];
		};
	}
}
#pragma once

#include "fea/inc/Mesh/Cells/Plane/Tri6.h"

namespace fea
{
	namespace mesh
	{
		namespace cells
		{
			class Cell;
		}
		namespace points
		{
			class Section;
		}
		namespace sections
		{
			class Mesh;
			class Fiber;
			class Section;
			class Generic;
		}
	}
}

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			class Warping
			{
				friend class sections::Mesh;
				friend class points::Section;
				friend class sections::Fiber;
				friend class sections::Section;
				friend class sections::Generic;

			private:
				//constructors
				Warping(const Section*);

				//destructor
				virtual ~Warping(void);

			public:
				//data
				bool computed(void) const;
				const double* stress_bounds(void) const;
				const double* warping_bounds(void) const;

			private:
				//analysis
				virtual void clear(void);
				virtual bool solve(void);
				virtual void force(void);
				virtual void shear(void);
				virtual void scale(void);
				virtual void adjust(void);
				virtual void center(void);
				virtual bool dof_map(void);
				virtual bool compute(void);
				virtual void inertias(void);
				virtual void allocate(void);
				virtual void stiffness(void);

				//setup
				virtual Warping* setup(void);

				//data
				Mesh* m_mesh;
				unsigned m_nn, m_ne;
				const cells::Cell* m_cell;
				int *m_row_map, *m_col_map, *m_row_triplet, *m_col_triplet;
				double m_I, m_c[2], m_A[3], m_W[6], *m_u, *m_f, *m_K, m_ub[6], m_sb[18];
			};
		}
	}
}
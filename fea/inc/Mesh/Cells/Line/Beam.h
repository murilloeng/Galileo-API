#pragma once

//mat
#include "linear/matrix.h"

//fea
#include "Mesh/Cells/Line/Line.h"

namespace fea
{
	namespace mesh
	{
		namespace cells
		{
			class Beam : public Line
			{
				friend class Cell;

			protected:
				//constructors
				Beam(unsigned = 2);

				//destructor
				virtual ~Beam(void) override;

			public:
				//type
				virtual cells::type type(void) const override;

				//interpolation
				virtual double* strain(double*, const double*) const;
				virtual double* function(double*, const double*) const override;
				virtual double* gradient(double*, const double*) const override;

				virtual void strain(mat::matrix&, const elements::Element*, double) const;
				virtual void function(mat::matrix&, const elements::Element*, double) const;

			private:
				//misc
				virtual void eigen(void) const;
				virtual void eigen(double) const;
				virtual void kinematic(void) const;

				virtual mat::matrix field(double) const;
				virtual mat::matrix strain(double) const;

				virtual void shear_stiffness(double) const;
				virtual void initial_stiffness(double) const;
				virtual void bending_stiffness(double) const;
				virtual void warping_stiffness(double) const;

				virtual void list(void) const;
				virtual void setup(const elements::Element*) const;
				virtual void apply(mat::matrix&, bool, double) const;
				virtual void complete(mat::matrix&, bool, double) const;

				//data
				static double m_L;
				static unsigned m_nfu, m_nft, m_nsu, m_nst, m_lfu[12], m_lsu[12];

				static mat::matrix m_Kw, m_K0, m_P, m_P1, m_P2, m_D, m_C, m_S;
				static mat::matrix m_I2, m_I3, m_H, m_A, m_E, m_Ac, m_K, m_Ks, m_Kb;
			};
		}
	}
}
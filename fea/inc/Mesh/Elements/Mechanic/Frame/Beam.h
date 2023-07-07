#pragma once

//fea
#include "Mesh/Elements/Mechanic/Frame/Frame.h"

namespace fea
{
	namespace models
	{
		class Model;
	}
	namespace boundary
	{
		namespace loads
		{
			class Line_Force;
			class Line_Moment;
		}
	}
}

namespace fea
{
	namespace mesh
	{
		namespace elements
		{
			class Beam : public Frame
			{
				friend class Element;
				friend class models::Model;

			protected:
				//constructors
				Beam(void);

				//destructor
				virtual ~Beam(void) override;

			public:
				//types
				virtual unsigned load_set(void) const override;
				virtual unsigned cell_set(void) const override;

				//data
				static bool shear(void);
				static bool shear(bool);
				static bool mixed(void);
				static bool mixed(bool);
				static bool high_order(void);
				static bool high_order(bool);

				double connection(unsigned) const;
				double connection(unsigned, double);

			protected:
				//analysis
				virtual double* load_line_force(double*, const boundary::loads::Line_Force*) const = 0;
				virtual double* load_line_moment(double*, const boundary::loads::Line_Moment*) const = 0;
				virtual double* reference_force(double*, const boundary::loads::Element*) const override;

				//data
				double m_p[2];
				static bool m_shear;
				static bool m_mixed;
				static bool m_high_order;
			};
		}
	}
}
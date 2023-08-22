#pragma once

//fea
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Beam.h"

namespace fea
{
	namespace mesh
	{
		namespace elements
		{
			class Beam2 : public Beam
			{
			protected:
				//constructors
				Beam2(void);

				//destructor
				virtual ~Beam2(void) override;

			public:
				//types
				virtual uint64_t state_set(void) const override;
				virtual unsigned stress_set(void) const override;
				virtual unsigned dof_set(unsigned) const override;

			protected:
				//analysis
				virtual bool check(void) const override;

				virtual double* load_line_force(double*, const boundary::loads::Line_Force*) const override;
				virtual double* load_line_moment(double*, const boundary::loads::Line_Moment*) const override;
			};
		}
	}
}
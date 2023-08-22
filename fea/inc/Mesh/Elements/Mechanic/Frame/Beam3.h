#pragma once

//fea
#include "fea/inc/Mesh/Elements/Mechanic/Frame/Beam.h"

namespace fea
{
	namespace mesh
	{
		namespace elements
		{
			enum class warping : unsigned;
		}
	}
	namespace models
	{
		class Model;
	}
}

namespace fea
{
	namespace mesh
	{
		namespace elements
		{
			class Beam3 : public Beam
			{
				friend class models::Model;

			protected:
				//constructors
				Beam3(void);

				//destructor
				virtual ~Beam3(void) override;

			public:
				//data
				static elements::warping warping(void);
				static elements::warping warping(elements::warping);

				//name
				static const char* warping_name(void);
				static const char* warping_name(elements::warping);

				//types
				virtual uint64_t state_set(void) const override;
				virtual unsigned stress_set(void) const override;
				virtual unsigned dof_set(unsigned) const override;

			protected:
				//analysis
				virtual double* load_line_force(double*, const boundary::loads::Line_Force*) const override;
				virtual double* load_line_moment(double*, const boundary::loads::Line_Moment*) const override;

				//data
				static elements::warping m_warping;
			};
		}
	}
}
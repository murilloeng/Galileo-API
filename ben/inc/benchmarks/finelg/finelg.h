#pragma once

namespace tests
{
	namespace finelg
	{
		namespace sections
		{
			void rectangle(void);
			void profile_I(void);
			void profile_C(void);
		}
		namespace buckling
		{
			void pinned_bending(void);
		}
		namespace static_linear
		{
			void pinned_middle_force(void);
			void cantilever_tip_force(void);
		}
		namespace static_nonlinear
		{
			namespace shear
			{
				void cf_tip_force(void);
				void ss_mid_force(void);
				void ss_mid_moment(void);
			}
			namespace elastic
			{
				void pinned_cable(void);
				void cantilever_arc(void);
				void cantilever_axial(void);
				void cantilever_bending(void);
				void cantilever_torsion(void);
				void cantilever_buckling(void);
				void pinned_middle_force(void);
				void cantilever_tip_force(void);
			}
			namespace inelastic
			{

			}
		}
	}
}
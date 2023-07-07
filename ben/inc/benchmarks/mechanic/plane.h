#pragma once

namespace tests
{
	namespace plane
	{
		namespace static_linear
		{
			void cantilever_axial(void);
			void cantilever_shear(void);
			void cantilever_bending(void);
		}
		namespace static_nonlinear
		{
			namespace elastic
			{
				void cantilever_axial(void);
				void cantilever_shear(void);
				void cantilever_bending(void);
			}
			namespace inelastic
			{
				void cantilever_axial(void);
				void cantilever_shear(void);
				void cantilever_bending(void);
			}
		}
	}
}
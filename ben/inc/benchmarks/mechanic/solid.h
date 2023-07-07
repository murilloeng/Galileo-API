#pragma once

namespace tests
{
	namespace solid
	{
		namespace state
		{
			void rigid_body(void);
		}
		namespace static_linear
		{
			void cantilever_axial(void);
			void cantilever_shear(void);
			void cantilever_bending(void);
		}
		namespace static_nonlinear
		{
			void cantilever_axial(void);
			void cantilever_shear(void);
			void cantilever_bending(void);
		}
		namespace dynamic_nonlinear
		{
			void cantilever_T(void);
		}
	}
}
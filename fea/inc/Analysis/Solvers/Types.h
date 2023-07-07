#pragma once

namespace fea
{
	namespace analysis
	{
		namespace solvers
		{
			enum class type : unsigned
			{
				drift				= 1 << 0,
				modal				= 1 << 1,
				state				= 1 << 2,
				buckling			= 1 << 3,
				static_linear		= 1 << 4,
				static_nonlinear	= 1 << 5,
				dynamic_linear		= 1 << 6,
				dynamic_nonlinear	= 1 << 7,
				transient_linear	= 1 << 8,
				transient_nonlinear	= 1 << 9,
				last
			};
		}
	}
}
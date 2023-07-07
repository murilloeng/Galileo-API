#pragma once

namespace fea
{
	namespace boundary
	{
		namespace loads
		{
			enum class type : unsigned
			{
				line_force		= 1 << 0,
				line_moment		= 1 << 1,
				plane_force		= 1 << 2,
				plane_moment	= 1 << 3,
				solid_force		= 1 << 4,
				last
			};
		}
	}
}
#pragma once

namespace fea
{
	namespace topology
	{
		namespace curves
		{
			enum class type : unsigned
			{
				line =			1 << 0,
				circle_arc =	1 << 1,
				last
			};
		}
	}
}
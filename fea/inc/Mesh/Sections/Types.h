#pragma once

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			enum class type : unsigned
			{
				box			= 1 << 0,
				ring		= 1 << 1,
				round		= 1 << 2,
				generic		= 1 << 3,
				rectangle	= 1 << 4,
				profile_C	= 1 << 5,
				profile_I	= 1 << 6,
				profile_L	= 1 << 7,
				profile_T	= 1 << 8,
				profile_X	= 1 << 9,
				profile_Z	= 1 << 10,
				last
			};
		}
	}
}
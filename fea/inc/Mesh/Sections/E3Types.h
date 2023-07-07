#pragma once

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			enum class E3Type : unsigned
			{
				ub	= 1 << 0,
				uc	= 1 << 1,
				ubp	= 1 << 2,
				ipe	= 1 << 3,
				hea	= 1 << 4,
				heb	= 1 << 5,
				hem	= 1 << 6,
				chs	= 1 << 7,
				rhs	= 1 << 8,
				shs	= 1 << 9,
				last
			};
		}
	}
}
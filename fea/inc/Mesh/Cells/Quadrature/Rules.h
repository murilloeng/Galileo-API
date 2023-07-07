#pragma once

namespace fea
{
	namespace mesh
	{
		namespace cells
		{
			enum class rule : unsigned
			{
				lobatto		= 1 << 0,
				legendre	= 1 << 1,
				last
			};
		}
	}
}
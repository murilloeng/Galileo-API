#pragma once

namespace fea
{
	namespace mesh
	{
		namespace joints
		{
			enum class type : unsigned
			{
				hinge		= 1 << 0,
				pinned		= 1 << 1,
				rigid2		= 1 << 2,
				rigid3		= 1 << 3,
				revolute	= 1 << 4,
				spherical	= 1 << 5,
				last
			};
		}
	}
}
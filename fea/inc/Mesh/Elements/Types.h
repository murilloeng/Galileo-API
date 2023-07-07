#pragma once

namespace fea
{
	namespace mesh
	{
		namespace elements
		{
			enum class type : unsigned
			{
				//1D
				bar2		= 1 << 0,
				bar3		= 1 << 1,
				beam2C		= 1 << 2,
				beam2T		= 1 << 3,
				beam3C		= 1 << 4,
				beam3T		= 1 << 5,
				//2D
				plane		= 1 << 6,
				plate		= 1 << 7,
				shell		= 1 << 8,
				warping		= 1 << 9,
				membrane	= 1 << 10,
				//3D
				solid		= 1 << 11,
				//heat
				heat		= 1 << 12,
				//last
				last
			};
		}
	}
}
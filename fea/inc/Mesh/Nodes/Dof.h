#pragma once

namespace fea
{
	namespace mesh
	{
		namespace nodes
		{
			enum class dof : unsigned
			{
				warping_1			= 1 <<  6,
				warping_2			= 1 <<  7,
				warping_3			= 1 <<  8,
				rotation_1 			= 1 <<  3,
				rotation_2 			= 1 <<  4,
				rotation_3 			= 1 <<  5,
				temperature			= 1 <<  9,
				translation_1 		= 1 <<  0,
				translation_2 		= 1 <<  1,
				translation_3 		= 1 <<  2,
				slip_rotation_3		= 1 << 12,
				slip_rotation_2		= 1 << 13,
				slip_translation_1	= 1 << 10,
				slip_translation_2	= 1 << 11,
				last				= 1 << 14
			};
		}
	}
}
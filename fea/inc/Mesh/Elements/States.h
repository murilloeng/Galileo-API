#pragma once

//std
#include <cstdint>

namespace fea
{
	namespace mesh
	{
		namespace elements
		{
			enum class state : uint64_t
			{
				//point
				strain_11			= 1ULL << 0,
				strain_22			= 1ULL << 1,
				strain_33			= 1ULL << 2,
				strain_12			= 1ULL << 3,
				strain_13			= 1ULL << 4,
				strain_23			= 1ULL << 5,
				stress_11			= 1ULL << 6,
				stress_22			= 1ULL << 7,
				stress_33			= 1ULL << 8,
				stress_12			= 1ULL << 9,
				stress_13			= 1ULL << 10,
				stress_23			= 1ULL << 11,
				plastic_strain_11	= 1ULL << 12,
				plastic_strain_22	= 1ULL << 13,
				plastic_strain_33	= 1ULL << 14,
				plastic_strain_12	= 1ULL << 15,
				plastic_strain_13	= 1ULL << 16,
				plastic_strain_23	= 1ULL << 17,
				//section
				force_1				= 1ULL << 18,
				force_2				= 1ULL << 19,
				force_3				= 1ULL << 20,
				moment_1			= 1ULL << 21,
				moment_2			= 1ULL << 22,
				moment_3			= 1ULL << 23,
				bi_shear_1			= 1ULL << 24,
				bi_shear_2			= 1ULL << 25,
				bi_shear_3			= 1ULL << 26,
				bi_moment_1			= 1ULL << 27,
				bi_moment_2			= 1ULL << 28,
				bi_moment_3			= 1ULL << 29,
				//shell
				force_11			= 1ULL << 30,
				force_22			= 1ULL << 31,
				force_12			= 1ULL << 32,
				force_13			= 1ULL << 33,
				force_23			= 1ULL << 34,
				moment_11			= 1ULL << 35,
				moment_22			= 1ULL << 36,
				moment_12			= 1ULL << 37,
				//heat
				heat_flux			= 1ULL << 38,
				//last
				last
			};
		}
	}
}
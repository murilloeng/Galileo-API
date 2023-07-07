#pragma once

namespace fea
{
	namespace boundary
	{
		namespace time
		{
			enum class type : unsigned
			{
				custom,
				sine_wave,
				polynomial,
				last
			};
		}
	}
}
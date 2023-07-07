#pragma once

namespace fea
{
	namespace mesh
	{
		namespace elements
		{
			enum class warping : unsigned
			{
				vlasov,
				benscoter,
				saint_venant,
				last
			};
		}
	}
}
#pragma once

//fea
#include "Mesh/Elements/Element.h"

namespace fea
{
	namespace models
	{
		class Model;
	}

	namespace mesh
	{
		namespace elements
		{
			class Mechanic : public Element
			{
				friend class models::Model;

			protected:
				//constructors
				Mechanic(void);

				//destructor
				virtual ~Mechanic(void) override;

			public:
				//data
				static bool geometric(void);
				static bool geometric(bool);
				static bool inelastic(void);
				static bool inelastic(bool);

				//types
				virtual unsigned stress_set(void) const = 0;
				virtual points::type point(void) const override;

			protected:
				//data
				static bool m_geometric;
				static bool m_inelastic;
			};
		}
	}
}
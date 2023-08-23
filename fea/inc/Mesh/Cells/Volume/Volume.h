#pragma once

//fea
#include "fea/inc/Mesh/Cells/Cell.h"

namespace fea
{
	namespace mesh
	{
		namespace cells
		{
			class Volume : public Cell
			{
			protected:
				//constructors
				Volume(unsigned);

				//destructor
				virtual ~Volume(void) override;

			public:
				//topology
				virtual unsigned points(void) const override;
				virtual unsigned dimension(void) const override;

			protected:
				//draw
				virtual void draw(unsigned) const override;
			};
		}
	}
}
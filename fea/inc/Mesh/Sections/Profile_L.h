#pragma once

#include "Mesh/Sections/Profile.h"

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			class Profile_L : public Profile
			{
				friend class Section;

			protected:
				//constructors
				Profile_L(void);

				//destructor
				virtual ~Profile_L(void) override;

			public:
				//type
				virtual sections::type type(void) const override;

				//walls
				virtual unsigned walls(void) const override;
				virtual void wall(unsigned, double*, double*) const override;

				//analysis
				virtual void topology_1(void) const;
				virtual void topology_2(void) const;
				virtual void update(void) const override;
				virtual bool check_sizes(void) const override;
			};
		}
	}
}
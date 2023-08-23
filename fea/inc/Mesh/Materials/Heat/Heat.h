#pragma once

//fea
#include "fea/inc/Mesh/Materials/Material.h"

namespace fea
{
	namespace mesh
	{
		namespace materials
		{
			class Heat : public Material
			{
				friend class Material;

			protected:
				//constructors
				Heat(void);

				//destructor
				virtual ~Heat(void) override;

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
				//type
				virtual materials::type type(void) const override;

				//data
				virtual double capacity(void) const;
				virtual double capacity(double);
				virtual double condutivity(void) const;
				virtual double condutivity(double);

			protected:
				//data
				double m_capacity;
				double m_condutivity;
			};
		}
	}
}
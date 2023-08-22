#pragma once

//fea
#include "fea/inc/Mesh/Points/Point.h"

namespace fea
{
	namespace mesh
	{
		namespace points
		{
			enum class state : unsigned;
		}
	}
}

namespace fea
{
	namespace mesh
	{
		namespace points
		{
			class Mechanic : public Point
			{
			public:
				//constructors
				Mechanic(void);

				//destructor
				virtual ~Mechanic(void) override;

				//type
				virtual points::type type(void) const override;

				//analysis
				virtual void reset(void);
				virtual void prepare(unsigned, unsigned, unsigned);
				virtual void prepare(const elements::Element*) override;

				virtual void update(void) override;
				virtual void restore(void) override;

				//data
				state m_state_old;
				state m_state_new;

				unsigned m_stress_size;
				unsigned m_damage_size;
				unsigned m_stress_types;
				unsigned m_hardening_size;

				double* m_damage_old;
				double* m_damage_new;
				double* m_hardening_old;
				double* m_hardening_new;
				double* m_residual_stress;
				double* m_plastic_strain_old;
				double* m_plastic_strain_new;
			};
		}
	}
}
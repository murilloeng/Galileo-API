#pragma once

//fea
#include "Mesh/Elements/Mechanic/Frame/Frame.h"

namespace fea
{
	namespace models
	{
		class Model;
	}
	namespace boundary
	{
		namespace loads
		{
			class Line_Force;
		}
	}
}

namespace fea
{
	namespace mesh
	{
		namespace elements
		{
			class Bar : public Frame
			{
				friend class Element;
				friend class models::Model;

			protected:
				//contructors
				Bar(void);

				//destructors
				virtual ~Bar(void) override;

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

			public:
				//types
				virtual unsigned cell_set(void) const override;
				virtual unsigned load_set(void) const override;
				virtual uint64_t state_set(void) const override;
				virtual unsigned stress_set(void) const override;

				//data
				bool cable(bool);
				bool cable(void) const;

				static bool strain(bool);
				static bool strain(void);

				double length(double);
				double length(void) const;

				double residual_stress(double);
				double residual_stress(void) const;

			protected:
				//analysis
				virtual void apply(void) override;
				virtual void record(void) override;

				//formulation
				virtual double* load_line_force(double*, const boundary::loads::Line_Force*) const = 0;
				virtual double* reference_force(double*, const boundary::loads::Element*) const override;

				//data
				bool m_cable;
				static bool m_strain;
				double m_l, m_f, m_k, m_e, m_de, m_he, m_sr;
			};
		}
	}
}
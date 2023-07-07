#pragma once

namespace equations
{
	namespace beam3TL
	{
		void test(void);
		void setup(void);

		void energy(double*, const double*);
		void stiffness(double*, const double*);
		void internal_force(double*, const double*);

		void section_strains_s(double*, const double*);
		void section_strains_h(double*, const double*);

		void section_stiffness_s(double*, const double*);
		void section_stiffness_h(double*, const double*);

		void section_resultants_s(double*, const double*);
		void section_resultants_h(double*, const double*);

		void global_rotation(double*, double*, const double*);
	}
}
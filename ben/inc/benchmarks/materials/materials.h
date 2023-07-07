#pragma once

namespace tests
{
	namespace materials
	{
		namespace von_mises
		{
			void shell(void);
			void full_3D(void);
			void biaxial(void);
			void uniaxial(void);
			void shear_2D(void);
			void shear_3D(void);
			void plane_stress(void);
		}
		namespace drucker_prager
		{
			void shell(void);
			void full_3D(void);
			void biaxial(void);
			void uniaxial(void);
			void shear_2D(void);
			void shear_3D(void);
			void plane_stress(void);
		}
		namespace bresler_pister
		{
			void shell(void);
			void full_3D(void);
			void biaxial(void);
			void uniaxial(void);
			void shear_2D(void);
			void shear_3D(void);
			void plane_stress(void);
		}
	}
}
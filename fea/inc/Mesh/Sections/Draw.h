#pragma once

//std
#include <vector>

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			enum class mode : unsigned
			{
				mesh		= 1 << 0,
				torsion		= 1 << 1,
				shear_2		= 1 << 2,
				shear_3		= 1 << 3,
				topology	= 1 << 4,
				resultant	= 1 << 5,
				last
			};
			enum class what : unsigned
			{
				warping_profile			= 1 << 0,
				warping_stress_12		= 1 << 1,
				warping_stress_13		= 1 << 2,
				warping_shear_norm		= 1 << 3,
				resultant_stress_11		= 1 << 4,
				resultant_stress_12		= 1 << 5,
				resultant_stress_13		= 1 << 6,
				resultant_von_mises		= 1 << 7,
				resultant_shear_norm	= 1 << 8,
				last
			};
			struct Draw
			{
				//draw
				static unsigned types(void);

				//names
				static const char* name_mode(mode);
				static const char* name_what(what);
				static const char* name_mode(void);
				static const char* name_what(void);

				//data
				static mode m_mode;
				static what m_what;
				static bool m_axes;
				static bool m_edges;
				static bool m_faces;
				static bool m_shape;
				static bool m_print;
				static bool m_walls;
				static bool m_points;
				static bool m_rebars;
				static double m_scale;
				static double m_color_edge[4];
				static double m_color_face[4];
			};
		}
	}
}
#pragma once

namespace tests
{
	namespace beam
	{
		namespace state
		{
			void axial(void);
			void shear(void);
			void torsion(void);
			void bending(void);
			void rigid_body(void);
			void cantilever_tip_moment(void);
		}
		namespace modal
		{
			void clamped(void);
			void pinned_beam(void);
			void cantilever_beam(void);
		}
		namespace buckling
		{
			void frame(void);
			void column(void);
			void pinned_C(void);
		}
		namespace static_linear
		{
			void shear(void);
			void load_axial(void);
			void load_shear(void);
			void load_torsion(void);
			void load_bending(void);
			void cantilever_tip_force(void);
			void cantilever_tip_moment(void);
			void cantilever_uniform_force(void);
		}
		namespace static_nonlinear
		{
			namespace elastic
			{
				namespace plane
				{
					void shear(void);
					void frame_lee(void);
					void arc_circle(void);
					void frame_roof(void);
					void self_weight(void);
					void frame_square(void);
					void frame_square(void);
					void frame_portal(void);
					void arc_asymmetric(void);
					void frame_williams(void);
					void frame_buckling(void);
					void column_buckling(void);
					void von_mises_buckling(void);
					void arc_shallow_pinned(void);
					void arc_shallow_clamped(void);
					void cantilever_tip_force(void);
					void cantilever_tip_moment(void);
				}
				namespace space
				{
					void helix(void);
					void sequence(void);
					void dome_framed(void);
					void cantilever_C(void);
					void arc_cantilever(void);
					void hockling_cable(void);
					void deployable_ring(void);
					void frame_right_angle(void);
					void cantilever_right_angle(void);
				}
			}
			namespace inelastic
			{
				namespace steel
				{
					void point_test_3(void);
					void point_test_4(void);
					void column_buckling(void);
					void cantilever_axial(void);
					void cantilever_shear(void);
					void cantilever_bending(void);
					void propped_force_midspan(void);
					void propped_force_distributed(void);
				}
				namespace concrete
				{
					void frame_square(void);
					void column_buckling(void);
					void pinned_tip_moment(void);
					void cantilever_tip_force(void);
				}
			}
		}
		namespace dynamic_linear
		{
			namespace plane
			{
				void simple_beam(void);
			}
			namespace space
			{

			}
		}
		namespace dynamic_nonlinear
		{
			namespace elastic
			{
				namespace plane
				{
					void frame_lee(void);
					void arc_shallow(void);
					void blade_rotation(void);
					void von_mises_buckling(void);
					void cantilever_impulse(void);
					void cantilever_tip_force(void);
				}
				namespace space
				{
					void frame_lee(void);
					void arc_shallow(void);
					void cantilever_T(void);
					void cantilever_C(void);
					void cantilever_I(void);
					void cantilever_L(void);
					void cantilever_mixed(void);
				}
			}
		}
	}
}
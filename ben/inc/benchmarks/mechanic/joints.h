#pragma once

namespace tests
{
	namespace joint
	{
		namespace pinned
		{
			namespace static_linear
			{
				void frame(void);
			}
			namespace static_nonlinear
			{
				void shear(void);
				void torsion(void);
				void scissor(void);
				void cantilever(void);
				void multi_scissor(void);
			}
		}
		namespace hinge
		{
			namespace static_nonlinear
			{
				void axial(void);
				void shear(void);
				void torsion(void);
				void bending(void);
				void cantilever(void);
				void two_beams_axial(void);
				void two_beams_torsion(void);
			}
		}
		namespace rigid2
		{
			namespace state
			{
				void motion(void);
			}
			namespace static_nonlinear
			{
				void force(void);
				void stiffness(void);
			}
		}
		namespace rigid3
		{
			namespace state
			{
				void motion(void);
			}
			namespace static_nonlinear
			{
				void force(void);
				void moment(void);
				void beam_torsion(void);
				void beam_bending(void);
			}
		}
	}
}
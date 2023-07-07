#pragma once

namespace tests
{
	namespace solvers
	{
		namespace drift
		{
			void single(void);
			namespace beam
			{
				void deployable_ring(void);
			}
		}
		namespace newmark
		{
			void gravity(void);
			void vibration(void);
			void single_pendulum_2D(void);
			void single_pendulum_3D(void);
			void double_pendulum_2D(void);
		}
		namespace runge_kutta
		{
			void gravity(void);
			void vibration(void);
			void single_pendulum_2D(void);
			void single_pendulum_3D(void);
			void double_pendulum_2D(void);
		}
		namespace newton_raphson
		{
			void test_1D(void);
		}
	}
}
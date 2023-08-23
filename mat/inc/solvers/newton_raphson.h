#pragma once

//std
#include <functional>

//mat
#include "mat/inc/linear/matrix.h"
#include "mat/inc/linear/vector.h"
#include "mat/inc/solvers/strategies.h"

namespace mat
{
	namespace solvers
	{
		class newton_raphson
		{
		public:
			//constructors
			newton_raphson(void);

			//destructor
			virtual ~newton_raphson(void);

			//data
			void size(unsigned);
			void save(const char*) const;

		private:
			//solve
			void clear(void);
			void apply(void);
			void print(void);
			void setup(void);
			void update(void);
			void residue(void);
			void allocate(void);
			void predictor(void);
			void corrector(void);
			void load_predictor(void);
			void load_corrector(void);

		public:
			//solve
			void step(void);
			void solve(void);

			//data
			bool m_silent;
			bool m_equilibrium;
			strategy m_strategy;
			std::function<bool(void)> m_stop;
			std::function<void(void)> m_update;
			std::function<void(void)> m_restore;
			std::function<void(unsigned)> m_run_interface;
			std::function<void(double*, double*, const double*)> m_system;

			unsigned m_step, m_attempt, m_iteration, m_watch_dof;
			unsigned m_step_max, m_attempt_max, m_iteration_max, m_nd;
			double m_l_old, m_l_new, m_dl, m_dl0, m_ddl, m_tolerance, *m_data;

			matrix m_Kt;
			vector m_x_old, m_x_new, m_dx, m_dxt, m_ddx, m_ddxr, m_ddxt, m_r, m_fi, m_fe;
		};
	}
}
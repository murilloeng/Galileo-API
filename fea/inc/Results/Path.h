#pragma once

//std
#include <functional>

namespace fea
{
	namespace results
	{
		class Results;
	}
}

namespace fea
{
	namespace results
	{
		class Path
		{
			friend class Results;

		public:
			//constructors
			Path(void);

			//destructor
			virtual ~Path(void);

			//data
			unsigned steps(void) const;

			const double* dof(void) const;
			const double* step(void) const;
			const double* solver(void) const;

			const double** energy(void) const;
			const double** support(void) const;

			const double**** node(void) const;
			const double**** joint(void) const;
			const double**** element(void) const;

			static const Results* results(void);

			std::function<void(const char*)> text(void) const;
			std::function<void(const char*)> text(std::function<void(const char*)>);

			//data
			bool read(void);
			void clear(void);

		protected:
			//data
			void allocate(void);

			//read
			bool read_node(void);
			bool read_joint(void);
			bool read_solver(void);
			bool read_energy(void);
			bool read_element(void);
			bool read_support(void);

			//clear
			void clear_nodes(void);
			void clear_sizes(void);
			void clear_joints(void);
			void clear_solver(void);
			void clear_energies(void);
			void clear_elements(void);
			void clear_supports(void);

			//allocate
			void setup(void);
			void allocate_nodes(void);
			void allocate_sizes(void);
			void allocate_joints(void);
			void allocate_solver(void);
			void allocate_energies(void);
			void allocate_elements(void);
			void allocate_supports(void);

		private:
			//data
			unsigned m_steps;

			double* m_dof;
			double* m_step;
			double* m_solver;

			double** m_energy;
			double** m_supports;

			double*** m_nodes[3];

			double**** m_joints;
			double**** m_elements;

			std::function<void(const char*)> m_text;
			static const results::Results* m_results;
			unsigned m_nn, m_ne, m_nj, m_ns, *m_nd, *m_njt, *m_njn, *m_net, *m_nen;
		};
	}
}
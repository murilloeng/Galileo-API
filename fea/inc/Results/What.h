#pragma once

//std
#include <cstdio>

namespace fea
{
	namespace results
	{
		class What
		{
		public:
			//constructors
			What(void);

			//destructor
			virtual ~What(void);

			//serialization
			void load(FILE*);
			void save(FILE*) const;

			//data
			bool solver(bool);
			bool states(bool);
			bool joints(bool);
			bool energies(bool);
			bool elements(bool);
			bool supports(bool);
			bool velocities(bool);
			bool accelerations(bool);

			bool solver(void) const;
			bool states(void) const;
			bool joints(void) const;
			bool energies(void) const;
			bool elements(void) const;
			bool supports(void) const;
			bool velocities(void) const;
			bool accelerations(void) const;

		private:
			//data
			bool m_solver;
			bool m_states;
			bool m_joints;
			bool m_energies;
			bool m_elements;
			bool m_supports;
			bool m_velocities;
			bool m_accelerations;
		};
	}
}
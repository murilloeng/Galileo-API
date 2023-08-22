#pragma once

//std
#include <cstdint>

namespace fea
{
	namespace models
	{
		class Model;
	}
	namespace results
	{
		class Results;
		class Limit_Dof;
		class Limit_State;
		enum class type : unsigned;
	}
}

namespace fea
{
	namespace results
	{
		class Canvas
		{
			friend class Results;

		public:
			//constructors
			Canvas(void);

			//destructor
			virtual ~Canvas(void);

			//data
			virtual unsigned steps(void) const;
			virtual unsigned dof_set(void) const;
			virtual uint64_t state_set(void) const;

			virtual unsigned step(unsigned);
			virtual unsigned dof_index(unsigned);
			virtual unsigned type_index(unsigned);
			virtual unsigned state_index(unsigned);

			virtual unsigned step(void) const;
			virtual unsigned dof_index(void) const;
			virtual unsigned type_index(void) const;
			virtual unsigned state_index(void) const;

			virtual results::type type(void) const;
			virtual results::type type(results::type);

			virtual double dof(unsigned) const;
			virtual double state(unsigned, unsigned) const;

			virtual const double* position(unsigned) const;
			virtual void rotation(double*, unsigned) const;
			virtual double position(unsigned, unsigned) const;
			virtual double rotation(unsigned, unsigned) const;

			virtual const double* path_position(unsigned, unsigned) const;

			virtual double* color_node(double*, unsigned) const;
			virtual double* color_element(double*, unsigned, unsigned) const;

			virtual const Limit_Dof& limit_dof(void) const;
			virtual const Limit_State& limit_state(void) const;

			static const Results* results(void);

			//data
			virtual void read(void);
			virtual void clear(void);

		protected:
			//setup
			virtual void apply_dof(void);
			virtual void apply_state(void);

			//read
			virtual void read_dof(void);
			virtual void read_states(void);
			virtual void read_positions(void);
			virtual void read_rotations(void);

			//clear
			virtual void clear_dof(void);
			virtual void clear_states(void);
			virtual void clear_positions(void);
			virtual void clear_rotations(void);

			//bound
			virtual void bound_dof(void);
			virtual void bound_states(void);

			//allocate
			virtual void allocate_dof(void);
			virtual void allocate_states(void);
			virtual void allocate_positions(void);
			virtual void allocate_rotations(void);

		private:
			//data
			unsigned m_nn;
			unsigned m_ne;
			unsigned m_steps;
			unsigned m_dof_set;
			uint64_t m_state_set;

			double*** m_dof[3];
			double**** m_states;
			double*** m_positions;
			double*** m_rotations;

			unsigned m_step;
			unsigned m_dof_index;
			unsigned m_type_index;
			unsigned m_state_index;

			results::type m_type;
			Limit_Dof* m_limit_dof[3];
			Limit_State* m_limit_states;

			static const results::Results* m_results;
		};
	}
}
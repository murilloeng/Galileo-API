#pragma once

//std
#include <cstdio>

namespace fea
{
	//forward declarations
	namespace models
	{
		class Model;
	}
	namespace analysis
	{
		class Assembler;
		namespace solvers
		{
			class Solver;
			enum class type : unsigned;
		}
	}

	namespace analysis
	{
		class Analysis
		{
			friend class models::Model;
			friend class analysis::Assembler;

		protected:
			//constructors
			Analysis(void);

			//destructor
			virtual ~Analysis(void);

			//setup
			void setup(void);

			//serialization
			void load(FILE*);
			void save(FILE*) const;

			bool load_results(void);
			bool save_results(void) const;

		public:
			//data
			bool solved(void) const;
			Assembler* assembler(void) const;
			solvers::Solver* solver(void) const;
			solvers::Solver* solver(solvers::type);
			
			static models::Model* model(void);

			//solve
			bool solve(bool = false);

		protected:
			//analysis
			bool check(void) const;
			void finish(void) const;
			void prepare(void) const;
			void apply_dof(void) const;

			//data
			bool m_solved;
			Assembler* m_assembler;
			solvers::Solver* m_solver;
			static models::Model* m_model;
		};
	}
}
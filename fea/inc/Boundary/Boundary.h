#pragma once

//std
#include <cstdio>
#include <vector>
#include <climits>

namespace fea
{
	namespace models
	{
		class Model;
	}
	namespace mesh
	{
		class Mesh;
		namespace nodes
		{
			enum class dof : unsigned;
		}
	}
	namespace analysis
	{
		class Analysis;
		class Assembler;
	}
	namespace boundary
	{
		class Initial;
		class Support;
		class Dependency;
		namespace time
		{
			class Time;
			enum class type : unsigned;
		}
		namespace loads
		{
			class Load_Set;
			class Load_Case;
			enum class type : unsigned;
		}
	}
}

namespace fea
{
	namespace boundary
	{
		class Boundary
		{
			friend class mesh::Mesh;
			friend class models::Model;
			friend class analysis::Analysis;
			friend class analysis::Assembler;

		protected:
			//constructors
			Boundary(void);

			//destructor
			virtual ~Boundary(void);

			//serialization
			void load(FILE*);
			void load_times(FILE*);
			void load_initials(FILE*);
			void load_supports(FILE*);
			void load_load_sets(FILE*);
			void load_load_cases(FILE*);
			void load_dependencies(FILE*);

			void save(FILE*) const;
			void save_times(FILE*) const;
			void save_initials(FILE*) const;
			void save_supports(FILE*) const;
			void save_load_sets(FILE*) const;
			void save_load_cases(FILE*) const;
			void save_dependencies(FILE*) const;

			bool load_results(void);
			bool save_results(void) const;

		public:
			//model
			models::Model* model(void) const;

			//data
			time::Time* time(unsigned) const;
			Initial* initial(unsigned) const;
			Support* support(unsigned) const;
			Dependency* dependency(unsigned) const;
			loads::Load_Set* load_set(unsigned) const;
			loads::Load_Case* load_case(unsigned) const;

			//lists
			const std::vector<time::Time*>& times(void) const;
			const std::vector<Initial*>& initials(void) const;
			const std::vector<Support*>& supports(void) const;
			const std::vector<Dependency*>& dependencies(void) const;
			const std::vector<loads::Load_Set*>& load_sets(void) const;
			const std::vector<loads::Load_Case*>& load_cases(void) const;

			//add
			time::Time* add_time(time::type);

			Initial* add_initial(unsigned, mesh::nodes::dof, double, double);

			Support* add_support(unsigned, mesh::nodes::dof);
			Support* add_support(unsigned, mesh::nodes::dof, unsigned);
			Support* add_support(unsigned, mesh::nodes::dof, double, double, double);

			loads::Load_Set* add_load_set(const char* = "");

			loads::Load_Case* add_load_case(const char* = "");
			loads::Load_Case* add_load_case(unsigned, mesh::nodes::dof, double = 1, unsigned = UINT_MAX);
			loads::Load_Case* add_load_case(loads::type, std::vector<unsigned> = {}, double = 1, unsigned = UINT_MAX);

			loads::Load_Case* add_self_weight(const char*, const double*, double = 9.81);
			loads::Load_Case* add_self_weight(const char*, mesh::nodes::dof, double = 9.81);

			Dependency* add_dependency(unsigned, mesh::nodes::dof, unsigned, mesh::nodes::dof, double = 1, double = 0);
			Dependency* add_dependency(unsigned, mesh::nodes::dof, std::vector<unsigned>, std::vector<mesh::nodes::dof>);

			//remove
			void remove_time(unsigned);
			void remove_node(unsigned);
			void remove_element(unsigned);
			void remove_initial(unsigned);
			void remove_support(unsigned);
			void remove_load_set(unsigned);
			void remove_load_case(unsigned);
			void remove_dependency(unsigned);

		private:
			//analysis
			bool check(void) const;
			void prepare(void) const;
			void apply_dof(void) const;

			void mesh_union(void);
			void mesh_split(void);

			void record(void) const;
			void finish(void) const;

			//draw
			void draw_loads(void) const;
			void draw_supports(void) const;

			//merge
			void merge_nodes(unsigned, unsigned) const;

		protected:
			//data
			static models::Model* m_model;

			std::vector<time::Time*> m_times;
			std::vector<Initial*> m_initials;
			std::vector<Support*> m_supports;
			std::vector<Dependency*> m_dependencies;
			std::vector<loads::Load_Set*> m_load_sets;
			std::vector<loads::Load_Case*> m_load_cases;
		};
	}
}
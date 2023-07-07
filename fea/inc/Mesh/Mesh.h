#pragma once

//std
#include <cstdio>
#include <vector>
#include <climits>

namespace mat
{
	class vec3;
}
namespace fea
{
	namespace models
	{
		class Model;
	}
	namespace mesh
	{
		namespace nodes
		{
			class Node;
			enum class dof : unsigned;
		}
		namespace cells
		{
			class Cell;
			enum class type : unsigned;
		}
		namespace joints
		{
			class Joint;
			enum class type : unsigned;
		}
		namespace materials
		{
			class Material;
			enum class type : unsigned;
		}
		namespace sections
		{
			class Section;
			enum class type : unsigned;
		}
		namespace elements
		{
			class Element;
			enum class type : unsigned;
		}
	}
	namespace boundary
	{
		class Boundary;
	}
	namespace analysis
	{
		class Assembler;
	}
}

namespace fea
{
	namespace mesh
	{
		class Mesh
		{
			friend class models::Model;
			friend class boundary::Boundary;
			friend class analysis::Assembler;

		protected:
			//constructors
			Mesh(void);

			//destructor
			virtual ~Mesh(void);

			//serialization
			virtual void load(FILE*);
			virtual void load_nodes(FILE*);
			virtual void load_cells(FILE*);
			virtual void load_joints(FILE*);
			virtual void load_sections(FILE*);
			virtual void load_elements(FILE*);
			virtual void load_materials(FILE*);

			virtual void save(FILE*) const;
			virtual void save_nodes(FILE*) const;
			virtual void save_cells(FILE*) const;
			virtual void save_joints(FILE*) const;
			virtual void save_sections(FILE*) const;
			virtual void save_elements(FILE*) const;
			virtual void save_materials(FILE*) const;

			virtual bool load_results(void);
			virtual bool save_results(void) const;

		public:
			//model
			virtual models::Model* model(void) const;

			//data
			virtual nodes::Node* node(unsigned) const;
			virtual cells::Cell* cell(unsigned) const;
			virtual joints::Joint* joint(unsigned) const;
			virtual sections::Section* section(unsigned) const;
			virtual elements::Element* element(unsigned) const;
			virtual materials::Material* material(unsigned) const;

			//bound
			virtual void bounds(double*) const;

			virtual double size(void) const;
			static double size(const double*);

			//step
			virtual unsigned step(void) const;

			//state
			virtual double max(nodes::dof) const;
			virtual double min(nodes::dof) const;
			virtual double mean(nodes::dof) const;

			//lists
			virtual const std::vector<nodes::Node*>& nodes(void) const;
			virtual const std::vector<cells::Cell*>& cells(void) const;
			virtual const std::vector<joints::Joint*>& joints(void) const;
			virtual const std::vector<sections::Section*>& sections(void) const;
			virtual const std::vector<elements::Element*>& elements(void) const;
			virtual const std::vector<materials::Material*>& materials(void) const;

			//merge
			virtual void merge(std::vector<unsigned>, std::vector<unsigned>);
			virtual void merge_query(std::vector<unsigned>&, std::vector<unsigned>&) const;

			//add
			virtual nodes::Node* add_node(const double*);
			virtual nodes::Node* add_node(double, double, double);

			virtual cells::Cell* add_cell(cells::type);

			virtual joints::Joint* add_joint(joints::type, std::vector<unsigned> = {});

			virtual sections::Section* add_section(sections::type);

			virtual elements::Element* add_element(const elements::Element*);
			virtual elements::Element* add_element(elements::type, std::vector<unsigned> = {}, unsigned = 0, unsigned = 0);

			virtual materials::Material* add_material(materials::type);

			//remove
			virtual void remove_node(unsigned);
			virtual void remove_cell(unsigned);
			virtual void remove_joint(unsigned);
			virtual void remove_section(unsigned);
			virtual void remove_element(unsigned);
			virtual void remove_material(unsigned);

			virtual void remove_nodes(std::vector<unsigned>);
			virtual void remove_cells(std::vector<unsigned>);
			virtual void remove_joints(std::vector<unsigned>);
			virtual void remove_sections(std::vector<unsigned>);
			virtual void remove_elements(std::vector<unsigned>);
			virtual void remove_materials(std::vector<unsigned>);

			//rotation
			virtual double* rotation_grad(const joints::Joint*, double*) const;
			virtual double* rotation_rows(const joints::Joint*, double*) const;
			virtual double* rotation_cols(const joints::Joint*, double*) const;
			virtual double* rotation_diag(const joints::Joint*, double*, const double*) const;

			virtual double* rotation_grad(const elements::Element*, double*) const;
			virtual double* rotation_rows(const elements::Element*, double*) const;
			virtual double* rotation_cols(const elements::Element*, double*) const;
			virtual double* rotation_diag(const elements::Element*, double*, const double*) const;

			//transform
			virtual void move_nodes(const std::vector<unsigned>&, const mat::vec3&, bool = false);
			virtual void scale_nodes(const std::vector<unsigned>&, const mat::vec3&, double, bool = false);
			virtual void rotate_nodes(const std::vector<unsigned>&, const mat::vec3&, const mat::vec3&, bool = false);
			virtual void mirror_nodes(const std::vector<unsigned>&, const mat::vec3&, const mat::vec3&, bool = false);

			virtual void move_elements(const std::vector<unsigned>&, const mat::vec3&, bool = false);
			virtual void scale_elements(const std::vector<unsigned>&, const mat::vec3&, double, bool = false);
			virtual void rotate_elements(const std::vector<unsigned>&, const mat::vec3&, const mat::vec3&, bool = false);
			virtual void mirror_elements(const std::vector<unsigned>&, const mat::vec3&, const mat::vec3&, bool = false);

		protected:
			//analysis
			virtual bool check(void);
			virtual void prepare(void);
			virtual void finish(void);
			virtual void record(void);
			virtual void update(void);
			virtual void restore(void);
			virtual void apply_dof(void);
			virtual void energy_sum(void);

			//draw
			virtual void draw_nodes(void) const;
			virtual void draw_joints(void) const;
			virtual void draw_numbers(void) const;
			virtual void draw_elements(void) const;

			//transform
			virtual void elements_set(std::vector<unsigned>&, const std::vector<unsigned>&) const;
			virtual void elements_copy(const std::vector<unsigned>&, const std::vector<unsigned>&, unsigned);

			//data
			static models::Model* m_model;

			std::vector<nodes::Node*> m_nodes;
			std::vector<cells::Cell*> m_cells;
			std::vector<joints::Joint*> m_joints;
			std::vector<sections::Section*> m_sections;
			std::vector<elements::Element*> m_elements;
			std::vector<materials::Material*> m_materials;
		};
	}
}
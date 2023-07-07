//std
#include <cmath>

//fea
#include "Model/Model.h"

#include "Mesh/Mesh.h"
#include "Mesh/Nodes/Dof.h"
#include "Mesh/Cells/Types.h"
#include "Mesh/Sections/Types.h"
#include "Mesh/Elements/Types.h"
#include "Mesh/Materials/Types.h"
#include "Mesh/Elements/Mechanic/Mechanic.h"

#include "Boundary/Boundary.h"

#include "Analysis/Analysis.h"
#include "Analysis/Solvers/Types.h"
#include "Analysis/Solvers/Modal.h"

//ben
#include "benchmarks/mechanic/bar.h"

//modal
void tests::bar::modal::von_mises_truss_2D(void)
{
	//parameters
	const double l = 1.00e+0;
	const double a = 30 * M_PI / 180;

	const double b = l * cos(a);
	const double h = l * sin(a);

	//model
	fea::models::Model model("von mises truss 2D", "benchmarks/bar/modal");

	//nodes
	model.mesh()->add_node( 0, h, 0);
	model.mesh()->add_node(+b, 0, 0);
	model.mesh()->add_node(-b, 0, 0);

	//cells
	model.mesh()->add_cell(fea::mesh::cells::type::bar);

	//sections
	model.mesh()->add_section(fea::mesh::sections::type::rectangle);

	//materials
	model.mesh()->add_material(fea::mesh::materials::type::steel);

	//elements
	fea::mesh::elements::Mechanic::geometric(true);
	model.mesh()->add_element(fea::mesh::elements::type::bar2, {1, 0});
	model.mesh()->add_element(fea::mesh::elements::type::bar2, {2, 0});

	//supports
	model.boundary()->add_support(0, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(1, fea::mesh::nodes::dof::translation_3);
	model.boundary()->add_support(2, fea::mesh::nodes::dof::translation_1);
	model.boundary()->add_support(2, fea::mesh::nodes::dof::translation_2);
	model.boundary()->add_support(2, fea::mesh::nodes::dof::translation_3);

	//initials
	model.boundary()->add_initial(0, fea::mesh::nodes::dof::translation_1, 0, 0);
	model.boundary()->add_initial(0, fea::mesh::nodes::dof::translation_2, 0, 0);

	//loads
	model.boundary()->add_load_case();

	//solver
	model.analysis()->solver(fea::analysis::solvers::type::modal);
	model.analysis()->solver()->watch_dof(0, fea::mesh::nodes::dof::translation_2);

	//solve
	model.analysis()->solve();

	//save
	model.save();
}
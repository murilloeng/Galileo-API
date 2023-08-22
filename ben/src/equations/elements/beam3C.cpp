// //std
// #include <ctime>

// //mat
// #include "mat/inc/misc/util.h"
// #include "mat/inc/misc/stress.h"
// #include "mat/inc/linear/vector.h"

// //fea
// #include "fea/inc/Model/Model.h"

// #include "fea/inc/Mesh/Mesh.h"
// #include "fea/inc/Mesh/Cells/Types.h"
// #include "fea/inc/Mesh/Sections/Mesh.h"
// #include "fea/inc/Mesh/Sections/Fiber.h"
// #include "fea/inc/Mesh/Sections/Types.h"
// #include "fea/inc/Mesh/Elements/Types.h"
// #include "fea/inc/Mesh/Cells/Line/Beam.h"
// #include "fea/inc/Mesh/Materials/Types.h"
// #include "fea/inc/Mesh/Sections/Rectangle.h"
// #include "fea/inc/Mesh/Elements/Mechanic/Mechanic.h"
// #include "fea/inc/Mesh/Elements/Mechanic/Frame/Beam3C.h"
// #include "fea/inc/Mesh/Elements/Mechanic/Frame/Warping.h"
// #include "fea/inc/Mesh/Materials/Mechanic/Steel.h"

// //ben
// #include "ben/inc/equations/elements/beam3C.h"

// //data
// static mat::vector fl;
// static mat::matrix Kh, Kl, *Bs;
// static const unsigned test = 3;
// static fea::models::Model model;
// static const double L = 1.00e+02;
// static const double w = 5.00e-01;
// static const double h = 1.00e+01;
// static const double E = 2.10e+06;
// static const double v = 3.33e-01;
// static unsigned nr, nc, ns, nd, nf, np;
// static const fea::mesh::elements::warping warping = fea::mesh::elements::warping::saint_venant;

// //cell
// static void cell_gradient(unsigned i)
// {
// 	//shape
// 	double s;
// 	Bs[i].resize(ns, nd);
// 	mat::matrix Bc(ns, nf);
// 	model.mesh()->cell(0)->point(&s, i);
// 	((fea::mesh::cells::Beam*) model.mesh()->cell(0))->strain(Bc, model.mesh()->element(0), s);
// 	//warping
// 	if(warping == fea::mesh::elements::warping::saint_venant)
// 	{
// 		Bs[i].span(0, 0, ns, 6) = Bc.span(0, 6, ns, 6);
// 	}
// 	else
// 	{
// 		Bs[i].span(0, 0, ns, 6) = Bc.span(0,  9, ns, 6);
// 		Bs[i].span(0, 6, ns, 3) = Bc.span(0,  6, ns, 3);
// 		Bs[i].span(0, 9, ns, 3) = Bc.span(0, 15, ns, 3);
// 	}
// }
// static void cell_strain_nonlinear(mat::vector& eh, const mat::vector& es)
// {
// 	//linear
// 	eh.zeros();
// 	eh.span(0, 0, ns, 1) = es;
// 	//2nd order curvature
// 	eh(ns + 0) = es(4) * es(5);
// 	eh(ns + 1) = (es(3) * es(3) + es(4) * es(4)) / 2;
// 	eh(ns + 2) = (es(3) * es(3) + es(5) * es(5)) / 2;
// 	//strech and bending curvature
// 	eh(4) += es(0) * es(4) - es(1) * es(3);
// 	eh(5) += es(0) * es(5) - es(2) * es(3);
// 	eh(0) += (es(0) * es(0) + es(1) * es(1) + es(2) * es(2)) / 2;
// }
// static void cell_gradient_nonlinear(mat::matrix& Bh, const mat::vector& es)
// {
// 	//linear
// 	Bh.zeros();
// 	for(unsigned i = 0; i < ns; i++)
// 	{
// 		Bh(i, i) = 1;
// 	}
// 	//nonlinear
// 	Bh(0, 0) += es(0);
// 	Bh(0, 1) += es(1);
// 	Bh(0, 2) += es(2);
// 	Bh(4, 0) += es(4);
// 	Bh(4, 1) -= es(3);
// 	Bh(4, 3) -= es(1);
// 	Bh(4, 4) += es(0);
// 	Bh(5, 0) += es(5);
// 	Bh(5, 2) -= es(3);
// 	Bh(5, 3) -= es(2);
// 	Bh(5, 5) += es(0);
// 	Bh(ns + 0, 4) = es(5);
// 	Bh(ns + 0, 5) = es(4);
// 	Bh(ns + 1, 3) = es(3);
// 	Bh(ns + 1, 4) = es(4);
// 	Bh(ns + 2, 3) = es(3);
// 	Bh(ns + 2, 5) = es(5);
// }
// static void cell_hessian_nonlinear(mat::matrix& Kg, const mat::vector& es)
// {
// 	//data
// 	Kg.zeros();
// 	mat::vector eh(ns + 3);
// 	cell_strain_nonlinear(eh, es);
// 	const mat::vector sh = Kh * eh;
// 	//hessian
// 	Kg(4, 4) = sh(ns + 1);
// 	Kg(5, 5) = sh(ns + 2);
// 	Kg(0, 4) = Kg(4, 0) = +sh(4);
// 	Kg(0, 5) = Kg(5, 0) = +sh(5);
// 	Kg(3, 1) = Kg(1, 3) = -sh(4);
// 	Kg(3, 2) = Kg(2, 3) = -sh(5);
// 	Kg(5, 4) = Kg(4, 5) = +sh(ns + 0);
// 	Kg(3, 3) = sh(ns + 1) + sh(ns + 2);
// 	Kg(0, 0) = Kg(1, 1) = Kg(2, 2) = sh(0);
// }

// static void local_elastic(void)
// {

// }

// //section
// static void section_gradient(mat::matrix& H, const fea::mesh::sections::Fiber& fiber, unsigned index)
// {
// 	//data
// 	const double wt = fiber.warping(index, 0);
// 	const double n2 = fiber.warping(index, 1);
// 	const double n3 = fiber.warping(index, 2);
// 	const double x2 = fiber.position(index, 0);
// 	const double x3 = fiber.position(index, 1);
// 	const double dwt2 = fiber.gradient(index, 0, 0);
// 	const double dwt3 = fiber.gradient(index, 0, 1);
// 	const double dn22 = fiber.gradient(index, 1, 0);
// 	const double dn23 = fiber.gradient(index, 1, 1);
// 	const double dn32 = fiber.gradient(index, 2, 0);
// 	const double dn33 = fiber.gradient(index, 2, 1);
// 	const double c2 = model.mesh()->section(0)->shear_center(0);
// 	const double c3 = model.mesh()->section(0)->shear_center(1);
// 	//gradient
// 	H.zeros();
// 	H[0 + 3 * 0] = 1;
// 	H[0 + 3 * 4] = +x3;
// 	H[0 + 3 * 5] = -x2;
// 	H[1 + 3 * 1] = dn22;
// 	H[1 + 3 * 2] = dn32;
// 	H[2 + 3 * 1] = dn23;
// 	H[2 + 3 * 2] = dn33;
// 	H[1 + 3 * 3] = dwt2 - x3 + c3;
// 	H[2 + 3 * 3] = dwt3 + x2 - c2;
// 	//higher-order terms
// 	H[0 + 3 * (ns + 0)] = -x2 * x3;
// 	H[0 + 3 * (ns + 1)] = +x3 * x3;
// 	H[0 + 3 * (ns + 2)] = +x2 * x2;
// 	//bi-moment terms
// 	if(warping != fea::mesh::elements::warping::saint_venant)
// 	{
// 		H[0 + 3 * 6] = wt;
// 		H[0 + 3 * 7] = n2 - x2;
// 		H[0 + 3 * 8] = n3 - x3;
// 	}
// 	//bi-shear terms
// 	if(warping == fea::mesh::elements::warping::benscoter)
// 	{
// 		H[1 + 3 *  9] = dwt2;
// 		H[2 + 3 *  9] = dwt3;
// 		H[2 + 3 * 10] = dn23;
// 		H[1 + 3 * 11] = dn32;
// 		H[1 + 3 * 10] = dn22 - 1;
// 		H[2 + 3 * 11] = dn33 - 1;
// 	}
// }

// //prepare
// static void prepare(void)
// {
// 	//data
// 	const unsigned st = 
// 		unsigned(mat::stress::sxx) | 
// 		unsigned(mat::stress::sxy) | 
// 		unsigned(mat::stress::sxz);
// 	//cell
// 	ns = 
// 		warping == fea::mesh::elements::warping::vlasov ? 9 : 
// 		warping == fea::mesh::elements::warping::benscoter ? 12 : 
// 		warping == fea::mesh::elements::warping::saint_venant ? 6 : 0;
// 	np = warping == fea::mesh::elements::warping::saint_venant ? 2 : 5;
// 	nd = warping == fea::mesh::elements::warping::saint_venant ? 6 : 12;
// 	nf = warping == fea::mesh::elements::warping::saint_venant ? 12 : 18;
// 	//gradient
// 	Bs = new mat::matrix[np];
// 	for(unsigned i = 0; i < np; i++)
// 	{
// 		cell_gradient(i);
// 	}
// 	//stiffness
// 	fl.resize(nd);
// 	Kl.resize(nd, nd);
// 	mat::matrix C(3, 3), H(3, ns + 3);
// 	Kh.resize(ns + 3, ns + 3).zeros();
// 	((fea::mesh::materials::Mechanic*) model.mesh()->material(0))->elastic_stiffness(C.mem(), st);
// 	for(const fea::mesh::sections::Fiber& fiber : model.mesh()->section(0)->mesh_local()->fibers())
// 	{
// 		for(unsigned i = 0; i < model.mesh()->section(0)->cell().points(); i++)
// 		{
// 			section_gradient(H, fiber, i);
// 			const double w = fiber.weight(i);
// 			const double d = fiber.jacobian(i);
// 			Kh += w * d * H.transpose() * C * H;
// 		}
// 	}
// }
// static void finish(void)
// {
// 	delete[] Bs;
// }

// //setup
// static void setup(void)
// {
// 	//nodes
// 	model.mesh()->add_node(0, 0, 0);
// 	model.mesh()->add_node(L, 0, 0);

// 	//section
// 	model.mesh()->add_section(fea::mesh::sections::type::rectangle);
// 	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->width(w);
// 	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->height(h);
// 	((fea::mesh::sections::Rectangle*) model.mesh()->section(0))->size(w / 2);

// 	//cell
// 	model.mesh()->add_cell(fea::mesh::cells::type::beam);

// 	//materials
// 	model.mesh()->add_material(fea::mesh::materials::type::steel);
// 	((fea::mesh::materials::Steel*) model.mesh()->material(0))->poisson_ratio(v);
// 	((fea::mesh::materials::Steel*) model.mesh()->material(0))->elastic_modulus(E);

// 	//elements
// 	model.mesh()->add_element(fea::mesh::elements::type::beam3C, {0, 1});

// 	//setup
// 	model.mesh()->section(0)->prepare();
// }

// //tests
// void fun(double* f, const double* x)
// {
// 	//eh -> sh
// 	if(test == 0)
// 	{
// 		mat::vector(f, ns + 3) = Kh * mat::vector(x, ns + 3);
// 	}
// 	//es -> eh
// 	else if(test == 1)
// 	{
// 		mat::vector eh(f, ns + 3);
// 		cell_strain_nonlinear(eh, mat::vector(x, ns));
// 	}
// 	//es -> ss
// 	else if(test == 2)
// 	{
// 		mat::vector eh(ns + 3);
// 		mat::matrix Bh(ns + 3, ns);
// 		cell_strain_nonlinear(eh, mat::vector(x, ns));
// 		cell_gradient_nonlinear(Bh, mat::vector(x, ns));
// 		mat::vector(f, ns) = Bh.transpose() * Kh * eh;
// 	}
// 	//dp -> U
// 	else if(test == 3)
// 	{
// 		f[0] = 0;
// 		double s, w;
// 		mat::vector dp(x, nd), es(ns), eh(ns + 3);
// 		for(unsigned i = 0; i < np; i++)
// 		{
// 			es = Bs[i] * dp;
// 			cell_strain_nonlinear(eh, es);
// 			w = model.mesh()->cell(0)->point(&s, i);
// 			f[0] += w * L / 2 * eh.inner(Kh * eh) / 2;
// 		}
// 	}
// 	//dp -> fp
// 	else if(test == 4)
// 	{
// 		double s, w;
// 		mat::vector(f, nd).zeros();
// 		mat::matrix Bh(ns + 3, ns);
// 		mat::vector dp(x, nd), es(ns), eh(ns + 3);
// 		for(unsigned i = 0; i < np; i++)
// 		{
// 			es = Bs[i] * dp;
// 			cell_strain_nonlinear(eh, es);
// 			w = model.mesh()->cell(0)->point(&s, i);
// 			mat::vector(f, nd) += w * L / 2 * Bs[i].transpose() * Bh.transpose() * Kh * eh;
// 		}
// 	}
// }
// void dev(double* K, const double* x)
// {
// 	//eh -> sh
// 	if(test == 0)
// 	{
// 		mat::matrix(K, ns + 3, ns + 3) = Kh;
// 	}
// 	//es -> eh
// 	else if(test == 1)
// 	{
// 		mat::matrix Bh(K, ns + 3, ns);
// 		cell_gradient_nonlinear(Bh, mat::vector(x, ns));
// 	}
// 	//es -> ss
// 	else if(test == 2)
// 	{
// 		mat::vector eh(ns + 3);
// 		mat::matrix Bh(ns + 3, ns), Kg(ns, ns);
// 		cell_strain_nonlinear(eh, mat::vector(x, ns));
// 		cell_hessian_nonlinear(Kg, mat::vector(x, ns));
// 		cell_gradient_nonlinear(Bh, mat::vector(x, ns));
// 		mat::matrix(K, ns, ns) = Bh.transpose() * Kh * Bh + Kg;
// 	}
// 	//dp -> U
// 	else if(test == 3)
// 	{
// 		double s, w;
// 		mat::vector(K, nd).zeros();
// 		mat::matrix Bh(ns + 3, ns);
// 		mat::vector dp(x, nd), es(ns), eh(ns + 3);
// 		for(unsigned i = 0; i < np; i++)
// 		{
// 			es = Bs[i] * dp;
// 			cell_strain_nonlinear(eh, es);
// 			cell_gradient_nonlinear(Bh, es);
// 			w = model.mesh()->cell(0)->point(&s, i);
// 			mat::vector(K, nd) += w * L / 2 * Bs[i].transpose() * Bh.transpose() * Kh * eh;
// 		}
// 	}
// 	//dp -> fp
// 	else if(test == 4)
// 	{
// 		double s, w;
// 		mat::matrix(K, nd, nd).zeros();
// 		mat::matrix Bh(ns + 3, ns), Kg(ns, ns);
// 		mat::vector dp(x, nd), es(ns), eh(ns + 3);
// 		for(unsigned i = 0; i < np; i++)
// 		{
// 			es = Bs[i] * dp;
// 			cell_strain_nonlinear(eh, es);
// 			cell_hessian_nonlinear(Kg, es);
// 			cell_gradient_nonlinear(Bh, es);
// 			w = model.mesh()->cell(0)->point(&s, i);
// 			mat::matrix(K, nd, nd) += w * L / 2 * Bs[i].transpose() * (Bh.transpose() * Kh * Bh + Kg) * Bs[i];
// 		}
// 	}
// }
// void test_sizes(void)
// {
// 	//eh -> sh
// 	if(test == 0)
// 	{
// 		nr = nc = ns + 3;
// 	}
// 	//es -> eh
// 	else if (test == 1)
// 	{
// 		nc = ns;
// 		nr = ns + 3;
// 	}
// 	//es -> ss
// 	else if (test == 2)
// 	{
// 		nr = nc = ns;
// 	}
// 	//dp -> U
// 	else if(test == 3)
// 	{
// 		nr = 1;
// 		nc = nd;
// 	}
// 	//dp -> fp
// 	else if(test == 4)
// 	{
// 		nr = nd;
// 		nc = nd;
// 	}
// }

// void equations::beam3C::test(void)
// {
// 	setup();
// 	prepare();
// 	test_sizes();
// 	srand(time(nullptr));
// 	const unsigned ns = 100000;
// 	double* x = (double*) alloca(nc * sizeof(double));
// 	for(unsigned i = 0; i < ns; i++)
// 	{
// 		printf("%05d ", i);
// 		mat::vector(x, nc).randu();
// 		if(!mat::drift(fun, dev, x, nr, nc, 1e-3, 1e-8))
// 		{
// 			break;
// 		}
// 	}
// 	finish();
// }
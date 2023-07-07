#pragma once

//fea
#include "Mesh/Joints/Joint.h"

namespace fea
{
	namespace mesh
	{
		namespace joints
		{
			class Pinned : public Joint
			{
				friend class Joint;

			protected:
				//constructor
				Pinned(void);

				//destructor
				virtual ~Pinned(void);

				//serialization
				virtual void load(FILE*);
				virtual void save(FILE*) const;

			public:
				//data
				bool fixed(void) const;
				bool fixed(bool);

				double clearance(void) const;
				double clearance(double);
				double orientation(void) const;
				double orientation(double);
				double moment_yield(void) const;
				double moment_yield(double);
				double plastic_modulus(void) const;
				double plastic_modulus(double);

				const double* stiffness(void) const;
				const double* stiffness(const double*);
				const double* stiffness(unsigned, double);
				const double* stiffness(double, double, double);

				//types
				virtual joints::type type(void) const override;
				virtual unsigned nodes_max(void) const override;
				virtual unsigned state_set(void) const override;
				virtual unsigned dof_set(unsigned) const override;

			protected:
				//analysis
				virtual void apply(void) override;

				virtual bool check(void) const override;
				virtual void configure(void) override;

				virtual void update(void) override;
				virtual void restore(void) override;

				virtual void record(void) override;

				//formulation
				virtual double* internal_force(double*) const override;

				virtual double* inertia(double*) const override;
				virtual double* damping(double*) const override;
				virtual double* stiffness(double*) const override;

				//draw
				virtual void draw(void) const override;

				//data
				bool m_fixed;

				double m_clearance;
				double m_orientation;
				double m_stiffness[3];
				double m_moment_yield;

				double m_hardening_old;
				double m_hardening_new;

				double m_plastic_modulus;
				double m_plastic_rotation_old;
				double m_plastic_rotation_new;

				double m_f[3];
				double m_k[5];
			};
		}
	}
}
#pragma once

//std
#include <cstdio>

namespace fea
{
	namespace mesh
	{
		class Mesh;
		namespace materials
		{
			enum class type : unsigned;
		}
	}
}

namespace fea
{
	namespace mesh
	{
		namespace materials
		{
			class Material
			{
				friend class mesh::Mesh;

			protected:
				//constructors
				Material(void);

				//destructor
				virtual ~Material(void);

				//serialization
				virtual void load(FILE*);
				virtual void save(FILE*) const;

				//create
				static Material* create(Material*&, const Material*);
				static Material* create(Material*&, materials::type, const Material* = nullptr);

			public:
				//data
				static Mesh* mesh(void);

				virtual const char* label(void) const;
				virtual const char* label(const char*);

				virtual double specific_mass(void) const;
				virtual double specific_mass(double);

				//type
				virtual materials::type type(void) const = 0;

				//name
				virtual const char* name(void) const;
				static const char* name(materials::type);

				//index
				virtual unsigned index(void) const;

				//analysis
				virtual bool check(void) const;

			protected:
				//data
				static Mesh* m_mesh;

				char m_label[200];
				double m_specific_mass;
			};
		}
	}
}
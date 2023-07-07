#pragma once

//std
#include <vector>

namespace fea
{
	namespace mesh
	{
		class Mesh;
		namespace sections
		{
			class Section;
			enum class E3Type : unsigned;
		}
	}
}

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			class E3Section
			{
				friend class mesh::Mesh;
				friend class mesh::sections::Section;

			private:
				//constructors
				E3Section(E3Type);

				//destructor
				virtual ~E3Section(void);

				//catalog
				static void load_ub(void);
				static void load_uc(void);
				static void load_ubp(void);
				static void load_ipe(void);
				static void load_hea(void);
				static void load_heb(void);
				static void load_hem(void);
				static void load_chs(void);
				static void load_rhs(void);
				static void load_shs(void);

			public:
				//data
				E3Type type(void) const;
				const char* name(void) const;

				//catalog
				static void load_catalog(void);
				static void clear_catalog(void);
				static const std::vector<E3Section*>& catalog(void);

				//name
				static const char* type_name(E3Type);
				virtual const char* type_name(void) const;

				//add
				virtual void add(double) const;

				//index
				static unsigned index(E3Type);
				virtual unsigned index(void) const;

			private:
				//add
				virtual void add_box(double) const;
				virtual void add_ring(double) const;
				virtual void add_profile_I(double) const;

				//data
				E3Type m_type;
				char m_name[50];
				double m_data[5];

				static mesh::Mesh* m_mesh;
				static const double m_table_ub[];
				static const double m_table_uc[];
				static const double m_table_ubp[];
				static const double m_table_ipe[];
				static const double m_table_hea[];
				static const double m_table_heb[];
				static const double m_table_hem[];
				static const double m_table_chs[];
				static const double m_table_rhs[];
				static const double m_table_shs[];
				static std::vector<E3Section*> m_catalog;
			};
		}
	}
}
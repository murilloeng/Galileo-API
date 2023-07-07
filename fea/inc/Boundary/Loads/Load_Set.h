#pragma once

//std
#include <vector>

namespace fea
{
	namespace boundary
	{
		class Boundary;
		namespace loads
		{
			class Set_Case;
		}
	}
	namespace analysis
	{
		class Assembler;
	}
}

namespace fea
{
	namespace boundary
	{
		namespace loads
		{
			class Load_Set
			{
				friend class boundary::Boundary;
				friend class analysis::Assembler;

			protected:
				//constructors
				Load_Set(void);

				//destructor
				virtual ~Load_Set(void);

				//serialization
				void load(FILE*);
				void save(FILE*) const;

			public:
				//data
				Set_Case* set_case(unsigned);

				const char* label(void) const;
				const char* label(const char*);

				static Boundary* boundary(void);

				const std::vector<Set_Case*>& set_cases(void) const;

				//list
				void remove_set_case(unsigned);
				Set_Case* add_set_case(bool, double, unsigned);

				//index
				unsigned index(void) const;

			protected:
				//analysis
				bool check(void) const;

				//data
				char m_label[200];
				static Boundary* m_boundary;
				std::vector<Set_Case*> m_set_cases;
			};
		}
	}
}
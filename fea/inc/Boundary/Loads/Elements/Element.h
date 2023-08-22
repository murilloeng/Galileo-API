#pragma once

//std
#include <vector>

//fea
#include "fea/inc/Boundary/Loads/Load.h"

namespace fea
{
	namespace boundary
	{
		class Boundary;
		namespace loads
		{
			class Load_Case;
			enum class type : unsigned;
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
			class Element : public Load
			{
				friend class Load_Case;
				friend class boundary::Boundary;
				friend class analysis::Assembler;

			protected:
				//constructors
				Element(void);

				//destructor
				virtual ~Element(void);

				//serialization
				virtual void load(FILE*) override;
				virtual void save(FILE*) const override;

				//create
				static void create(loads::Element*&, loads::type);

			public:
				//name
				static const char* name(loads::type);
				virtual const char* name(void) const;

				//type
				virtual loads::type type(void) const = 0;

				//data
				std::vector<unsigned>& elements(void);
				const std::vector<unsigned>& elements(void) const;

				//index
				unsigned index(void) const;

			protected:
				//draw
				virtual void draw(void) const override;

				//analysis
				virtual void prepare(void) override;
				virtual bool check(void) const override;

				//data
				std::vector<unsigned> m_elements;
			};
		}
	}
}
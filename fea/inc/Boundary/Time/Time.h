#pragma once

//std
#include <cstdio>

namespace fea
{
	namespace boundary
	{
		namespace time
		{
			enum class type : unsigned;
		}
		class Boundary;
	}
}

namespace fea
{
	namespace boundary
	{
		namespace time
		{
			class Time
			{
				friend class boundary::Boundary;

			protected:
				//constructor
				Time(void);

				//destructor
				virtual ~Time(void);

				//serialization
				virtual void load(FILE*);
				virtual void save(FILE*) const;

				//create
				static void create(Time*&, const Time*);
				static void create(Time*&, time::type, const Time* = nullptr);

			public:
				//data
				static Boundary* boundary(void);

				virtual const char* label(void) const;
				virtual const char* label(const char*);

				//type
				virtual time::type type(void) const = 0;

				//name
				static const char* name(time::type);
				virtual const char* name(void) const;

				//index
				unsigned index(void) const;

				//value
				virtual double value(double) const = 0;

			protected:
				//data
				char m_label[200];
				static Boundary* m_boundary;
			};
		}
	}
}
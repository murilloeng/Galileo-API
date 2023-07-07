#pragma once

//std
#include <string>

namespace fea
{
	namespace mesh
	{
		class Mesh;
		namespace points
		{
			enum class type : unsigned;
		}
		namespace elements
		{
			class Element;
		}
	}
}

namespace fea
{
	namespace mesh
	{
		namespace points
		{
			class Point
			{
				friend class Mesh;
				friend class elements::Element;

			public:
				//constructors
				Point(void);

				//destructor
				virtual ~Point(void);

				//create
				static Point* create(Point*&, const Point*);
				static Point* create(Point*&, points::type, const Point* = nullptr);

				//type
				virtual points::type type(void) const = 0;

				//index
				virtual unsigned index(void) const;

				//analysis
				virtual void update(void);
				virtual void restore(void);
				virtual void prepare(const elements::Element*);

				//data
				const elements::Element* m_element;
			};
		}
	}
}
#pragma once

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			class History;
		}
	}
}

namespace fea
{
	namespace mesh
	{
		namespace sections
		{
			class Point
			{
				friend class History;

			public:
				//constructors
				Point(void);

				//destructor
				virtual ~Point(void);

				//serialization
				virtual void load(FILE*);
				virtual void save(FILE*) const;

				//data
				virtual double stress(unsigned) const;

			private:
				//data
				double m_stress[3];
			};
		}
	}
}
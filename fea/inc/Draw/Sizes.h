#pragma once

//std
#include <cstdio>

namespace fea
{
	namespace draw
	{
		class Sizes
		{
		public:
			//constructors
			Sizes(void);

			//destructor
			virtual ~Sizes(void);

			//serialization
			void load(FILE*);
			void save(FILE*) const;

			//data
			double loads(double);
			double joints(double);
			double curves(double);
			double graphs(double);
			double supports(double);

			double loads(void) const;
			double joints(void) const;
			double curves(void) const;
			double graphs(void) const;
			double supports(void) const;

			//config
			void reset(void);

		private:
			//data
			double m_loads;
			double m_joints;
			double m_curves;
			double m_graphs;
			double m_supports;
		};
	}
}
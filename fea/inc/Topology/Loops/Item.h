#pragma once

//std
#include <cstdio>

namespace fea
{
	namespace topology
	{
		class Topology;
		namespace curves
		{
			class Curve;
		}
		namespace loops
		{
			class Loop;
		}
		namespace surfaces
		{
			class Surface;
		}
	}
}

namespace fea
{
	namespace topology
	{
		namespace loops
		{
			class Item
			{
			private:
				//constructors
				Item(Topology*, unsigned, bool);

				//destructor
				~Item(void);

				//serialization
				void load(FILE*);
				void save(FILE*) const;

			public:
				//data
				bool inversed(bool);
				bool inversed(void) const;

				unsigned index(unsigned);
				unsigned index(void) const;

				curves::Curve* curve(unsigned);
				curves::Curve* curve(void) const;

				surfaces::Surface* surface(unsigned);
				surfaces::Surface* surface(void) const;

			private:
				//data
				bool m_inversed;
				unsigned m_index;
				Topology* m_topology;

				//friends
				friend class Loop;
				friend class Topology;
				friend class surfaces::Surface;
			};
		}
	}
}
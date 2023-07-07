#pragma once

//std
#include <cstdio>
#include <vector>

namespace fea
{
	namespace topology
	{
		class Topology;
		namespace loops
		{
			class Item;
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
			class Loop
			{
			private:
				//constructors
				Loop(Topology*);

				//destructor
				~Loop(void);

				//serialization
				void load(FILE*);
				void save(FILE*) const;

			public:
				//data
				Item* item(unsigned);
				const Item* item(unsigned) const;
				const std::vector<Item*>& items(void);

				//items
				void remove_item(unsigned);
				Item* add_item(unsigned, bool);

				//check
				bool closed(void) const;

			private:
				//data
				Topology* m_topology;
				std::vector<Item*> m_items;

				//friends
				friend class Topology;
				friend class surfaces::Surface;
			};
		}
	}
}
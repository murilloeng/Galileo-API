#pragma once

//std
#include <cstdio>

namespace fea
{
	namespace draw
	{
		class What
		{
		public:
			//constructors
			What(void);

			//destructor
			virtual ~What(void);

			//serialization
			void load(FILE*);
			void save(FILE*) const;

			//data
			bool axes(bool);
			bool edges(bool);
			bool faces(bool);
			bool scale(bool);
			bool nodes(bool);
			bool loads(bool);
			bool graph(bool);
			bool joints(bool);
			bool volume(bool);
			bool points(bool);
			bool curves(bool);
			bool elements(bool);
			bool supports(bool);
			bool surfaces(bool);
			bool deformed(bool);
			unsigned load_case(unsigned);

			bool axes(void) const;
			bool edges(void) const;
			bool faces(void) const;
			bool scale(void) const;
			bool nodes(void) const;
			bool loads(void) const;
			bool graph(void) const;
			bool joints(void) const;
			bool volume(void) const;
			bool points(void) const;
			bool curves(void) const;
			bool elements(void) const;
			bool supports(void) const;
			bool surfaces(void) const;
			bool deformed(void) const;
			unsigned load_case(void) const;

			//config
			void reset(void);

		private:
			//data
			bool m_axes;
			bool m_edges;
			bool m_faces;
			bool m_scale;
			bool m_nodes;
			bool m_loads;
			bool m_graph;
			bool m_joints;
			bool m_volume;
			bool m_points;
			bool m_curves;
			bool m_elements;
			bool m_supports;
			bool m_surfaces;
			bool m_deformed;
			unsigned m_load_case;
		};
	}
}
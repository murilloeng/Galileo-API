#pragma once

//std
#include <cstdio>

namespace fea
{
	namespace draw
	{
		class Numbers
		{
		public:
			//constructors
			Numbers(void);

			//destructor
			virtual ~Numbers(void);

			//serialization
			void load(FILE*);
			void save(FILE*) const;

			//data
			bool nodes(bool);
			bool joints(bool);
			bool points(bool);
			bool curves(bool);
			bool elements(bool);
			bool surfaces(bool);
			unsigned font_size(unsigned);

			bool nodes(void) const;
			bool joints(void) const;
			bool points(void) const;
			bool curves(void) const;
			bool elements(void) const;
			bool surfaces(void) const;
			unsigned font_size(void) const;

			//config
			void reset(void);

		private:
			//data
			bool m_nodes;
			bool m_joints;
			bool m_points;
			bool m_curves;
			bool m_elements;
			bool m_surfaces;
			unsigned m_font_size;
		};
	}
}
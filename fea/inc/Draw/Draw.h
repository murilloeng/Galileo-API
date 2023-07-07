#pragma once

//std
#include <cstdio>

namespace fea
{
	namespace draw
	{
		class What;
		class View;
		class Sizes;
		class Colors;
		class Numbers;
		class Palette;
		class Selection;
	}
}

namespace fea
{
	namespace draw
	{
		class Draw
		{
		public:
			//constructors
			Draw(void);

			//destructor
			virtual ~Draw(void);

			//serialization
			void load(FILE*);
			void save(FILE*) const;

			//data
			bool mode(bool);
			bool mode(void) const;

			What* what(void);
			Sizes* sizes(void);
			Colors* colors(void);
			Numbers* numbers(void);
			Palette* palette(void);
			Selection* selection(void);

			//config
			void reset(void);

		protected:
			//data
			bool m_mode;
			What* m_what;
			Sizes* m_sizes;
			Colors* m_colors;
			Numbers* m_numbers;
			Palette* m_palette;
			Selection* m_selection;
		};
	}
}
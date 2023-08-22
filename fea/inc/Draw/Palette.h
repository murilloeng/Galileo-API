#pragma once

//std
#include <vector>

namespace fea
{
	namespace draw
	{
		class Draw;
	}
}

namespace fea
{
	namespace draw
	{
		class Palette
		{
		private:
			//constructors
			Palette(bool catalog = false);

			//destructor
			~Palette(void);

			//catalog
			static void load_jet(void);
			static void load_greys(void);
			static void load_plasma(void);
			static void load_whylrd(void);
			static void load_viridis(void);
			static void load_moreland(void);
			static void load_spectral(void);

		public:
			//data
			unsigned index(unsigned);
			unsigned index(void) const;

			double value_min(double);
			double value_max(double);
			double value_min(void) const;
			double value_max(void) const;

			unsigned size(void) const;
			const char* name(void) const;
			const double* colors(void) const;

			static const std::vector<Palette*>& catalog(void);

			//color
			double* color(double*, double, double = 1) const;

		private:
			//catalog
			void load_from_catalog(void);
			static void load_catalog(void);
			static void clear_catalog(void);

			//data
			char m_name[50];
			unsigned m_size;
			double* m_colors;
			unsigned m_index;
			double m_v_min, m_v_max;

			static double m_table_jet[];
			static double m_table_greys[];
			static double m_table_plasma[];
			static double m_table_whylrd[];
			static double m_table_viridis[];
			static double m_table_moreland[];
			static double m_table_spectral[];
			static std::vector<Palette*> m_catalog;

			//friends
			friend class Draw;
		};
	}
}
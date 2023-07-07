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
			//friends
			friend class Draw;

		private:
			//constructors
			Palette(bool);

			//destructor
			virtual ~Palette(void);

			//catalog
			static void load_jet(void);
			static void load_greys(void);
			static void load_plasma(void);
			static void load_whylrd(void);
			static void load_viridis(void);
			static void load_moreland(void);
			static void load_spectral(void);

		public:
			//load
			virtual void load_from_catalog(unsigned);

			//catalog
			static void load_catalog(void);
			static void clear_catalog(void);
			static const std::vector<Palette*>& catalog(void);

			//data
			static unsigned index(void);
			virtual unsigned size(void) const;
			virtual const char* name(void) const;
			virtual const double* colors(void) const;

			//color
			virtual double* color(double*, double, double, double, double = 1) const;

		private:
			//data
			char m_name[50];
			unsigned m_size;
			double* m_colors;

			static unsigned m_index;
			static double m_table_jet[];
			static double m_table_greys[];
			static double m_table_plasma[];
			static double m_table_whylrd[];
			static double m_table_viridis[];
			static double m_table_moreland[];
			static double m_table_spectral[];
			static std::vector<Palette*> m_catalog;
		};
	}
}
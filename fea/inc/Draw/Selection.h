#pragma once

//std
#include <vector>

namespace fea
{
	namespace draw
	{
		class Selection
		{
		public:
			//constructors
			Selection(void);

			//destructor
			virtual ~Selection(void);

			//data
			void clear(void);
			std::vector<unsigned>& nodes(void);
			std::vector<unsigned>& points(void);
			std::vector<unsigned>& curves(void);
			std::vector<unsigned>& joints(void);
			std::vector<unsigned>& elements(void);
			std::vector<unsigned>& surfaces(void);

		protected:
			//data
			std::vector<unsigned> m_nodes;
			std::vector<unsigned> m_points;
			std::vector<unsigned> m_curves;
			std::vector<unsigned> m_joints;
			std::vector<unsigned> m_elements;
			std::vector<unsigned> m_surfaces;
		};
	}
}
#pragma once

namespace fea
{
	namespace results
	{
		class Limit_State
		{
		public:
			//constructors
			Limit_State(void);

			//destructor
			virtual ~Limit_State(void);

			//data
			double m_value[2];
			unsigned m_step[2];
			unsigned m_node[2];
			unsigned m_element[2];
		};
	}
}
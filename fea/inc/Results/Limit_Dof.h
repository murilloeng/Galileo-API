#pragma once

namespace fea
{
	namespace results
	{
		class Limit_Dof
		{
		public:
			//constructors
			Limit_Dof(void);

			//destructor
			virtual ~Limit_Dof(void);

			//data
			double m_value[2];
			unsigned m_step[2];
			unsigned m_node[2];
		};
	}
}
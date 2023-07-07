#pragma once

namespace fea
{
	namespace results
	{
		class Results;
	}
}

namespace fea
{
	namespace results
	{
		class Bounding_Box
		{
			friend class Results;

			public:
				//constructors
				Bounding_Box(void);

				//destructor
				virtual ~Bounding_Box(void);

				//data
				unsigned type(void);
				double size(void) const;
				double size(unsigned) const;

				//transform
				void apply(void) const;
				void apply(double*) const;
				void invert(double*) const;

				//update
				void update(void);
				void update(unsigned);

			private:
				//update
				void update_mesh(void);
				void update_topology(void);
				void update_positions(void);

				unsigned type(void) const;

				//data
				double m_x[3][2][3];
				static const Results* m_results;
		};
	}
}
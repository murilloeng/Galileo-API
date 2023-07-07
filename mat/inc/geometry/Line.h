#pragma once

namespace mat
{
	class Line
	{
	public:
		//constructors
		Line(void);
		Line(const double*);
		Line(const double*, const double*);

		//destructor
		virtual ~Line(void);

		//data
		const double* point(unsigned) const;
		const double* point(unsigned, const double*);
		const double* point(unsigned, double, double, double);

		//geometry
		double length(void) const;
		double* direction(double*) const;

		double* point(double*, double) const;

		double distance(const double*) const;
		double distance(double, double, double) const;

		bool intersection(const Line&, double*) const;

	private:
		//data
		double m_x[2][3];
	};
}
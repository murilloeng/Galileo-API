#pragma once

//mat
#include "mat/inc/linear/matrix.h"

namespace mat
{
	class vector : public matrix
	{
	public:
		//constructors
		vector(void);
		vector(const matrix&);
		vector(const double*, unsigned);
		vector(unsigned, mode = mode::null);
		vector(std::initializer_list<double>);
		vector(double*, unsigned, mode = mode::null);

		//destructor
		virtual ~vector(void);

		//size
		vector& resize(unsigned);
		vector& resize(unsigned, unsigned);

		//linear
		vector unit(void) const;
		matrix outer(void) const;
		matrix outer(const vector&) const;
		double inner(const vector&) const;
	};
}
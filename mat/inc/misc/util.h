#pragma once

//std
#include <ctime>
#include <cstdint>

namespace mat
{
	int sign(int);
	int sign(bool);
	int sign(double);

	void swap(double&, double&);
	void swap(unsigned&, unsigned&);

	unsigned delta(unsigned, unsigned);
	unsigned delta(unsigned, unsigned, unsigned);

	bool bit_set(unsigned, unsigned);
	unsigned char bit_index(unsigned);
	unsigned char bit_count(unsigned);
	unsigned char bit_index(unsigned, unsigned);
	unsigned char bit_search(unsigned, unsigned char);

	bool bit_set(uint64_t, uint64_t);
	unsigned char bit_index(uint64_t);
	unsigned char bit_count(uint64_t);
	unsigned char bit_index(uint64_t, uint64_t);
	unsigned char bit_search(uint64_t, unsigned char);

	double randu(double = 0, double = 1);
	double bound(double, double = -1, double = +1);

	double fn(double, unsigned);
	double dfn(double, unsigned);
	double funt(double, unsigned);
	double cost(double, unsigned);
	double sint(double, unsigned);

	double* inv_color(double*);
	double* inv_color(double*, const double*);
	double* map_color(double*, const double**, double);

	char* time_format(char*, const time_t&, bool = true);

	void histogram(const double*, double*, unsigned, unsigned, unsigned);

	bool drift(void(*)(double*, const double*), void(*)(double*, const double*), double*, unsigned, unsigned, double, double);
}
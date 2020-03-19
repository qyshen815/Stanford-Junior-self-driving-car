#ifndef SHAPETABLES_H
#define SHAPETABLES_H

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <string>
#include <fstream>

#ifndef INT16_MAX 
#define INT16_MAX		(32767)
#endif

#ifndef UINT16_MAX
#define UINT16_MAX		(65535)
#endif

#ifndef INT16_MIN
#define INT16_MIN		(-32767-1)
#endif

template<typename T>
class ShapeTable
{
	private:
	int16_t min, max; // Table indices (INT16_MIN <= min < max < INT16_MAX)
	double a, s, x0;  // Non-centered, non-normalized Guassian distribution parameters.
	bool invert;      // Invert output values (0 -> a, a -> 0)
	T* table;         // Will contain (max - min) values of type T with domain [min,max).

	bool init()
	{
		if (!(min < max && max < INT16_MAX)) return false;
		if (table == NULL) table = new T[max-min+1];
		if (a == 0.) 
		{
			a = 1.;
		}

		return true;
	}

	public:
	// Inputs:
	// _a   Global maximum
	// _s   Sigma (width)
	// _x0  Mean
	//
	// For a table whose sum == 1, set _a = 0.

	ShapeTable() : min(INT16_MAX), max(INT16_MIN), a(0.), s(0.), x0(0.), invert(false), table(NULL) { }
	
	ShapeTable(uint16_t _min, uint16_t _max, double _a, double _s, double _x0) : 
		min(_min), max(_max), a(_a), s(_s), x0(_x0), invert(false), table(NULL) 
		{ GenerateGaussianTable(); }

	ShapeTable(const ShapeTable<T>& _st) : 
		min(_st.min), max(_st.max), a(_st.a), s(_st.s), x0(_st.x0), invert(_st.invert), table(NULL)
	{
		if (!init()) return;
		for (int16_t i = min; i <= max; i ++)
			table[i-min] = _st[i];
	}

	ShapeTable(const std::string& filename) : 
		s(0.), x0(0.), invert(false), table(NULL)
		{ Load(filename); }
	
	~ShapeTable() { if (table != NULL) delete [] table; }

	void Load(const std::string& filename)
	{
		char* buffer = NULL;

		{	
			std::ifstream in(filename.c_str(), std::ios::binary);
			if (!in.good() || in.eof() || !in.is_open()) { return; }
			in.seekg(0, std::ios_base::beg);
			std::ifstream::pos_type begin_pos = in.tellg();
			in.seekg(0, std::ios_base::end);
			int length = static_cast<int>(in.tellg() - begin_pos);
			in.seekg(0, std::ios_base::beg);
			buffer = new char[length];
			in.read(buffer, length);
			in.close();
		}

		if (buffer != NULL)
		{
			min = *reinterpret_cast<uint16_t*>(&buffer[0]);
			max = *reinterpret_cast<uint16_t*>(&buffer[sizeof(uint16_t)*1]);
			a   = *reinterpret_cast<double*>  (&buffer[sizeof(uint16_t)*2]);

			if (table != NULL) delete [] table;
			table = new T[max-min+1];

			T* offset = reinterpret_cast<T*>(&buffer[sizeof(uint16_t)*2 + sizeof(double)]);

			for (int i = min; i <= max; i ++)
				table[i-min] = offset[i-min];

			delete [] buffer;
		}
	}
	
	void SetParams(uint16_t _min, uint16_t _max, double _a, double _s, double _x0)
		{ min = _min; max = _max; a = _a; s = _s; x0 = _x0; }

	void GenerateGaussianTable()
	{
		bool normalize = a == 0.;
		if (!init()) return;
		double total = 0.;

		for (int16_t i = min; i <= max; i ++)
			total += table[i-min] = (T)(a*exp(-((double)i-x0)*((double)i-x0)/(2.*s*s)));

		if (normalize)
			for (int16_t i = min; i <= max; i ++)
				table[i-min] /= total;
	}

	void GenerateExponentialTable()
	{
		bool normalize = a == 0.;
		if (!init()) return;
		double total = 0.;

		for (int16_t i = min; i <= max; i ++)
			total += table[i-min] = (T)(a*exp(s*(double)i));

		if (normalize)
			for (int16_t i = min; i <= max; i ++)
				table[i-min] /= total;
	}

	void ApplyCompression(T threshold, double ratio)
	{
		double gain = a / ((a - threshold)/ratio + threshold);
		
		for (int16_t i = min; i <= max; i ++)
		{
			if (table[i-min] > threshold) table[i-min] = 
				(T)(gain*(((double)table[i-min]-threshold)/ratio + threshold));
			else table[i-min] *= gain;
		}
	}

	void InvertValues()
	{
		invert = !invert;
	}

	void MoveDomain(int16_t first, int16_t second)
	{
		int size = max-min+1;
		if (abs(first-second)+1 != size) return;
		T* new_table = new T[size];

		// Flip domain
		if (first > second)
		{
			for (int16_t i = 0; i < size; i ++)
				new_table[size-i-1] = table[i];
		
			min = second;
			max = first;
		}
		else
		{
			min = first;
			max = second;
		}

		for (int16_t i = 0; i < size; i ++)
			table[i] = new_table[i];

		delete [] new_table;
	}

	T operator [] (int16_t i) const
	{ 
		if (invert)
			return ((min <= i && i <= max) ? (T)(a - table[i-min]) : (T)(a)); 
		else
			return ((min <= i && i <= max) ? table[i-min] : (T)0.); 
	}

	void Export(T* tbl, int size)
	{
		for (int i = 0; i < size; i ++)
			tbl[i] = (*this)[i];
	}
};

#endif


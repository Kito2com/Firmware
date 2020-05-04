#pragma once

#include <cstdio>
#include <cstring>
#include <assert.h>

#if defined(SUPPORT_STDIOSTREAM)
#include <iostream>
#include <iomanip>
#endif // defined(SUPPORT_STDIOSTREAM)

#include <matrix/math.hpp>

#define SAMPLE_N 200  // Sample count

/**
 * 平均、分散を逐次計算する。
 */
class Statistics
{
private:
	/**
	 * number of sample
	 */
	int sampleN;

	/**
	 * current sample No
	 */
	int curN {0};

	const float data_v3[3] {0, 0, 0};

	/**
	 * previous average
	 */
	matrix::Vector<float, 3> ave1 {data_v3};

	/**
	 * average
	 */
	matrix::Vector<float, 3> ave {data_v3};

	/**
	 * previous variance
	 */
	matrix::Vector<float, 3> var1 {data_v3};

	/**
	 * variance
	 */
	matrix::Vector<float, 3> var {data_v3};


public:
	// Constructors
	Statistics():sampleN(SAMPLE_N) {}

	struct Range_error {};

	Statistics(const int sample_n):sampleN(sample_n)
	{
	  assert(sample_n > 0);
	}

	~Statistics() {}

	/**
	 * input : sample value
	 * function : calculation of average and variance
	 */
	void apply(const matrix::Vector<float, 3> sample) {

	  if (curN >= sampleN) return;

	  curN += 1;
	  // calculate average
	  for (int i = 0; i < 3; ++i) {
	    ave(i) = ( (curN-1) * ave1(i) + sample(i) ) / curN;
	  }
	  // ca1culate variance
	  for (int i = 0; i < 3; ++i) {
	    var(i) = ( (curN-1) * (var1(i)*var1(i) + ave1(i)*ave1(i))  + sample(i)*sample(i) )/ curN - ave(i)*ave(i);
	  }
	  // save
	  for (int i = 0; i < 3; ++i) {
	      ave1(i) = ave(i);
	      var1(i) = var(i);
	  }
	}

	matrix::Vector<float, 3> getAve() {return ave;}
	matrix::Vector<float, 3> getAve1() {return ave1;}

	matrix::Vector<float, 3> getVar() {return var;}
	matrix::Vector<float, 3> getVar1() {return var1;}

	/**
	 * function : clear each property
	 */
	void reset() {
	  curN = 0;
	  for (int i = 0; i < 3; ++i) {
	    ave1(i) = 0;
	    ave(i) = 0;
	    var1(i) = 0;
	    var(i) = 0;
	  }
	}

	int getSampleN() {return sampleN;}

	int getCurN() {return curN;}

};


template<typename Type, size_t  M>
bool isEqualVec(const matrix::Vector<Type, M> &x,
             const matrix::Vector<Type, M> &y, const Type eps=1e-4f) {

    bool equal = true;

    for (size_t i = 0; i < M; i++) {
        if (fabs(x(i) - y(i)) > eps) {
            equal = false;
            break;
        }
        if (equal == false) break;
    }

    if (!equal) {
        static const size_t n = 10*M;
        char * buf = new char[n];
        x.write_string(buf, n);
        printf("not equal\n x=%s\n", buf);
        y.write_string(buf, n);
        printf(" y=%s\n", buf);
        delete[] buf;
    }
    return equal;
}

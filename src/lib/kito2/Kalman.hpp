#pragma once

#include <cstdio>
#include <cstring>
#include <assert.h>

#if defined(SUPPORT_STDIOSTREAM)
#include <iostream>
#include <iomanip>
#endif // defined(SUPPORT_STDIOSTREAM)

#include <matrix/math.hpp>
#include <float.h>

#define DELT_T  0.002   // Sampling cycle time (sec)
#define EPSILON 0.001   // 可観測性を確保するための微係数（デフォルト）　0 〜 0.01
#define GAMMA 1         // 共分散行列 P の初期値係数

/**
 * 状態変数 x の 配列index および 次元 を示す enum class ← template class が絡むと上手く行かない
 */
//enum class X {x, x1, xd, xd1, y, y1, yd, yd1, z, z1, zd, zd1, n};
enum {X_x, X_x1, X_xd, X_xd1, X_y, X_y1, X_yd, X_yd1, X_z, X_z1, X_zd, X_zd1, X_n};

/**
 * 入力値 u の 配列index および 次元 を示す enum class ← template class が絡むと上手く行かない
 */
//enum class U {x, xd, y, yd, z, zd, n};
enum {U_x, U_xd, U_y, U_yd, U_z, U_zd, U_n};

/**
 * 観測値 y の 配列index および 次元 を示す enum class ← template class が絡むと上手く行かない
 */
//enum class Y {x, y, z, n};
enum {Y_x, Y_y, Y_z, Y_n};

/**
 * 静止状態クラス
 */
struct Static
{
public:
	/**
	 * 係数行列Aを得る。
	 * dt：サンプリング周期 (s)
	 * eps：可観測微係数
	 */
	static matrix::Matrix<float,X_n,X_n>
	  getA(const float dt = DELT_T, const float eps = EPSILON)
	{
	  assert(dt >= 0.001 && dt =< 0.01 );
	  assert(eps > 0 && eps <0.01);
	  float a[X_n][X_n] = {
	    {           1,             0,   dt,    0, 0, 0, 0, 0, 0, 0, 0, 0},
	    {           0,             1, dt/4, dt/4, 0, 0, 0, 0, 0, 0, 0, 0},
	    {        1/dt,         -1/dt,  1.5, -1.5, 0, 0, 0, 0, 0, 0, 0, 0},
	    {2*(1+eps)/dt, -2*(1-eps)/dt,    0,   -1, 0, 0, 0, 0, 0, 0, 0, 0},
	    {0, 0, 0, 0,            1,             0,   dt,    0, 0, 0, 0, 0},
	    {0, 0, 0, 0,            0,             1, dt/4, dt/4, 0, 0, 0, 0},
	    {0, 0, 0, 0,         1/dt,         -1/dt,  1.5, -1.5, 0, 0, 0, 0},
	    {0, 0, 0, 0, 2*(1+eps)/dt, -2*(1-eps)/dt,    0,   -1, 0, 0, 0, 0},
	    {0, 0, 0, 0, 0, 0, 0, 0,            1,             0,   dt,    0},
	    {0, 0, 0, 0, 0, 0, 0, 0,            0,             1, dt/4, dt/4},
	    {0, 0, 0, 0, 0, 0, 0, 0,         1/dt,         -1/dt,  1.5, -1.5},
	    {0, 0, 0, 0, 0, 0, 0, 0, 2*(1+eps)/dt, -2*(1-eps)/dt,    0,   -1}
	  };
	  matrix::Matrix<float,X_n,X_n> A {a};
	  return A;
	};
};

/**
 * 運動状態クラス
 */
struct Kinetic
{
public:
	/**
	 * 係数行列Aを得る。
	 * dt：サンプリング周期 (s)
	 * epsilon：可観測微係数
	 */
	static matrix::Matrix<float,X_n,X_n>
	  getA(const float dt = DELT_T, const float eps = EPSILON)
	{
	  assert(dt >= 0.001 && dt =< 0.01 );
	  assert(eps > 0 && eps <0.01);
	  float a[X_n][X_n] = {
	    {           1,             0,   dt,    0, 0, 0, 0, 0, 0, 0, 0, 0},
	    {         0.5,           0.5, dt/4, dt/4, 0, 0, 0, 0, 0, 0, 0, 0},
	    {        1/dt,         -1/dt,  1.5, -1.5, 0, 0, 0, 0, 0, 0, 0, 0},
	    {2*(1+eps)/dt, -2*(1-eps)/dt,    0,   -1, 0, 0, 0, 0, 0, 0, 0, 0},
	    {0, 0, 0, 0,            1,             0,   dt,    0, 0, 0, 0, 0},
	    {0, 0, 0, 0,          0.5,           0.5, dt/4, dt/4, 0, 0, 0, 0},
	    {0, 0, 0, 0,         1/dt,         -1/dt,  1.5, -1.5, 0, 0, 0, 0},
	    {0, 0, 0, 0, 2*(1+eps)/dt, -2*(1-eps)/dt,    0,   -1, 0, 0, 0, 0},
	    {0, 0, 0, 0, 0, 0, 0, 0,            1,             0,   dt,    0},
	    {0, 0, 0, 0, 0, 0, 0, 0,          0.5,           0.5, dt/4, dt/4},
	    {0, 0, 0, 0, 0, 0, 0, 0,         1/dt,         -1/dt,  1.5, -1.5},
	    {0, 0, 0, 0, 0, 0, 0, 0, 2*(1+eps)/dt, -2*(1-eps)/dt,    0,   -1}
	  };
	  matrix::Matrix<float,X_n,X_n> A {a};
	  return A;
	};

};

/**
 * 回転運動クラス
 */
struct Rotate
{
public:
	/**
	 * 係数行列Cを得る。
	 * phi：ロール角 (rad)
	 * theta：ピッチ角 (rad)
	 */
	static matrix::Matrix<float,Y_n,X_n> getC(const double phi, const double theta)
	{
	  float c[Y_n][X_n] = {
	    {0, 0, 1, 0, 0, 0,                             0, 0, 0, 0,         static_cast<float>(-sin(theta)), 0},
	    {0, 0, 0, 0, 0, 0,  static_cast<float>(cos(phi)), 0, 0, 0, static_cast<float>(sin(phi)*cos(theta)), 0},
	    {0, 0, 0, 0, 0, 0, static_cast<float>(-sin(phi)), 0, 0, 0, static_cast<float>(cos(phi)*cos(theta)), 0}
	  };
	  matrix::Matrix<float,Y_n,X_n>  C {c};
	  return C;
	};

	/**
	 * 係数行列Cを更新する。
	 * phi：ロール角 (rad)
	 * theta：ピッチ角 (rad)
	 * matC：係数行列C、out
	 */
	static void updC(const double phi, const double theta, matrix::Matrix<float,Y_n,X_n>& matC)
	{
	  matC(Y_x, X_zd) = static_cast<float>(-sin(theta));
	  matC(Y_y, X_yd) = static_cast<float>(cos(phi));
	  matC(Y_y, X_zd) = static_cast<float>(sin(phi)*cos(theta));
	  matC(Y_z, X_yd) = static_cast<float>(-sin(phi));
	  matC(Y_z, X_zd) = static_cast<float>(cos(phi)*cos(theta));
	  return;
	};

};

/**
 * 並進運動クラス
 */
struct Translate
{
public:
        /**
         * 静止状態：係数行列Cを得る。
         */
        static matrix::Matrix<float,Y_n,X_n> getC_static()
        {
          float c[Y_n][X_n] = {
            {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0}
          };
          matrix::Matrix<float,Y_n,X_n>  C {c};
          return C;
        };

	/**
	 * 運動状態：係数行列Cを得る。
	 * aero：空気抵抗(1/s), A/m
	 * angR：角速度 (rad/s)
	 */
	static matrix::Matrix<float,Y_n,X_n>
	  getC_kinetic(const matrix::Vector<float,Y_n>& aero, const matrix::Vector<float,Y_n>& angR)
	{
	  float c[Y_n][X_n] = {
	    {aero(Y_x), 0, 1, 0, -angR(Y_z), 0, 0, 0, angR(Y_y), 0, 0, 0},
	    {angR(Y_z), 0, 0, 0, aero(Y_y), 0,  1, 0, -angR(Y_x), 0, 0, 0},
	    {-angR(Y_y), 0, 0, 0, angR(Y_x), 0, 0, 0, aero(Y_z), 0, 1, 0}
	  };
	  matrix::Matrix<float,Y_n,X_n>  C {c};
	  return C;
	};

	/**
	 * 係数行列Cを更新する。
	 * angR：角速度 (rad/s)
	 * matC：係数行列C、out
	 */
	static void updC(const matrix::Vector<float,Y_n>& angR,
			 matrix::Matrix<float,Y_n,X_n>& matC)
	{
	  matC(Y_x, X_y) = -angR(Y_z);
	  matC(Y_x, X_z) = angR(Y_y);
	  matC(Y_y, X_x) = angR(Y_z);
	  matC(Y_y, X_z) = -angR(Y_x);
	  matC(Y_z, X_x) = -angR(Y_y);
	  matC(Y_z, X_y) = angR(Y_x);
	  return;
	};

};

/**
 * 静止時／運動時、回転／並進 による違いを、フラグによって統一的に扱う Kalman Filter。
 */
class Kalman
{
private:
	const float deltT {DELT_T};	// sampling cycle time (sec)
	const float obsEps {EPSILON};	// compensative coefficient for observability
	const float gamma {GAMMA};	// 共分散行列 P の初期値係数

	bool m_isStatic, m_isRotate;

	// dynamics matrix
	matrix::Matrix<float,X_n,X_n> matA;

	// input matrix
	matrix::Matrix<float,X_n,U_n> matB;

	// output matrix
	matrix::Matrix<float,Y_n,X_n> matC;

	// state covariance
	matrix::Matrix<float,X_n,X_n> matP;
	matrix::Matrix<float,X_n,X_n> matPb;

	// system noise covariance
	matrix::Matrix<float,U_n,U_n> matQ;

	// output covariance
	matrix::Matrix<float,Y_n,Y_n> matR;

	// state vector
	matrix::Vector<float,X_n> vecX;
	matrix::Vector<float,X_n> vecXb;

	// input vector
	matrix::Vector<float,U_n> vecU;

	// output vector
	matrix::Vector<float,Y_n> vecY;

	// BQBt
	matrix::Matrix<float,X_n,X_n> matBQBt;

	// kalman gain
	matrix::Matrix<float,X_n,Y_n> matG;
	matrix::Matrix<float,Y_n,Y_n> matCPbCtR;

	// unit matrix
	matrix::Matrix<float,X_n,X_n> matI;

public:
	Kalman(bool isStatic, bool isRotate, matrix::Vector<float,Y_n> angOrAero,
	       matrix::Vector<float,U_n> vecQ, matrix::Vector<float,Y_n> vecR,
	       matrix::Vector<float,U_n> inVecU)
	      : m_isStatic(isStatic), m_isRotate(isRotate), vecU(inVecU)
	{
	  matB.setZero();

	  matP.setIdentity();
	  matP = gamma * matP;
	  matPb.setZero();

	  matQ.setZero();

	  matR.setZero();
	  matR(Y_x,Y_x) = vecR(0);
	  matR(Y_y,Y_y) = vecR(1);
	  matR(Y_z,Y_z) = vecR(2);

	  float initAngR[3] {0, 0, 0};
	  matrix::Vector<float,Y_n> initAngRv {initAngR};

	  vecX.setZero();
	  vecXb.setZero();

	  vecU.setZero();

	  vecY.setZero();

	  matBQBt.setZero();

	  matI.identity();

	  if (isStatic)
	    {
	      matA = Static::getA(deltT, obsEps);
	      if (isRotate)
		{
		  matC = Rotate::getC(angOrAero(0), angOrAero(1));
		}
	      else
		{
		  matC = Translate::getC_static();
		}
	    }
	  else
	    {
	      matA = Kinetic::getA(deltT, obsEps);
	      matB(X_x,U_x) = 1.0F;
	      matB(X_xd,U_xd) = 1.0F;
	      matB(X_y,U_y) = 1.0F;
	      matB(X_yd,U_yd) = 1.0F;
	      matB(X_z,U_z) = 1.0F;
	      matB(X_zd,U_zd) = 1.0F;
	      if (isRotate)
		{
		  matC = Rotate::getC(angOrAero(0), angOrAero(1));
		}
	      else
		{
		  matC = Translate::getC_kinetic(angOrAero,initAngRv );
		}
	      matQ(U_x,U_x) = vecQ(0);
	      matQ(U_xd,U_xd) = vecQ(1);
	      matQ(U_y,U_y) = vecQ(2);
	      matQ(U_yd,U_yd) = vecQ(3);
	      matQ(U_z,U_z) = vecQ(4);
	      matQ(U_zd,U_zd) = vecQ(5);
	      matBQBt = matB * matQ * matB.transpose();
	    }
	};

	~Kalman() {};

	matrix::Matrix<float,X_n,X_n> getMatA()
	{
	  return matA;
	}
	matrix::Matrix<float,X_n,U_n> getMatB()
	{
	  return matB;
	}
	matrix::Matrix<float,Y_n,X_n> getMatC()
	{
	  return matC;
	}
	matrix::Matrix<float,X_n,X_n> getMatP()
	{
	  return matP;
	}
	matrix::Matrix<float,X_n,X_n> getMatPb()
	{
	  return matPb;
	}

	matrix::Matrix<float,U_n,U_n> getMatQ()
	{
	  return matQ;
	}
	matrix::Matrix<float,Y_n,Y_n> getMatR()
	{
	  return matR;
	}
	matrix::Matrix<float,X_n,Y_n> getMatG()
	{
	  return matG;
	}

	matrix::Vector<float,X_n> getVecX()
	{
	  return vecX;
	}
	matrix::Vector<float,X_n> getVecXb()
	{ return vecXb;

	}
	matrix::Vector<float,U_n> getVecU()
	{
	  return vecU;
	}
	matrix::Vector<float,Y_n> getVecY()
	{
	  return vecY;
	}
	matrix::Matrix<float,X_n,X_n> getMatBQBt()
	{
	  return matBQBt;
	}

	void setX(const matrix::Vector<float,X_n> x)
	{
	  vecX = x;
	}

	matrix::Vector<float,X_n> calcXb()
	{
	  vecXb = matA * vecX + matB * vecU;
	  return vecXb;
	}

	matrix::Matrix<float,X_n,X_n> calcPb()
	{
	  matPb = matA * matP * matA.transpose() + matBQBt;
	  return matPb;
	}

	// invert matrix(3,3)
	int mat_invert3(matrix::Matrix<float,3,3>& src, matrix::Matrix<float,3,3>& dst)
	{
	  float det = src(0,0) * (src(1,1) * src(2,2) - src(1,2) * src(2,1)) -
		      src(0,1) * (src(1,0) * src(2,2) - src(1,2) * src(2,0)) +
		      src(0,2) * (src(1,0) * src(2,1) - src(1,1) * src(2,0));

          if (fabsf(det) < FLT_EPSILON) {
                  return PX4_ERROR;        // Singular matrix
          }

          dst(0,0) = (src(1,1) * src(2,2) - src(1,2) * src(2,1)) / det;
          dst(1,0) = (src(1,2) * src(2,0) - src(1,0) * src(2,2)) / det;
          dst(2,0) = (src(1,0) * src(2,1) - src(1,1) * src(2,0)) / det;
          dst(0,1) = (src(0,2) * src(2,1) - src(0,1) * src(2,2)) / det;
          dst(1,1) = (src(0,0) * src(2,2) - src(0,2) * src(2,0)) / det;
          dst(2,1) = (src(0,1) * src(2,0) - src(0,0) * src(2,1)) / det;
          dst(0,2) = (src(0,1) * src(1,2) - src(0,2) * src(1,1)) / det;
          dst(1,2) = (src(0,2) * src(1,0) - src(0,0) * src(1,2)) / det;
          dst(2,2) = (src(0,0) * src(1,1) - src(0,1) * src(1,0)) / det;

	  return PX4_OK;
	}

	// calc kalman_gain:G
	matrix::Matrix<float,X_n,Y_n> calcG()
	{
	  matCPbCtR = matC * matPb * matC.transpose() + matR;
	  matrix::Matrix<float,Y_n,Y_n> matInv;
	  if (mat_invert3(matCPbCtR, matInv) == PX4_OK)
	    {
	      matG = matPb * matC.transpose() * matInv;
	    }
	  return matG;
	}

	// calc state variant of x
	matrix::Vector<float,X_n> calcX()
	{
	  vecX = vecXb + matG * (vecY - matC * vecXb);
	  return vecX;
	}

	// calc P
	matrix::Matrix<float,X_n,X_n> calcP()
	{
	  matP = (matI - matG * matC) * matPb;
	  return matP;
	}

	/**
	 * 係数行列Cを更新する。
	 */
	void updC(matrix::Vector<float,Y_n> angOrRate)
	{
	  if (m_isRotate)
	    {
	      Rotate::updC(static_cast<double>(angOrRate(0)), static_cast<double>(angOrRate(1)), matC);
	    }
	  else
	    {
	      Translate::updC(angOrRate, matC);
	    }
	}

	/**
	 * 観測値 y を設定する。
	 */
	void setY(const matrix::Vector<float,Y_n> y)
	{
	  vecY = y;
	}

	/**
	 * Filterを更新する。
	 */
	void updFilter()
	{
	  calcXb();
	  calcPb();
	  calcG();
	  calcX();
	  calcP();
	}

};


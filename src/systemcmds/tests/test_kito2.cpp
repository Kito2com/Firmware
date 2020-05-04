
#include <unit_test.h>
#include <matrix/math.hpp>

#include <kito2/Statistics.hpp>
#include <kito2/Kalman.hpp>
#include <assert.h>

class Kito2Test : public UnitTest
{
public:
	virtual bool run_tests();

private:
	bool statisticsTests();
	bool kalmanTests();
};


bool Kito2Test::run_tests()
{
	ut_run_test(statisticsTests);
	ut_run_test(kalmanTests);

	return (_tests_failed == 0);
};


ut_declare_test_c(test_kito2, Kito2Test)


bool Kito2Test::statisticsTests()
{
	printf(" 1. default constructor\n");
	Statistics statdef;
	int sampleN = statdef.getSampleN();
	ut_assert("sampleN == 200", sampleN == 200);

	printf(" 2. constructor with parameter\n");
	Statistics stat100(100);
	sampleN = stat100.getSampleN();
	ut_assert("sampleN == 100", sampleN == 100);

	float vec0[3] {0, 0, 0};
	matrix::Vector<float, 3> matVec0 {vec0};

	printf(" 3. Ave initialize\n");
	matrix::Vector<float, 3> matTest = stat100.getAve();
	ut_test(isEqualVec(matTest, matVec0));

	printf(" 4. Ave1 initialize\n");
	matTest = statdef.getAve1();
	ut_test(isEqualVec(matTest, matVec0));

	printf(" 5. Var initialize\n");
	matTest = stat100.getVar();
	ut_test(isEqualVec(matTest, matVec0));

	printf(" 6. Var1 initialize\n");
	matTest = stat100.getVar1();
	ut_test(isEqualVec(matTest, matVec0));

	float vecsamp[3] {1.2, 2.3, 3.4};
	matrix::Vector<float, 3> vecAdd {vecsamp};

	printf(" 7. CurN at 1\n");
	stat100.apply(vecsamp);
	int curN = stat100.getCurN();
	ut_assert("curN == 1", curN == 1);

	printf(" 8. Ave at 1\n");
	matTest = stat100.getAve();
	ut_test(isEqualVec(matTest, vecAdd));

	printf(" 9. Var at 1\n");
	matrix::Vector<float, 3> var1 = stat100.getVar();
	ut_test(isEqualVec(var1, matVec0));

	float vecsamp2[3] {0.8, 1.7, 2.6};
	stat100.apply(vecsamp2);

	printf("10. CurN at 2\n");
	curN = stat100.getCurN();
	ut_assert("curN == 2", curN == 2);

	printf("11. Ave at 2\n");
	matTest = stat100.getAve();
	float ans[3] {1, 2, 3};
	matrix::Vector<float, 3> vecAns {ans};
	ut_test(isEqualVec(matTest, vecAns));

	printf("12. Var at 2\n");
	float vec3_2[3] {0.04, 0.09, 0.16};
	matrix::Vector<float, 3> avec3_2 {vec3_2};
	matrix::Vector<float, 3> tvec3_2 = stat100.getVar();
	ut_test(isEqualVec(tvec3_2, avec3_2));

	stat100.reset();

	printf("13. Ave at reset\n");
	matTest = stat100.getAve();
	ut_test(isEqualVec(matTest, matVec0));

	printf("14. Ave1 at reset\n");
	matTest = stat100.getAve1();
	ut_test(isEqualVec(matTest, matVec0));

	printf("15. Var at reset\n");
	matTest = stat100.getVar();
	ut_test(isEqualVec(matTest, matVec0));

	printf("16. Var1 at reset\n");
	matTest = stat100.getVar1();
	ut_test(isEqualVec(matTest, matVec0));

	printf("17. CurN at rest\n");
	curN = stat100.getCurN();
	ut_assert("curN == 0", curN == 0);

	printf("18. sample count saturate\n");
	Statistics stat3(3);
	for (int i=0; i<10; ++i) {
	    stat3.apply(vecAdd);
	  }
	curN = stat3.getCurN();
	ut_assert("curN == 3", curN == 3);

//	printf("Annex: constructor invalid parameter\n");
//	const int m20 = -20;
//	Statistics statM20(m20);

	return true;
};

bool Kito2Test::kalmanTests()
{
  printf(" 1. Static getA\n");
  float aS[12][12] = {
    {           1,             0,   0.002,    0, 0, 0, 0, 0, 0, 0, 0, 0},
    {           0,             1, 0.002/4, 0.002/4, 0, 0, 0, 0, 0, 0, 0, 0},
    {        1/0.002,         -1/0.002,  1.5, -1.5, 0, 0, 0, 0, 0, 0, 0, 0},
    {2*(1+0.001)/0.002, -2*(1-0.001)/0.002,    0,   -1, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0,            1,             0,   0.002,    0, 0, 0, 0, 0},
    {0, 0, 0, 0,            0,             1, 0.002/4, 0.002/4, 0, 0, 0, 0},
    {0, 0, 0, 0,         1/0.002,         -1/0.002,  1.5, -1.5, 0, 0, 0, 0},
    {0, 0, 0, 0, 2*(1+0.001)/0.002, -2*(1-0.001)/0.002,    0,   -1, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0,            1,             0,   0.002,    0},
    {0, 0, 0, 0, 0, 0, 0, 0,            0,             1, 0.002/4, 0.002/4},
    {0, 0, 0, 0, 0, 0, 0, 0,         1/0.002,         -1/0.002,  1.5, -1.5},
    {0, 0, 0, 0, 0, 0, 0, 0, 2*(1+0.001)/0.002, -2*(1-0.001)/0.002,    0,   -1}
  };
  matrix::Matrix<float,12,12> ansAs {aS};
  matrix::Matrix<float,12,12> testA = Static::getA(0.002, 0.001);
  ut_test(isEqual(testA, ansAs));

  printf("Annex1: Kinetic::getA invalid dt\n");
  testA = Kinetic::getA(0, 0.001);

  printf(" 2. Static getA_ epsilon default\n");
  testA = Static::getA(0.002);
  ut_test(isEqual(testA, ansAs));

  printf(" 3. Kinetic getA\n");
  float aK[12][12] = {
    {           1,             0,   0.011,    0, 0, 0, 0, 0, 0, 0, 0, 0},
    {         0.5,           0.5, 0.011/4, 0.011/4, 0, 0, 0, 0, 0, 0, 0, 0},
    {        1/0.011,         -1/0.011,  1.5, -1.5, 0, 0, 0, 0, 0, 0, 0, 0},
    {2*(1+0.001)/0.011, -2*(1-0.001)/0.011,    0,   -1, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0,            1,             0,   0.011,    0, 0, 0, 0, 0},
    {0, 0, 0, 0,          0.5,            0.5, 0.011/4, 0.011/4, 0, 0, 0, 0},
    {0, 0, 0, 0,         1/0.011,         -1/0.011,  1.5, -1.5, 0, 0, 0, 0},
    {0, 0, 0, 0, 2*(1+0.001)/0.011, -2*(1-0.001)/0.011,    0,   -1, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0,            1,             0,   0.011,    0},
    {0, 0, 0, 0, 0, 0, 0, 0,          0.5,           0.5, 0.011/4, 0.011/4},
    {0, 0, 0, 0, 0, 0, 0, 0,         1/0.011,         -1/0.011,  1.5, -1.5},
    {0, 0, 0, 0, 0, 0, 0, 0, 2*(1+0.001)/0.011, -2*(1-0.001)/0.011,    0,   -1}
  };
  matrix::Matrix<float,12,12> ansAk {aK};
  testA = Kinetic::getA(0.011, 0.001);
  ut_test(isEqual(testA, ansAk));

  printf(" 4. Rotate getC\n");
  float cR[3][12] = {
    {0, 0, 1, 0, 0, 0,                                0, 0, 0, 0,            static_cast<float>(-sin(M_PI/6)), 0},
    {0, 0, 0, 0, 0, 0,  static_cast<float>(cos(M_PI/3)), 0, 0, 0, static_cast<float>(sin(M_PI/3)*cos(M_PI/6)), 0},
    {0, 0, 0, 0, 0, 0, static_cast<float>(-sin(M_PI/3)), 0, 0, 0, static_cast<float>(cos(M_PI/3)*cos(M_PI/6)), 0}
  };
  matrix::Matrix<float,3,12> ansCr {cR};
  matrix::Matrix<float,3,12> testC = Rotate::getC(M_PI/3,M_PI/6);
  ut_test(isEqual(testC, ansCr));

  printf(" 5. Rotate updC\n");
  double phi = M_PI/4;
  double theta = 0;
  ansCr(0,10) = static_cast<float>(-sin(theta));
  ansCr(1,6) = static_cast<float>(cos(phi));
  ansCr(1,10) = static_cast<float>(sin(phi)*cos(theta));
  ansCr(2,6) = static_cast<float>(-sin(phi));
  ansCr(2,10) = static_cast<float>(cos(phi)*cos(theta));
  Rotate::updC(phi,theta,testC);
  ut_test(isEqual(testC, ansCr));

  printf(" 6. Translate getC_static\n");
  float cT_static[3][12] = {
    {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0}
  };
  matrix::Matrix<float,3,12> ansCt_static {cT_static};
  testC = Translate::getC_static();
  ut_test(isEqual(testC, ansCt_static));

  printf(" 7. Translate getC_kinetic\n");
  float cT_kinetic[3][12] = {
    {0.5342, 0, 1,     0,   -567, 0, 0, 0, -444.0, 0, 0, 0},
    {   567, 0, 0,     0, 0.5342, 0, 1, 0, -333.3, 0, 0, 0},
    { 444.0, 0, 0,     0,  333.3, 0, 0, 0, 0.5342, 0, 1, 0}
  };
  matrix::Matrix<float,3,12> ansCtk_static {cT_kinetic};
  float aero[3] {0.5342, 0.5342, 0.5342};
  matrix::Vector<float,3> aeroV {aero};
  float angR[3] {333.3, -444.0, 567};
  matrix::Vector<float,3> angRV {angR};
  testC = Translate::getC_kinetic(aeroV,angRV);
  ut_test(isEqual(testC, ansCtk_static));

  printf(" 8. Translate updC\n");
  float cT[3][12] = {
    {0.5342, 0, 1,     0,   321.0, 0, 0, 0, 432.1, 0, 0, 0},
    {-321.0, 0, 0,     0, 0.5342, 0, 1, 0, 123.4, 0, 0, 0},
    {-432.1, 0, 0,     0,  -123.4, 0, 0, 0, 0.5342, 0, 1, 0}
  };
  matrix::Matrix<float,3,12> ansCt {cT};
  float angR2[3] {-123.4, 432.1, -321.0};
  matrix::Vector<float,3> angRV2 {angR2};
  Translate::updC(angRV2, testC);
  ut_test(isEqual(testC, ansCt));

  //---
  printf(" 9. Kalman constructor:Static,Rotate:matA\n");

  matrix::Vector<float,6> vecU;
  vecU.setZero();

  float initAng[3] {M_PI/3, M_PI/6, M_PI/4};
  matrix::Vector<float,3> initAngV {initAng};
  float q[6] {1.1, 2.2, 3.3, 4.4, 5.5, 6.6};
  matrix::Vector<float,6> vecQ {q};
  float r[3] {7.7, 8.8, 9.9};
  matrix::Vector<float,3> vecR {r};

  Kalman kalmanSR(true,true,initAngV, vecQ, vecR, vecU);
  testA = kalmanSR.getMatA();
  ut_test(isEqual(testA, ansAs));

  printf("10. Kalman constructor:Static,Rotate:matB\n");
  matrix::Matrix<float,12,6> ansBs;
  ansBs.setZero();
  matrix::Matrix<float,12,6> testB = kalmanSR.getMatB();
  ut_test(isEqual(testB, ansBs));

  printf("11. Kalman constructor:Static,Rotate:matC\n");
  phi = M_PI/3;
  theta = M_PI/6;
  ansCr(0,10) = static_cast<float>(-sin(theta));
  ansCr(1,6) = static_cast<float>(cos(phi));
  ansCr(1,10) = static_cast<float>(sin(phi)*cos(theta));
  ansCr(2,6) = static_cast<float>(-sin(phi));
  ansCr(2,10) = static_cast<float>(cos(phi)*cos(theta));
  testC = kalmanSR.getMatC();
  ut_test(isEqual(testC, ansCr));

  printf("12. Kalman constructor:Static,Rotate:matP\n");
  matrix::Matrix<float,12,12> matP;
  matP.setIdentity();
  matP = static_cast<float>(1) * matP;
  matrix::Matrix<float,12,12> testP = kalmanSR.getMatP();
  ut_test(isEqual(testP, matP));

  printf("13. Kalman constructor:Static,Rotate:matQ\n");
  matrix::Matrix<float,6,6> matQ;
  matQ.setZero();
  matrix::Matrix<float,6,6> testQ = kalmanSR.getMatQ();
  ut_test(isEqual(testQ, matQ));

  printf("14. Kalman constructor:Static,Rotate:matR\n");
  matrix::Matrix<float,3,3> matR;
  matR.setZero();
  matR(0,0) = vecR(0);
  matR(1,1) = vecR(1);
  matR(2,2) = vecR(2);
  matrix::Matrix<float,3,3> testR = kalmanSR.getMatR();
  ut_test(isEqual(testR,matR));

//---
  printf("15. Kalman constructor:Static,Translate:matA\n");
  Kalman kalmanST(true,false,initAngV, vecQ, vecR, vecU);
  testA = kalmanST.getMatA();
  ut_test(isEqual(testA, ansAs));

  printf("16. Kalman constructor:Static,Translate:matB\n");
  testB = kalmanST.getMatB();
  ut_test(isEqual(testB, ansBs));

  printf("17. Kalman constructor:Static,Translate:matC\n");
  float cST[3][12] = {
    {0, 0, 1, 0, 0 ,0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0}
  };
  matrix::Matrix<float,3,12> ansCst {cST};
  testC = kalmanST.getMatC();
  ut_test(isEqual(testC,ansCst));

  printf("18. Kalman constructor:Static,Translate:matP\n");
  testP = kalmanST.getMatP();
  ut_test(isEqual(testP,matP));

  printf("19. Kalman constructor:Static,Translate:matQ\n");
  testQ = kalmanST.getMatQ();
  ut_test(isEqual(testQ,matQ));

  printf("20. Kalman constructor:Static,Translate:matR\n");
  testR = kalmanST.getMatR();
  ut_test(isEqual(testR,matR));

  //---
  printf("21. Kalman constructor:Kinetic,Rotate:matA\n");

  vecU(0) = -0.10; vecU(1) = -0.11; vecU(2) = -0.12; vecU(3) = -0.13; vecU(4) = -0.14; vecU(5) = -0.15;

  Kalman kalmanKR(false, true, initAngV, vecQ, vecR, vecU);
  float aK2[12][12] = {
    {           1,             0,   0.002,    0, 0, 0, 0, 0, 0, 0, 0, 0},
    {         0.5,           0.5, 0.002/4, 0.002/4, 0, 0, 0, 0, 0, 0, 0, 0},
    {        1/0.002,         -1/0.002,  1.5, -1.5, 0, 0, 0, 0, 0, 0, 0, 0},
    {2*(1+0.001)/0.002, -2*(1-0.001)/0.002,    0,   -1, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0,            1,             0,   0.002,    0, 0, 0, 0, 0},
    {0, 0, 0, 0,          0.5,            0.5, 0.002/4, 0.002/4, 0, 0, 0, 0},
    {0, 0, 0, 0,         1/0.002,         -1/0.002,  1.5, -1.5, 0, 0, 0, 0},
    {0, 0, 0, 0, 2*(1+0.001)/0.002, -2*(1-0.001)/0.002,    0,   -1, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0,            1,             0,   0.002,    0},
    {0, 0, 0, 0, 0, 0, 0, 0,          0.5,           0.5, 0.002/4, 0.002/4},
    {0, 0, 0, 0, 0, 0, 0, 0,         1/0.002,         -1/0.002,  1.5, -1.5},
    {0, 0, 0, 0, 0, 0, 0, 0, 2*(1+0.001)/0.002, -2*(1-0.001)/0.002,    0,   -1}
  };
  matrix::Matrix<float,12,12> ansAk2 {aK2};
  testA = kalmanKR.getMatA();
  ut_test(isEqual(testA,ansAk2));

  printf("22. Kalman constructor:Kinetic,Rotate:matB\n");
  float bK[12][6] = {
    {1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0},
    {0, 1, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0},
    {0, 0, 1, 0, 0, 0},
    {0, 0, 0, 0, 0, 0},
    {0, 0, 0, 1, 0, 0},
    {0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 1, 0},
    {0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1},
    {0, 0, 0, 0, 0, 0}
  };
  matrix::Matrix<float,12,6> ansBk {bK};
  testB = kalmanKR.getMatB();
  ut_test(isEqual(testB,ansBk));

  printf("23. Kalman constructor:Kinetic,Rotate:matC\n");
  testC = kalmanKR.getMatC();
  ut_test(isEqual(testC,ansCr));

  printf("24. Kalman constructor:Kinetic,Rotate:matP\n");
  testP = kalmanKR.getMatP();
  ut_test(isEqual(testP,matP));

  printf("25. Kalman constructor:Kinetic,Rotate:matQ\n");
  matrix::Matrix<float,6,6> matQk;
  matQk.setZero();
  matQk(0,0) = vecQ(0);
  matQk(1,1) = vecQ(1);
  matQk(2,2) = vecQ(2);
  matQk(3,3) = vecQ(3);
  matQk(4,4) = vecQ(4);
  matQk(5,5) = vecQ(5);
  testQ = kalmanKR.getMatQ();
  ut_test(isEqual(testQ,matQk));

  printf("26. Kalman constructor:Kinetic,Rotate:matR\n");
  testR = kalmanKR.getMatR();
  ut_test(isEqual(testR,matR));

  //---
  printf("27. Kalman constructor:Kinetic,Tranlate:matA\n");
  Kalman kalmanKT(false, false, initAngV, vecQ, vecR, vecU);
  testA = kalmanKT.getMatA();
  ut_test(isEqual(testA,ansAk2));

  printf("28. Kalman constructor:Kinetic,Tranlate:matB\n");
  testB = kalmanKT.getMatB();
  ut_test(isEqual(testB,ansBk));

  printf("29. Kalman constructor:Kinetic,Tranlate:matC\n");
  matrix::Matrix<float,3,12> ansCkt;
  ansCkt.setZero();
  ansCkt(0,0) = initAngV(0);
  ansCkt(0,2) = 1;
  ansCkt(1,4) = initAngV(1);
  ansCkt(1,6) = 1;
  ansCkt(2,8) = initAngV(2);
  ansCkt(2,10) = 1;
  testC = kalmanKT.getMatC();
  ut_test(isEqual(testC,ansCkt));

  printf("30. Kalman constructor:Kinetic,Tranlate:matP\n");
  testP = kalmanKT.getMatP();
  ut_test(isEqual(testP,matP));

  printf("31. Kalman constructor:Kinetic,Tranlate:matQ\n");
  testQ = kalmanKT.getMatQ();
  ut_test(isEqual(testQ,matQk));

  printf("32. Kalman constructor:Kinetic,Tranlate:matR\n");
  testR = kalmanKT.getMatR();
  ut_test(isEqual(testR,matR));

  //---
  printf("33. Kalman constructor:Static:matBQBt\n");
  matrix::Matrix<float,12,12> ansBQBt;
  ansBQBt.setZero();
  matrix::Matrix<float,12,12> testBQBt = kalmanST.getMatBQBt();
  ut_test(isEqual(testBQBt,ansBQBt));

  printf("34. Kalman constructor:Kinetic:matBQBt\n");
  ansBQBt = testB * testQ * testB.transpose();
  testBQBt = kalmanKT.getMatBQBt();
  ut_test(isEqual(testBQBt,ansBQBt));

  //---
  printf("35. Kalman:calcXb\n");
  float x1[12] = {0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.1,1.2};
  matrix::Vector<float,12> vecX1 {x1};
  kalmanSR.setX(vecX1);
  matrix::Vector<float,12> ansXb = kalmanSR.getMatA() * vecX1;
//  ansXb.print();
  matrix::Vector<float,12> testXb = kalmanSR.calcXb();
  ut_test(isEqualVec(testXb,ansXb));

  printf("36. Kalman:calcPb\n");
  matrix::Matrix<float,12,12> ktA = kalmanKT.getMatA();
  matrix::Matrix<float,12,12> ktP = kalmanKT.getMatP();
  matrix::Matrix<float,12,12> ansPb = ktA * ktP * ktA.transpose() + testBQBt;
  matrix::Matrix<float,12,12> testPb = kalmanKT.calcPb();
  ut_test(isEqual(testPb,ansPb));

  //---
  printf("37. Kalman:mat_invert3\n");
  float src[3][3] = {
    {-1,  1, 2},
    { 3, -1, 1},
    {-1,  3, 4}
  };
  matrix::Matrix<float,3,3> matSrc {src};
  float inv[3][3] = {
    {-0.7,  0.2,  0.3},
    {-1.3, -0.2,  0.7},
    { 0.8,  0.2, -0.2}
  };
  matrix::Matrix<float,3,3> matInv {inv};
  matrix::Matrix<float,3,3> testInv;
  if (kalmanKT.mat_invert3(matSrc,testInv) == PX4_OK)
    {
      ut_test(isEqual(testInv,matInv));
//      testInv.print();
    }
  else
    {
      printf("not nominal!\n");
    }

  printf("38. Kalman:mat_invert3:error\n");
  float err[3][3] = {
    { 3, -1, 5},
    { 2,  6, 4},
    { 5,  5, 9}
  };
  matrix::Matrix<float,3,3> matErr {err};
  int rc = kalmanKT.mat_invert3(matErr,testInv);
  ut_assert("not nominal!", rc == PX4_ERROR);

  //---
  printf("39. Kalman:calc gain\n");
  matrix::Matrix<float,3,12> ansC = kalmanKT.getMatC();
  matrix::Matrix<float,3,3> ansR = kalmanKT.getMatR();
  matSrc = ansC * ansPb * ansC.transpose() + ansR;
  matrix::Matrix<float,12,3> ansG, testG;
  if (kalmanKT.mat_invert3(matSrc, matInv) == PX4_OK)
    {
      printf("\t invert matrix is ok.\n");
      ansG = ansPb * ansC.transpose() * matInv;
      testG = kalmanKT.calcG();
      ut_test(isEqual(testG, ansG));
    }

  //---
  printf("40. Kalman:calc state variant x\n");
  float y[3] = {1.1, -2.2, 3.3};
  matrix::Vector<float,3> vecY {y};
  kalmanKT.setX(vecX1);
  ansXb = kalmanKT.calcXb();
  ansPb = kalmanKT.calcPb();
  ansG = kalmanKT.calcG();
  matrix::Vector<float,12> ansX = ansXb + ansG * (vecY - kalmanKT.getMatC() * ansXb);
  kalmanKT.setY(vecY);
  matrix::Vector<float,12> testX = kalmanKT.calcX();
  ut_test(isEqual(testX,ansX));

  //---
  printf("41. Kalman:calc P\n");
  matrix::Matrix<float,12,12> matI;
  matI.identity();
  matrix::Matrix<float,12,12> ansP = (matI - ansG * kalmanKT.getMatC()) * ansPb;
  testP = kalmanKT.calcP();
  ut_test(isEqual(testP, ansP));

  return true;
};

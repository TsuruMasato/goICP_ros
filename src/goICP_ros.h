#include "jly_goicp.h"

namespace goICP_ros_namespace
{

class GoICP_ros : GoICP
{
public:
	int Nm, Nd;
	POINT3D * pModel, * pData;

	ROTNODE initNodeRot;
	TRANSNODE initNodeTrans;

	DT3D dt;

	ROTNODE optNodeRot;
	TRANSNODE optNodeTrans;

	GoICP_ros(){};
  ~GoICP_ros(){};

  bool run(){};

  float Register();
	void BuildDT();

	float MSEThresh;
	float SSEThresh;
	float icpThresh;

	float optError;
	Matrix optR;
	Matrix optT;

	clock_t clockBegin;

	float trimFraction;
	int inlierNum;
	bool doTrim;

private:
	//temp variables
	float * normData;
	float * minDis;
	float** maxRotDis;
	float * maxRotDisL;
	POINT3D * pDataTemp;
	POINT3D * pDataTempICP;
	
	ICP3D<float> icp3d;
	float * M_icp;
	float * D_icp;

	float ICP(Matrix& R_icp, Matrix& t_icp);
	float InnerBnB(float* maxRotDisL, TRANSNODE* nodeTransOut);
	float OuterBnB();
	void Initialize();
	void Clear();

};



}
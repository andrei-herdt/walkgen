#include <mpc-walkgen/humanoid/walkgen-abstract.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <stdio.h>


#if _SECURE_SCL
//#error MOEP
#endif

#if _HAS_ITERATOR_DEBUGGING
//#error MOEP2
#endif

using namespace Eigen;
using namespace MPCWalkgen;


void dumpTrajectory(MPCSolution & solution, std::vector<std::ofstream*> & data_vec);
bool checkFiles(std::ifstream & f1, std::ifstream & f2);
int copyFile(const std::string & source, const std::string & destination);

int main() {
  // Logging:
  // --------
  const int nbFile=10;

  std::vector<std::string> name_vec(nbFile);
  name_vec[0]="CoM_X";
  name_vec[1]="CoM_Y";
  name_vec[2]="CoP_X";
  name_vec[3]="CoP_Y";
  name_vec[4]="LF_X";
  name_vec[5]="LF_Y";
  name_vec[6]="RF_X";
  name_vec[7]="RF_Y";
  name_vec[8]="RF_Z";
  name_vec[9]="LF_Z";

  std::vector<std::ofstream*> data_vec(nbFile);
  std::vector<std::ifstream*> ref_vec(nbFile);
  for (int i = 0; i < nbFile; ++i) {
    data_vec[i] = new std::ofstream((name_vec[i]+".data").c_str());
    ref_vec[i] = new std::ifstream((name_vec[i]+".ref").c_str());
  }


  // Robot parameters (HRP-2):
  // -------------------------
  FootData leftFoot;
  leftFoot.anklePositionInLocalFrame << 0, 0, 0.105;
  leftFoot.soleHeight = 0.138;
  leftFoot.soleWidth = 0.2172;
  FootData rightFoot;
  rightFoot.anklePositionInLocalFrame << 0, 0, 0.105;
  rightFoot.soleHeight = 0.138;
  rightFoot.soleWidth = 0.2172;

  HipYawData leftHipYaw;
  leftHipYaw.lowerBound = -0.523599;
  leftHipYaw.upperBound = 0.785398;
  leftHipYaw.lowerVelocityBound = -3.54108;
  leftHipYaw.upperVelocityBound = 3.54108;
  leftHipYaw.lowerAccelerationBound = -0.1;
  leftHipYaw.upperAccelerationBound = 0.1;
  HipYawData rightHipYaw = leftHipYaw;

  MPCData mpcData;

  RobotData robotData(leftFoot, rightFoot, leftHipYaw, rightHipYaw, 0.0);

  // Feasible hulls:
  // ---------------
  const int nbVertFeet = 5;
  // Feasible foot positions
  double DefaultFPosEdgesX[nbVertFeet] = {-0.28, -0.2, 0.0, 0.2, 0.28};
  double DefaultFPosEdgesY[nbVertFeet] = {-0.2, -0.3, -0.4, -0.3, -0.2};

  robotData.leftFootHull.resize(nbVertFeet);
  robotData.rightFootHull.resize(nbVertFeet);
  for (int i=0; i < nbVertFeet; ++i) {
    robotData.leftFootHull.x(i)=DefaultFPosEdgesX[i];
    robotData.leftFootHull.y(i)=DefaultFPosEdgesY[i];
    robotData.rightFootHull.x(i)=DefaultFPosEdgesX[i];
    robotData.rightFootHull.y(i)=-DefaultFPosEdgesY[i];
  }

  // Constraints on the CoP
  const int nbVertCoP = 4;
  double DefaultCoPSSEdgesX[nbVertCoP] = {0.0686, 0.0686, -0.0686, -0.0686};
  double DefaultCoPSSEdgesY[nbVertCoP] = {0.029, -0.029, -0.029, 0.029};
  double DefaultCoPDSEdgesX[nbVertCoP] = {0.0686, 0.0686, -0.0686, -0.0686};
  double DefaultCoPDSEdgesY[nbVertCoP] = {0.029, -0.229, -0.229, 0.029};

  robotData.CoPLeftSSHull.resize(nbVertCoP);
  robotData.CoPRightSSHull.resize(nbVertCoP);
  robotData.CoPLeftDSHull.resize(nbVertCoP);
  robotData.CoPRightDSHull.resize(nbVertCoP);
  for (int i = 0; i < nbVertCoP; ++i) {
    robotData.CoPLeftSSHull.x(i) = DefaultCoPSSEdgesX[i];
    robotData.CoPLeftSSHull.y(i) = DefaultCoPSSEdgesY[i];
    robotData.CoPLeftDSHull.x(i) = DefaultCoPDSEdgesX[i];
    robotData.CoPLeftDSHull.y(i) = DefaultCoPDSEdgesY[i];

    robotData.CoPRightSSHull.x(i) = DefaultCoPSSEdgesX[i];
    robotData.CoPRightSSHull.y(i) =- DefaultCoPSSEdgesY[i];
    robotData.CoPRightDSHull.x(i) = DefaultCoPDSEdgesX[i];
    robotData.CoPRightDSHull.y(i) =- DefaultCoPDSEdgesY[i];
  }

  // Create and initialize generator:
  // -------------------------------
  WalkgenAbstract * walk = createWalkgen(CURRENT_QPSOLVERTYPE);

  walk->init(robotData, mpcData);
  //	const RigidBodySystem *robot = walk->robot();// Not used yet

  // Run:
  // ----
  double velocity = 0.05;
  double t = 3.0;
  for (; t < 5; t += 0.005){
    MPCSolution solution = walk->online(t);
    dumpTrajectory(solution, data_vec);
  }

  walk->reference(velocity, 0, 0);
  for (; t < 10; t += 0.005){
    MPCSolution solution = walk->online(t);
    dumpTrajectory(solution, data_vec);
  }

  //walk->reference(0, velocity, 0);
  walk->reference(velocity, 0, 0);
  for (; t < 20; t += 0.005){
    MPCSolution solution = walk->online(t);
    dumpTrajectory(solution, data_vec);
  }

  //walk->reference(velocity, 0, velocity);
  walk->reference(0, 0, 0);
  for (; t < 30; t += 0.005){
    MPCSolution solution = walk->online(t);
    dumpTrajectory(solution, data_vec);
  }


  for(unsigned i = 0; i < data_vec.size(); ++i){
    data_vec[i]->close();
  }


  // Reopen the files:
  // -----------------
  std::vector<std::ifstream*> check_vec(nbFile);
  for(int i = 0;i < nbFile; ++i){
    check_vec[i] = new std::ifstream((name_vec[i]+".data").c_str());
  }


  bool success = true;
  for(unsigned i = 0; i < check_vec.size();++i){
    // if the reference file exists, compare with the previous version.
    if (*ref_vec[i]){
      if (!checkFiles(*check_vec[i],*ref_vec[i])){
        success = false;
      }
    }
    // otherwise, create it
    else{
      copyFile((name_vec[i]+".data"), (name_vec[i]+".ref"));
    }
    check_vec[i]->close();
    ref_vec[i]->close();
  }

  for (int i=0; i < nbFile; ++i) {
    delete check_vec[i];
    delete ref_vec[i];
    delete data_vec[i];
  }
  delete walk;

  return (success)?0:1;
}

void dumpTrajectory(MPCSolution &solution, std::vector<std::ofstream*> &data_vec) {
  for (int i = 0; i < solution.state_vec[0].CoMTrajX_.rows(); ++i) {
    *data_vec[0] << solution.state_vec[0].CoMTrajX_(i) << " " << solution.state_vec[1].CoMTrajX_(i) << " " << solution.state_vec[2].CoMTrajX_(i) << std::endl;
    *data_vec[1] << solution.state_vec[0].CoMTrajY_(i) << " " << solution.state_vec[1].CoMTrajY_(i) << " " << solution.state_vec[2].CoMTrajY_(i) << std::endl;
    *data_vec[2] << solution.CoPTrajX(i)  << std::endl;
    *data_vec[3] << solution.CoPTrajY(i) << std::endl;
    *data_vec[4] << solution.state_vec[0].leftFootTrajX_(i) << " " << solution.state_vec[1].leftFootTrajX_(i) << " " << solution.state_vec[2].leftFootTrajX_(i) << std::endl;
    *data_vec[5] << solution.state_vec[0].leftFootTrajY_(i) << " " << solution.state_vec[1].leftFootTrajY_(i) << " " << solution.state_vec[2].leftFootTrajY_(i) << std::endl;
    *data_vec[6] << solution.state_vec[0].rightFootTrajX_(i) << " " << solution.state_vec[1].rightFootTrajX_(i) << " " << solution.state_vec[2].rightFootTrajX_(i) << std::endl;
    *data_vec[7] << solution.state_vec[0].rightFootTrajY_(i) << " " << solution.state_vec[1].rightFootTrajY_(i) << " " << solution.state_vec[2].rightFootTrajY_(i) << std::endl;
    *data_vec[8] << solution.state_vec[0].rightFootTrajZ_(i) << " " << solution.state_vec[1].rightFootTrajZ_(i) << " " << solution.state_vec[2].rightFootTrajZ_(i) << std::endl;
    *data_vec[9] << solution.state_vec[0].leftFootTrajZ_(i) << " " << solution.state_vec[1].leftFootTrajZ_(i) << " " << solution.state_vec[2].leftFootTrajZ_(i) << std::endl;
  }
}

bool checkFiles(std::ifstream & fich1, std::ifstream & fich2) {
  bool equal=1;
  if (fich1 && fich2) {
    std::string lignef1;
    std::string lignef2;
    while (std::getline( fich1, lignef1) && std::getline( fich2, lignef2) && equal) {
      if (strcmp(lignef1.c_str(),lignef2.c_str())!=0) {
        equal = 0;
      }
    }
  }

  return equal;
}

int copyFile(const std::string & source, const std::string & destination) {
  std::ifstream ifs(source.c_str(), std::ios::binary);
  std::ofstream ofs(destination.c_str(), std::ios::binary);
  ofs << ifs.rdbuf();
  ofs.close();
  ifs.close();

  return 0;
}

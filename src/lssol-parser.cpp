#include <mpc-walkgen/lssol-parser.h>

//#include <lssol/lssol.h>
//TODO: How can we compile without the LSSOL sources?

using namespace MPCWalkgen;
using namespace Eigen;


LSSOLParser::LSSOLParser(int num_vars, int num_constr)
	:QPSolver(num_vars, num_constr)
	,kx_(num_vars)
	,bb_(1)
	,lambda_(num_vars + num_constr)
	,bu_(num_vars + num_constr)
	,bl_(num_vars + num_constr)
	,leniw_(num_vars)
	,lenw_(2 * num_vars * num_vars + 10 * num_vars + 6*num_constr)
	,war_(lenw_)
	,iwar_(leniw_)
	,inform_(0)
	,iter_(0)
	,obj_(0)
	,useCholesky_(true)
{
	//sendOption("Print Level = 0");
	//sendOption("Problem Type = QP4");
	//sendOption("warm start");
}

LSSOLParser::~LSSOLParser(){}

void LSSOLParser::Init() {}

void LSSOLParser::solve(VectorXd & qpSolution,
			VectorXi & constraints,
			VectorXd & initialSolution,
			VectorXi & initialConstraints,
			bool useWarmStart){
 
	reorderInitialSolution(initialSolution, initialConstraints);


	// Pile up XL and BL
	bl_.segment(0,      num_vars_) = var_l_bounds_vec_().block(0,0,num_vars_,1);
	bl_.segment(num_vars_, num_constr_) = constr_l_bounds_vec_().block(0,0,num_constr_,1);

	// Pile up XU and BU
	bu_.segment(0,      num_vars_) = var_u_bounds_vec_().block(0,0,num_vars_,1);
	bu_.segment(num_vars_, num_constr_) = constr_u_bounds_vec_().block(0,0,num_constr_,1);

	if (useWarmStart){
		qpSolution = initialSolution;
		constraints = initialConstraints;
	}else{
		if (qpSolution.rows() != num_vars_){
			qpSolution.setZero(num_vars_);
		}else{
			qpSolution.fill(0);
		}
		if (constraints.rows()!=num_vars_ + num_constr_){
			constraints.setZero(num_vars_ + num_constr_);
		}else{
			constraints.fill(0);
		}
	}

	for(int i=0;i<num_vars_;++i){
		kx_(i)=i+1;
	}

	// The error 6 of lssol "An input parameter is invalid" may be
	// difficult to debug. The following tests can help.
	assert(bl_.size() >= num_vars_ + num_constr_);
	assert(bu_.size() >= num_vars_ + num_constr_);
	assert(gradient_vec_().size() >= num_vars_);
	assert(constraints.size() >= num_vars_ + num_constr_);
	assert(leniw_>=num_vars_);
	assert((num_constr_ > 0)  || (lenw_ >=10*num_vars_));
	assert((num_constr_ == 0) || (lenw_ >=2*num_vars_*num_vars_ + 10*num_vars_+ 6*num_constr_));


//	lssol_(&num_vars_, &num_vars_,
//			&num_constr_, &num_constr_, &num_vars_,
//			matrixA_().data(), bl_.data(), bu_.data(),gradient_vec_().data(),
//			constraints.data(), kx_.data(), qpSolution.data(),
//			matrixQ_.cholesky().data(), bb_.data(), &inform_, &iter_, &obj_, lambda_.data(),
//			iwar_.data(), &leniw_, war_.data(), &lenw_);


	reorderSolution(qpSolution, constraints, initialConstraints);
}

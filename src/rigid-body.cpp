#include <mpc-walkgen/rigid-body.h>
#include <mpc-walkgen/debug.h>

using namespace MPCWalkgen;


RigidBody::RigidBody():mpc_parameters_(NULL)
,robot_data_(NULL)
,dyn_build_p_(NULL)
{}

RigidBody::~RigidBody() {}

void RigidBody::Init(const MPCParameters *mpc_parameters_p) {
	mpc_parameters_ = mpc_parameters_p;

	int num_dynamics = mpc_parameters_->GetNumRecomputations();
	dynamics_qp_vec_.resize(num_dynamics);

	int num_samples = mpc_parameters_->num_samples_act();
	int num_unst_modes = 1;//TODO: unstable modes
	motion_act_.SetZero(num_samples, num_unst_modes);
	wrench_.SetZero();
}

void RigidBody::Init(const RobotData *robot_data_p) {
	robot_data_ = robot_data_p;
}

void RigidBody::Init(DynamicsBuilder *dyn_build_p) {
	dyn_build_p_ = dyn_build_p;
}

void RigidBody::ComputeDynamics(SystemOrder dynamics_order) {
	assert(state_.z(0) > kEps);
	assert(mpc_parameters_->num_samples_horizon_max > 0);

	int num_samples = mpc_parameters_->num_samples_horizon_max;
	int num_dynamics = mpc_parameters_->GetNumRecomputations();

	// Build vector of sampling periods:
	// ---------------------------------
	std::vector<double> sampling_periods_vec(num_samples, mpc_parameters_->period_qpsample);
	int period_num = 0;
	for(; period_num < mpc_parameters_->num_samples_first_fine_period; period_num++) {
		sampling_periods_vec[period_num] = mpc_parameters_->period_recomputation;
	}
	for(; period_num < mpc_parameters_->num_samples_first_coarse_period + mpc_parameters_->num_samples_first_fine_period - 1; period_num++) {
		sampling_periods_vec[period_num] = mpc_parameters_->period_inter_samples;
	}

	int num_cycles_ext = static_cast<int>(mpc_parameters_->period_qpsample / mpc_parameters_->period_inter_samples);
	int num_cycles_int = static_cast<int>(mpc_parameters_->period_inter_samples / mpc_parameters_->period_recomputation);
	double first_period = mpc_parameters_->period_recomputation;

	std::vector<LinearDynamics>::iterator dyn_it = dynamics_qp_vec_.begin();
	double last_period = 0.;
	double first_coarse_period = mpc_parameters_->period_qpsample;
	for (int k = 0; k < num_cycles_ext; ++k) {
		double first_fine_period = mpc_parameters_->period_inter_samples;
		for (int i = 0; i < num_cycles_int; ++i) {
			sampling_periods_vec.back() = last_period;
			sampling_periods_vec.at(mpc_parameters_->num_samples_first_fine_period) = first_fine_period;
			sampling_periods_vec.at(mpc_parameters_->num_samples_first_fine_period + mpc_parameters_->num_samples_first_coarse_period - 1) = first_coarse_period;
			if (sampling_periods_vec.back() < kEps) {
				sampling_periods_vec.pop_back();
				dyn_build_p_->Build(dynamics_order, *dyn_it, state_.z(0), sampling_periods_vec, num_samples, false);
				Debug::Cout("sampling_periods_vec", sampling_periods_vec);
				sampling_periods_vec.push_back(0.);
			} else {
				dyn_build_p_->Build(dynamics_order, *dyn_it, state_.z(0), sampling_periods_vec, num_samples, false);
				Debug::Cout("sampling_periods_vec", sampling_periods_vec);
			}
			++dyn_it;
			first_fine_period -= mpc_parameters_->period_recomputation;
			last_period += mpc_parameters_->period_recomputation;
		}
		first_coarse_period -= mpc_parameters_->period_inter_samples;
	}

	num_samples = mpc_parameters_->num_samples_act();
	sampling_periods_vec.resize(num_samples);
	std::fill(sampling_periods_vec.begin(), sampling_periods_vec.end(), mpc_parameters_->period_actsample);
	dyn_build_p_->Build(dynamics_order, dynamics_act_, state_.z(0), sampling_periods_vec, num_samples, true);
}

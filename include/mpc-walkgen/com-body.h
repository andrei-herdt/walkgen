#pragma once
#ifndef MPC_WALKGEN_COM_BODY_H
#define MPC_WALKGEN_COM_BODY_H

#include <mpc-walkgen/types.h>
#include <mpc-walkgen/rigid-body.h>

namespace MPCWalkgen{

class CoMBody:public RigidBody{

	//
	// Public methods:
	//
public:
	CoMBody();
	virtual ~CoMBody();

	virtual void Interpolate(MPCSolution &solution, double current_time, const Reference &ref);
	//
	// Private methods:
	//
private:
	void InterpolateTrunkYaw(MPCSolution &solution,
			const Reference &ref
	);

};
}
#endif // MPC_WALKGEN_COM_BODY_H

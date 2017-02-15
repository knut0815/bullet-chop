#include "CHOP_CPlusPlusBase.h"

/*

This example file implements a class that does 2 different things depending on
if a CHOP is connected to the CPlusPlus CHOPs input or not.
The example is timesliced, which is the more complex way of working.

If an input is connected the node will output the same number of channels as the
input and divide the first 'N' samples in the input channel by 2. 'N' being the current
timeslice size. This is noteworthy because if the input isn't changing then the output
will look wierd since depending on the timeslice size some number of the first samples
of the input will get used.

If no input is connected then the node will output a smooth sine wave at 120hz.
*/

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"

#include <iostream>
#include <vector>
#include <array>
#include <random>

// To get more help about these functions, look at CHOP_CPlusPlusBase.h
class BulletCHOP : public CHOP_CPlusPlusBase
{
public:
	BulletCHOP(const OP_NodeInfo* info);
	virtual ~BulletCHOP();

	virtual void		getGeneralInfo(CHOP_GeneralInfo* ) override;
	virtual bool		getOutputInfo(CHOP_OutputInfo*) override;
	virtual const char*	getChannelName(int index, void* reserved) override;

	virtual void		execute(const CHOP_Output*,
								OP_Inputs*,
								void* reserved) override;


	virtual int			getNumInfoCHOPChans() override;
	virtual void		getInfoCHOPChan(int index,
										OP_InfoCHOPChan* chan) override;

	virtual bool		getInfoDATSize(OP_InfoDATSize* infoSize) override;
	virtual void		getInfoDATEntries(int index,
											int nEntries,
											OP_InfoDATEntries* entries) override;

	virtual void		setupParameters(OP_ParameterManager* manager) override;
	virtual void		pulsePressed(const char* name) override;

private:

	void createCollisionPlane(const btVector3 &normal, const btVector3 &position);

	// We don't need to store this pointer, but we do for the example.
	// The OP_NodeInfo class store information about the node that's using
	// this instance of the class (like its name).
	const OP_NodeInfo *myNodeInfo;

	// In this example this value will be incremented each time the execute()
	// function is called, then passes back to the CHOP 
	int	myExecuteCount;
	double myOffset;

	btBroadphaseInterface* broadphase;
	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btMLCPSolver* solver;
	btDiscreteDynamicsWorld* dynamicsWorld;

	btCollisionShape* groundShape;

	std::vector<btCollisionShape*> collisionPlaneShapes;
	std::vector<btRigidBody*> collisionPlanes;
	
	btCollisionShape* collisionShape;
	std::vector<btRigidBody*> rigidBodies;

	btRigidBody* groundRigidBody;

	bool bulletInitialized;
};

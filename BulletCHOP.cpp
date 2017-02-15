/* Shared Use License: This file is owned by Derivative Inc. (Derivative) and
 * can only be used, and/or modified for use, in conjunction with 
 * Derivative's TouchDesigner software, and only if you are a licensee who has
 * accepted Derivative's TouchDesigner license or assignment agreement (which
 * also govern the use of this file).  You may share a modified version of this
 * file with another authorized licensee of Derivative's TouchDesigner software.
 * Otherwise, no redistribution or sharing of this file, with or without
 * modification, is permitted.
 */

#include "BulletCHOP.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>

// These functions are basic C function, which the DLL loader can find
// much easier than finding a C++ Class.
// The DLLEXPORT prefix is needed so the compile exports these functions from the .dll
// you are creating
extern "C"
{

DLLEXPORT
int
GetCHOPAPIVersion(void)
{
	// Always return CHOP_CPLUSPLUS_API_VERSION in this function.
	return CHOP_CPLUSPLUS_API_VERSION;
}

DLLEXPORT
CHOP_CPlusPlusBase*
CreateCHOPInstance(const OP_NodeInfo* info)
{
	// Return a new instance of your class every time this is called.
	// It will be called once per CHOP that is using the .dll
	return new BulletCHOP(info);
}

DLLEXPORT
void
DestroyCHOPInstance(CHOP_CPlusPlusBase* instance)
{
	// Delete the instance here, this will be called when
	// Touch is shutting down, when the CHOP using that instance is deleted, or
	// if the CHOP loads a different DLL
	delete (BulletCHOP*)instance;
}

};


BulletCHOP::BulletCHOP(const OP_NodeInfo* info) : myNodeInfo(info)
{
	myExecuteCount = 0;
	myOffset = 0.0;

	// General Bullet objects
	broadphase = new btDbvtBroadphase();
	collisionConfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	//solver = new btSequentialImpulseConstraintSolver;
	solver = new btMLCPSolver(new btSolveProjectedGaussSeidel());

	// Configure world dynamics
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
	dynamicsWorld->setGravity(btVector3(0, -50, 0)); // was 10
	dynamicsWorld->getSolverInfo().m_erp2 = 0.f;
	dynamicsWorld->getSolverInfo().m_globalCfm = 0.f;
	dynamicsWorld->getSolverInfo().m_numIterations = 1;
	dynamicsWorld->getSolverInfo().m_solverMode = SOLVER_SIMD;
	dynamicsWorld->getSolverInfo().m_splitImpulse = false;

	// Shapes and collision planes
	const int scale = 20;
	createCollisionPlane(btVector3(0, 1, 0), btVector3(0, -scale, 0));
	createCollisionPlane(btVector3(0, -1, 0), btVector3(0, scale, 0));
	createCollisionPlane(btVector3(1, 0, 0), btVector3(-scale, 0, 0));
	createCollisionPlane(btVector3(-1, 0, 0), btVector3(scale, 0, 0));
	createCollisionPlane(btVector3(0, 0, 1), btVector3(0, 0, -scale));
	createCollisionPlane(btVector3(0, 0, -1), btVector3(0, 0, scale));
	bulletInitialized = false;
}

void BulletCHOP::createCollisionPlane(const btVector3 &normal, const btVector3 &position)
{
	collisionPlaneShapes.push_back(new btStaticPlaneShape(normal, 1));

	btDefaultMotionState* motion = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), position));
	btRigidBody::btRigidBodyConstructionInfo construction(0, motion, collisionPlaneShapes.back(), btVector3(0, 0, 0));
	collisionPlanes.push_back(new btRigidBody(construction));
	collisionPlanes.back()->setRestitution(1.0f);
	dynamicsWorld->addRigidBody(collisionPlanes.back());
}

BulletCHOP::~BulletCHOP()
{
	// Delete all of the rigid bodies we created 
	for (size_t i = 0; i < rigidBodies.size(); ++i)
	{
		dynamicsWorld->removeRigidBody(rigidBodies[i]);
		delete rigidBodies[i]->getMotionState();
		delete rigidBodies[i];
	}

	delete collisionShape;

	for (size_t i = 0; i < collisionPlanes.size(); ++i)
	{
		dynamicsWorld->removeRigidBody(collisionPlanes[i]);
		delete collisionPlanes[i]->getMotionState();
		delete collisionPlanes[i];

		delete collisionPlaneShapes[i];
	}

	delete dynamicsWorld;
	delete solver;
	delete collisionConfiguration;
	delete dispatcher;
	delete broadphase;
}

void
BulletCHOP::getGeneralInfo(CHOP_GeneralInfo* ginfo)
{
	ginfo->cookEveryFrameIfAsked = true;
	ginfo->inputMatchIndex = 0;
}

bool
BulletCHOP::getOutputInfo(CHOP_OutputInfo* info)
{
	if (info->opInputs->getNumInputs() > 0)
	{
		return false;
	}
	else
	{
		info->numChannels = 1;
		info->sampleRate = 120;
		return true;
	}
}

const char*
BulletCHOP::getChannelName(int index, void* reserved)
{
	return "bullet";
}

void
BulletCHOP::execute(const CHOP_Output* output, OP_Inputs* inputs, void* reserved)
{
	myExecuteCount++;

	double scale = inputs->getParDouble("Scale");
	const OP_CHOPInput	*cinput = inputs->getInputCHOP(0);

	if (inputs->getNumInputs() > 0 && output->numChannels >= 3)
	{
		if (!bulletInitialized)
		{
			std::cout << "Initializing bullet" << std::endl;
			const float *tx = cinput->getChannelData(0);
			const float *ty = cinput->getChannelData(1);
			const float *tz = cinput->getChannelData(2);

			collisionShape = new btSphereShape(0.5f);
			
			btScalar mass = 4;
			btVector3 inertia(0, 0, 0);
			collisionShape->calculateLocalInertia(mass, inertia);

			for (int j = 0; j < output->numSamples; j++)
			{
				btTransform startTransform;
				startTransform.setOrigin(btVector3(tx[j], ty[j], tz[j]));

				btDefaultMotionState* motion = new btDefaultMotionState(startTransform);
				btRigidBody::btRigidBodyConstructionInfo construction(mass, motion, collisionShape, inertia);

				rigidBodies.push_back(new btRigidBody(construction));
				dynamicsWorld->addRigidBody(rigidBodies[j]);
				rigidBodies[j]->setLinearVelocity(btVector3(0.0f, -10.0f, 0.0f));
				rigidBodies[j]->setRestitution(0.9f);
				
				static std::random_device rd;
				static std::mt19937 mt(rd());
				static std::uniform_real_distribution<double> dist(-200.0,200.0);

				rigidBodies[j]->applyCentralImpulse(btVector3(dist(mt), dist(mt), dist(mt)));
			}
			std::cout << "Initialized " << rigidBodies.size() << " rigid bodies" << std::endl;

			bulletInitialized = true;
		}
		else
		{
			const float *rx = cinput->getChannelData(3);
			const float *ry = cinput->getChannelData(4);
			const float *rz = cinput->getChannelData(5);
			
			// Run the simulation
			dynamicsWorld->stepSimulation(1 / 60.0f);

			for (size_t i = 0; i < rigidBodies.size(); ++i)
			{

				
				btTransform trans;
				rigidBodies[i]->getMotionState()->getWorldTransform(trans);

				auto txNext = trans.getOrigin().getX();
				auto tyNext = trans.getOrigin().getY();
				auto tzNext = trans.getOrigin().getZ();
				auto quat = trans.getRotation();

				for (size_t j = 0; j < output->numSamples; ++j)
				{
					output->channels[0][i] = static_cast<float>(txNext);
					output->channels[1][i] = static_cast<float>(tyNext);
					output->channels[2][i] = static_cast<float>(tzNext);
					output->channels[3][i] = rx[i];
					output->channels[4][i] = ry[i];
					output->channels[5][i] = rz[i];
				}
			}
		}
	}
}

int
BulletCHOP::getNumInfoCHOPChans()
{
	return 2;
}

void
BulletCHOP::getInfoCHOPChan(int index,
										OP_InfoCHOPChan* chan)
{
	// This function will be called once for each channel we said we'd want to return
	// In this example it'll only be called once.

	if (index == 0)
	{
		chan->name = "executeCount";
		chan->value = (float)myExecuteCount;
	}

	if (index == 1)
	{
		chan->name = "offset";
		chan->value = (float)myOffset;
	}
}

bool		
BulletCHOP::getInfoDATSize(OP_InfoDATSize* infoSize)
{
	infoSize->rows = 2;
	infoSize->cols = 2;
	// Setting this to false means we'll be assigning values to the table
	// one row at a time. True means we'll do it one column at a time.
	infoSize->byColumn = false;
	return true;
}

void
BulletCHOP::getInfoDATEntries(int index,
										int nEntries,
										OP_InfoDATEntries* entries)
{
	// It's safe to use static buffers here because Touch will make it's own
	// copies of the strings immediately after this call returns
	// (so the buffers can be reuse for each column/row)
	static char tempBuffer1[4096];
	static char tempBuffer2[4096];

	if (index == 0)
	{
		// Set the value for the first column
		strcpy_s(tempBuffer1, "executeCount");
		entries->values[0] = tempBuffer1;

		// Set the value for the second column
		sprintf_s(tempBuffer2, "%d", myExecuteCount);
		entries->values[1] = tempBuffer2;
	}

	if (index == 1)
	{
		// Set the value for the first column
		strcpy_s(tempBuffer1, "offset");
		entries->values[0] = tempBuffer1;

		// Set the value for the second column
		sprintf_s(tempBuffer2, "%g", myOffset);
		entries->values[1] = tempBuffer2;
	}
}

void
BulletCHOP::setupParameters(OP_ParameterManager* manager)
{
	// speed
	{
		OP_NumericParameter	np;

		np.name = "Speed";
		np.label = "Speed";
		np.defaultValues[0] = 1.0;
		np.minSliders[0] = -10.0;
		np.maxSliders[0] =  10.0;
		
		ParAppendResult res = manager->appendFloat(np);
		assert(res == PARAMETER_APPEND_SUCCESS);
	}

	// scale
	{
		OP_NumericParameter	np;

		np.name = "Scale";
		np.label = "Scale";
		np.defaultValues[0] = 1.0;
		np.minSliders[0] = -10.0;
		np.maxSliders[0] =  10.0;
		
		ParAppendResult res = manager->appendFloat(np);
		assert(res == PARAMETER_APPEND_SUCCESS);
	}

	// shape
	{
		OP_StringParameter	sp;

		sp.name = "Shape";
		sp.label = "Shape";

		sp.defaultValue = "Sine";

		const char *names[] = { "Sine", "Square", "Ramp" };
		const char *labels[] = { "Sine", "Square", "Ramp" };

		ParAppendResult res = manager->appendMenu(sp, 3, names, labels);
		assert(res == PARAMETER_APPEND_SUCCESS);
	}

	// pulse
	{
		OP_NumericParameter	np;

		np.name = "Reset";
		np.label = "Reset";
		
		ParAppendResult res = manager->appendPulse(np);
		assert(res == PARAMETER_APPEND_SUCCESS);
	}

}

void 
BulletCHOP::pulsePressed(const char* name)
{
	if (!strcmp(name, "Reset"))
	{
		bulletInitialized = false;
	}
}


#include "StdAfx.h"

#include <functional>

#include <cmodel.h>

#include "Physics_Environment.h"
#include "Physics.h"
#include "Physics_Object.h"
#include "Physics_ShadowController.h"
#include "Physics_PlayerController.h"
#include "Physics_FluidController.h"
#include "Physics_DragController.h"
#include "Physics_MotionController.h"
#include "Physics_Constraint.h"
#include "Physics_Collision.h"
#include "Physics_VehicleController.h"
#include "Physics_SoftBody.h"
#include "miscmath.h"
#include "convert.h"

#include <mutex>

#if DEBUG_DRAW
	#include "DebugDrawer.h"
#endif

// Multithreading stuff

//#define USE_PARALLEL_DISPATCHER
//#define USE_PARALLEL_SOLVER // NOT COMPLETE

#if defined(USE_PARALLEL_DISPATCHER) || defined(USE_PARALLEL_SOLVER)
	#define MULTITHREADED
#endif

#ifdef USE_PARALLEL_DISPATCHER
	#include "BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"
#endif

#ifdef USE_PARALLEL_SOLVER
	//#include "BulletMultiThreaded/btParallelConstraintSolver.h"
#endif

#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"

#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

/*****************************
* MISC. CLASSES
*****************************/

class IDeleteQueueItem {
	public:
		virtual void Delete() = 0;
};

template <typename T>
class CDeleteProxy : public IDeleteQueueItem {
	public:
		CDeleteProxy(T *pItem) : m_pItem(pItem) {}
		virtual void Delete() { delete m_pItem; }
	private:
		T *m_pItem;
};

class CDeleteQueue {
	public:
		void Add(IDeleteQueueItem *pItem) {
			m_list.push_back(pItem);
		}

		template <typename T>
		void QueueForDelete(T *pItem) {
			Add(new CDeleteProxy<T>(pItem));
		}

		void DeleteAll() {
			for (int i = m_list.size()-1; i >= 0; --i) {
				m_list[i]->Delete();
			}
			m_list.clear();
		}
	private:
		std::vector<IDeleteQueueItem *> m_list;
};

bool CCollisionSolver::needBroadphaseCollision(btBroadphaseProxy *proxy0, btBroadphaseProxy *proxy1) const {
	btRigidBody *body0 = btRigidBody::upcast((btCollisionObject *)proxy0->m_clientObject);
	btRigidBody *body1 = btRigidBody::upcast((btCollisionObject *)proxy1->m_clientObject);
	if (!body0 || !body1) {
		btCollisionObject *colObj0 = (btCollisionObject *)proxy0->m_clientObject;
		btCollisionObject *colObj1 = (btCollisionObject *)proxy1->m_clientObject;

		// Check if one of them is a soft body
		if (colObj0->getInternalType() & btCollisionObject::CO_SOFT_BODY || colObj1->getInternalType() & btCollisionObject::CO_SOFT_BODY) {
			return true;
		}

		if (body0)
			return !(body0->isStaticObject());
		
		if (body1)
			return !(body1->isStaticObject());

		return false;
	}

	CPhysicsObject *pObject0 = (CPhysicsObject *)body0->getUserPointer();
	CPhysicsObject *pObject1 = (CPhysicsObject *)body1->getUserPointer();

	bool collides = NeedsCollision(pObject0, pObject1);
	collides = collides && (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask);
	collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);

	if (!collides) {
		// Clean this pair from the cache
		m_pEnv->GetBulletEnvironment()->getBroadphase()->getOverlappingPairCache()->removeOverlappingPair(proxy0, proxy1, m_pEnv->GetBulletEnvironment()->getDispatcher());
	}
			
	return collides;
}

bool CCollisionSolver::NeedsCollision(CPhysicsObject *pObject0, CPhysicsObject *pObject1) const {
	if (pObject0 && pObject1) {
		if (!pObject0->IsCollisionEnabled() || !pObject1->IsCollisionEnabled())
			return false;

		// No kinematic->static collisions
		if ((pObject0->GetObject()->getCollisionFlags() & btCollisionObject::CF_KINEMATIC_OBJECT && pObject1->IsStatic())
		 || (pObject1->GetObject()->getCollisionFlags() & btCollisionObject::CF_KINEMATIC_OBJECT && pObject0->IsStatic()))
			return false;

		// No static->static collisions
		if (pObject0->IsStatic() && pObject1->IsStatic())
			return false;

		// No shadow->shadow collisions
		if (pObject0->GetShadowController() && pObject1->GetShadowController())
			return false;

		if ((pObject0->GetCallbackFlags() & CALLBACK_ENABLING_COLLISION) || (pObject1->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE)) return false;
		if ((pObject1->GetCallbackFlags() & CALLBACK_ENABLING_COLLISION) || (pObject0->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE)) return false;

		if (m_pSolver && !m_pSolver->ShouldCollide(pObject0, pObject1, pObject0->GetGameData(), pObject1->GetGameData())) 
			return false;
	} else {
		// One of the objects has no phys object...
		if (pObject0 && !pObject0->IsCollisionEnabled())
			return false;

		if (pObject1 && !pObject1->IsCollisionEnabled())
			return false;
	}

	return true;
}

void SerializeWorld_f(const CCommand &args) {
	if (args.ArgC() != 3) {
		Msg("Usage: vphysics_serialize <index> <name>\n");
		return;
	}

	CPhysicsEnvironment *pEnv = (CPhysicsEnvironment *)g_Physics.GetActiveEnvironmentByIndex(atoi(args.Arg(2)));
	if (pEnv) {
		btDiscreteDynamicsWorld *pWorld = (btDiscreteDynamicsWorld *)pEnv->GetBulletEnvironment();
		Assert(pWorld);

		btSerializer *pSerializer = new btDefaultSerializer;
		pWorld->serialize(pSerializer);

		// FIXME: We shouldn't be using this. Find the appropiate method from valve interfaces.
		const char *pName = args.Arg(3);
		FILE *pFile = fopen(pName, "wb");
		if (pFile) {
			fwrite(pSerializer->getBufferPointer(), pSerializer->getCurrentBufferSize(), 1, pFile);
			fclose(pFile);
		} else {
			Warning("Couldn't open \"%s\" for writing!\n", pName);
		}
	} else {
		Warning("Invalid environment index supplied!\n");
	}
}

static ConCommand cmd_serializeworld("vphysics_serialize", SerializeWorld_f, "Serialize environment by index (usually 0=server, 1=client)\n\tDumps the file out to the exe directory.");

/*******************************
* CLASS CObjectTracker
*******************************/

class CObjectTracker {
	public:
		CObjectTracker(CPhysicsEnvironment *pEnv, IPhysicsObjectEvent *pObjectEvents) {
			m_pEnv = pEnv;
			m_pObjEvents = pObjectEvents;
		}

		int GetActiveObjectCount() const {
			return m_activeObjects.size();
		}

		void GetActiveObjects(IPhysicsObject **pOutputObjectList) const {
			if (!pOutputObjectList) return;

			int size = m_activeObjects.size();
			for (int i = 0; i < size; i++) {
				pOutputObjectList[i] = m_activeObjects[i];
			}
		}

		void SetObjectEventHandler(IPhysicsObjectEvent *pEvents) {
			m_pObjEvents = pEvents;
		}

		void ObjectRemoved(CPhysicsObject *pObject) {
			auto it = std::find(m_activeObjects.begin(), m_activeObjects.end(), pObject);
			if (it != m_activeObjects.end())
				m_activeObjects.erase(it);
		}

		void Tick() {
			btDiscreteDynamicsWorld *pBulletEnv = m_pEnv->GetBulletEnvironment();
			btCollisionObjectArray &colObjArray = pBulletEnv->getCollisionObjectArray();
			for (int i = 0; i < colObjArray.size(); i++) {
				CPhysicsObject *pObj = (CPhysicsObject *)colObjArray[i]->getUserPointer();
				if (!pObj)
					continue;
 
				// Don't add objects marked for delete
				if (pObj->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE) {
					continue;
				}

				if (colObjArray[i]->getActivationState() != pObj->GetLastActivationState()) {
					int newState = colObjArray[i]->getActivationState();

					if (!pObj->UpdateSleepState(newState)) {
						continue;
					}

					auto it = std::find(m_activeObjects.begin(), m_activeObjects.end(), pObj);
					switch (pObj->GetSleepState()) {
						case CPhysicsObject::SleepState::Awake:
							// Don't add the object twice!
							if (it == m_activeObjects.end())
								m_activeObjects.push_back(pObj);
							break;
						case CPhysicsObject::SleepState::Sleep:
							ObjectRemoved(pObj);
							break;
					}

					if (m_pObjEvents) {
						switch (pObj->GetSleepState()) {
							// FIXME: Objects may call objectwake twice if they go from disable_deactivation -> active_tag
							case CPhysicsObject::SleepState::Awake:
								m_pObjEvents->ObjectWake(pObj);
								break;
							case CPhysicsObject::SleepState::Sleep:
								m_pObjEvents->ObjectSleep(pObj);
								break;
						}
					}

					pObj->SetLastActivationState(newState);
				}
			}
		}

	private:
		CPhysicsEnvironment *m_pEnv;
		IPhysicsObjectEvent *m_pObjEvents;

		std::vector<CPhysicsObject *> m_activeObjects;
};

/*******************************
* CLASS CPhysicsCollisionData
*******************************/

class CPhysicsCollisionData : public IPhysicsCollisionData {
	public:
		CPhysicsCollisionData(btManifoldPoint *manPoint) {
			ConvertDirectionToHL(manPoint->m_normalWorldOnB, m_surfaceNormal);
			ConvertPosToHL(manPoint->getPositionWorldOnA(), m_contactPoint);
			ConvertPosToHL(manPoint->m_lateralFrictionDir1, m_contactSpeed);	// FIXME: Need the correct variable from the manifold point
		}

		// normal points toward second object (object index 1)
		void GetSurfaceNormal(Vector &out) {
			out = m_surfaceNormal;
		}

		// contact point of collision (in world space)
		void GetContactPoint(Vector &out) {
			out = m_contactPoint;
		}

		// speed of surface 1 relative to surface 0 (in world space)
		void GetContactSpeed(Vector &out) {
			out = m_contactSpeed;
		}

	private:
		Vector m_surfaceNormal;
		Vector m_contactPoint;
		Vector m_contactSpeed;
};

/*********************************
* CLASS CCollisionEventListener
*********************************/

class CCollisionEventListener {
	public:
		CCollisionEventListener(CPhysicsEnvironment *pEnv) {
			m_pEnv = pEnv;
			m_pCallback = NULL;
			memset(&m_tmpEvent, 0, sizeof(m_tmpEvent));
		}

		void nearCallback(btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo)
		{
			btCollisionObject* colObj0 = (btCollisionObject*)collisionPair.m_pProxy0->m_clientObject;
			btCollisionObject* colObj1 = (btCollisionObject*)collisionPair.m_pProxy1->m_clientObject;

			//CPhysicsObject* physObA = (CPhysicsObject*)colObj0->getUserPointer();
			//CPhysicsObject* physObB = (CPhysicsObject*)colObj1->getUserPointer();

			// These are our own internal objects, don't do callbacks on them.
			if (colObj0->getInternalType() == btCollisionObject::CO_GHOST_OBJECT || colObj1->getInternalType() == btCollisionObject::CO_GHOST_OBJECT) {
				dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
				return;
			}

			if (dispatcher.needsCollision(colObj0, colObj1))
			{
				btCollisionObjectWrapper obj0Wrap(0, colObj0->getCollisionShape(), colObj0, colObj0->getWorldTransform(), -1, -1);
				btCollisionObjectWrapper obj1Wrap(0, colObj1->getCollisionShape(), colObj1, colObj1->getWorldTransform(), -1, -1);

				//dispatcher will keep algorithms persistent in the collision pair
				if (!collisionPair.m_algorithm)
				{
					collisionPair.m_algorithm = dispatcher.findAlgorithm(&obj0Wrap, &obj1Wrap, 0, BT_CONTACT_POINT_ALGORITHMS);
				}

				if (collisionPair.m_algorithm)
				{
					btManifoldResult contactPointResult(&obj0Wrap, &obj1Wrap);

					if (dispatchInfo.m_dispatchFunc == btDispatcherInfo::DISPATCH_DISCRETE)
					{
						//discrete collision detection query

						collisionPair.m_algorithm->processCollision(&obj0Wrap, &obj1Wrap, dispatchInfo, &contactPointResult);

						auto manifold = contactPointResult.getPersistentManifold();

						if (manifold == nullptr) {
							return;
						}

						int contacts = manifold->getNumContacts();
						for (int i = 0; i < contacts; i++) {
							if (m_pCallback) {
								auto& manPoint = manifold->getContactPoint(i);
								preSolveContact(colObj0, colObj1, &manPoint);
							}
						}

						dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);

						for (int i = 0; i < contacts; i++) {
							if (m_pCallback) {
								auto& manPoint = manifold->getContactPoint(i);
								postSolveContact(colObj0, colObj1, &manPoint);
								friction(colObj0, colObj1, &manPoint);
							}
						}

						return;
					}
					else
					{
						//continuous collision detection query, time of impact (toi)
						btScalar toi = collisionPair.m_algorithm->calculateTimeOfImpact(colObj0, colObj1, dispatchInfo, &contactPointResult);
						if (dispatchInfo.m_timeOfImpact > toi)
							dispatchInfo.m_timeOfImpact = toi;
					}
				}
			}

			dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
		}

		// TODO: Optimize this, heavily!
		void preSolveContact(const btCollisionObject *colObj0, const btCollisionObject *colObj1, btManifoldPoint *cp) {
			CPhysicsObject *pObj0 = (CPhysicsObject *)colObj0->getUserPointer();
			CPhysicsObject *pObj1 = (CPhysicsObject *)colObj1->getUserPointer();
			if (pObj0->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE || pObj1->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE)
				return;

			//auto rb0 = btRigidBody::upcast(colObj0);
			//auto rb1 = btRigidBody::upcast(colObj1);

			unsigned int flags0 = pObj0->GetCallbackFlags();
			unsigned int flags1 = pObj1->GetCallbackFlags();

			//Msg("preCollision: %f %f\n", cp->getAppliedImpulse(), m_tmpEvent.collisionSpeed);

			// Clear it
			memset(&m_tmpEvent, 0, sizeof(m_tmpEvent));
			m_tmpEvent.collisionSpeed = 0.0; // Invalid pre-collision
			m_tmpEvent.deltaCollisionTime = 1.0; // FIXME: Find a way to track the real delta time
			m_tmpEvent.isCollision = (flags0 & flags1 & CALLBACK_GLOBAL_COLLISION) ? true : false; // False when either one of the objects don't have CALLBACK_GLOBAL_COLLISION
			m_tmpEvent.isShadowCollision = ((flags0 ^ flags1) & CALLBACK_SHADOW_COLLISION) ? true : false; // True when only one of the objects is a shadow (if both are shadow, it's handled by the game)

			m_tmpEvent.pObjects[0] = pObj0;
			m_tmpEvent.pObjects[1] = pObj1;	
			m_tmpEvent.surfaceProps[0] = pObj0->GetMaterialIndex();
			m_tmpEvent.surfaceProps[1] = pObj1->GetMaterialIndex();

			if ((pObj0->IsStatic() && !(flags1 & CALLBACK_GLOBAL_COLLIDE_STATIC)) || (pObj1->IsStatic() && !(flags0 & CALLBACK_GLOBAL_COLLIDE_STATIC))) {
				m_tmpEvent.isCollision = false;
			}

			if (!m_tmpEvent.isCollision && !m_tmpEvent.isShadowCollision) return;

			CPhysicsCollisionData data(cp);
			m_tmpEvent.pInternalData = &data;

			if (m_pCallback) {
				m_pCallback->PreCollision(&m_tmpEvent);
			}
		}

		// TODO: Optimize this, heavily!
		void postSolveContact(const btCollisionObject *colObj0, const btCollisionObject *colObj1, btManifoldPoint *cp) {
			// FIXME: Problem with bullet code, only one solver body created for static objects!
			// There could be more than one static object created by us!
			CPhysicsObject *pObj0 = (CPhysicsObject *)colObj0->getUserPointer();
			CPhysicsObject *pObj1 = (CPhysicsObject *)colObj1->getUserPointer();
			if (pObj0->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE || pObj1->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE)
				return;

			if (!m_tmpEvent.isCollision && !m_tmpEvent.isShadowCollision) return;

			auto rb0 = btRigidBody::upcast(colObj0);
			auto rb1 = btRigidBody::upcast(colObj1);

			auto invMass0 = rb0->getInvMass();
			auto invMass1 = rb1->getInvMass();

			btScalar combinedMass = 0.0;
			if (invMass0 != 0.0) {
				combinedMass += 1.0 / invMass0;
			}
			if (invMass1 != 0.0) {
				combinedMass += 1.0 / invMass1;
			}

			if (combinedMass != 0.0) {
				m_tmpEvent.collisionSpeed = ConvertForceImpulseToHL(abs(cp->getAppliedImpulse())) / (float)combinedMass; // Speed of body 1 rel to body 2 on axis of constraint 
				//Msg("postCollision: %f %f\n", cp->getAppliedImpulse(), m_tmpEvent.collisionSpeed);
			}
			else {
				m_tmpEvent.collisionSpeed = 0.0;
			}

			// FIXME: Find a way to track the real delta time
			// IVP tracks this delta time between object pairs
			m_tmpEvent.deltaCollisionTime = 1.0;
			/*
			m_tmpEvent.isCollision = (flags0 & flags1 & CALLBACK_GLOBAL_COLLISION); // False when either one of the objects don't have CALLBACK_GLOBAL_COLLISION
			m_tmpEvent.isShadowCollision = (flags0 ^ flags1) & CALLBACK_SHADOW_COLLISION; // True when only one of the objects is a shadow

			m_tmpEvent.pObjects[0] = pObj0;
			m_tmpEvent.pObjects[1] = pObj1;	
			m_tmpEvent.surfaceProps[0] = pObj0 ? pObj0->GetMaterialIndex() : 0;
			m_tmpEvent.surfaceProps[1] = pObj1 ? pObj1->GetMaterialIndex() : 0;
			*/

			CPhysicsCollisionData data(cp);
			m_tmpEvent.pInternalData = &data;

			if (m_pCallback) {
				m_pCallback->PostCollision(&m_tmpEvent);
			}
		}

		void friction(const btCollisionObject *colObj0, const btCollisionObject *colObj1, btManifoldPoint *cp) {
			CPhysicsObject *pObj0 = (CPhysicsObject *)colObj0->getUserPointer();
			CPhysicsObject *pObj1 = (CPhysicsObject *)colObj1->getUserPointer();
			if (pObj0->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE || pObj1->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE)
				return;
			
			if (!(pObj1->GetCallbackFlags() & CALLBACK_GLOBAL_FRICTION))
				return;

			// FRICTION CALLBACK
			// FIXME: We need to find the energy used by the friction! Bullet doesn't provide this in the manifold point.
			// This may not be the proper variable but whatever, as of now it works.

			auto rb1 = btRigidBody::upcast(colObj1);
			auto invMass = rb1->getInvMass();

			//Msg("friction: %f %f\n", cp->getAppliedImpulse(), invMass);

			auto energy = abs(cp->getAppliedImpulse() * invMass);

			if (energy > 0.05f) {
				CPhysicsCollisionData data(cp);

				if (m_pCallback) {
					m_pCallback->Friction(pObj1,
						ConvertEnergyToHL(energy),
						pObj1->GetMaterialIndex(),
						pObj0->GetMaterialIndex(),
						&data);
				}
			}
		}

		void SetCollisionEventCallback(IPhysicsCollisionEvent *pCallback) {
			m_pCallback = pCallback;
		}

	private:
		CPhysicsEnvironment *m_pEnv;
		IPhysicsCollisionEvent *m_pCallback;

		vcollisionevent_t m_tmpEvent;
};

/*******************************
* CLASS CPhysicsEnvironment
*******************************/

// Environment ConVars
// Change the hardcoded min and max if you change the min and max here!
#ifdef MULTITHREADED
static void vphysics_numthreads_Change(IConVar *var, const char *pOldValue, float flOldValue);
static ConVar vphysics_numthreads("vphysics_numthreads", "4", FCVAR_ARCHIVE, "Amount of threads to use in simulation (don't set this too high).", true, 1, true, 8, vphysics_numthreads_Change);

static void vphysics_numthreads_Change(IConVar *var, const char *pOldValue, float flOldValue) {
	int newVal = vphysics_numthreads.GetInt();
	if (newVal <= 0 || newVal > 8) return;

	Msg("VPhysics: Resizing to %d threads\n", newVal);

	for (int i = 0; i < g_Physics.GetActiveEnvironmentCount(); i++) {
		((CPhysicsEnvironment *)g_Physics.GetActiveEnvironmentByIndex(i))->ChangeThreadCount(newVal);
	}
}
#endif

class btCollisionDispatcherCustom : public btCollisionDispatcher
{
public:
	btCollisionDispatcherCustom(CPhysicsEnvironment* env, btCollisionConfiguration* collisionConfiguration) : btCollisionDispatcher(collisionConfiguration), pEnv(env) {}

	virtual ~btCollisionDispatcherCustom() {}

	CPhysicsEnvironment* Env() {
		return pEnv;
	}

private:
	CPhysicsEnvironment* pEnv;
};

CPhysicsEnvironment::CPhysicsEnvironment() {
	m_deleteQuick		= false;
	m_bUseDeleteQueue	= false;
	m_inSimulation		= false;
	m_bConstraintNotify = false;
	m_pDebugOverlay		= NULL;
	m_pConstraintEvent	= NULL;
	m_pObjectEvent		= NULL;
	m_pObjectTracker	= NULL;
	m_pCollisionEvent	= NULL;

	m_pBulletBroadphase		= NULL;
	m_pBulletConfiguration	= NULL;
	m_pBulletDispatcher		= NULL;
	m_pBulletEnvironment	= NULL;
	m_pBulletGhostCallback	= NULL;
	m_pBulletSolver			= NULL;

	m_timestep = 0.f;
	m_invPSIScale = 0.f;
	m_simPSICurrent = 0;
	m_simPSI = 0;

#ifdef MULTITHREADED
	// Maximum number of parallel tasks (number of threads in the thread support)
	// Good to set it to the same amount of CPU cores on the system.
	// TODO: The game dev needs to be able to configure this value. Expose it in the interface.
	int maxTasks = vphysics_numthreads.GetInt();

	maxTasks = max(maxTasks, 1);
	maxTasks = min(maxTasks, 8);

	// Shared thread pool (used by both solver and dispatcher)
	m_pSharedThreadPool = new btThreadPool;
	m_pSharedThreadPool->startThreads(maxTasks);
#endif

	btDefaultCollisionConstructionInfo cci;
	m_pBulletConfiguration = new btSoftBodyRigidBodyCollisionConfiguration(cci);

#ifdef USE_PARALLEL_DISPATCHER
	m_pBulletDispatcher = new btCollisionDispatcherMt(m_pBulletConfiguration, m_pSharedThreadPool);
#else
	m_pBulletDispatcher = new btCollisionDispatcherCustom(this, m_pBulletConfiguration);
#endif

#ifdef USE_PARALLEL_SOLVER
	m_pBulletSolver = new btParallelConstraintSolver(m_pSharedThreadPool);
#else
	m_pBulletSolver = new btSequentialImpulseConstraintSolver;
#endif

	auto nearCallback = [](btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo) {
		auto env = static_cast<btCollisionDispatcherCustom*>(&dispatcher)->Env();
		env->m_pCollisionListener->nearCallback(collisionPair, dispatcher, dispatchInfo);
	};
	m_pBulletDispatcher->setNearCallback(nearCallback);

	m_pCollisionListener = new CCollisionEventListener(this);

	m_pBulletBroadphase = new btDbvtBroadphase;

	// Note: The soft body solver (last default-arg in the constructor) is used for OpenCL stuff (as per the Soft Body Demo)
	m_pBulletEnvironment = new btSoftRigidDynamicsWorld(m_pBulletDispatcher, m_pBulletBroadphase, m_pBulletSolver, m_pBulletConfiguration);

	m_pBulletGhostCallback = new btGhostPairCallback;
	m_pCollisionSolver = new CCollisionSolver(this);
	m_pBulletBroadphase->getOverlappingPairCache()->setOverlapFilterCallback(m_pCollisionSolver);
	m_pBulletBroadphase->getOverlappingPairCache()->setInternalGhostPairCallback(m_pBulletGhostCallback);

	m_pDeleteQueue = new CDeleteQueue;
	m_pPhysicsDragController = new CPhysicsDragController;
	m_pObjectTracker = new CObjectTracker(this, NULL);

	m_perfparams.Defaults();
	memset(&m_stats, 0, sizeof(m_stats));

	// Soft body stuff
	m_softBodyWorldInfo.m_broadphase = m_pBulletBroadphase;
	m_softBodyWorldInfo.m_dispatcher = m_pBulletDispatcher;

	m_softBodyWorldInfo.m_sparsesdf.Initialize();

	m_pBulletEnvironment->getSolverInfo().m_solverMode |= SOLVER_SIMD;

	// TODO: Threads solve any oversized batches (>32?), otherwise solving done on main thread.
	/*
	m_pBulletEnvironment->getSolverInfo().m_minimumSolverBatchSize = 128; // Combine islands up to this many constraints
	m_pBulletEnvironment->getDispatchInfo().m_allowedCcdPenetration = 0.0001f;
	m_pBulletEnvironment->setApplySpeculativeContactRestitution(true);
	*/
	//m_pBulletEnvironment->getDispatchInfo().m_dispatchFunc = btDispatcherInfo::DISPATCH_CONTINUOUS;

	//m_simPSIs = 0;
	//m_invPSIscale = 0;

	m_pBulletEnvironment->setInternalTickCallback(TickCallback, (void *)this);

#if DEBUG_DRAW
	m_debugdraw = new CDebugDrawer(m_pBulletEnvironment);
#endif
}

CPhysicsEnvironment::~CPhysicsEnvironment() {
#if DEBUG_DRAW
	delete m_debugdraw;
#endif
	SetQuickDelete(true);

	for (int i = m_objects.size() - 1; i >= 0; --i) {
		delete m_objects[i];
	}

	for (int i = m_softBodies.size() - 1; i >= 0; --i) {
		delete m_softBodies[i];
	}

	m_objects.clear();
	m_softBodies.clear();
	CleanupDeleteList();

	delete m_pDeleteQueue;
	delete m_pPhysicsDragController;

	delete m_pBulletEnvironment;
	delete m_pBulletSolver;
	delete m_pBulletBroadphase;
	delete m_pBulletDispatcher;
	delete m_pBulletConfiguration;
	delete m_pBulletGhostCallback;

#ifdef MULTITHREADED
	m_pSharedThreadPool->stopThreads();
	delete m_pSharedThreadPool;
#endif

	delete m_pCollisionListener;
	delete m_pCollisionSolver;
	delete m_pObjectTracker;
}

void CPhysicsEnvironment::ChangeThreadCount(int newThreadCount) {
#ifdef MULTITHREADED
	m_pSharedThreadPool->resizeThreads(newThreadCount);
#endif
}

// UNEXPOSED

void CPhysicsEnvironment::TickCallback(btDynamicsWorld *world, btScalar timeStep) {
	if (!world) return;

	CPhysicsEnvironment *pEnv = (CPhysicsEnvironment *)world->getWorldUserInfo();
	if (pEnv)
		pEnv->BulletTick(timeStep);
}

IVPhysicsDebugOverlay *g_pDebugOverlay = NULL;
void CPhysicsEnvironment::SetDebugOverlay(CreateInterfaceFn debugOverlayFactory) {
	if (debugOverlayFactory)
		g_pDebugOverlay = (IVPhysicsDebugOverlay *)debugOverlayFactory(VPHYSICS_DEBUG_OVERLAY_INTERFACE_VERSION, NULL);

#if DEBUG_DRAW
	if (g_pDebugOverlay)
		m_debugdraw->SetDebugOverlay(g_pDebugOverlay);
#endif
}

IVPhysicsDebugOverlay *CPhysicsEnvironment::GetDebugOverlay() {
	return g_pDebugOverlay;
}

btIDebugDraw *CPhysicsEnvironment::GetDebugDrawer() {
	return (btIDebugDraw *)m_debugdraw;
}

void CPhysicsEnvironment::SetGravity(const Vector &gravityVector) {
	btVector3 temp;
	ConvertPosToBull(gravityVector, temp);

	m_pBulletEnvironment->setGravity(temp);
	m_softBodyWorldInfo.m_gravity = temp;
}

void CPhysicsEnvironment::GetGravity(Vector *pGravityVector) const {
	if (!pGravityVector) return;

	btVector3 temp = m_pBulletEnvironment->getGravity();
	ConvertPosToHL(temp, *pGravityVector);
}

void CPhysicsEnvironment::SetAirDensity(float density) {
	m_pPhysicsDragController->SetAirDensity(density);

	// Density is kg/in^3 from HL
	m_softBodyWorldInfo.air_density = density / CUBIC_METERS_PER_CUBIC_INCH;
}

float CPhysicsEnvironment::GetAirDensity() const {
	return m_pPhysicsDragController->GetAirDensity();
}

IPhysicsObject *CPhysicsEnvironment::CreatePolyObject(const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams) {
	IPhysicsObject *pObject = CreatePhysicsObject(this, pCollisionModel, materialIndex, position, angles, pParams, false);

	m_objects.push_back(pObject);
	return pObject;
}

IPhysicsObject *CPhysicsEnvironment::CreatePolyObjectStatic(const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams) {
	IPhysicsObject *pObject = CreatePhysicsObject(this, pCollisionModel, materialIndex, position, angles, pParams, true);

	m_objects.push_back(pObject);
	return pObject;
}

// Deprecated. Create a sphere model using collision interface.
IPhysicsObject *CPhysicsEnvironment::CreateSphereObject(float radius, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams, bool isStatic) {
	IPhysicsObject *pObject = CreatePhysicsSphere(this, radius, materialIndex, position, angles, pParams, isStatic);

	m_objects.push_back(pObject);
	return pObject;
}

void CPhysicsEnvironment::DestroyObject(IPhysicsObject *pObject) {
	if (!pObject) return;
	
	Assert(std::find(m_deadObjects.begin(), m_deadObjects.end(), pObject) == m_deadObjects.end());	// If you hit this assert, the object is already on the list!

	auto it = std::find(m_objects.begin(), m_objects.end(), pObject);
	if (it != m_objects.end())
		m_objects.erase(it);

	m_pObjectTracker->ObjectRemoved((CPhysicsObject *)pObject);

	if (m_inSimulation || m_bUseDeleteQueue) {
		// We're still in the simulation, so deleting an object would be disastrous here. Queue it!
		((CPhysicsObject *)pObject)->AddCallbackFlags(CALLBACK_MARKED_FOR_DELETE);
		
		m_deadObjects.push_back(pObject);
	} else {
		delete pObject;
	}
}

IPhysicsSoftBody *CPhysicsEnvironment::CreateSoftBody() {
	CPhysicsSoftBody *pSoftBody = ::CreateSoftBody(this);

	if (pSoftBody)
		m_softBodies.push_back(pSoftBody);

	return pSoftBody;
}

IPhysicsSoftBody *CPhysicsEnvironment::CreateSoftBodyFromVertices(const Vector *vertices, int numVertices, const softbodyparams_t *pParams) {
	CPhysicsSoftBody *pSoftBody = ::CreateSoftBodyFromVertices(this, vertices, numVertices, pParams);

	if (pSoftBody)
		m_softBodies.push_back(pSoftBody);

	return pSoftBody;
}

IPhysicsSoftBody *CPhysicsEnvironment::CreateSoftBodyRope(const Vector &pos, const Vector &end, int resolution, const softbodyparams_t *pParams) {
	CPhysicsSoftBody *pSoftBody = ::CreateSoftBodyRope(this, pos, end, resolution, pParams);

	if (pSoftBody)
		m_softBodies.push_back(pSoftBody);

	return pSoftBody;
}

IPhysicsSoftBody *CPhysicsEnvironment::CreateSoftBodyPatch(const Vector *corners, int resx, int resy, const softbodyparams_t *pParams) {
	CPhysicsSoftBody *pSoftBody = ::CreateSoftBodyPatch(this, corners, resx, resy, pParams);

	if (pSoftBody)
		m_softBodies.push_back(pSoftBody);

	return pSoftBody;
}

void CPhysicsEnvironment::DestroySoftBody(IPhysicsSoftBody *pSoftBody) {
	if (!pSoftBody) return;

	auto it = std::find(m_softBodies.begin(), m_softBodies.end(), pSoftBody);
	if (it != m_softBodies.end())
		m_softBodies.erase(it);
	
	if (m_inSimulation || m_bUseDeleteQueue) {
		m_pDeleteQueue->QueueForDelete(pSoftBody);
	} else {
		delete pSoftBody;
	}
}

IPhysicsFluidController *CPhysicsEnvironment::CreateFluidController(IPhysicsObject *pFluidObject, fluidparams_t *pParams) {
	CPhysicsFluidController *pFluid = ::CreateFluidController(this, (CPhysicsObject *)pFluidObject, pParams);
	if (pFluid) {
		m_fluids.push_back(pFluid);
	}

	return pFluid;
}

void CPhysicsEnvironment::DestroyFluidController(IPhysicsFluidController *pController) {
	auto it = std::find(m_fluids.begin(), m_fluids.end(), pController);
	if (it != m_fluids.end()) {
		m_fluids.erase(it);
	}
	delete pController;
}

IPhysicsSpring *CPhysicsEnvironment::CreateSpring(IPhysicsObject *pObjectStart, IPhysicsObject *pObjectEnd, springparams_t *pParams) {
	return ::CreateSpringConstraint(this, pObjectStart, pObjectEnd, pParams);
}

void CPhysicsEnvironment::DestroySpring(IPhysicsSpring *pSpring) {
	if (!pSpring) return;

	CPhysicsConstraint *pConstraint = (CPhysicsConstraint *)pSpring;

	if (m_deleteQuick) {
		IPhysicsObject *pObj0 = pConstraint->GetReferenceObject();
		if (pObj0 && !pObj0->IsStatic())
			pObj0->Wake();

		IPhysicsObject *pObj1 = pConstraint->GetAttachedObject();
		if (pObj1 && !pObj1->IsStatic())
			pObj1->Wake();
	}

	if (m_inSimulation) {
		pConstraint->Deactivate();
		m_pDeleteQueue->QueueForDelete(pSpring);
	} else {
		delete pSpring;
	}
}

IPhysicsConstraint *CPhysicsEnvironment::CreateRagdollConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll) {
	return ::CreateRagdollConstraint(this, pReferenceObject, pAttachedObject, pGroup, ragdoll);
}

IPhysicsConstraint *CPhysicsEnvironment::CreateHingeConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge) {
	return ::CreateHingeConstraint(this, pReferenceObject, pAttachedObject, pGroup, hinge);
}

IPhysicsConstraint *CPhysicsEnvironment::CreateFixedConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed) {
	return ::CreateFixedConstraint(this, pReferenceObject, pAttachedObject, pGroup, fixed);
}

IPhysicsConstraint *CPhysicsEnvironment::CreateSlidingConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding) {
	return ::CreateSlidingConstraint(this, pReferenceObject, pAttachedObject, pGroup, sliding);
}

IPhysicsConstraint *CPhysicsEnvironment::CreateBallsocketConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket) {
	return ::CreateBallsocketConstraint(this, pReferenceObject, pAttachedObject, pGroup, ballsocket);
}

IPhysicsConstraint *CPhysicsEnvironment::CreatePulleyConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley) {
	return ::CreatePulleyConstraint(this, pReferenceObject, pAttachedObject, pGroup, pulley);
}

IPhysicsConstraint *CPhysicsEnvironment::CreateLengthConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length) {
	return ::CreateLengthConstraint(this, pReferenceObject, pAttachedObject, pGroup, length);
}

IPhysicsConstraint *CPhysicsEnvironment::CreateGearConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_gearparams_t &gear) {
	return ::CreateGearConstraint(this, pReferenceObject, pAttachedObject, pGroup, gear);
}

IPhysicsConstraint *CPhysicsEnvironment::CreateUserConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, IPhysicsUserConstraint *pConstraint) {
	return ::CreateUserConstraint(this, pReferenceObject, pAttachedObject, pGroup, pConstraint);
}

void CPhysicsEnvironment::DestroyConstraint(IPhysicsConstraint *pConstraint) {
	if (!pConstraint) return;

	if (m_deleteQuick) {
		IPhysicsObject *pObj0 = pConstraint->GetReferenceObject();
		if (pObj0 && !pObj0->IsStatic())
			pObj0->Wake();

		IPhysicsObject *pObj1 = pConstraint->GetAttachedObject();
		if (pObj1 && !pObj1->IsStatic())
			pObj1->Wake();
	}

	if (m_inSimulation) {
		pConstraint->Deactivate();
		m_pDeleteQueue->QueueForDelete(pConstraint);
	} else {
		delete pConstraint;
	}
}

IPhysicsConstraintGroup *CPhysicsEnvironment::CreateConstraintGroup(const constraint_groupparams_t &groupParams) {
	return ::CreateConstraintGroup(this, groupParams);
}

void CPhysicsEnvironment::DestroyConstraintGroup(IPhysicsConstraintGroup *pGroup) {
	delete pGroup;
}

IPhysicsShadowController *CPhysicsEnvironment::CreateShadowController(IPhysicsObject *pObject, bool allowTranslation, bool allowRotation) {
	CShadowController *pController = ::CreateShadowController(pObject, allowTranslation, allowRotation);
	if (pController) {
		m_controllers.push_back(pController);
	}

	return pController;
}

void CPhysicsEnvironment::DestroyShadowController(IPhysicsShadowController *pController) {
	if (!pController) return;

	auto it = std::find(m_controllers.begin(), m_controllers.end(), (CShadowController*)pController);
	if (it != m_controllers.end()) {
		m_controllers.erase(it);
	}

	delete pController;
}

IPhysicsPlayerController *CPhysicsEnvironment::CreatePlayerController(IPhysicsObject *pObject) {
	CPlayerController *pController = ::CreatePlayerController(this, pObject);
	if (pController) {
		m_controllers.push_back(pController);
	}

	return pController;
}

void CPhysicsEnvironment::DestroyPlayerController(IPhysicsPlayerController *pController) {
	if (!pController) return;

	auto it = std::find(m_controllers.begin(), m_controllers.end(), (CPlayerController*)pController);
	if (it != m_controllers.end()) {
		m_controllers.erase(it);
	}

	delete pController;
}

IPhysicsMotionController *CPhysicsEnvironment::CreateMotionController(IMotionEvent *pHandler) {
	CPhysicsMotionController *pController = (CPhysicsMotionController *)::CreateMotionController(this, pHandler);
	if (pController) {
		m_controllers.push_back(pController);
	}

	return pController;
}

void CPhysicsEnvironment::DestroyMotionController(IPhysicsMotionController *pController) {
	if (!pController) return;

	auto it = std::find(m_controllers.begin(), m_controllers.end(), (CPhysicsMotionController*)pController);
	if (it != m_controllers.end()) {
		m_controllers.erase(it);
	}

	delete pController;
}

IPhysicsVehicleController *CPhysicsEnvironment::CreateVehicleController(IPhysicsObject *pVehicleBodyObject, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace) {
	return ::CreateVehicleController(this, (CPhysicsObject *)pVehicleBodyObject, params, nVehicleType, pGameTrace);
}

void CPhysicsEnvironment::DestroyVehicleController(IPhysicsVehicleController *pController) {
	delete pController;
}

void CPhysicsEnvironment::SetCollisionSolver(IPhysicsCollisionSolver *pSolver) {
	m_pCollisionSolver->SetHandler(pSolver);
}

static ConVar cvar_substeps("vphysics_substeps", "4", FCVAR_REPLICATED, "Sets the amount of simulation substeps (higher number means higher precision)", true, 1, false, 0);
void CPhysicsEnvironment::Simulate(float deltaTime) {
	Assert(m_pBulletEnvironment);

	// Input deltaTime is how many seconds have elapsed since the previous frame
	// phys_timescale can scale this parameter however...
	// Can cause physics to slow down on the client environment when the game's window is not in focus
	// Also is affected by the tickrate as well

	if (deltaTime > 1.0 || deltaTime < 0.0) {
		deltaTime = 0;
	} else if (deltaTime > 0.1) {
		deltaTime = 0.1f;
	}

	// sim PSI: How many substeps are done in a single simulation step
	m_simPSI = cvar_substeps.GetInt() != 0 ? cvar_substeps.GetInt() : 1;
	m_simPSICurrent = m_simPSI; // Substeps left in this step
	m_numSubSteps = m_simPSI;
	m_curSubStep = 0;
	
	// Simulate no less than 1 ms
	if (deltaTime > 0.0001) {
		// Now mark us as being in simulation. This is used for callbacks from bullet mid-simulation
		// so we don't end up doing stupid things like deleting objects still in use
		m_inSimulation = true;

		m_subStepTime = m_timestep;
		
		// Okay, how this fixed timestep shit works:
		// The game sends in deltaTime which is the amount of time that has passed since the last frame
		// Bullet will add the deltaTime to its internal counter
		// When this internal counter exceeds m_timestep (param 3 to the below), the simulation will run for fixedTimeStep seconds
		// If the internal counter does not exceed fixedTimeStep, bullet will just interpolate objects so the game can render them nice and happy
		m_pBulletEnvironment->stepSimulation(deltaTime, 4, m_timestep);

		// No longer in simulation!
		m_inSimulation = false;
	}

#if DEBUG_DRAW
	m_debugdraw->DrawWorld();
#endif

	// FIXME: See if this is even needed here
	m_softBodyWorldInfo.m_sparsesdf.GarbageCollect();
}

bool CPhysicsEnvironment::IsInSimulation() const {
	return m_inSimulation;
}

float CPhysicsEnvironment::GetSimulationTimestep() const {
	return m_timestep;
}

void CPhysicsEnvironment::SetSimulationTimestep(float timestep) {
	m_timestep = timestep;
}

float CPhysicsEnvironment::GetSimulationTime() const {
	NOT_IMPLEMENTED
	return 0.0;
}

void CPhysicsEnvironment::ResetSimulationClock() {
	NOT_IMPLEMENTED
}

float CPhysicsEnvironment::GetNextFrameTime() const {
	NOT_IMPLEMENTED
	return 0.0;
}

void CPhysicsEnvironment::SetCollisionEventHandler(IPhysicsCollisionEvent *pCollisionEvents) {
	m_pCollisionListener->SetCollisionEventCallback(pCollisionEvents);
	m_pCollisionEvent = pCollisionEvents;
}

void CPhysicsEnvironment::SetObjectEventHandler(IPhysicsObjectEvent *pObjectEvents) {
	m_pObjectEvent = pObjectEvents;

	m_pObjectTracker->SetObjectEventHandler(pObjectEvents);
}

void CPhysicsEnvironment::SetConstraintEventHandler(IPhysicsConstraintEvent *pConstraintEvents) {
	m_pConstraintEvent = pConstraintEvents;
}

void CPhysicsEnvironment::SetQuickDelete(bool bQuick) {
	m_deleteQuick = bQuick;
}

int CPhysicsEnvironment::GetActiveObjectCount() const {
	return m_pObjectTracker->GetActiveObjectCount();
}

void CPhysicsEnvironment::GetActiveObjects(IPhysicsObject **pOutputObjectList) const {
	return m_pObjectTracker->GetActiveObjects(pOutputObjectList);
}

int CPhysicsEnvironment::GetObjectCount() const {
	return m_objects.size();
}

const IPhysicsObject **CPhysicsEnvironment::GetObjectList(int *pOutputObjectCount) const {
	if (pOutputObjectCount) {
		*pOutputObjectCount = m_objects.size();
	}

	return (const IPhysicsObject **)m_objects.data();
}

bool CPhysicsEnvironment::TransferObject(IPhysicsObject *pObject, IPhysicsEnvironment *pDestinationEnvironment) {
	if (!pObject || !pDestinationEnvironment) return false;

	if (pDestinationEnvironment == this) {
		((CPhysicsObject *)pObject)->TransferToEnvironment(this);
		m_objects.push_back(pObject);
		if (pObject->IsFluid()) {
			m_fluids.push_back(((CPhysicsObject *)pObject)->GetFluidController());
		}

		return true;
	} else {
		auto it = std::find(m_objects.begin(), m_objects.end(), pObject);
		if (it != m_objects.end())
			m_objects.erase(it);

		if (pObject->IsFluid()) {
			auto fit = std::find(m_fluids.begin(), m_fluids.end(), ((CPhysicsObject *)pObject)->GetFluidController());
			if (fit != m_fluids.end())
				m_fluids.erase(fit);
		}

		return pDestinationEnvironment->TransferObject(pObject, pDestinationEnvironment);
	}
}

void CPhysicsEnvironment::CleanupDeleteList() {
	for (size_t i = 0; i < m_deadObjects.size(); i++) {
		delete m_deadObjects[i];
	}

	m_deadObjects.clear();
	m_pDeleteQueue->DeleteAll();
}

void CPhysicsEnvironment::EnableDeleteQueue(bool enable) {
	m_bUseDeleteQueue = enable;
}

bool CPhysicsEnvironment::Save(const physsaveparams_t &params) {
	NOT_IMPLEMENTED
	return false;
}

void CPhysicsEnvironment::PreRestore(const physprerestoreparams_t &params) {
	NOT_IMPLEMENTED
}

bool CPhysicsEnvironment::Restore(const physrestoreparams_t &params) {
	NOT_IMPLEMENTED
	return false;
}

void CPhysicsEnvironment::PostRestore() {
	NOT_IMPLEMENTED
}

bool CPhysicsEnvironment::IsCollisionModelUsed(CPhysCollide *pCollide) const {
	for (size_t i = 0; i < m_objects.size(); i++) {
		if (((CPhysicsObject *)m_objects[i])->GetObject()->getCollisionShape() == pCollide->GetCollisionShape())
			return true;
	}

	// Also scan the to-be-deleted objects list
	for (size_t i = 0; i < m_deadObjects.size(); i++) {
		if (((CPhysicsObject *)m_deadObjects[i])->GetObject()->getCollisionShape() == pCollide->GetCollisionShape())
			return true;
	}

	return false;
}

void CPhysicsEnvironment::TraceRay(const Ray_t &ray, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace) {
	if (!ray.m_IsRay || !pTrace) return;

	btVector3 vecStart, vecEnd;
	ConvertPosToBull(ray.m_Start + ray.m_StartOffset, vecStart);
	ConvertPosToBull(ray.m_Start + ray.m_StartOffset + ray.m_Delta, vecEnd);

	// TODO: Override this class to use the mask and trace filter.
	btCollisionWorld::ClosestRayResultCallback cb(vecStart, vecEnd);
	m_pBulletEnvironment->rayTest(vecStart, vecEnd, cb);

	pTrace->fraction = (float)cb.m_closestHitFraction;
	ConvertPosToHL(cb.m_hitPointWorld, pTrace->endpos);
	ConvertDirectionToHL(cb.m_hitNormalWorld, pTrace->plane.normal);
}

// Is this function ever called?
// TODO: This is a bit more complex, bullet doesn't support compound sweep tests.
void CPhysicsEnvironment::SweepCollideable(const CPhysCollide *pCollide, const Vector &vecAbsStart, const Vector &vecAbsEnd, const QAngle &vecAngles, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace) {
	NOT_IMPLEMENTED
}

class CFilteredConvexResultCallback : public btCollisionWorld::ClosestConvexResultCallback {
	public:
		CFilteredConvexResultCallback(IPhysicsTraceFilter *pFilter, unsigned int mask, const btVector3 &convexFromWorld, const btVector3 &convexToWorld):
		btCollisionWorld::ClosestConvexResultCallback(convexFromWorld, convexToWorld) {
			m_pTraceFilter = pFilter;
			m_mask = mask;
		}

		virtual bool needsCollision(btBroadphaseProxy *proxy0) const {
			btCollisionObject *pColObj = (btCollisionObject *)proxy0->m_clientObject;
			CPhysicsObject *pObj = (CPhysicsObject *)pColObj->getUserPointer();
			if (pObj && !m_pTraceFilter->ShouldHitObject(pObj, m_mask)) {
				return false;
			}

			bool collides = (proxy0->m_collisionFilterGroup & m_collisionFilterMask) != 0;
			collides = collides && (m_collisionFilterGroup & proxy0->m_collisionFilterMask);

			return collides;
		}

	private:
		IPhysicsTraceFilter *m_pTraceFilter;
		unsigned int m_mask;
};

void CPhysicsEnvironment::SweepConvex(const CPhysConvex *pConvex, const Vector &vecAbsStart, const Vector &vecAbsEnd, const QAngle &vecAngles, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace) {
	if (!pConvex || !pTrace) return;
	
	btVector3 vecStart, vecEnd;
	ConvertPosToBull(vecAbsStart, vecStart);
	ConvertPosToBull(vecAbsEnd, vecEnd);

	btMatrix3x3 matAng;
	ConvertRotationToBull(vecAngles, matAng);

	btTransform transStart, transEnd;
	transStart.setOrigin(vecStart);
	transStart.setBasis(matAng);

	transEnd.setOrigin(vecEnd);
	transEnd.setBasis(matAng);

	btConvexShape *pShape = (btConvexShape *)pConvex;

	CFilteredConvexResultCallback cb(pTraceFilter, fMask, vecStart, vecEnd);
	m_pBulletEnvironment->convexSweepTest(pShape, transStart, transEnd, cb, 0.0001f);

	pTrace->fraction = (float)cb.m_closestHitFraction;
	ConvertPosToHL(cb.m_hitPointWorld, pTrace->endpos);
	ConvertDirectionToHL(cb.m_hitNormalWorld, pTrace->plane.normal);
}

void CPhysicsEnvironment::GetPerformanceSettings(physics_performanceparams_t *pOutput) const {
	if (!pOutput) return;

	*pOutput = m_perfparams;
}

void CPhysicsEnvironment::SetPerformanceSettings(const physics_performanceparams_t *pSettings) {
	if (!pSettings) return;

	m_perfparams = *pSettings;
}

void CPhysicsEnvironment::ReadStats(physics_stats_t *pOutput) {
	if (!pOutput) return;

	*pOutput = m_stats;
}

void CPhysicsEnvironment::ClearStats() {
	memset(&m_stats, 0, sizeof(m_stats));
}

unsigned int CPhysicsEnvironment::GetObjectSerializeSize(IPhysicsObject *pObject) const {
	NOT_IMPLEMENTED
	return 0;
}

void CPhysicsEnvironment::SerializeObjectToBuffer(IPhysicsObject *pObject, unsigned char *pBuffer, unsigned int bufferSize) {
	NOT_IMPLEMENTED
}

IPhysicsObject *CPhysicsEnvironment::UnserializeObjectFromBuffer(void *pGameData, unsigned char *pBuffer, unsigned int bufferSize, bool enableCollisions) {
	NOT_IMPLEMENTED
	return NULL;
}

void CPhysicsEnvironment::EnableConstraintNotify(bool bEnable) {
	// Notify game about broken constraints?
	m_bConstraintNotify = bEnable;
}

// FIXME: What do? Source SDK mods call this every frame in debug builds.
void CPhysicsEnvironment::DebugCheckContacts() {
	
}

void CPhysicsEnvironment::SetAlternateGravity(const Vector& gravityVector)
{
	NOT_IMPLEMENTED
}

void CPhysicsEnvironment::GetAlternateGravity(Vector* pGravityVector) const
{
	NOT_IMPLEMENTED
}

float CPhysicsEnvironment::GetDeltaFrameTime(int maxTicks) const
{
	//NOT_IMPLEMENTED
	return 0.0f;
}

void CPhysicsEnvironment::ForceObjectsToSleep(IPhysicsObject** pList, int listCount)
{
	NOT_IMPLEMENTED
}

void CPhysicsEnvironment::SetPredicted(bool bPredicted)
{
	NOT_IMPLEMENTED
}

bool CPhysicsEnvironment::IsPredicted(void)
{
	//NOT_IMPLEMENTED

	return false;
}

void CPhysicsEnvironment::SetPredictionCommandNum(int iCommandNum)
{
	//NOT_IMPLEMENTED
}

int CPhysicsEnvironment::GetPredictionCommandNum(void)
{
	//NOT_IMPLEMENTED
	return 0;
}

void CPhysicsEnvironment::DoneReferencingPreviousCommands(int iCommandNum)
{
	//NOT_IMPLEMENTED
}

void CPhysicsEnvironment::RestorePredictedSimulation(void)
{
	//NOT_IMPLEMENTED
}

void CPhysicsEnvironment::DestroyCollideOnDeadObjectFlush(CPhysCollide*)
{
	NOT_IMPLEMENTED
}

// UNEXPOSED
btSoftRigidDynamicsWorld *CPhysicsEnvironment::GetBulletEnvironment() {
	return m_pBulletEnvironment;
}

// UNEXPOSED
float CPhysicsEnvironment::GetInvPSIScale() {
	return m_invPSIScale;
}

// UNEXPOSED
void CPhysicsEnvironment::BulletTick(btScalar dt) {
	// Dirty hack to spread the controllers throughout the current simulation step
	if (m_simPSICurrent) {
		m_invPSIScale = 1.0f / (float)m_simPSICurrent;
		m_simPSICurrent--;
	} else {
		m_invPSIScale = 0;
	}

	m_pPhysicsDragController->Tick(dt);

	{
		for (size_t i = 0; i < m_controllers.size(); i++)
			m_controllers[i]->Tick((float)dt);

		for (size_t i = 0; i < m_fluids.size(); i++)
			m_fluids[i]->Tick((float)dt);
	}

	m_inSimulation = false;

	// Update object sleep states
	m_pObjectTracker->Tick();

	if (!m_bUseDeleteQueue) {
		CleanupDeleteList();
	}

	if (m_pCollisionEvent) {
		m_pCollisionEvent->PostSimulationFrame();
	}

	m_inSimulation = true;
	m_curSubStep++;
}

// UNEXPOSED
CPhysicsDragController *CPhysicsEnvironment::GetDragController() {
	return m_pPhysicsDragController;
}

CCollisionSolver *CPhysicsEnvironment::GetCollisionSolver() {
	return m_pCollisionSolver;
}

btVector3 CPhysicsEnvironment::GetMaxLinearVelocity() const {
	return btVector3(HL2BULL(m_perfparams.maxVelocity), HL2BULL(m_perfparams.maxVelocity), HL2BULL(m_perfparams.maxVelocity));
}

btVector3 CPhysicsEnvironment::GetMaxAngularVelocity() const {
	return btVector3(HL2BULL(m_perfparams.maxAngularVelocity), HL2BULL(m_perfparams.maxAngularVelocity), HL2BULL(m_perfparams.maxAngularVelocity));
}

// ==================
// EVENTS
// ==================

// UNEXPOSED
void CPhysicsEnvironment::HandleConstraintBroken(CPhysicsConstraint *pConstraint) {
	if (m_bConstraintNotify && m_pConstraintEvent)
		m_pConstraintEvent->ConstraintBroken(pConstraint);
}

// UNEXPOSED
void CPhysicsEnvironment::HandleFluidStartTouch(CPhysicsFluidController *pController, CPhysicsObject *pObject) {
	if (pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE) return;

	if (m_pCollisionEvent)
		m_pCollisionEvent->FluidStartTouch(pObject, pController);
}

// UNEXPOSED
void CPhysicsEnvironment::HandleFluidEndTouch(CPhysicsFluidController *pController, CPhysicsObject *pObject) {
	if (pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE) return;

	if (m_pCollisionEvent)
		m_pCollisionEvent->FluidEndTouch(pObject, pController);
}

// UNEXPOSED
void CPhysicsEnvironment::HandleObjectEnteredTrigger(CPhysicsObject *pTrigger, CPhysicsObject *pObject) {
	if (pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE) return;

	if (m_pCollisionEvent) {
		// Doesn't work
		//m_pCollisionEvent->ObjectEnterTrigger(pTrigger, pObject);
	}
}

// UNEXPOSED
void CPhysicsEnvironment::HandleObjectExitedTrigger(CPhysicsObject *pTrigger, CPhysicsObject *pObject) {
	if (pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE) return;

	if (m_pCollisionEvent) {
		// Doesn't work
		//m_pCollisionEvent->ObjectLeaveTrigger(pTrigger, pObject);
	}
}

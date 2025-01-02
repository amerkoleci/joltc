// Copyright (c) Amer Koleci and Contributors.
// Distributed under the MIT license. See the LICENSE file in the project root for more information.

#include "joltc.h"
#include <stdbool.h>
#include <stdlib.h> // malloc, free
#include <string.h> // memset
// STL includes
#include <iostream>

static void TraceImpl(const char* message)
{
	// Print to the TTY
	std::cout << message << std::endl;
}

namespace Layers
{
	static constexpr JPH_ObjectLayer NON_MOVING = 0;
	static constexpr JPH_ObjectLayer MOVING = 1;
	static constexpr JPH_ObjectLayer NUM_LAYERS = 2;
};

namespace BroadPhaseLayers
{
	static constexpr JPH_BroadPhaseLayer NON_MOVING(0);
	static constexpr JPH_BroadPhaseLayer MOVING(1);
	static constexpr uint32_t NUM_LAYERS(2);
};

int main(void)
{
	if (!JPH_Init())
		return 1;

	JPH_SetTraceHandler(TraceImpl);
	//JPH_SetAssertFailureHandler(JPH_AssertFailureFunc handler);

	JPH_JobSystem* jobSystem = JPH_JobSystemThreadPool_Create(nullptr);

	// We use only 2 layers: one for non-moving objects and one for moving objects
	JPH_ObjectLayerPairFilter* objectLayerPairFilterTable = JPH_ObjectLayerPairFilterTable_Create(2);
	JPH_ObjectLayerPairFilterTable_EnableCollision(objectLayerPairFilterTable, Layers::NON_MOVING, Layers::MOVING);
	JPH_ObjectLayerPairFilterTable_EnableCollision(objectLayerPairFilterTable, Layers::MOVING, Layers::NON_MOVING);

	// We use a 1-to-1 mapping between object layers and broadphase layers
	JPH_BroadPhaseLayerInterface* broadPhaseLayerInterfaceTable = JPH_BroadPhaseLayerInterfaceTable_Create(2, 2);
	JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(broadPhaseLayerInterfaceTable, Layers::NON_MOVING, BroadPhaseLayers::NON_MOVING);
	JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(broadPhaseLayerInterfaceTable, Layers::MOVING, BroadPhaseLayers::MOVING);

	JPH_ObjectVsBroadPhaseLayerFilter* objectVsBroadPhaseLayerFilter = JPH_ObjectVsBroadPhaseLayerFilterTable_Create(broadPhaseLayerInterfaceTable, 2, objectLayerPairFilterTable, 2);

	JPH_PhysicsSystemSettings settings = {};
	settings.maxBodies = 65536;
	settings.numBodyMutexes = 0;
	settings.maxBodyPairs = 65536;
	settings.maxContactConstraints = 65536;
	settings.broadPhaseLayerInterface = broadPhaseLayerInterfaceTable;
	settings.objectLayerPairFilter = objectLayerPairFilterTable;
	settings.objectVsBroadPhaseLayerFilter = objectVsBroadPhaseLayerFilter;
	JPH_PhysicsSystem* system = JPH_PhysicsSystem_Create(&settings);
	JPH_BodyInterface* bodyInterface = JPH_PhysicsSystem_GetBodyInterface(system);

	JPH_BodyID floorId = {};
	{
		// Next we can create a rigid body to serve as the floor, we make a large box
		// Create the settings for the collision volume (the shape). 
		// Note that for simple shapes (like boxes) you can also directly construct a BoxShape.
		JPH_Vec3 boxHalfExtents = { 100.0f, 1.0f, 100.0f };
		JPH_BoxShape* floorShape = JPH_BoxShape_Create(&boxHalfExtents, JPH_DEFAULT_CONVEX_RADIUS);

		JPH_Vec3 floorPosition = { 0.0f, -1.0f, 0.0f };
		JPH_BodyCreationSettings* floorSettings = JPH_BodyCreationSettings_Create3(
			(const JPH_Shape*)floorShape,
			&floorPosition,
			nullptr, // Identity, 
			JPH_MotionType_Static,
			Layers::NON_MOVING);

		// Create the actual rigid body
		floorId = JPH_BodyInterface_CreateAndAddBody(bodyInterface, floorSettings, JPH_Activation_DontActivate);
		JPH_BodyCreationSettings_Destroy(floorSettings);
	}

	// Sphere
	JPH_BodyID sphereId = {};
	{
		JPH_SphereShape* sphereShape = JPH_SphereShape_Create(50.0f);
		JPH_Vec3 spherePosition = { 0.0f, 2.0f, 0.0f };
		JPH_BodyCreationSettings* sphereSettings = JPH_BodyCreationSettings_Create3(
			(const JPH_Shape*)sphereShape,
			&spherePosition,
			nullptr, // Identity, 
			JPH_MotionType_Dynamic,
			Layers::MOVING);

		sphereId = JPH_BodyInterface_CreateAndAddBody(bodyInterface, sphereSettings, JPH_Activation_Activate);
		JPH_BodyCreationSettings_Destroy(sphereSettings);
	}

	// Now you can interact with the dynamic body, in this case we're going to give it a velocity.
	// (note that if we had used CreateBody then we could have set the velocity straight on the body before adding it to the physics system)
	JPH_Vec3 sphereLinearVelocity = { 0.0f, -5.0f, 0.0f };
	JPH_BodyInterface_SetLinearVelocity(bodyInterface, sphereId, &sphereLinearVelocity);

	{
		static constexpr float	cCharacterHeightStanding = 1.35f;
		static constexpr float	cCharacterRadiusStanding = 0.3f;
		static constexpr float	cCharacterHeightCrouching = 0.8f;
		static constexpr float	cCharacterRadiusCrouching = 0.3f;
		static constexpr float	cInnerShapeFraction = 0.9f;

		JPH_CapsuleShape* capsuleShape = JPH_CapsuleShape_Create(0.5f * cCharacterHeightStanding, cCharacterRadiusStanding);
		JPH_Vec3 position = { 0, 0.5f * cCharacterHeightStanding + cCharacterRadiusStanding, 0 };
		auto mStandingShape = JPH_RotatedTranslatedShape_Create(&position, nullptr, (JPH_Shape*)capsuleShape);

		JPH_CharacterVirtualSettings characterSettings{};
		JPH_CharacterVirtualSettings_Init(&characterSettings);
		characterSettings.base.shape = (const JPH_Shape*)mStandingShape;
		characterSettings.base.supportingVolume = { {0, 1, 0}, -cCharacterRadiusStanding }; // Accept contacts that touch the lower sphere of the capsule
		static const JPH_RVec3 characterVirtualPosition = { -5.0f, 0, 3.0f };

		auto mAnimatedCharacterVirtual = JPH_CharacterVirtual_Create(&characterSettings, &characterVirtualPosition, nullptr, 0, system);
	}

	JPH_SixDOFConstraintSettings jointSettings;
	JPH_SixDOFConstraintSettings_Init(&jointSettings);

	// We simulate the physics world in discrete time steps. 60 Hz is a good rate to update the physics system.
	const float cDeltaTime = 1.0f / 60.0f;

	// Optional step: Before starting the physics simulation you can optimize the broad phase. This improves collision detection performance (it's pointless here because we only have 2 bodies).
	// You should definitely not call this every frame or when e.g. streaming in a new level section as it is an expensive operation.
	// Instead insert all new objects in batches instead of 1 at a time to keep the broad phase efficient.
	JPH_PhysicsSystem_OptimizeBroadPhase(system);

	// Now we're ready to simulate the body, keep simulating until it goes to sleep
	uint32_t step = 0;
	while (JPH_BodyInterface_IsActive(bodyInterface, sphereId))
	{
		// Next step
		++step;

		// Output current position and velocity of the sphere
		JPH_RVec3 position;
		JPH_Vec3 velocity;

		JPH_BodyInterface_GetCenterOfMassPosition(bodyInterface, sphereId, &position);
		JPH_BodyInterface_GetLinearVelocity(bodyInterface, sphereId, &velocity);
		std::cout << "Step " << step << ": Position = (" << position.x << ", " << position.y << ", " << position.z << "), Velocity = (" << velocity.x << ", " << velocity.y << ", " << velocity.z << ")" << std::endl;

		// If you take larger steps than 1 / 60th of a second you need to do multiple collision steps in order to keep the simulation stable. Do 1 collision step per 1 / 60th of a second (round up).
		const int cCollisionSteps = 1;

		// Step the world
		JPH_PhysicsSystem_Update(system, cDeltaTime, cCollisionSteps, jobSystem);
	}

	// Remove the destroy sphere from the physics system. Note that the sphere itself keeps all of its state and can be re-added at any time.
	JPH_BodyInterface_RemoveAndDestroyBody(bodyInterface, sphereId);

	// Remove and destroy the floor
	JPH_BodyInterface_RemoveAndDestroyBody(bodyInterface, floorId);

	JPH_JobSystem_Destroy(jobSystem);

	JPH_PhysicsSystem_Destroy(system);
	JPH_Shutdown();
	return 0;
}

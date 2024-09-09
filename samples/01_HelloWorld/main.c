// Copyright (c) Amer Koleci and Contributors.
// Distributed under the MIT license. See the LICENSE file in the project root for more information.

#include "joltc.h"
#include <stdbool.h>
#include <stdlib.h> // malloc, free
#include <string.h> // memset

int main(int argc, char** argv) 
{
	if (!JPH_Init())
		return 1;

	JPH_PhysicsSystemSettings settings = {};
	JPH_PhysicsSystem* system = JPH_PhysicsSystem_Create(&settings);

	JPH_Shutdown();
	return 0;
}

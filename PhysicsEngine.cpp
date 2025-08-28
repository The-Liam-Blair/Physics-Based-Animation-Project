#define GLM_ENABLE_EXPERIMENTAL

#include "PhysicsEngine.h"

#include <map>
#include <numeric>

#include "Application.h"
#include "Camera.h"
#include "Force.h"

#include <glm/gtx/matrix_cross_product.hpp>
#include <glm/gtx/orthonormalize.hpp>
#include <glm/ext.hpp>

#include <array>
#include <vector>
#include <time.h>

#define SPHERE_COUNT 50
#define GridSizeX 50
#define GridSizeY 50


using namespace glm;

// Sphere template used during the init function and the table
RigidBody sphereTemplate;
RigidBody groundPlane;

// List of spheres
std::vector<RigidBody> spheres;

// List of recorded positions
std::vector<vec3> initialPositions;

// Grid class that stores each square's position, spheres inside it, and the number of each sphere inside it.
class Grid
{
public:
	vec3 pos = vec3();
	std::vector<RigidBody*> Spheres;
	bool isOccupied;
};

// Init a grid 2D array, 50 * 50 squares
Grid GridCollisionBoxes[GridSizeX][GridSizeY];

/**
 * \brief Generates a single new sphere at runtime (When the S key is pressed). The sphere is given a random position, velocity and mass.
 */
void GenerateNewSphere()
{
	// Copy a pre-existing sphere for simplicity.
	RigidBody newSphere = spheres[0];

	// Generate new random position. Similar to the position assignment intially done to all spheres, but now also creates a random y value
	// position between 2.0 and 40.0.
	newSphere.SetPosition(vec3
	((rand() % int(groundPlane.Scale().x * 2) - 2) - (groundPlane.Scale().x - 2),
		((rand() % 38) + 2.0),
		(rand() % int(groundPlane.Scale().z * 2) - 2) - (groundPlane.Scale().z - 2)
	));

	// Generate random velocity for the sphere.
	newSphere.SetVelocity(vec3(rand() % 40 - 19, rand() % 40 - 19, rand() % 40 - 19));
	int sphereType = 1 + rand() % 3;

		// Normal sphere: mass of 1 (red), 2 (green), 3 (blue). Radii of 1, 2, 3 respectively.
		// Use a switch statement to assign mass and colour to each sphere depending on the sphereType.
		switch (sphereType)
		{
		case 1:
			newSphere.SetColor(vec4(1.0f, 0.0f, 0.0f, 1.0f));
			newSphere.SetMass(1.0);
			break;
		case 2:
			newSphere.SetMass(2.0);
			newSphere.SetColor(vec4(0.0f, 1.0f, 0.0f, 1.0f));
			newSphere.SetScale(vec3{ 2.f });
			break;
		case 3:
			newSphere.SetMass(3.0);
			newSphere.SetColor(vec4(0.0f, 0.0f, 1.0f, 1.0f));
			newSphere.SetScale(vec3{ 3.f });
			break;

		// Black spheres denote an error with the sphere assignment.
		default:
			newSphere.SetMass(1.0);
			newSphere.SetColor(vec4(0.0f, 0.0f, 0.0f, 1.0f));
			std::cout << "Error assigning sphere weight/colour for newly generated sphere " << spheres.size() + 1 << std::endl;
			break;
		}

	// Add sphere to the list of drawn spheres now that it has been fully initialised.
	spheres.push_back(newSphere);
}


/**
 * \brief Performs the narrow-phase collision detection and resolution for sphere-to-sphere collisions.
 * \param Spheres List of spheres occupying a given grid square, calculated from broad-phase collision detection.
 */
void SphereImpulse(std::vector<RigidBody*> Spheres)
{
	// Loop through every sphere, and then perform a second loop to compare that sphere to the other spheres (O(n^2)). For optimisation, the 2nd loop index is always initialised as i+1 so that
	// it doesn't perform meaningless checks (For example, checking if spheres 1 and 3 are colliding, then later checking if spheres 3 and 1 are colliding).
	for (int i = 0; i < Spheres.size(); i++)
	{
		for (int j = i + 1; j < Spheres.size(); j++)
		{		
			// Get the distance between both spheres. Then, get their combined radius and check if the distance between the spheres is less than their combined radii.
			vec3 dist = Spheres[i]->Position() - Spheres[j]->Position();
			float radiusSum = Spheres[i]->Scale().x + Spheres[j]->Scale().x;
			float d = dot(dist, dist);
			if (d <= radiusSum * radiusSum)
			{
				// Find impulse vector of the first object. Caclulate the collision normal w.r.t the first object, then apply the impulse formula.
				vec3 normal = normalize(Spheres[i]->Position() - Spheres[j]->Position());

				float penetration = abs(radiusSum - length(dist));

				// If an intersection is found, first correct positions. This is done evenly by moving both objects back by half the distance between each other.
				Spheres[i]->SetPosition(Spheres[i]->Position() + ((penetration * 0.5f) + 0.001) * normal);
				Spheres[j]->SetPosition(Spheres[j]->Position() - ((penetration * 0.5f) + 0.001) * normal);

				// If a sphere is ever pushed under the table (due to a collision from above for example), reset its position back to onto the table and correct the other sphere's position so that it
				// is no longer intersecting. The normal is still used to translate each sphere to make it realistic.
				if (Spheres[i]->Position().y < 2.0)
				{
					Spheres[i]->SetPosition(vec3(Spheres[i]->Position().x, 2.0f, Spheres[i]->Position().z));
					Spheres[j]->SetPosition(Spheres[j]->Position() - ((penetration + 0.001) * normal));
				}
				else if (Spheres[j]->Position().y < 2.0)
				{
					Spheres[j]->SetPosition(vec3(Spheres[j]->Position().x, 2.0f, Spheres[j]->Position().z));
					Spheres[i]->SetPosition(Spheres[i]->Position() + ((penetration + 0.001) * normal));
				}

				// Calculate the 1D relative velocity between both spheres
				float relVel = dot(Spheres[i]->Velocity() - Spheres[j]->Velocity(), normal);

				// Calculate the impulse force using the impulse formula.
				float impulse = -((1.0 + Spheres[i]->CoefficientOfRestitution()) * relVel) / (1.0f / Spheres[i]->Mass() + 1.0f / Spheres[j]->Mass());

				// Apply an impulse force to both spheres, multiplied by the normal which determines the impulse force direction.
				Spheres[i]->ApplyImpulse(impulse * normal);
				Spheres[j]->ApplyImpulse(-impulse * normal);

			}
		}
	}
}

/**
 * \brief Checks for the spheres that are inside each grid square, records each sphere inside the grid.
 * \return The updated list of grid squares that have 2+ spheres inside them.
 */
void UpdateGridBroadPhase()
{
	// Clear list of stored spheres per grid before calculating new colliding spheres
	for (int i = 0; i < GridSizeX; i++)
	{
		for (int j = 0; j < GridSizeY; j++)
		{
			GridCollisionBoxes[i][j].Spheres.clear();
			GridCollisionBoxes[i][j].isOccupied = false;
		}
	}

	// For each sphere, calculate the grid squares that it intersects with, and add the sphere to each intersected grid square.
	// (This approach enables spheres to be added to multiple grid squares when applicable).
	for (int i = 0; i < spheres.size(); i++)
	{
		// Calculate the range of grid squares that the sphere intersects with.
		int gridXMin = std::max(0, int((spheres[i].Position().x - spheres[i].Scale().x) / 4) + 25);
		int gridXMax = std::min(49, int((spheres[i].Position().x + spheres[i].Scale().x) / 4) + 25);

		int gridYMin = std::max(0, int((spheres[i].Position().z - spheres[i].Scale().z) / 4) + 25);
		int gridYMax = std::min(49, int((spheres[i].Position().z + spheres[i].Scale().z) / 4) + 25);

		// Loop through all the intersected grid squares and add the sphere to each one.
		for (int gridX = gridXMin; gridX <= gridXMax; gridX++)
		{
			for (int gridY = gridYMin; gridY <= gridYMax; gridY++)
			{
				GridCollisionBoxes[gridX][gridY].Spheres.push_back(&spheres[i]);
			}
		}
	}

	// Loop through every grid square, and if the grid square has 2+ spheres inside it, set the grid square's isOccupied flag to true.
	for (int i = 0; i < GridSizeX; i++)
	{
		for (int j = 0; j < GridSizeY; j++)
		{
			if (GridCollisionBoxes[i][j].Spheres.size() >= 2)
			{
				GridCollisionBoxes[i][j].isOccupied = true;
			}
		}
	}
}

/**
 * \brief Detects and resolves sphere-to-table collisions. Includes the table top, and the x and z axes walls.
 */
void TableImpulse()
{
	// Loop through every sphere
	for (int i = 0; i < spheres.size(); i++)
	{
		// Bounce on x axes walls.
		if (spheres[i].Position().x > groundPlane.Scale().x - spheres[i].Scale().x || spheres[i].Position().x < -groundPlane.Scale().x + spheres[i].Scale().x)
		{
			// Determine which wall was hit based on position on the x axis. This decides the collision normal, which is used to correctly set the impulse vector to bounce in the opposite direction.
			// Also, corrects position during this phase.
			vec3 normal;
			if (spheres[i].Position().x > groundPlane.Scale().x - spheres[i].Scale().x)
			{
				spheres[i].SetPosition(vec3(groundPlane.Scale().x - spheres[i].Scale().x, spheres[i].Position().y, spheres[i].Position().z));
				normal = vec3(1.0f, 0.0f, 0.0);
			}
			else
			{
				spheres[i].SetPosition(vec3(-groundPlane.Scale().x + spheres[i].Scale().x, spheres[i].Position().y, spheres[i].Position().z));
				normal = vec3(-1.0f, 0.0f, 0.0f);
			}
			float relVel = dot(spheres[i].Velocity(), normal);
			// Calculate the impulse force (mass component is set to 1 as for objects with larger mass, this increases the resultant impules which shouldn't happen), and apply the impulse to the sphere.
			float impulse = -((1.0 + spheres[i].CoefficientOfRestitution()) * relVel / (1.0f / spheres[i].Mass()));
			spheres[i].ApplyImpulse(impulse * normal);
		}

		// Bounce on z axes walls
		// Same operations apply to this code block compared with the above code block, but for the z axis instead.
		if (spheres[i].Position().z > groundPlane.Scale().z - spheres[i].Scale().x || spheres[i].Position().z < -groundPlane.Scale().z + spheres[i].Scale().x)
		{
			vec3 normal;
			if (spheres[i].Position().z > groundPlane.Scale().z - spheres[i].Scale().x)
			{
				spheres[i].SetPosition(vec3(spheres[i].Position().x, spheres[i].Position().y, groundPlane.Scale().z - spheres[i].Scale().x));
				normal = vec3(0.0f, 0.0f, 1.0);
			}
			else
			{
				spheres[i].SetPosition(vec3(spheres[i].Position().x, spheres[i].Position().y, -groundPlane.Scale().z + spheres[i].Scale().x));
				normal = vec3(0.0f, 0.0f, -1.0f);
			}
			float relVel = dot(spheres[i].Velocity(), normal);
			float impulse = -((1.0 + spheres[i].CoefficientOfRestitution()) * relVel / (1.0f / spheres[i].Mass()));
			spheres[i].ApplyImpulse(impulse * normal);
		}

		// Bounce on the table top
		if (spheres[i].Position().y < spheres[i].Scale().y + 1.f)
		{
			spheres[i].SetPosition(vec3(spheres[i].Position().x, spheres[i].Scale().y + 1.f, spheres[i].Position().z));
			vec3 normal = vec3(0.0f, 1.0f, 0.0f);
			float relVel = dot(spheres[i].Velocity(), normal);

			// If downward velocity is low, lower coefficient of restitution to 0.2 to prevent jittering from spheres laying on the table.
			// Otherwise, for objects with sufficient downward velocity, set coefficient of restitutio to 0.8. Normally it is 1.0 but its temporarily lowered so that
			// the sphere will lose energy per bounce and will eventually lay on the table.
			if (relVel >= -1) { spheres[i].SetCoefficientOfRestitution(0.2f); }
			else { spheres[i].SetCoefficientOfRestitution(0.8f); }

			float impulse = -((1.0 + spheres[i].CoefficientOfRestitution()) * relVel / (1.0f / spheres[i].Mass()));
			spheres[i].ApplyImpulse(impulse * normal);

			// Reset coefficient of restitution back to 1.0so it does not interfere with other collisions
			spheres[i].SetCoefficientOfRestitution(1.f);
		}
	}
}

/**
 * \brief Update each sphere's position and velocity per frame using a standard Euler integration formula. Also applies any built-up forces and impulses to the sphere.
 * \param dt Delta time.
 */
void Integrate(float dt)
{
	for (int i = 0; i < spheres.size(); i++)
	{
		// Calculate the new velocIty and position of the particle using the integration formula. Includes adding the impulse and force values.
		vec3 newVel = spheres[i].Velocity() + 
			(1 / spheres[i].Mass()) * (spheres[i].AccumulatedForce() * dt) + 
			((1 / spheres[i].Mass()) * spheres[i].AccumulatedImpulse());
		vec3 newPos = spheres[i].Position() + (newVel * dt);

		// Set new position and velocity values for this particle.
		spheres[i].SetPosition(newPos);
		spheres[i].SetVelocity(newVel);
	}
}

/**
 * \brief Called whenever a sphere is generated for the first time, it attempts to find a valid position that will not immediately intersect another sphere.
 * \return A valid position, non-intersecting position (Or (-1, -1, -1) if a valid position could not be found).
 */
vec3 GetSpherePosition()
{
	int spherePosCounter = 0;

	// Loop up to 100 times per sphere placement test. This means that the program will attempt to place a new sphere up to 100 times before throwing an error.
	while (spherePosCounter < 100)
	{
		// Reset the position check flag to false
		bool badPosition = false;

		// Generate a random position thats within the table boundaries, and also on the table.
		vec3 possiblePosition = vec3
		((rand() % int(groundPlane.Scale().x * 2) - 2) - (groundPlane.Scale().x - 2),
			4.0f,
			(rand() % int(groundPlane.Scale().z * 2) - 2) - (groundPlane.Scale().z - 2));

		// If the list of previously-recorded positions is empty, skip checking and return the position as it is valid. Also add that position to the positions list for future use.
		if (initialPositions.size() == 0)
		{
			initialPositions.push_back(possiblePosition);
			return possiblePosition;
		}

		// For a non-empty positions list, iterate through all the positions in the list. If the distance between the currently-proposed position and a recorded one is less than 2.0 (intersecting)
		// then set the flag to true and end the for loop. Also increment the position counter for the while loop.
		for (int i = 0; i < initialPositions.size(); i++)
		{
			if (distance(possiblePosition, initialPositions[i]) < 2.0)
			{
				badPosition = true;
				spherePosCounter++;
				break;
			}
		}

		// If the position flag is still false, and was not set to true in the previous loop, that means the proposed position is valid. So, add that position to the positions list for future use, and return the valid position.
		if (!badPosition)
		{
			initialPositions.push_back(possiblePosition);
			return possiblePosition;
		}
	}
	// Error message with an error return statement. Error is called when a valid sphere position could not be found after 100 attempts. Outputs the sphere that failed to be placed, and the total sphere count requested.
	std::cout << "ERROR: Failed to successfully place sphere after 100 attempts. Reduce the sphere count." << std::endl;
	std::cout << "Number of spheres that were requested to be generated: " << spheres.size() << std::endl;
	std::cout << "Error detected on sphere number " << initialPositions.size() - 1 << std::endl;
	return vec3{ -1.0f };
}


/**
 * \brief Check if all sphere positions are valid (If none were set to (-1, -1, -1) during the sphere generation process).
 * \return True if all sphere positions are valid, false if not.
 */
bool PhysicsEngine::SpherePositionsValid()
{
	for each (RigidBody spherePos in spheres)
	{
		if (spherePos.Position() == vec3{ -1.0f }) { return false; }
	}
	return true;
}


/**
 * \brief Initialises the physics engine, including the ground plane, spheres, and grid positions.
 * \param camera Camera object.
 * \param meshDb Mesh database for the spheres and ground plane.
 * \param shaderDb Shader database.
 */
void PhysicsEngine::Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Creates new random seed for the random function (Based on current time). Seed determines the values the rand() function generates, so generates new positions everytime the program is run.
	// Without this, the starting positions and velocities of the spheres would always be the same at execution start.
	srand(time(0));

	// Get a few meshes/shaders from the databases
	auto defaultShader = shaderDb.Get("default");

	meshDb.Add("cube", Mesh(MeshDataFromWavefrontObj("resources/models/cube.obj")));
	meshDb.Add("sphere", Mesh(MeshDataFromWavefrontObj("resources/models/ball.obj")));

	// Initialise meshes for the spheres and ground plane
	auto sphereMesh = meshDb.Get("sphere");
	auto cubeMesh = meshDb.Get("cube");


	// Init ground plane
	groundPlane.SetMesh(cubeMesh);
	groundPlane.SetShader(defaultShader);
	groundPlane.SetScale(vec3(100.0, 1.0, 100.0));
	groundPlane.SetPosition(vec3(0.0f, 0.0f, 0.0f));

	// Setup the template sphere
	sphereTemplate.SetMesh(sphereMesh);
	sphereTemplate.SetShader(defaultShader);
	sphereTemplate.SetMass(1.0);
	sphereTemplate.SetCoefficientOfRestitution(1.f);
	sphereTemplate.SetScale(vec3{ 1.0f });

	// For each sphere, initialise it, determine it's position so that it does not intersect with other spheres initially.
	for (int i = 0; i < SPHERE_COUNT; i++)
	{
		// Init sphere
		RigidBody aSphere;

		// Copy template over.
		aSphere = sphereTemplate;

		// Set sphere position, using a function that returns a valid, non-intersecting position.
		aSphere.SetPosition(GetSpherePosition());

		// End the loop, called whenever a sphere position is (-1, -1, -1) which denotes no more valid positions for the table size.
		if (aSphere.Position() == vec3{ -1.0f })
		{
			break;
		}

		// Set the sphere velocity. Using the random function, which has a range of -20 < sphere.velocity < 20
		aSphere.SetVelocity(vec3(rand() % 40 - 19, 0.0f, rand() % 40 - 19));

		// Generate random value between 1 and 3 to determine the mass + colour of each sphere
		int sphereType = 1 + rand() % 3;

		// Use a switch statement to assign mass and colour to each sphere depending on the sphereType.
		switch (sphereType)
		{
		case 1:
			// Red spheres
			aSphere.SetColor(vec4(1.0f, 0.0f, 0.0f, 1.0f));
			break;
		case 2:
			// Green spheres
			aSphere.SetMass(2.0);
			aSphere.SetColor(vec4(0.0f, 1.0f, 0.0f, 1.0f));
			aSphere.SetScale(vec3{2.f});
			break;
		case 3:
			// Blue spheres
			aSphere.SetMass(3.0);
			aSphere.SetColor(vec4(0.0f, 0.0f, 1.0f, 1.0f));
			aSphere.SetScale(vec3{ 3.f });
			break;

			// Black spheres denote an error with the sphere assignment.
		default:
			aSphere.SetMass(1.0);
			aSphere.SetColor(vec4(0.0f, 0.0f, 0.0f, 1.0f));
			std::cout << "Error assigning sphere weight/colour for sphere " << spheres.size() + 1 << std::endl;
			break;
		}

		spheres.push_back(aSphere);
	}
	

	// Setup Grid Positions, used for broad-phase collision detection

	// Counters to assign values to the 2D array
	int counterX = 0;
	int counterY = 0;

	// The 2d array uses i and j to set positions per grid square, and so starts from -25 to 25 to cover the entire table, whose centre is at (0, 0).
	for (int i = -25; i < GridSizeX - 25; i++)
	{
		for (int j = -25; j < GridSizeY - 25; j++)
		{
			// Each grid stores a vec2 representing its scale. So, if a grid stores (-100, -100), it represents a square that spans from (-100, -100) to (-98, -98).
			// In short, to find a grid's actual size/position, add 2 to the stored vec2: the vec2 represents one corner, and the vec2 + 2 represents the opposite corner.
			GridCollisionBoxes[counterX][counterY].Spheres = *new std::vector<RigidBody*>;
			GridCollisionBoxes[counterX][counterY].pos = vec3(i * 4 + 2, 2.0, j * 4 + 2);
			counterX++;
		}
		counterY++;
		counterX = 0;
	}

	// Generate camera with initial position
	camera = Camera(vec3(100, 60, 100));
}


// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
	// Perform integration on all spheres
	Integrate(deltaTime);

	// Clear all previous forces, then add in the gravity force
	for (int i = 0; i < spheres.size(); i++)
	{
		spheres[i].ClearForcesImpulses();
		Force::Gravity(spheres[i]);
	}

	// Update broad-phase collision detection
	UpdateGridBroadPhase();

	// Calculate and resolve sphere-to-sphere collisions
	for (int i = 0; i < GridSizeX; i++)
	{
		for (int j = 0; j < GridSizeY; j++)
		{
			if (GridCollisionBoxes[i][j].isOccupied)
			{
				SphereImpulse(GridCollisionBoxes[i][j].Spheres);
			}
		}
	}

	// Calculate and resolve sphere-to-table collisions
	TableImpulse();
}


// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	// Draw all spheres and the ground plane.
	for (int i = 0; i < spheres.size(); i++)
	{
		spheres[i].Draw(viewMatrix, projMatrix);
	}
	groundPlane.Draw(viewMatrix, projMatrix);
}


void PhysicsEngine::HandleInputKey(int keyCode, bool pressed)
{
	// OnKeyDown events only, not key releases.
	if (pressed)
	{
		// Spawn a new sphere based on input.
		switch (keyCode)
		{
		// Randomly generate a new sphere
		case GLFW_KEY_1:
			GenerateNewSphere();
			break;

		default:
			break;
		}
	}
}
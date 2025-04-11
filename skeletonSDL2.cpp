//DH2323 skeleton code, Lab2 (SDL2 version)
#include <cstddef>
#include <iostream>
#include <glm/glm.hpp>
#include <limits>
#include "SDL2Auxiliary.h"
#include "TestModel.h"

using namespace std;
using glm::vec3;
using glm::mat3;

struct Intersection
{
	vec3 position;
	float distance;
	int triangleIndex;
};

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES

const int SCREEN_WIDTH = 500;
const int SCREEN_HEIGHT = 500;
SDL2Aux *sdlAux;
int t;

vector<Triangle> triangles;

// ----------------------------------------------------------------------------
// FUNCTIONS

void Update(void);
void Draw(void);

bool ClosestIntersection(
	vec3 start,
	vec3 dir,
	const vector<Triangle>& triangles,
	Intersection& closestIntersection
);

int main( int argc, char* argv[] )
{
	sdlAux = new SDL2Aux(SCREEN_WIDTH, SCREEN_HEIGHT);
	t = SDL_GetTicks();	// Set start value for timer.

	while (!sdlAux->quitEvent())
	{
		Update();
		Draw();
	}
	sdlAux->saveBMP("screenshot.bmp");
	return 0;
}

void Update(void)
{
	// Compute frame time:
	int t2 = SDL_GetTicks();
	float dt = float(t2-t);
	t = t2;
	cout << "Render time: " << dt << " ms." << endl;
}

void Draw()
{
	sdlAux->clearPixels();

	for( int y=0; y<SCREEN_HEIGHT; ++y )
	{
		for( int x=0; x<SCREEN_WIDTH; ++x )
		{
			vec3 color( 1, 0.5, 0.5 );
			sdlAux->putPixel(x, y, color);
		}
	}
	sdlAux->render();
}

bool ClosestIntersection(
	vec3 start,
	vec3 dir,
	const vector<Triangle>& triangles,
	Intersection& closestIntersection
)
{
	float max = std::numeric_limits<float>::max();
	closestIntersection = {vec3(max, max, max), max, -1};

	for( size_t index = 0; index < triangles.size(); ++index)
	{
		vec3 v0 = triangles[index].v0;
		vec3 v1 = triangles[index].v1;
		vec3 v2 = triangles[index].v2;
		vec3 e1 = v1 - v0;
		vec3 e2 = v2 - v0;
		vec3 b = start - v0;
		mat3 A( -dir, e1, e2 );
		mat3 At(b, e1, e2);

		// If t < 0 then skip the rest of the calculation
		float t = glm::determinant(At) / glm::determinant(A);
		if(t < 0)
		{
			continue;
		}

		vec3 x = glm::inverse(A) * b; // Intersection point

		// Update closestIntersection
		float distance = glm::distance(start, x); 
		if(distance < closestIntersection.distance)
		{
			closestIntersection.distance = distance;
			closestIntersection.position = x;
			closestIntersection.triangleIndex = index;
		}
	}
}
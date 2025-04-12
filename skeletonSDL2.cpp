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

float focalLength = (float)SCREEN_HEIGHT;
vec3 cameraPos( 0, 0, -3 );

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

	LoadTestModel(triangles);

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
			vec3 color( 0, 0, 0 );
			Intersection closestIntersection;
			if(ClosestIntersection(cameraPos, vec3( x-(SCREEN_WIDTH/2), y-(SCREEN_HEIGHT/2), focalLength ), triangles, closestIntersection))
			{
				color = triangles[closestIntersection.triangleIndex].color;	
			}
			
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
	bool result = false;

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
		// float t = glm::determinant(At) / glm::determinant(A);
		// if(t < 0)
		// {
		// 	continue;
		// }

		vec3 tuv = glm::inverse(A) * b;
		float t = tuv.x;
		float u = tuv.y;
		float v = tuv.z;

		// Update closestIntersection
		if(u >= 0.0f && v >= 0.0f && (u + v) <= 1.0f && t >= 0.0f && t < closestIntersection.distance)
		{
			closestIntersection.distance = t;
			closestIntersection.position = start + t * dir;
			closestIntersection.triangleIndex = index;
			result = true;
		}
	}

	return result;
}
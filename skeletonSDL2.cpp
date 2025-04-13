// DH2323 skeleton code, Lab2 (SDL2 version)
#include "SDL2Auxiliary.h"
#include "TestModel.h"
#include <cstddef>
#include <glm/glm.hpp>
#include <iostream>
#include <limits>

using namespace std;
using glm::mat3;
using glm::vec3;

struct Intersection {
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
vec3 cameraPos(0, 0, -3);
mat3 R(1.0f);
float yaw = 0.0f;
float moveSpeed = 0.05f;
float rotateSpeed = 0.025f;

// Light source
vec3 lightPos(0, -0.5, -0.7);
vec3 lightColor = 14.f * vec3(1, 1, 1);

// ----------------------------------------------------------------------------
// FUNCTIONS

void Update(void);
void Draw(void);

bool ClosestIntersection(vec3 start, vec3 dir,
                         const vector<Triangle> &triangles,
                         Intersection &closestIntersection);
vec3 DirectLight(const Intersection &i);

int main(int argc, char *argv[]) {
  sdlAux = new SDL2Aux(SCREEN_WIDTH, SCREEN_HEIGHT);
  t = SDL_GetTicks(); // Set start value for timer.

  LoadTestModel(triangles);

  while (!sdlAux->quitEvent()) {
    Update();
    Draw();
  }
  sdlAux->saveBMP("screenshot.bmp");
  return 0;
}

void Update(void) {
  // Compute frame time:
  int t2 = SDL_GetTicks();
  float dt = float(t2 - t);
  t = t2;
  cout << "Render time: " << dt << " ms." << endl;
  const Uint8 *keystate = SDL_GetKeyboardState(NULL);
  if (keystate[SDL_SCANCODE_W]) {
    cameraPos += moveSpeed * vec3(R[0][2], R[1][2], R[2][2]);
  }
  if (keystate[SDL_SCANCODE_S]) {
    cameraPos -= moveSpeed * vec3(R[0][2], R[1][2], R[2][2]);
  }
  if (keystate[SDL_SCANCODE_A]) {
    yaw -= rotateSpeed;
  }
  if (keystate[SDL_SCANCODE_D]) {
    yaw += rotateSpeed;
  }
  R = mat3(cos(yaw), 0, sin(yaw), 0, 1, 0, -sin(yaw), 0, cos(yaw));
}

void Draw() {
  sdlAux->clearPixels();

  for (int y = 0; y < SCREEN_HEIGHT; ++y) {
    for (int x = 0; x < SCREEN_WIDTH; ++x) {
      vec3 color(0, 0, 0);
      Intersection closestIntersection;
      if (ClosestIntersection(cameraPos,
                              vec3(x - (SCREEN_WIDTH / 2),
                                   y - (SCREEN_HEIGHT / 2), focalLength) *
                                  R,
                              triangles, closestIntersection)) {
        color = triangles[closestIntersection.triangleIndex].color;
      }

      sdlAux->putPixel(x, y, color);
    }
  }
  sdlAux->render();
}

bool ClosestIntersection(vec3 start, vec3 dir,
                         const vector<Triangle> &triangles,
                         Intersection &closestIntersection) {
  float max = std::numeric_limits<float>::max();
  closestIntersection = {vec3(max, max, max), max, -1};
  bool result = false;

  for (size_t index = 0; index < triangles.size(); ++index) {
    vec3 v0 = triangles[index].v0;
    vec3 v1 = triangles[index].v1;
    vec3 v2 = triangles[index].v2;
    vec3 e1 = v1 - v0;
    vec3 e2 = v2 - v0;
    vec3 b = start - v0;
    mat3 A(-dir, e1, e2);
    mat3 At(b, e1, e2);
    mat3 Au(-dir, b, e2);
    mat3 Av(-dir, e1, b);

    // If t < 0 then skip the rest of the calculation
    float t = glm::determinant(At) / glm::determinant(A);
    float u = glm::determinant(Au) / glm::determinant(A);
    float v = glm::determinant(Av) / glm::determinant(A);
    if (t < 0) {
      continue;
    }

    // vec3 tuv = glm::inverse(A) * b;
    // float t = tuv.x;
    // float u = tuv.y;
    // float v = tuv.z;

    // Update closestIntersection
    if (u >= 0.0f && v >= 0.0f && (u + v) <= 1.0f && t >= 0.0f &&
        t < closestIntersection.distance) {
      closestIntersection.distance = t;
      closestIntersection.position = start + t * dir;
      closestIntersection.triangleIndex = index;
      result = true;
    }
  }

  return result;
}

vec3 DirectLight(const Intersection &i) {
  float r = glm::distance(lightPos, i.position);
  vec3 lightDir = glm::normalize(lightPos - i.position);
  vec3 normal = triangles[i.triangleIndex].normal;
  vec3 directIllumination =
      (lightColor * glm::max(glm::dot(lightDir, normal), 0.0f)) / (4 * r * r);
  return directIllumination;
}

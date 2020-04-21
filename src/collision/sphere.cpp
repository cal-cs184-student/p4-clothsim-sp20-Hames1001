#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.
    Vector3D pmPosition = pm.position;
    Vector3D pmRadius = pmPosition - origin;
    if (pmRadius.norm() <= radius) {
        auto collision = origin + pmRadius.unit() * radius;
        pm.position = pm.last_position + (1 - friction) * (collision - pm.last_position);
    }

}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}

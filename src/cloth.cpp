#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}
bool checkIfPinned(int x, int y, vector<vector<int>> pinned) {
    
    for (const auto vect : pinned) {
       if (vect[0] == x && vect[1] == y) {
            return true;
        }
    }
    return false;
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
    //r * (row) + c
    for (int y = 0; y < num_height_points; y++) {
        for (int x = 0; x < num_width_points; x++) {
            double xx = (double) x * (width/num_width_points);
            double yy = (double) y * (height/num_height_points);
            double z = fRand(-1.0/1000, 1.0/1000);
            Vector3D position;
            if (orientation == HORIZONTAL) {
                position = Vector3D(xx, 1.0, yy);
            }
            if (orientation == VERTICAL) {
                position = Vector3D(xx, yy, z);
            }
            bool pin = checkIfPinned(x, y, pinned);
            PointMass pm = PointMass(position, pin);
            point_masses.emplace_back(pm);
        }
    }
    
    for (int y = 0; y < num_height_points; y++) {
        for (int x = 0; x < num_width_points; x++) {
            //Left of pm
            if (x > 0) {
                PointMass *a = &point_masses[y * num_width_points + x - 1];
                PointMass *b = &point_masses[y * num_width_points + x];
                Spring s = Spring(a, b, STRUCTURAL);
                springs.emplace_back(s);
            }
            //Above
            if (y > 0) {
                PointMass *a = &point_masses[(y - 1) * num_width_points + x];
                PointMass *b = &point_masses[y * num_width_points + x];
                Spring s = Spring(a, b, STRUCTURAL);
                springs.emplace_back(s);
            }
            //Upper Left
            if (x > 0 && y > 0) {
                PointMass *a = &point_masses[(y - 1) * num_width_points + x - 1];
                PointMass *b = &point_masses[y * num_width_points + x];
                Spring s = Spring(a, b, SHEARING);
                springs.emplace_back(s);
            }
            //upper Right
            if (x+1 < num_width_points && y > 0) {
                PointMass *a = &point_masses[(y - 1) * num_width_points + x + 1];
                PointMass *b = &point_masses[y * num_width_points + x];
                Spring s = Spring(a, b, SHEARING);
                springs.emplace_back(s);
            }
            //two above
            if ((y - 2) >= 0) {
                PointMass *a = &point_masses[(y - 2) * num_width_points + x];
                PointMass *b = &point_masses[y * num_width_points + x];
                Spring s = Spring(a, b, BENDING);
                springs.emplace_back(s);
            }
            //Two left from point mass
            if ((x - 2) >= 0) {
                PointMass *a = &point_masses[y * num_width_points + x - 2];
                PointMass *b = &point_masses[y * num_width_points + x];
                Spring s = Spring(a, b, BENDING);
                springs.emplace_back(s);
            }
        
        }
    }
    

}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
    Vector3D acceleration = Vector3D(0,0,0);
    for (Vector3D exForce : external_accelerations) {
        acceleration += exForce;
    }
    for (PointMass &pm : point_masses) {
        pm.forces = acceleration * mass;
       
    }
    for (Spring &s : springs) {
        if ((cp->enable_structural_constraints && s.spring_type == STRUCTURAL) ||
            (cp->enable_shearing_constraints && s.spring_type == SHEARING) ||
            (cp->enable_bending_constraints && s.spring_type == BENDING)) {
            
            Vector3D p_aMinusP_b = s.pm_a->position - s.pm_b->position;
            //Vector3D opp = s.pm_b->position - s.pm_a->position;
            double Force_s;
            if (s.spring_type == BENDING) {
                Force_s = cp->ks * 0.2 * (p_aMinusP_b.norm() - s.rest_length);
            } else {
                Force_s = cp->ks * (p_aMinusP_b.norm() - s.rest_length);
            }
            Vector3D forceVector = Force_s * p_aMinusP_b.unit();
            //Vector3D opposite = Force_s * opp.unit();
            
            s.pm_a->forces += -forceVector;
            s.pm_b->forces += forceVector;
            
        }
    }

  // TODO (Part 2): Use Verlet integration to compute new point mass positions
    for (PointMass &pm : point_masses) {
        if (!pm.pinned) {
            Vector3D x_t = pm.position;
            Vector3D v_t = pm.position - pm.last_position;
            Vector3D a_t = pm.forces / mass;
            
            pm.last_position = pm.position;
            pm.position = x_t + (1.0 - (cp->damping / 100)) * v_t + a_t * pow(delta_t, 2);
            
        }
    }

   
  // TODO (Part 4): Handle self-collisions.
    for (auto &pm : point_masses) {
        self_collide(pm, simulation_steps);;
    }
    

  // TODO (Part 3): Handle collisions with other primitives.
    for (PointMass &pm : point_masses) {
        
        for (auto primitives : *collision_objects) {
            primitives->collide(pm);
        }
    }


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  
    
    for (Spring &s : springs) {
        auto p_a = s.pm_a->position;
        auto p_b = s.pm_b->position;
        double length = (p_a - p_b).norm() - s.rest_length * 1.2;
        if (length > 0) {
            Vector3D offset = (p_a - p_b).unit() * length;
            if (!s.pm_a->pinned && !s.pm_b->pinned) {
                s.pm_a->position -= 0.5 * offset;
                s.pm_b->position += 0.5 * offset;
            } else if (!s.pm_a->pinned) {
                s.pm_a->position -= offset;
            }else if (!s.pm_b->pinned) {
                s.pm_b->position += offset;
            } else {
                continue;
            }
        }
    }
     

}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
    for (PointMass &pm : point_masses) {
        float val = hash_position(pm.position);
        if (map[val]) {
            map[val]->push_back(&pm);
        } else {
            map[val] = new vector<PointMass *>;
            map[val]->push_back(&pm);
        }
    }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
    float val = hash_position(pm.position);
    Vector3D correct;
    bool hashed = false;
    
    if (map[val]) {
        for (auto *pointMasses : *map[val]) {
            auto checkThickness = pm.position - pointMasses->position;
            if (checkThickness.norm() <= 2 * thickness) {
                if (pointMasses != &pm) {
                    correct = checkThickness.unit() * (2 * thickness - checkThickness.norm());
                    hashed = true;
                }
            }
        }
    }
    if (hashed) {
        pm.position += correct / simulation_steps;
    }

}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
    double w = 3 * width / num_width_points;
    double h = 3 * height / num_height_points;
    double t = fmax(w, h);
    Vector3D tP = Vector3D(fmod(pos.x, w), fmod(pos.y, h), fmod(pos.z, t));
    

    return tP.y * tP.z + tP.x;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}

#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

//        Comment-in this part when you implement the constructor
        Vector2D s2e = end - start;
        Vector2D avg_s2e = s2e / (num_nodes - 1);
        masses.resize(num_nodes);
        springs.resize(num_nodes - 1);

        for(int i = 0; i < num_nodes; i++){
            Vector2D pos = start + i * avg_s2e;
            masses[i] = new Mass(pos, node_mass, false);
        }

        for(int i = 0; i < num_nodes - 1; i++){
            springs[i] = new Spring(masses[i], masses[i+1], k);
        }

        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }

    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D a2b = (s->m2->position - s->m1->position);
            Vector2D force_ab = - s->k * (a2b.norm() - s->rest_length) * (s->m2->position - s->m1->position).unit();
            s->m1->forces += -force_ab;
            s->m2->forces += force_ab;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += m->mass * gravity;
                Vector2D a = m->forces / m->mass;

                //explicit method
/*                Vector2D v_t = m->velocity;
                m->velocity += a * delta_t;
                m->position += v_t * delta_t;  */

                //semi-implicit method
                Vector2D v_t = m->velocity;
                m->velocity += a * delta_t;
                m->position += m->velocity * delta_t;               
                
                // TODO (Part 2): Add global damping
                float kd = 0.00005;
                m->velocity += - kd * m->velocity; 
            }
            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D a2b = (s->m2->position - s->m1->position);
            if(!s->m1->pinned)
                s->m1->position += 0.5 * (a2b.norm() - s->rest_length) * a2b.unit();
            if(!s->m2->pinned)
                s->m2->position += - 0.5 * (a2b.norm() - s->rest_length) * a2b.unit();
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                Vector2D a = gravity;
//                m->position += (m->position - m->last_position) +  a * delta_t * delta_t;
                
                // TODO (Part 4): Add global Verlet damping
                double damping_factor = 0.00005;
                m->position += (1 - damping_factor) * (m->position - m->last_position) +  a * delta_t * delta_t;

                m->last_position = temp_position;
            }
        }
    }
}

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

        Vector2D dir = end - start;

        for (int i = 0; i < num_nodes; ++i) {
            Vector2D pos = start + dir * i / num_nodes;
            masses.push_back(new Mass(pos, node_mass, false));
            if (i != 0) {
                springs.push_back(new Spring(masses[i - 1], masses[i], k));
            }
        }

        //  Comment-in this part when you implement the constructor
        for (auto& i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    // 欧拉法
    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto& s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D ab = s->m2->position - s->m1->position; // a指向b的向量
            Vector2D fab = s->k * ab / ab.norm() * (ab.norm() - s->rest_length);    // b对a施加的力
            s->m1->forces += fab;   // 累加前面节点对a节点的作用力
            s->m2->forces += -fab;   // b节点的作用力
        
            // 添加弹簧内部阻力，即相对速度在ab方向上的投影
            float kd = 0.01;
            Vector2D vab = s->m1->velocity - s->m2->velocity;   // ab的相对速度
            Vector2D fa = kd * -ab / ab.norm() * vab * -ab / ab.norm();    // a的阻力
            Vector2D fb = kd * ab / ab.norm() * -vab * ab / ab.norm(); // b的阻力
            s->m1->forces += fa;
            s->m2->forces += fb;
        }

        for (auto& m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += m->mass * gravity; // 添加重力

                Vector2D accelerate = m->forces / m->mass;
#define EXPLICIT 0
#if EXPLICIT
                // 显式欧拉法
                Vector2D nextV = m->velocity + accelerate * delta_t;
                m->position += m->velocity * delta_t;
                m->velocity = nextV;
#else
                // 使用隐式欧拉法
                m->velocity += accelerate * delta_t;
                m->position += m->velocity * delta_t;
#endif // EXPLICIT

                // TODO (Part 2): Add global damping
                float damping_factor = 0.000005;
                m->position *= 1 - damping_factor;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto& s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D ab = s->m2->position - s->m1->position; // a指向b的向量
            Vector2D fab = s->k * ab / ab.norm() * (ab.norm() - s->rest_length);    // b对a施加的力
            s->m1->forces += fab;   // 累加前面节点对a节点的作用力
            s->m2->forces += -fab;   // b节点的作用力
        }

        for (auto& m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += m->mass * gravity; // 添加重力
                Vector2D accelerate = m->forces / m->mass;

                float damping_factor = 0.000005;
                m->position += (1 - damping_factor) * (m->position - m->last_position) + accelerate * pow(delta_t, 2);

                m->last_position = temp_position;
            }
            m->forces = Vector2D(0, 0);
        }
    }
}

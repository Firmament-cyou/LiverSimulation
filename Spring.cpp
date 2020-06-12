#include "Spring.h"
#include <algorithm>
#include <iostream>

using namespace std;

void Spring::update(float delta_time)
{
	updateAccelerations();
	updatePositions(delta_time);
	updateVelocities(delta_time);
}

void Spring::restoreVelocities()
{
	fill(velocities_.begin(), velocities_.end(), glm::vec3(0.0f, 0.0f, 0.0f));
}

void Spring::restoreParticles()
{
	vector<Vertex>& vertices = mesh_ptr_->vertices;
	for (auto& spr_c : springs_c_) {
		glm::vec3& pos = vertices[spr_c.v].Position;
		pos = spr_c.pos;
	}
	fill(velocities_.begin(), velocities_.end(), glm::vec3(0.0f, 0.0f, 0.0f));
}

void Spring::initSprings()
{
	vector<unsigned int>& indices = mesh_ptr_->indices;
	vector<Vertex>& vertices = mesh_ptr_->vertices;
	for (int i = 0; i < indices.size(); i += 3) {
		float dis1 = glm::distance(vertices[indices[i]].Position, vertices[indices[i + 1]].Position);
		float dis2 = glm::distance(vertices[indices[i]].Position, vertices[indices[i + 2]].Position);
		float dis3 = glm::distance(vertices[indices[i + 1]].Position, vertices[indices[i + 2]].Position);
		springs_p_.emplace_back(SprP{ indices[i], indices[i + 1], dis1 });
		springs_p_.emplace_back(SprP{ indices[i], indices[i + 2], dis2 });
		springs_p_.emplace_back(SprP{ indices[i + 1], indices[i + 2], dis3 });
	}
	for (int i = 0; i < vertices.size(); ++i) {
		springs_c_.emplace_back(SprC{ (unsigned int)i, vertices[i].Position });
	}
}

void Spring::updateAccelerations()
{
	fill(accelerations_.begin(), accelerations_.end(), glm::vec3(0.0f, 0.0f, 0.0f));
	vector<Vertex>& vertices = mesh_ptr_->vertices;
	for (auto& spr_p : springs_p_) {
		glm::vec3& pos1 = vertices[spr_p.v1].Position;
		glm::vec3& pos2 = vertices[spr_p.v2].Position;
		float dis = glm::distance(pos1, pos2);
		accelerations_[spr_p.v1] += glm::normalize(pos2 - pos1) * c_p * (dis - spr_p.length);
		accelerations_[spr_p.v2] += glm::normalize(pos1 - pos2) * c_p * (dis - spr_p.length);
	}
	for (auto& spr_c : springs_c_) {
		glm::vec3& pos = vertices[spr_c.v].Position;
		float dis = glm::distance(pos, spr_c.pos);
		if (dis < 0.0001f) continue; // 归一化零向量是未定义的
		accelerations_[spr_c.v] += glm::normalize(spr_c.pos - pos) * c_c * dis;
	}
	// 随便加个阻尼……
	for (int i = 0; i < accelerations_.size(); ++i) {
		accelerations_[i] -= velocities_[i] * 0.5f;
	}
}

inline void Spring::updateVelocities(float delta_time)
{
	for (int i = 0; i < velocities_.size(); ++i) {
		velocities_[i] += delta_time * accelerations_[i];
	}
}

inline void Spring::updatePositions(float delta_time)
{
	vector<Vertex>& vertices = mesh_ptr_->vertices;
	for (int i = 0; i < vertices.size(); ++i) {
		Vertex& vert = vertices[i];
		vert.Position += velocities_[i] * delta_time + accelerations_[i] * delta_time * delta_time / 2.0f;
	}
}

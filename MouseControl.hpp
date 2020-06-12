#pragma once
#include <glad/glad.h>
#include "Mesh.h"
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>

class MouseControl {
public:
	MouseControl(Mesh* mesh_ptr) : mesh_ptr_(mesh_ptr) {
		sparse_index_.resize(10000, -1);
		sparse_z_.resize(10000);
	}

	// 重点在于网格和坐标的转换
	void update(glm::mat4 translate) {
		if (vertex_index_ >= 0) return;
		std::fill(sparse_index_.begin(), sparse_index_.end(), -1);
		std::vector<Vertex>& vertices = mesh_ptr_->vertices;
		for (int i = 0; i < vertices.size(); ++i) {
			glm::vec4 pos = translate * glm::vec4(vertices[i].Position, 1.0f);
			int index = int((pos.x / pos.w + 1.0f) / 0.02f) * 100 + int((-pos.y / pos.w + 1.0f) / 0.02f);
			if (index < 0 || index >= 10000) continue;
			if (sparse_index_[index] >= 0 && sparse_z_[index] < pos.z / pos.w) continue;
			sparse_index_[index] = i;
			sparse_z_[index] = pos.z / pos.w;
		}
		translate_ = translate;
	}

	void getVertex(double xpos, double ypos) {
		// 事实上，double 类型的 xypos 得到的是像素值，为什么不用 int 呢？
		int index = int(xpos) / 8 * 100 + int(ypos) / 6;
		vertex_index_ = sparse_index_[index];
	}

	void releaseVertex() {
		vertex_index_ = -1;
	}

	void onDrag(double xpos, double ypos) {
		if (vertex_index_ < 0) return;
		Vertex& vert = mesh_ptr_->vertices[vertex_index_];
		glm::vec4 show = translate_ * glm::vec4(vert.Position, 1.0f);
		float x = (float(xpos) - 400.0f) / 400.0f;
		float y = -(float(ypos) - 300.0f) / 300.0f;
		glm::vec4 pos = glm::vec4(x, y, show.z / show.w, 1.0f);
		pos = glm::inverse(translate_) * pos;
		vert.Position.x = pos.x / pos.w;
		vert.Position.y = pos.y / pos.w;
		vert.Position.z = pos.z / pos.w;
	}

private:
	Mesh* mesh_ptr_;
	std::vector<int> sparse_index_; // 100 * 100 的稀疏网格
	std::vector<float> sparse_z_;
	int vertex_index_ = -1;
	glm::mat4 translate_;
};

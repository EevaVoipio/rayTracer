#pragma once
// solution code removed
#include "rtutil.hpp"
#include <vector>
#include "RTTriangle.hpp"
#include "RaycastResult.hpp"
#include "BvhNode.hpp"
#include "LinearBvhNode.hpp"

namespace FW
{

	class Bvh {
	public:
		uint32_t nodeCount = 0;
		std::vector<LinearBvhNode> nodes;
		mutable BvhNode root;
		void build(std::vector<int>& indices, BvhNode& node, std::vector<RTTriangle>& triangles);
		void buildSAH(BvhNode & node, std::vector<int>& indices, std::vector<RTTriangle>& triangles);
		AABB getBox(std::vector<int>& indices, std::vector<RTTriangle>& triangles, int start, int end);
		bool intersectBox(const BvhNode & node, const Vec3f & orig, const Vec3f & dir, float & tBox) const;
		bool intersectBox(const LinearBvhNode & node, const Vec3f & orig, const Vec3f & dir, float & tBox) const;
		int flatten(const BvhNode & node, int & index);
		Bvh();

	};
}
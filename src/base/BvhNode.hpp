#pragma once

// solution code removed
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include "rtutil.hpp"
	struct BvhNode
	{
		FW::AABB box; // Axis-aligned bounding box
		int startPrim, endPrim; // Indices in the global list
		std::unique_ptr<BvhNode> leftChild;
		std::unique_ptr<BvhNode> rightChild;
		bool leaf;
		int axis;

		BvhNode(int start, int end) {
			startPrim = start;
			endPrim = end;
			leaf = false;
		};

		BvhNode() {};

		friend std::ostream & operator<<(std::ostream& stream, const BvhNode& node) {
			return stream << node.startPrim << " " << node.endPrim << " " << node.box.min.x << " " << node.box.min.y << " " << node.box.min.z << " " << node.box.max.x << " " << node.box.max.y << " " << node.box.max.z << " " << node.leaf <<" ";
		};

		friend std::istream & operator>>(std::istream& stream, BvhNode& node) {
			stream >> node.startPrim >> node.endPrim >> node.box.min.x >> node.box.min.y >> node.box.min.z >> node.box.max.x >> node.box.max.y >> node.box.max.z >> node.leaf;
			return stream;
		};
	};


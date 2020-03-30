#pragma once
struct LinearBvhNode
{
	FW::AABB box; // Axis-aligned bounding box
	int startPrim, endPrim; // Indices in the global list

	int rightChildIndex;
	int index;
	bool leaf;
	int axis;

	LinearBvhNode(int start, int end) {
		startPrim = start;
		endPrim = end;
		leaf = false;
		rightChildIndex = -1;
	};


	friend std::ostream & operator<<(std::ostream& stream, const LinearBvhNode& node) {
		return stream << node.startPrim << " " << node.endPrim << " " << node.box.min.x << " " << node.box.min.y << " " << node.box.min.z << " " << node.box.max.x << " " << node.box.max.y << " " << node.box.max.z << " " 
			<< node.leaf << " " << node.axis << " " << node.index << " " << node.rightChildIndex;
	};

	friend std::istream & operator>>(std::istream& stream, LinearBvhNode& node) {
		stream >> node.startPrim >> node.endPrim >> node.box.min.x >> node.box.min.y >> node.box.min.z >> node.box.max.x >> node.box.max.y >> node.box.max.z >> node.leaf >> node.axis >> node.index >> node.rightChildIndex;
		return stream;
	};
};


#include "Bvh.hpp"
#include <iostream>
#include <algorithm>

// solution code removed

namespace FW
{

	int getMaxDimension(Vec3f dimensions) {
		int max = 0;
		if (dimensions.y > dimensions.x) {
			max = 1;
		}
		if (dimensions.z > dimensions.y) {
			max = 2;
		}
		return max;
	}

	void FW::Bvh::build(std::vector<int>& indices, BvhNode& node, std::vector<RTTriangle>& triangles) {
		node.box = getBox(indices, triangles, node.startPrim, node.endPrim);
		if (node.endPrim - node.startPrim >= 10) {
			int splitDimension = getMaxDimension(node.box.max - node.box.min);
			node.axis = splitDimension;
			FW::F32 splitCoord = node.box.min[splitDimension] + 0.5f * (node.box.max[splitDimension] - node.box.min[splitDimension]);
			int mid = node.startPrim;
			for (auto i = node.startPrim; i <= node.endPrim; i++) {

				if (triangles[indices[i]].centroid()[splitDimension] < splitCoord) {
					std::swap(indices[i], indices[mid]);
					mid++;
				}
			}
			if (node.startPrim == mid || node.endPrim == (mid-1)) {
				mid = (node.endPrim + node.startPrim) / 2;
			}
			node.leftChild = std::unique_ptr<BvhNode>(new BvhNode(node.startPrim, (mid-1)));
			build(indices, *node.leftChild,triangles);
			node.rightChild = std::unique_ptr<BvhNode>(new BvhNode(mid, node.endPrim));
			build(indices, *node.rightChild, triangles);
		} else {
			node.leaf = true;
		}
	}

	void FW::Bvh::buildSAH(BvhNode & node, std::vector<int>& indices, std::vector<RTTriangle>& triangles)
	{
		node.box = getBox(indices, triangles, node.startPrim, node.endPrim);
		float minCost = INFINITY;
		int splitDimension = -1;
		int triangleCount = (node.endPrim - node.startPrim + 1);
		int split = -1;
		int bestBucket = -1;
		typedef std::pair<int, AABB> splitInfo;
		int numBuckets = 12;
		std::vector<splitInfo> buckets(numBuckets);
		std::vector<splitInfo> backward(numBuckets);

		if (triangleCount >=16) {

			AABB centroidBox = AABB(triangles[indices[node.startPrim]].centroid(), triangles[indices[node.startPrim]].centroid());
			AABB leftBox, rightBox;

			for (int i = node.startPrim + 1; i <= node.endPrim; i++) {
				centroidBox.extend(triangles[indices[i]].centroid(), triangles[indices[i]].centroid());
			}

			for (int j = 0; j < 3; j++) {

				//Initialize buckets
				for (int i = 0; i < 12; i++) {
					buckets[i] = std::make_pair(0, AABB(FLT_MAX, FLT_MIN));
				}
				if (centroidBox.max[j] - centroidBox.min[j] == 0) {
					continue;
				}
				const float bbLenInv = 1.0f / (centroidBox.max[j] - centroidBox.min[j]);

				//Define to which bucket each triangle belongs
				for (int i = node.startPrim; i <= node.endPrim; ++i) {
					const auto & triangle = (triangles)[indices[i]];
					int idx = numBuckets * (triangle.centroid()[j] - centroidBox.min[j]) * bbLenInv;
					if (idx == numBuckets) --idx;
					if (idx >= 0 && idx < numBuckets) {
						++buckets[idx].first;
						buckets[idx].second.extend(triangle.min(), triangle.max());
					}
				}

				//Scan backwards to get right side for each bucket split
				backward[numBuckets - 1] = buckets[numBuckets - 1];
				for (int i = 1; i < numBuckets; ++i) {
					backward[numBuckets - i - 1] = backward[numBuckets - i];
					backward[numBuckets - i - 1].second.extend(buckets[numBuckets - i - 1].second);
					backward[numBuckets - i - 1].first += buckets[numBuckets - i - 1].first;
				}


				//Calculate cost for each bucket
				AABB lbox = buckets[0].second;
				int left = 0;
				for (int i = 0; i < numBuckets - 1; ++i) {
					lbox.extend(buckets[i].second);
					const float larea = lbox.area();
					const float rarea = backward[i + 1].second.area();
					const int rcnt = backward[i + 1].first;
					const int lcnt = triangleCount - rcnt;
					left += buckets[i].first;
					if (lcnt == 0) continue;

					const float cost = (lcnt * larea + rcnt * rarea);

					if (cost < minCost) {
						minCost = cost;
						bestBucket = i;
						splitDimension = j;
						leftBox = lbox;
						rightBox = backward[bestBucket + 1].second;
						split = node.startPrim + lcnt - 1;
					}

				}

			}

			if (bestBucket != -1) {
				node.axis = splitDimension;
				const float bbLenInv = 1.0f / (centroidBox.max[splitDimension] - centroidBox.min[splitDimension]);
				int off = node.startPrim;
				for (int i = node.startPrim; i <= node.endPrim; ++i) {
					const auto & triangle = (triangles)[indices[i]];
					int idx = numBuckets * (triangle.centroid()[splitDimension] - centroidBox.min[splitDimension]) * bbLenInv;
					if (idx == numBuckets) --idx;
					if (idx <= bestBucket) {
						std::swap(indices[i], indices[off++]);
					}
				}

					node.leftChild = std::unique_ptr<BvhNode>(new BvhNode(node.startPrim, split));
					buildSAH(*node.leftChild, indices, triangles);


					node.rightChild = std::unique_ptr<BvhNode>(new BvhNode(split + 1, node.endPrim));
					buildSAH(*node.rightChild, indices, triangles);
			}
			else {
				node.leaf = true;
			}

		}
		else {
			node.leaf = true;
		}
	}


	AABB Bvh::getBox(std::vector<int>& indices, std::vector<RTTriangle>& triangles, int start, int end) {
		Vec3f min = triangles[indices[start]].min();
		Vec3f max = triangles[indices[start]].max();
		for (auto i = start + 1; i <= end; i++) {
			if (triangles[indices[i]].min().x < min.x) {
				min.x = triangles[indices[i]].min().x;
			}
			if (triangles[indices[i]].min().y < min.y) {
				min.y = triangles[indices[i]].min().y;
			}
			if (triangles[indices[i]].min().z < min.z) {
				min.z = triangles[indices[i]].min().z;
			}
			if (triangles[indices[i]].max().x > max.x) {
				max.x = triangles[indices[i]].max().x;
			}
			if (triangles[indices[i]].max().y > max.y) {
				max.y = triangles[indices[i]].max().y;
			}
			if (triangles[indices[i]].max().z > max.z) {
				max.z = triangles[indices[i]].max().z;
			}
		}
		return AABB(min, max);
	}
	bool Bvh::intersectBox(const LinearBvhNode & node, const Vec3f & orig, const Vec3f & dir, float &tBox) const {

		float tmin = -INFINITY, tmax = tBox;
		//Vec3f invDir = (Vec3f(1.0f, 1.0f, 1.0f) / (dir));

		for (int i = 0; i < 3; i++) {
			float t1 = (node.box.min[i] - orig[i])*dir[i];
			float t2 = (node.box.max[i] - orig[i])*dir[i];

			tmin = max(tmin, min(t1, t2));
			tmax = min(tmax, max(t1, t2));

		}

		if (tmax >= max(tmin, 0.0f)) {
			//tBox = tmin;
			return true;
		}

		return false;//tmax >= max(tmin, 0.0f);
		/*Vec3f t1 = (node.box.min - orig) * dir;
		Vec3f t2 = (node.box.max - orig) * dir;
		Vec3f tmin = min(t1, t2);
		Vec3f tmax = max(t1, t2);
		tBox = min(tBox, tmin.max());
		return tmin.max() <= tmax.min();*/
		//float ttmin = max()

	}
	int Bvh::flatten(const BvhNode & node, int &index)
	{
		LinearBvhNode linearNode = LinearBvhNode(node.startPrim, node.endPrim);
		linearNode.axis = node.axis;
		linearNode.box = node.box;
		linearNode.leaf = node.leaf;
		linearNode.index = index;
		++index;
		++nodeCount;
		nodes.push_back(linearNode);
		if (!linearNode.leaf) {
			flatten(*node.leftChild, index);
			int rightChildIndex = linearNode.rightChildIndex = flatten(*node.rightChild, index);
			nodes[linearNode.index].rightChildIndex = rightChildIndex;
		}
		return linearNode.index;
		
	}

	Bvh::Bvh() {

	}

}

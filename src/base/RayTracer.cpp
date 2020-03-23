#define _CRT_SECURE_NO_WARNINGS

#include "base/Defs.hpp"
#include "base/Math.hpp"
#include "RayTracer.hpp"
#include <stdio.h>
#include "rtIntersect.inl"
#include <fstream>


#include "rtlib.hpp"


// Helper function for hashing scene data for caching BVHs
extern "C" void MD5Buffer(void* buffer, size_t bufLen, unsigned int* pDigest);


namespace FW
{
	template<class T>
	std::ostream & writeIndex(std::ostream& stream, const T& x);
	template<class T>
	std::istream& loadIndex(std::istream& stream, T& x);


	Vec2f getTexelCoords(Vec2f uv, const Vec2i size)
	{

		// YOUR CODE HERE (R3):
		// Get texel indices of texel nearest to the uv vector. Used in texturing.
		// uv should be mapped from [0,1]x[0,1] to the size of the image [0,X]x[0,Y] and
		// wrapped so that uv values beyond 1 are mapped back to [0,1] in a repeating manner
		// (so 0.5, 1.5, 2.5,... all map to 0.5)

		float intx, inty;
		float x = modf(uv.x, &intx);
		float y = modf(uv.y, &inty);
		x = modf((x + 1), &intx);
		y = modf((y + 1), &inty);

		/*int x = int(u * textureWidth) % (textureWidth - 1);
int y = int(v * textureHeight) % (textureHeight - 1);*/

		Vec2f coords = Vec2f(max(0.0f, x) * (size.x), max(0.0f, y) * (size.y));
		return coords;
	}

	Mat3f formBasis(const Vec3f& n) {
		Mat3f rotation;
		rotation.setCol(2, n);
		Vec3f t = n;
		float min = 0;
		for (auto i = 0; i < 3; i++) {
			if (abs(t[i]) < abs(t[min])) {
				min = i;
			}
		}
		t[min] = 1;
		t = cross(t, n).normalized();
		rotation.setCol(0, t);
		Vec3f b = cross(t, n);
		rotation.setCol(1, b);
		// YOUR CODE HERE (R4):
		return rotation;//Mat3f();
	}


	String RayTracer::computeMD5(const std::vector<Vec3f>& vertices)
	{
		unsigned char digest[16];
		MD5Buffer((void*)&vertices[0], sizeof(Vec3f)*vertices.size(), (unsigned int*)digest);

		// turn into string
		char ad[33];
		for (int i = 0; i < 16; ++i)
			::sprintf(ad + i * 2, "%02x", digest[i]);
		ad[32] = 0;

		return FW::String(ad);
	}


	// --------------------------------------------------------------------------


	RayTracer::RayTracer()
	{
	}

	RayTracer::~RayTracer()
	{
	}


	void RayTracer::loadHierarchy(const char* filename, std::vector<RTTriangle>& triangles)
	{
		std::ifstream infile(filename, std::ios::binary);
		// YOUR CODE HERE (R2):
		for (auto i = 0; i < triangles.size(); i++) {
			int j;
			loadIndex(infile, j);
			indices.push_back(j);
		}
		loadNodes(infile, bvh.root);
		m_triangles = &triangles;
	}

	void RayTracer::saveHierarchy(const char* filename, const std::vector<RTTriangle>& triangles) {
		std::ofstream outfile(filename, std::ios::binary);
		for (auto i = 0; i < indices.size(); i++) {
			writeIndex(outfile, indices[i]);
		}
		writeNodes(outfile, bvh.root);
		//writeNode(outfile, bvh.root);
		// YOUR CODE HERE (R2)
	}

	void writeNodes(std::ostream& stream, BvhNode& node) {
		if (node.leftChild != nullptr) {
			stream << node;
			writeNodes(stream, *node.leftChild);
			writeNodes(stream, *node.rightChild);
		}
		else {
			stream << node;
		}
	}

	void loadNodes(std::istream& stream, BvhNode& node) {
		stream >> node;
		if (!node.leaf) {
			node.leftChild = std::unique_ptr<BvhNode>(new BvhNode());
			loadNodes(stream, *node.leftChild);
			node.rightChild = std::unique_ptr<BvhNode>(new BvhNode());
			loadNodes(stream, *node.rightChild);
		}
	}

	template<class T>
	std::ostream& writeIndex(std::ostream& stream, const T& x) {
		return stream.write(reinterpret_cast<const char*>(&x), sizeof(x));
	}

	template<class T>
	std::istream& loadIndex(std::istream& stream, T& x) {
		return stream.read(reinterpret_cast<char*>(&x), sizeof(x));
	}



	void RayTracer::constructHierarchy(std::vector<RTTriangle>& triangles, SplitMode splitMode) {
		//std::vector<int> indices(triangles.size());
		for (auto i = 0; i < triangles.size(); i++) {
			indices.push_back(i);
		}
		// YOUR CODE HERE (R1
		bvh = Bvh();
		bvh.root = BvhNode(0, triangles.size() - 1);
		bvh.buildSAH(bvh.root, indices, triangles);
		m_triangles = &triangles;
	}


	RaycastResult RayTracer::raycast(const Vec3f& orig, const Vec3f& dir) const {
		++m_rayCount;
		RaycastResult castresult;

		float tBox;
		std::stack<BvhNode*> nodes;
		Vec3f invDir = (Vec3f(1.0f, 1.0f, 1.0f) / (dir));
		bool hitFound = false;
		float tFound = INFINITY;
		float tmin = 1.0f, umin = 0.0f, vmin = 0.0f;
		int imin = -1;
		bool nearestHitFound = false;

		bool hitsBox = bvh.intersectBox(bvh.root, orig, invDir, tBox);
		if (hitsBox) {
			nodes.push(&bvh.root);
		}
		while (!nodes.empty()) {
			BvhNode * current = nodes.top();
			nodes.pop();
			if (current->leftChild != nullptr && current->rightChild != nullptr) {
						hitsBox = bvh.intersectBox(*current->leftChild, orig, invDir, tBox);
						if (hitsBox && tBox < tFound) {
							float leftHit = tBox;
							hitsBox = bvh.intersectBox(*current->rightChild, orig, invDir, tBox);
							if (hitsBox && tBox < tFound) {
								if (leftHit < tBox) {
									nodes.push(current->rightChild.get());
									nodes.push(current->leftChild.get());
								}
								else {
									nodes.push(current->leftChild.get());
									nodes.push(current->rightChild.get());
								}
							}
							else {
								nodes.push(current->leftChild.get());
							}
						}
						else {
							hitsBox = bvh.intersectBox(*current->rightChild, orig, invDir, tBox);
							if (hitsBox && tBox < tFound) {
								nodes.push(current->rightChild.get());
							}
						}
					}
			else {
					for (size_t i = current->startPrim; i <= current->endPrim; i++) {
						float t, u, v;
						//if ((*m_triangles)[i].intersect_woop(orig, dir, t, u, v)) {
						if ((*m_triangles)[indices[i]].intersect_woop(orig, dir, t, u, v)) {
							if (t > 0.0f && t < tmin) {
								imin = i;
								tmin = t;
								tFound = t;
								hitFound = true;
								umin = u;
								vmin = v;

							}
						}
					}
				}
		}
		if (imin != -1) {
			castresult = RaycastResult(&(*m_triangles)[indices[imin]], tmin, umin, vmin, orig + tmin*dir, orig, dir);
			//castresult = RaycastResult(&(*m_triangles)[imin], tmin, umin, vmin, orig + tmin*dir, orig, dir);
		}


		return castresult;

	}

}
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
		return rotation;
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
		/*std::ifstream infile(filename, std::ios::binary);
		// YOUR CODE HERE (R2):
		for (auto i = 0; i < triangles.size(); i++) {
			int j;
			loadIndex(infile, j);
			indices.push_back(j);
		}
		loadNodes(infile, bvh.root);
		m_triangles = &triangles;*/
	}

	void RayTracer::saveHierarchy(const char* filename, const std::vector<RTTriangle>& triangles) {
		/*std::ofstream outfile(filename, std::ios::binary);
		for (auto i = 0; i < indices.size(); i++) {
			writeIndex(outfile, indices[i]);
		}
		writeNodes(outfile, bvh.root);
		// YOUR CODE HERE (R2)*/
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
		for (auto i = 0; i < triangles.size(); i++) {
			indices.push_back(i);
		}
		// YOUR CODE HERE (R1
		bvh = Bvh();
		bvh.root = BvhNode(0, triangles.size() - 1);
		//bvh.build(indices, bvh.root, triangles);
		bvh.buildSAH(bvh.root, indices, triangles);
		int rootIndex = 0;
		bvh.nodes.reserve(bvh.nodeCount);
		bvh.flatten(bvh.root, rootIndex);
		m_triangles = &triangles;
	}


	RaycastResult RayTracer::raycast(const Vec3f& orig, const Vec3f& dir, bool shadowRay) const {
		++m_rayCount;
		RaycastResult castresult;

		float tBox;
		//std::vector<BvhNode*> nodes;
		//nodes.reserve(20);
		Vec3f invDir = (Vec3f(1.0f, 1.0f, 1.0f) / (dir));
		Vec3f negDir = Vec3f(invDir.x < 0, invDir.y < 0, invDir.z < 0);
		bool hitFound = false;
		float tFound = INFINITY;
		float tmin = 1.0f, umin = 0.0f, vmin = 0.0f;
		int imin = -1;
		int nodesToVisit[64];
		int nodeIndex = 0;
		int toVisitOffset = 0;

		bool hitsBox = false;

		while (true) {
			LinearBvhNode  current = bvh.nodes[nodeIndex];
			hitsBox = bvh.intersectBox(current, orig, invDir, tFound);

			if (hitsBox ) {
				if (!current.leaf) {
					if (negDir[current.axis]) {
						nodesToVisit[toVisitOffset++] = nodeIndex + 1;
						nodeIndex = current.rightChildIndex;
					}
					else {
						nodesToVisit[toVisitOffset++] = current.rightChildIndex;
						++nodeIndex;
					}
				}
				else {
					for (size_t i = current.startPrim; i <= current.endPrim; i++) {
						float t, u, v;
						if ((*m_triangles)[indices[i]].intersect_woop(orig, dir, t, u, v)) {
							if (shadowRay) {
								if (t > 0.01f && t < tmin) {
									imin = i;
									tmin = t;
									tFound = t;
									umin = u;
									vmin = v;
									break;
								}
							}
							else if (t > 0.00f && t < tmin) {
								auto triangle = (*m_triangles)[indices[i]];
								float alpha = triangle.m_material->diffuse.w;
								Texture& alphaTex = triangle.m_material->textures[MeshBase::TextureType_Alpha];
								if (alphaTex.exists()) {
									const Image& img = *alphaTex.getImage();
									Vec2f uv = (1 - v - u) * triangle.m_vertices[0].t + u * triangle.m_vertices[1].t + v * triangle.m_vertices[2].t;
									Vec2i texelCoords = getTexelCoords(uv, img.getSize());
									alpha = img.getVec4f(texelCoords).y;
								}
								if (alpha >= 0.5f) {
									imin = i;
									tmin = t;
									tFound = t;
									umin = u;
									vmin = v;
								}
							}
						}
					}
					if (toVisitOffset == 0) {
						break;
					}
					nodeIndex = nodesToVisit[--toVisitOffset];
				}

			}
			else {
				if (toVisitOffset == 0) {
					break;
				}
				nodeIndex = nodesToVisit[--toVisitOffset];
			}
		}
		if (imin != -1) {
			castresult = RaycastResult(&(*m_triangles)[indices[imin]], tmin, umin, vmin, orig + tmin*dir, orig, dir);
		}


		return castresult;

	}

}
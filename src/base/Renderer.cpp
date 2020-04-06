#include "Renderer.hpp"
#include "RayTracer.hpp"

#include <atomic>
#include <chrono>
#include <math.h> 


namespace FW {


Renderer::Renderer()
{
    m_aoRayLength = 0.5f;
	m_aoNumRays = 16;
	m_aaNumRays = 1;
    m_raysPerSecond = 0.0f;
}

Renderer::~Renderer()
{
}

void Renderer::gatherLightTriangles(RayTracer* rt) {
	// gather light triangles into vector for possible later use in the area light extra
	m_lightTriangles.clear();
	m_combinedLightArea = .0f;
	for (auto& tri : *rt->m_triangles)
	{
		MeshBase::Material* mat = tri.m_material;
		if (mat->emission.length() > .0f)
		{
			m_lightTriangles.push_back(&tri);
			m_combinedLightArea += tri.area();
		}
	}
}

timingResult Renderer::rayTracePicture( RayTracer* rt, Image* image, const CameraControls& cameraCtrl, ShadingMode mode )
{

    // measure time to render
	LARGE_INTEGER start, stop, frequency;
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&start); // Start time stamp	
	rt->resetRayCounter();

    // this has a side effect of forcing Image to reserve its memory immediately
    // otherwise we get a rendering bug & memory leak in OpenMP parallel code
    image->getMutablePtr();

    // get camera orientation and projection
    Mat4f worldToCamera = cameraCtrl.getWorldToCamera();
    Mat4f projection = Mat4f::fitToView(Vec2f(-1,-1), Vec2f(2,2), image->getSize())*cameraCtrl.getCameraToClip();

    // inverse projection from clip space to world space
    Mat4f invP = (projection * worldToCamera).inverted();

    // progress counter
    std::atomic<int> lines_done = 0;

    int height = image->getSize().y;
    int width = image->getSize().x;

	for (int j = 0; j < height; j++)
	for (int i = 0; i < width; i++)
		image->setVec4f(Vec2i(i, j), Vec4f(.0f)); //initialize image to 0

	// YOUR CODE HERE(R5):
	// remove this to enable multithreading (you also need to enable it in the project properties: C++/Language/Open MP support)
	#pragma omp parallel for
    for ( int j = 0; j < height; ++j )
    {
        // Each thread must have its own random generator
        Random rnd;
		std::vector<Vec2f> offsets;
		offsets.reserve(m_aaNumRays);
		if (m_aaNumRays > 1) {
			getSamplePositions(offsets, rnd);
		}
		else {
			offsets[0] = Vec2f(0.0f, 0.0f);
		}
        for ( int i = 0; i < width; ++i )
        {
			Vec4f aaColor(0, 0, 0, 0);
			float aaWeight = 0.0f;
			for (int aa = 0; aa < m_aaNumRays; ++aa)
			{
				// generate ray through pixel
				Vec2f offset = offsets[aa];//getSamplePosition(aa);
				float x = (i + 0.5f + offset.x) / image->getSize().x *  2.0f - 1.0f;
				float y = (j + 0.5f + offset.y) / image->getSize().y * -2.0f + 1.0f;

				// point on front plane in homogeneous coordinates
				Vec4f P0(x, y, 0.0f, 1.0f);
				// point on back plane in homogeneous coordinates
				Vec4f P1(x, y, 1.0f, 1.0f);

				// apply inverse projection, divide by w to get object-space points
				Vec4f Roh = (invP * P0);
				Vec3f Ro = (Roh * (1.0f / Roh.w)).getXYZ();
				Vec4f Rdh = (invP * P1);
				Vec3f Rd = (Rdh * (1.0f / Rdh.w)).getXYZ();

				// Subtract front plane point from back plane point,
				// yields ray direction.
				// NOTE that it's not normalized; the direction Rd is defined
				// so that the segment to be traced is [Ro, Ro+Rd], i.e.,
				// intersections that come _after_ the point Ro+Rd are to be discarded.
				Rd = Rd - Ro;

				// trace!
				RaycastResult hit = rt->raycast(Ro, Rd);

				// if we hit something, fetch a color and insert into image
				Vec4f color(0, 0, 0, 1);
				if (hit.tri != nullptr)
				{
					switch (mode)
					{
					case ShadingMode_Headlight:
						color = computeShadingHeadlight(hit, cameraCtrl);
						break;
					case ShadingMode_AmbientOcclusion:
						color = computeShadingAmbientOcclusion(rt, hit, cameraCtrl, rnd);
						break;
					case ShadingMode_Whitted:
						color = computeShadingWhitted(rt, hit, cameraCtrl, rnd, 0);
						break;
					}
				}
				float weight = 1; //Use simple box filter for now
				//float weight = getWeight(x) * getWeight(y);
				aaWeight += weight;
				aaColor += weight * color;
				// put pixel.
				//image->setVec4f(Vec2i(i, j), aaColor);
				//image->setVec4f(Vec2i(i, j), color);
			}
			aaColor /= aaWeight;
			image->setVec4f(Vec2i(i, j), aaColor);
        }

        // Print progress info
        ++lines_done;
        ::printf("%.2f%% \r", lines_done * 100.0f / height);
    }

	// how fast did we go?
	timingResult result;

	QueryPerformanceCounter(&stop); // Stop time stamp

	result.duration = (int)((stop.QuadPart - start.QuadPart) * 1000.0 / frequency.QuadPart); // Get timer result in milliseconds

	// calculate average rays per second
	result.rayCount = rt->getRayCount();
    m_raysPerSecond = 1000.0f * result.rayCount / result.duration;

    printf("\n");

	return result;
}


void Renderer::getTextureParameters(const RaycastResult& hit, Vec3f& diffuse, Vec3f& n, Vec3f& specular)
{
	MeshBase::Material* mat = hit.tri->m_material;
	// YOUR CODE HERE (R3):
	// using the barycentric coordinates of the intersection (hit.u, hit.v) and the
	// vertex texture coordinates hit.tri->m_vertices[i].t of the intersected triangle,
	// compute the uv coordinate of the intersection point.

	Vec2f uv = (1 - hit.v - hit.u) * hit.tri->m_vertices[0].t + hit.u * hit.tri->m_vertices[1].t + hit.v * hit.tri->m_vertices[2].t;
//	Vec2f uv = (u,v);// Vec2f(.0f);
	Texture& diffuseTex = mat->textures[MeshBase::TextureType_Diffuse]; //note: you can fetch other kinds of textures like this too. 
																		//By default specular maps, displacement maps and alpha stencils
																		//are loaded too if the .mtl file specifies them.
	if (diffuseTex.exists()) //check whether material uses a diffuse texture
	{
		const Image& img = *diffuseTex.getImage();
		//fetch diffuse color from texture
		Vec2i texelCoords = getTexelCoords(uv, img.getSize());
		diffuse = img.getVec4f(texelCoords).getXYZ();
		// YOUR CODE HERE (R3): uncomment the line below once you have implemented getTexelCoords.
	}
	Texture& normalTex = mat->textures[MeshBase::TextureType_Normal];
	if (normalTex.exists() && m_normalMapped) //check whether material uses a normal map
	{
		const Image& img = *normalTex.getImage();
		Vec2i texelCoords = getTexelCoords(uv, img.getSize());

		Vec3f normal = (2.0f * img.getVec4f(texelCoords).getXYZ() - 1.0f).normalized();
		Vec3f deltaPos1 = hit.tri->m_vertices[1].p - hit.tri->m_vertices[0].p;
		Vec3f deltaPos2 = hit.tri->m_vertices[2].p - hit.tri->m_vertices[0].p;
		Vec2f deltaUV1 = hit.tri->m_vertices[1].t - hit.tri->m_vertices[0].t;
		Vec2f deltaUV2 = hit.tri->m_vertices[2].t - hit.tri->m_vertices[0].t;
		float r = 1.0f / (deltaUV1.x * deltaUV2.y - deltaUV1.y * deltaUV2.x);
		Vec3f tangent = (deltaPos1 * deltaUV2.y - deltaPos2 * deltaUV1.y)*r;
		Vec3f bitangent = (deltaPos2 * deltaUV1.x - deltaPos1 * deltaUV2.x)*r;
		//EXTRA: do tangent space normal mapping
		//first, get texel coordinates as above, for the rest, see handout
		Mat3f tbn;
		tbn.setCol(0, tangent.normalized());
		tbn.setCol(1, bitangent.normalized());
		tbn.setCol(2, n.normalized());
		n = (tbn * normal);
	}

	Texture& specularTex = mat->textures[MeshBase::TextureType_Specular];
	if (specularTex.exists()) {
		const Image& img = *specularTex.getImage();
		Vec2i texelCoords = getTexelCoords(uv, img.getSize());
		specular = img.getVec4f(texelCoords).getXYZ();
	}
	// EXTRA: read a value from the specular texture into specular_mult.



}

Vec4f Renderer::computeShadingHeadlight(const RaycastResult& hit, const CameraControls& cameraCtrl)
{
	// get diffuse color
	MeshBase::Material* mat = hit.tri->m_material;
	Vec3f diffuse = mat->diffuse.getXYZ();
	Vec3f n(hit.tri->normal());
	n = ((1 - hit.v - hit.u) * hit.tri->m_vertices[0].n + hit.u * hit.tri->m_vertices[1].n + hit.v * hit.tri->m_vertices[2].n).normalized();
	Vec3f specular = mat->specular; // specular color. Not used in requirements, but you can use this in extras if you wish.

	if (m_useTextures)
		getTextureParameters(hit, diffuse, n, specular);

    // dot with view ray direction <=> "headlight shading"
	float d = fabs(dot(n, (hit.point - cameraCtrl.getPosition()).normalized()));
    // assign gray value (d,d,d)
	Vec3f shade = d;

	//Specular
	Vec3f reflection = (-(hit.point - cameraCtrl.getPosition()).normalized() - 2 * d * n).normalized();
	float sd = max(0.0f, (dot(hit.dir.normalized(), reflection)));
	float s = pow(sd, mat->glossiness);

	return Vec4f(shade*diffuse + 0.5f * (s*specular), 1.0f);
	//return Vec4f(shade*diffuse, 1.0f);
}


Vec4f Renderer::computeShadingAmbientOcclusion(RayTracer* rt, const RaycastResult& hit, const CameraControls& cameraCtrl, Random& rnd)
{
	Vec4f color;
	float numHits = 16;
	//float numHits = 0;// m_aoNumRays;
	Vec3f n(hit.tri->normal());

	for (auto i = 0; i < m_aoNumRays; i++) {
		Vec3f randomDir = rnd.getVec3f(-1, 1);
		while (pow(randomDir.x, 2) + pow(randomDir.y, 2) > 1) {
			randomDir = rnd.getVec3f(-1, 1);
		}
		randomDir.z = sqrt(1 - pow(randomDir.x, 2) - pow(randomDir.y, 2));
		Mat3f rotation = formBasis(n);
		randomDir = (rotation * randomDir).normalized();
		RaycastResult ambientHit = rt->raycast((hit.point - 0.001f*(hit.point - cameraCtrl.getPosition())), m_aoRayLength*randomDir);
		/*if (ambientHit.t > 0.0f && ambientHit <= m_aoRayLength) {
			++numHits;
		}*/
		if (ambientHit.tri != nullptr) {
			--numHits;
		}
		// YOUR CODE HERE (R4)
	}
	color = numHits / m_aoNumRays;
    return color;
}

Vec4f Renderer::computeShadingWhitted(RayTracer* rt, const RaycastResult& hit, const CameraControls& cameraCtrl, Random& rnd, int num_bounces)
{
	Vec4f color;
	MeshBase::Material* mat = hit.tri->m_material;
	Vec3f diffuse = mat->diffuse.getXYZ();
	Vec3f n(hit.tri->normal());
	n = ((1 - hit.v - hit.u) * hit.tri->m_vertices[0].n + hit.u * hit.tri->m_vertices[1].n + hit.v * hit.tri->m_vertices[2].n).normalized();
	Vec3f specular = mat->specular; // specular color. Not used in requirements, but you can use this in extras if you wish.

	if (m_useTextures)
		getTextureParameters(hit, diffuse, n, specular);

	float d = fabs(dot(n, (hit.point - cameraCtrl.getPosition()).normalized()));
	// assign gray value (d,d,d)
	Vec3f shade = d;

		//Specular
		//Vec3f reflection = (-(hit.point - cameraCtrl.getPosition()).normalized() - 2 * d * n).normalized();
		Vec3f reflection = (-(hit.point - m_pointLightPos).normalized() - 2 * d * n).normalized();
		float sd = max(0.0f, (dot((hit.point - cameraCtrl.getPosition()).normalized(), reflection)));

		float s = pow(sd, mat->glossiness);
		color = Vec4f(shade*diffuse + 0.5f * (s*specular), 1.0f);
		//Shadow
		Vec3f shadowRay = m_pointLightPos - hit.point;
		RaycastResult shadowHit = rt->raycast(hit.point - 0.001f*(hit.point - cameraCtrl.getPosition()), shadowRay);
		if (shadowHit.tri != nullptr) {
			color = Vec4f(0.0f, 0.0f, 0.0f, 1.0f);
		}

		
	if ((num_bounces)  < m_whittedBounces && specular != Vec3f(0.0f, 0.0f, 0.0f)) {
		Vec3f reflectionDir = (-(hit.point - cameraCtrl.getPosition()).normalized() - 2 * d * n).normalized();
		RaycastResult whittedHit = rt->raycast((hit.point - 0.001f*(hit.point - cameraCtrl.getPosition())), -10000.0f*reflectionDir);
		if (whittedHit.tri != nullptr) {
			color += Vec4f(specular, 0.0f) * (mat->glossiness/100) * computeShadingWhitted(rt, whittedHit, cameraCtrl, rnd, ++num_bounces);

		}
	}

	return color;
}

void Renderer::getSamplePositions(std::vector<Vec2f> & positions, Random& rnd) {
	float subDim = 1.0f / m_aaNumRays;
	for (int i = 0; i < m_aaNumRays; ++i) {
		float offset = rnd.getF32(i*subDim, (i + 1)*subDim);
		positions[i] = Vec2f(offset, offset);
	}
	std::random_shuffle(positions.begin(), positions.end());
}

/*Vec2f Renderer::getSamplePosition(int n) {
	// YOUR CODE HERE (R9)
	// Return a sample through the center of the Nth subpixel.
	// The starter code only supports one sample per pixel.
	int m_dim = sqrtf(m_aaNumRays);
	float sub_dim = 1.0f / m_dim;
	float x = (n % m_dim) * sub_dim + 0.5*sub_dim;
	float y = (n / m_dim) * sub_dim + 0.5*sub_dim;
	return Vec2f(x, y);
}*/

float Renderer::getWeight(float x) {
	float c = 0.325;
	float b = 0.35;
	//x = std::abs(2 * x);
	if (x < 1) {
		return ((12-9*b - 6*c) * x*x*x + (-18 + 12*b + 6*c)* x*x + (6 - 2*b)) * (1.0f / 6.0f);
	}
	else if (x < 2) {
		return ((-b - 6*c) * x*x*x + (6*b + 30*c) * x*x + (-12*b - 48*c) * x + (8*b + 24*c)) * (1.0f / 6.0f);
	}
	else {
		return 0.0f;
	}
}

} // namespace FW

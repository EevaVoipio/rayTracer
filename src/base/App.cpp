#define _CRT_SECURE_NO_WARNINGS

#include "App.hpp"
#include "base/Main.hpp"
#include "gpu/GLContext.hpp"
#include "3d/Mesh.hpp"
#include "io/File.hpp"
#include "io/StateDump.hpp"
#include "base/Random.hpp"

#include "RayTracer.hpp"
#include "rtlib.hpp"

#include <stdio.h>
#include <conio.h>
#include <map>
#include <algorithm>
#include <chrono>

#include <iostream>
#include <fstream>
#include <string>

using namespace FW;


//------------------------------------------------------------------------

bool fileExists(std::string fileName)
{
	return std::ifstream(fileName).good();
}

App::App(std::vector<std::string>& cmd_args)
	: m_commonCtrl(CommonControls::Feature_Default & ~CommonControls::Feature_RepaintOnF5),
	m_cameraCtrl(&m_commonCtrl, CameraControls::Feature_Default | CameraControls::Feature_StereoControls),
	m_action(Action_None),
	m_cullMode(CullMode_None),
	m_shadingMode	(Renderer::ShadingMode_Headlight),
	m_showRTImage	(false),
	m_glRTTexture	(0),
	m_numAORays		(16),
	m_numAARays		(1),
	m_aoRayLength	(1.0f),
	m_whittedBounces(3),
	m_toneMap		(false)
{

	m_commonCtrl.showFPS(true);
	m_commonCtrl.addStateObject(this);
	m_cameraCtrl.setKeepAligned(true);

	m_commonCtrl.addButton((S32*)&m_action, Action_LoadMesh, FW_KEY_M, "Load mesh or state... (M)");
	m_commonCtrl.addButton((S32*)&m_action, Action_ReloadMesh, FW_KEY_F5, "Reload mesh (F5)");
	m_commonCtrl.addButton((S32*)&m_action, Action_SaveMesh, FW_KEY_O, "Save mesh... (O)");
	m_commonCtrl.addSeparator();

	m_commonCtrl.addButton((S32*)&m_action, Action_ResetCamera, FW_KEY_NONE, "Reset camera");
	m_commonCtrl.addButton((S32*)&m_action, Action_EncodeCameraSignature, FW_KEY_NONE, "Encode camera signature");
	m_commonCtrl.addButton((S32*)&m_action, Action_DecodeCameraSignature, FW_KEY_NONE, "Decode camera signature...");
	m_window.addListener(&m_cameraCtrl);
	m_commonCtrl.addSeparator();

	m_commonCtrl.addButton((S32*)&m_action, Action_NormalizeScale, FW_KEY_NONE, "Normalize scale");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_FlipXY,                  FW_KEY_NONE,    "Flip X/Y");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_FlipYZ,                  FW_KEY_NONE,    "Flip Y/Z");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_FlipZ,                   FW_KEY_NONE,    "Flip Z");
	m_commonCtrl.addSeparator();

	m_commonCtrl.addButton((S32*)&m_action, Action_NormalizeNormals, FW_KEY_NONE, "Normalize normals");
	m_commonCtrl.addButton((S32*)&m_action, Action_FlipNormals, FW_KEY_NONE, "Flip normals");
	m_commonCtrl.addButton((S32*)&m_action, Action_RecomputeNormals, FW_KEY_NONE, "Recompute normals");
	m_commonCtrl.addButton((S32*)&m_action, Action_PlacePointLight, FW_KEY_HOME, "Place point light at camera (Home)");
	m_commonCtrl.addSeparator();

	m_commonCtrl.addToggle((S32*)&m_cullMode, CullMode_None, FW_KEY_NONE, "Disable backface culling");
	m_commonCtrl.addToggle((S32*)&m_cullMode, CullMode_CW, FW_KEY_NONE, "Cull clockwise faces");
	m_commonCtrl.addToggle((S32*)&m_cullMode, CullMode_CCW, FW_KEY_NONE, "Cull counter-clockwise faces");
	m_commonCtrl.addButton((S32*)&m_action, Action_FlipTriangles, FW_KEY_NONE, "Flip triangles");
	m_commonCtrl.addSeparator();

	//    m_commonCtrl.addButton((S32*)&m_action, Action_CleanMesh,               FW_KEY_NONE,    "Remove unused materials, denegerate triangles, and unreferenced vertices");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_CollapseVertices,        FW_KEY_NONE,    "Collapse duplicate vertices");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_DupVertsPerSubmesh,      FW_KEY_NONE,    "Duplicate vertices shared between multiple materials");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_FixMaterialColors,       FW_KEY_NONE,    "Override material colors with average over texels");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_DownscaleTextures,       FW_KEY_NONE,    "Downscale textures by 2x");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_ChopBehindNear,          FW_KEY_NONE,    "Chop triangles behind near plane");
	//    m_commonCtrl.addSeparator();


	m_settings.use_textures = true;
	m_settings.use_arealights = false;
	m_settings.enable_reflections = true;
	m_normalMapped = false;
	m_filterTextures = false;
	m_commonCtrl.addToggle(&(m_settings.use_textures), FW_KEY_NONE, "Use textures");
	m_commonCtrl.addToggle(&m_normalMapped, FW_KEY_NONE, "Use normal mapping");
	m_commonCtrl.addToggle(&m_filterTextures, FW_KEY_NONE, "Use trilinear texture filtering");
	m_commonCtrl.addToggle(&(m_settings.use_arealights), FW_KEY_NONE, "Use area lights");
	m_commonCtrl.addToggle(&(m_settings.enable_reflections), FW_KEY_NONE, "Enable reflections");
	m_commonCtrl.addToggle(&(m_toneMap), FW_KEY_NONE, "Enable tone mapping");
	m_commonCtrl.addSeparator();

	m_commonCtrl.addButton((S32*)&m_action, Action_TracePrimaryRays,        FW_KEY_ENTER,   "Trace Rays (ENTER)");
	m_commonCtrl.addToggle(&m_showRTImage,									FW_KEY_SPACE,	"Show Ray Tracer result (SPACE)" );

	m_commonCtrl.addToggle((S32*)&m_shadingMode, Renderer::ShadingMode_Headlight,			FW_KEY_F1,    "Headlight shading (F1)");
	m_commonCtrl.addToggle((S32*)&m_shadingMode, Renderer::ShadingMode_AmbientOcclusion, FW_KEY_F2, "Ambient occlusion shading (F2)");
	m_commonCtrl.addToggle((S32*)&m_shadingMode, Renderer::ShadingMode_Whitted, FW_KEY_F3, "Extra: shading using whitted integrator (F3)");
	m_commonCtrl.addToggle((S32*)&m_shadingMode, Renderer::ShadingMode_AreaLights, FW_KEY_F4, "Extra: shading using only area lights (F4)");

	m_commonCtrl.addSeparator();

	m_commonCtrl.beginSliderStack();
	m_commonCtrl.addSlider(&m_numAARays, 1, 128, true, FW_KEY_NONE, FW_KEY_NONE, "AA rays= %d");
	m_commonCtrl.addSlider(&m_numAORays, 1, 256, true, FW_KEY_NONE, FW_KEY_NONE, "AO rays= %d");
	m_commonCtrl.addSlider(&m_aoRayLength, 0.1f, 10.0f, false, FW_KEY_NONE, FW_KEY_NONE, "AO ray length= %.2f");
	m_commonCtrl.endSliderStack();

	m_window.addListener(this);
	m_window.addListener(&m_commonCtrl);

	m_window.setTitle("Assignment 1");
	m_commonCtrl.setStateFilePrefix("state_assignment1_");

	// allocate image for ray tracer results
	m_rtImage.reset(new Image( m_window.getSize(), ImageFormat::RGBA_Vec4f));
	m_renderer.reset(new Renderer);

	process_args(cmd_args);



	if (!m_settings.batch_render) {
		m_commonCtrl.loadState(m_commonCtrl.getStateFileName(1));
		m_settings.splitMode = SplitMode_Sah;
	}
	else {

		m_commonCtrl.loadState(cmd_args[1].c_str());

		std::cout << cmd_args[1].c_str() << std::endl;

		std::string path = cmd_args[1];

		size_t begin = path.find_last_of("/\\") + 1,
			end = path.find_last_of(".");

		m_results.state_name = path.substr(begin, end - begin);

		if (m_settings.sample_type == AO_sampling) {
			m_renderer->setAORayLength(m_settings.ao_length);
			m_renderer->setAONumRays(m_settings.spp);
			m_renderer->setAANumRays(1);
			m_shadingMode = Renderer::ShadingMode::ShadingMode_AmbientOcclusion;
		}
		else {
			m_renderer->setAANumRays(m_settings.spp);
			m_shadingMode = Renderer::ShadingMode::ShadingMode_Headlight;
		}
		m_renderer->setUseAreaLights(m_settings.use_arealights);
		m_renderer->setUseTextures(m_settings.use_textures);
		m_renderer->setUseReflections(m_settings.enable_reflections);
		m_renderer->setWhittedBounces(m_whittedBounces);
		m_renderer->setPointLightPos(m_pointLightPos);
		m_renderer->setNormalMapping(m_normalMapped);
		m_renderer->setTextureFiltering(m_filterTextures);

		timingResult res = m_renderer->rayTracePicture(m_rt.get(), m_rtImage.get(), m_cameraCtrl, (Renderer::ShadingMode)m_shadingMode);
		m_RTTextureNeedsUpload = true;

		m_results.trace_time = res.duration;
		m_results.rayCount = res.rayCount;
		if (m_settings.output_images) {
			FW::exportImage(std::string("images/"+m_results.state_name + ".png").c_str(), m_rtImage.get());
		}

		// :: alone refers to the local anonymous namespace (present if no other specified)
		bool created = !(::fileExists(cmd_args[2]));

		std::ofstream result(cmd_args[2], std::ios_base::out|std::ios_base::app);

		if (created)
			result << "set_name scene_name state_name build_time(ms) trace_time(ms) ray_count" << std::endl;

		result << cmd_args[3] << " " << m_results.scene_name << " " << m_results.state_name << " " << m_results.build_time << " " << m_results.trace_time << " " << m_results.rayCount << std::endl;

		exit(0);
	}
}

// returns the index of the needle in the haystack or -1 if not found
int find_argument(std::string needle, std::vector<std::string> haystack) {

	for (unsigned j = 0; j < haystack.size(); ++j)
		if (!haystack[j].compare(needle))
			return j;
	
	return -1;
}

void App::process_args(std::vector<std::string>& args) {

	// all of the possible cmd arguments and the corresponding enums (enum value is the index of the string in the vector)
	const std::vector<std::string> argument_names = { "-builder", "-spp", "-output_images", "-use_textures", "-bat_render", "-aa", "-ao", "-ao_length" };
	enum argument { arg_not_found = -1, builder = 0, spp = 1, output_images = 2, use_textures = 3, bat_render = 4, AA = 5, AO = 6, AO_length = 7 };

	// similarly a list of the implemented BVH builder types
	const std::vector<std::string> builder_names = { "none", "sah", "object_median", "spatial_median", "linear" };
	enum builder_type { builder_not_found = -1, builder_None = 0, builder_SAH = 1, builder_ObjectMedian = 2, builder_SpatialMedian = 3, builder_Linear = 4 };

	m_settings.batch_render = false;
	m_settings.output_images = false;
	m_settings.use_textures = true;
	m_settings.sample_type = AA_sampling;
	m_settings.ao_length = 1.0f;
	m_settings.spp = 1;
	m_settings.splitMode = SplitMode_Sah;

	for (unsigned i = 0; i < args.size(); ++i) {

		// try to recognize the argument
		argument cmd = argument(find_argument(args[i], argument_names));

		switch (cmd) {

		case bat_render:
			m_settings.batch_render = true;
			break;

		case output_images:
			m_settings.output_images = true;
			break;
		
		case use_textures:
			m_settings.use_textures = true;
			break;
		
		case AO:
			m_settings.sample_type = AO_sampling;
			break;

		case AA:
			m_settings.sample_type = AA_sampling;
			break;

		case spp:
			++i;
			m_settings.spp = std::stoi(args[i]);
			break;

		case AO_length:
			++i;
			m_settings.ao_length = std::stof(args[i]);
			break;

		case builder: {

			++i;
			builder_type type = builder_type(find_argument(args[i], builder_names));

			if (type==builder_not_found) {
				type = builder_SAH;
				std::cout << "BVH builder not recognized, using Surface Area Heuristic" << std::endl;
				break;
			}

			switch (type) {

			case builder_None:
				m_settings.splitMode = SplitMode_None;
				break;
			
			case builder_SAH:
				m_settings.splitMode = SplitMode_Sah;
				break;

			case builder_ObjectMedian:
				m_settings.splitMode = SplitMode_ObjectMedian;
				break;

			case builder_SpatialMedian:
				m_settings.splitMode = SplitMode_SpatialMedian;
				break;

			case builder_Linear:
				m_settings.splitMode = SplitMode_Linear;
				break;
			}

			break;
		}


		default:
			if (args[i][0] == '-')std::cout << "argument \"" << args[i] << "\" not found!" << std::endl;

		}
	}
	if (m_settings.batch_render)
		m_settings.use_textures = false;
}

//------------------------------------------------------------------------

App::~App()
{
}

//------------------------------------------------------------------------

bool App::handleEvent(const Window::Event& ev)
{
	if (ev.type == Window::EventType_Close)
	{
		m_window.showModalMessage("Exiting...");
		delete this;
		return true;
	}


	Action action = m_action;
	m_action = Action_None;
	String name;
	Mat4f mat;

	switch (action)
	{
	case Action_None:
		break;

	case Action_PlacePointLight:
		m_pointLightPos = m_cameraCtrl.getPosition();
		break;

	case Action_LoadMesh:
		name = m_window.showFileLoadDialog("Load mesh or state", getMeshImportFilter()+",dat:State file");
		if (name.getLength())
			if (name.endsWith(".dat"))
				m_commonCtrl.loadState(name);
			else
				loadMesh(name);
		break;

	case Action_ReloadMesh:
		if (m_meshFileName.getLength())
			loadMesh(m_meshFileName);
		break;

	case Action_SaveMesh:
		name = m_window.showFileSaveDialog("Save mesh", getMeshExportFilter());
		if (name.getLength())
			saveMesh(name);
		break;

	case Action_ResetCamera:
		if (m_mesh)
		{
			m_cameraCtrl.initForMesh(m_mesh.get());
			m_commonCtrl.message("Camera reset");
		}
		break;

	case Action_EncodeCameraSignature:
		m_window.setVisible(false);
		printf("\nCamera signature:\n");
		printf("%s\n", m_cameraCtrl.encodeSignature().getPtr());
		waitKey();
		break;

	case Action_DecodeCameraSignature:
	{
		m_window.setVisible(false);
		printf("\nEnter camera signature:\n");

		char buf[1024];
		if (scanf_s("%s", buf, FW_ARRAY_SIZE(buf)))
			m_cameraCtrl.decodeSignature(buf);
		else
			setError("Signature too long!");

		if (!hasError())
			printf("Done.\n\n");
		else
		{
			printf("Error: %s\n", getError().getPtr());
			clearError();
			waitKey();
		}
	}
		break;

	case Action_NormalizeScale:
		if (m_mesh)
		{
			Vec3f lo, hi;
			m_mesh->getBBox(lo, hi);
			m_mesh->xform(Mat4f::scale(Vec3f(2.0f / (hi - lo).max())) * Mat4f::translate((lo + hi) * -0.5f));
		}
		break;

	case Action_FlipXY:
		std::swap(mat.col(0), mat.col(1));
		if (m_mesh)
		{
			m_mesh->xform(mat);
			m_mesh->flipTriangles();
		}
		break;

	case Action_FlipYZ:
		std::swap(mat.col(1), mat.col(2));
		if (m_mesh)
		{
			m_mesh->xform(mat);
			m_mesh->flipTriangles();
		}
		break;

	case Action_FlipZ:
		mat.col(2) = -mat.col(2);
		if (m_mesh)
		{
			m_mesh->xform(mat);
			m_mesh->flipTriangles();
		}
		break;

	case Action_NormalizeNormals:
		if (m_mesh)
			m_mesh->xformNormals(mat.getXYZ(), true);
		break;

	case Action_FlipNormals:
		mat = -mat;
		if (m_mesh)
			m_mesh->xformNormals(mat.getXYZ(), false);
		break;

	case Action_RecomputeNormals:
		if (m_mesh)
			m_mesh->recomputeNormals();
		break;

	case Action_FlipTriangles:
		if (m_mesh)
			m_mesh->flipTriangles();
		break;

	case Action_CleanMesh:
		if (m_mesh)
			m_mesh->clean();
		break;

	case Action_CollapseVertices:
		if (m_mesh)
			m_mesh->collapseVertices();
		break;

	case Action_DupVertsPerSubmesh:
		if (m_mesh)
			m_mesh->dupVertsPerSubmesh();
		break;

	case Action_FixMaterialColors:
		if (m_mesh)
			m_mesh->fixMaterialColors();
		break;

	case Action_DownscaleTextures:
		if (m_mesh)
			downscaleTextures(m_mesh.get());
		break;

	case Action_ChopBehindNear:
		if (m_mesh)
		{
			Mat4f worldToClip = m_cameraCtrl.getCameraToClip() * m_cameraCtrl.getWorldToCamera();
			Vec4f pleq = worldToClip.getRow(2) + worldToClip.getRow(3);
			chopBehindPlane(m_mesh.get(), pleq);
		}
		break;
	case Action_TracePrimaryRays:
	{
		if (m_rt) {
			m_renderer->setAONumRays(m_numAORays);
			m_renderer->setAANumRays(m_numAARays);
			m_renderer->setAORayLength(m_aoRayLength);
			m_renderer->setWhittedBounces(m_whittedBounces);
			m_renderer->setUseReflections(m_settings.enable_reflections);
			m_renderer->setUseAreaLights(m_settings.use_arealights);
			m_renderer->setUseTextures(m_settings.use_textures);
			m_renderer->setPointLightPos(m_pointLightPos);
			m_renderer->setNormalMapping(m_normalMapped);
			m_renderer->setTextureFiltering(m_filterTextures);
			
			m_renderer->rayTracePicture(m_rt.get(), m_rtImage.get(), m_cameraCtrl, (Renderer::ShadingMode)m_shadingMode);
			m_RTTextureNeedsUpload = true;

			m_showRTImage = true;
		}
	}
		break;
	default:
		FW_ASSERT(false);
		break;
		}



	m_window.setVisible(true);

	if (ev.type == Window::EventType_Paint)
		renderFrame(m_window.getGL());
	m_window.repaint();
	return false;
		}

//------------------------------------------------------------------------

void App::readState(StateDump& d)
{
	String meshFileName;

	d.pushOwner("App");
	d.get(meshFileName,     "m_meshFileName");
	d.get((S32&)m_cullMode, "m_cullMode");
	d.get((S32&)m_numAORays, "m_numAORays");
	d.get((F32&)m_aoRayLength, "m_aoRayLength");
	d.get((Vec3f&)m_pointLightPos, "m_pointLightPos");
	d.popOwner();

	if (m_meshFileName != meshFileName && meshFileName.getLength()) {
		loadMesh(meshFileName);
	}
	}

//------------------------------------------------------------------------

void App::writeState(StateDump& d) const
{
	d.pushOwner("App");
	d.set(m_meshFileName,       "m_meshFileName");
	d.set((S32)m_cullMode,      "m_cullMode");
	d.set((S32&)m_numAORays,    "m_numAORays");
	d.set((F32&)m_aoRayLength,  "m_aoRayLength");
	d.set((Vec3f&)m_pointLightPos, "m_pointLightPos");
	d.popOwner();
}

//------------------------------------------------------------------------

void App::waitKey(void)
{
	printf("Press any key to continue . . . ");
	_getch();
	printf("\n\n");
}

//------------------------------------------------------------------------

void App::renderFrame(GLContext* gl)
{
	// make sure the RT image is correct size. If not, reallocate
	if ( m_rtImage->getSize() != m_window.getSize() )
	{
		m_rtImage.reset(new Image( m_window.getSize(), ImageFormat::RGBA_Vec4f ));
	}

	// if desired, show the ray traced result image
	if ( m_showRTImage )
	{
		glClearColor(0.2f, 0.4f, 0.8f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_DEPTH_TEST);

		// blit the image
		if (m_toneMap)
			blitRttToScreen(gl);
		else
			gl->drawImage( *m_rtImage, Vec2f(0,0) );

		m_commonCtrl.message(sprintf("%.2f Mrays/sec",m_renderer->getRaysPerSecond()/1000000.0f),"rayStats");
	}
	else
	{
		// nope, let's draw using OpenGL
		// Setup transformations.

		Mat4f worldToCamera = m_cameraCtrl.getWorldToCamera();
		Mat4f projection = gl->xformFitToView(Vec2f(-1.0f, -1.0f), Vec2f(2.0f, 2.0f)) * m_cameraCtrl.getCameraToClip();

		// Initialize GL state.

		glClearColor(0.2f, 0.4f, 0.8f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_DEPTH_TEST);

		if (m_cullMode == CullMode_None)
			glDisable(GL_CULL_FACE);
		else
		{
			glEnable(GL_CULL_FACE);
			glCullFace(GL_BACK);
			glFrontFace((m_cullMode == CullMode_CW) ? GL_CCW : GL_CW);
		}

		// No mesh => skip.

		if (!m_mesh)
		{
			gl->drawModalMessage("No mesh loaded!");
			return;
		}

		// Render.

		if (!gl->getConfig().isStereo)
			renderScene(gl, worldToCamera, projection);
		else
		{
			glDrawBuffer(GL_BACK_LEFT);
			renderScene(gl, m_cameraCtrl.getCameraToLeftEye() * worldToCamera, projection);
			glDrawBuffer(GL_BACK_RIGHT);
			glClear(GL_DEPTH_BUFFER_BIT);
			renderScene(gl, m_cameraCtrl.getCameraToRightEye() * worldToCamera, projection);
			glDrawBuffer(GL_BACK);
		}

		// Display status line.

		m_commonCtrl.message(sprintf("Triangles = %d, vertices = %d, materials = %d",
			m_mesh->numTriangles(),
			m_mesh->numVertices(),
			m_mesh->numSubmeshes()),
			"meshStats");
	}	// end GL draw code
}

//------------------------------------------------------------------------

void App::renderScene(GLContext* gl, const Mat4f& worldToCamera, const Mat4f& projection)
{
	// Draw mesh.
	if (m_mesh)
		m_mesh->draw(gl, worldToCamera, projection);
}


//------------------------------------------------------------------------



const static F32 posAttrib[] =
{
	-1, -1, 0, 1,
	1, -1, 0, 1,
	-1, 1, 0, 1,
	1, 1, 0, 1
};



const static F32 texAttrib[] =
{
	0, 0,
	1, 0,
	0, 1,
	1, 1
};


void App::blitRttToScreen(GLContext* gl)
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_DEPTH_TEST);


	// Create program.
	const char* progId = "App::blitToScreen";
	GLContext::Program* prog = gl->getProgram(progId);
	if (!prog)
	{
		prog = new GLContext::Program(
			FW_GL_SHADER_SOURCE(
			attribute vec4 posAttrib;
		attribute vec2 texAttrib;
		varying vec2 uv;
		void main()
		{
			gl_Position = posAttrib;
			uv = texAttrib;
		}
		),
			FW_GL_SHADER_SOURCE(
			uniform sampler2D texSampler;
		varying vec2 uv;
		uniform float reinhardLWhite;
		uniform float tonemapBoost;


		void main()
		{
			vec3 color = texture2D(texSampler, vec2(uv.x, 1.0 - uv.y)).rgb;
			color *= tonemapBoost;
			float L = 0.2126 * color.x + 0.7152 * color.y + 0.0722 * color.z;
			color *= (1.0 + L / reinhardLWhite / reinhardLWhite) / (1.0 + L);

			gl_FragColor = vec4(color, 1.0);
		}
		));
		gl->setProgram(progId, prog);
	}

	prog->use();

	gl->setAttrib(prog->getAttribLoc("posAttrib"), 4, GL_FLOAT, 0, posAttrib);
	gl->setAttrib(prog->getAttribLoc("texAttrib"), 2, GL_FLOAT, 0, texAttrib);
	gl->setUniform(prog->getUniformLoc("reinhardLWhite"), 3.0f);
	gl->setUniform(prog->getUniformLoc("tonemapBoost"), 1.0f);
	if (m_RTTextureNeedsUpload) {
		glDeleteTextures(1, &m_glRTTexture);
		m_glRTTexture = m_rtImage.get()->createGLTexture();
	}
	m_RTTextureNeedsUpload = false;

	GLint oldTex = 0;
	glGetIntegerv(GL_TEXTURE_BINDING_2D, &oldTex); //get currently bound texture so texture bind state can remain unchanged

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, m_glRTTexture);
	gl->setUniform(prog->getUniformLoc("texSampler"), 0);

	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glBindTexture(GL_TEXTURE_2D, oldTex);
	GLContext::checkErrors();
}



//------------------------------------------------------------------------


void App::loadMesh(const String& fileName)
{

	// find the scene name; the file path without preceding folders and file extension

	std::string path = std::string(fileName.getPtr());

	size_t begin = path.find_last_of("/\\")+1,
		     end = path.find_last_of(".");

	m_results.scene_name = path.substr(begin, end - begin);

	std::cout << "Scene name: " << m_results.scene_name << std::endl;

	m_window.showModalMessage(sprintf("Loading mesh from '%s'...", fileName.getPtr()));
	String oldError = clearError();
	std::unique_ptr<MeshBase> mesh((MeshBase*)importMesh(fileName));

	String newError = getError();

	if (restoreError(oldError))
	{
		m_commonCtrl.message(sprintf("Error while loading '%s': %s", fileName.getPtr(), newError.getPtr()));
		return;
	}

	m_meshFileName = fileName;

	m_mesh.reset(new Mesh<VertexPNTC>(*mesh));

	// fix input colors to white so we see something
	for ( S32 i = 0; i < m_mesh->numVertices(); ++i )
		m_mesh->mutableVertex(i).c = Vec3f(1, 1, 1);

	m_commonCtrl.message(sprintf("Loaded mesh from '%s'", fileName.getPtr()));

	// build the BVH!
	constructTracer();
}

//------------------------------------------------------------------------

void App::saveMesh(const String& fileName)
{
	if (!m_mesh)
	{
		m_commonCtrl.message("No mesh to save!");
		return;
	}

	m_window.showModalMessage(sprintf("Saving mesh to '%s'...", fileName.getPtr()));
	String oldError = clearError();
	exportMesh(fileName, m_mesh.get());
	String newError = getError();

	if (restoreError(oldError))
	{
		m_commonCtrl.message(sprintf("Error while saving '%s': %s", fileName.getPtr(), newError.getPtr()));
		return;
	}

	m_meshFileName = fileName;
	m_commonCtrl.message(sprintf("Saved mesh to '%s'", fileName.getPtr()));
}

//------------------------------------------------------------------------

// This function iterates over all the "sub-meshes" (parts of the object with different materials),
// heaps all the vertices and triangles together, and calls the BVH constructor.
// It is the responsibility of the tree to free the data when deleted.
// This functionality is _not_ part of the RayTracer class in order to keep it separate
// from the specifics of the Mesh class.
void App::constructTracer()
{
	// fetch vertex and triangle data ----->
	m_rtTriangles.clear();
	m_rtTriangles.reserve(m_mesh->numTriangles());


	for (int i = 0; i < m_mesh->numSubmeshes(); ++i)
	{
		const Array<Vec3i>& idx = m_mesh->indices(i);
		for (int j = 0; j < idx.getSize(); ++j)
		{

			const VertexPNTC &v0 = m_mesh->vertex(idx[j][0]),
						     &v1 = m_mesh->vertex(idx[j][1]),
							 &v2 = m_mesh->vertex(idx[j][2]);

			RTTriangle t = RTTriangle(v0, v1, v2);

			t.m_data.vertex_indices = idx[j];
			t.m_material = &(m_mesh->material(i));

			m_rtTriangles.push_back(t);
		}
	}


	// compute checksum

	m_rtVertexPositions.clear();
	m_rtVertexPositions.reserve(m_mesh->numVertices());
	for (int i = 0; i < m_mesh->numVertices(); ++i)
		m_rtVertexPositions.push_back(m_mesh->vertex(i).p);

	String md5 = RayTracer::computeMD5(m_rtVertexPositions);
	FW::printf("Mesh MD5: %s\n", md5.getPtr());

	// construct a new ray tracer (deletes the old one if there was one)
	m_rt.reset(new RayTracer());

	// YOUR CODE HERE (R2)
	// When you have implemented your BVH saving & loading, enable it here.
	bool tryLoadHierarchy = true;//false;

	// always construct when measuring performance
	if (m_settings.batch_render)
		tryLoadHierarchy = false;

	if (tryLoadHierarchy)
	{
		// check if saved hierarchy exists

#ifdef _WIN64
		String hierarchyCacheFile = String("Hierarchy-") + md5 + String("-x64.bin");
#else
		String hierarchyCacheFile = String("Hierarchy-") + md5 + String("-x86.bin");
#endif

		if (fileExists(hierarchyCacheFile.getPtr()))
		{
			// yes, load!
			m_rt->loadHierarchy(hierarchyCacheFile.getPtr(), m_rtTriangles);
			::printf("Loaded hierarchy from %s\n", hierarchyCacheFile.getPtr());
		}
		else
		{
			// no, construct...
			LARGE_INTEGER start, stop, frequency;
			QueryPerformanceFrequency(&frequency);
			QueryPerformanceCounter(&start); // Start time stamp		

			m_rt->constructHierarchy(m_rtTriangles, m_settings.splitMode);

			QueryPerformanceCounter(&stop); // Stop time stamp

			int build_time = (int)((stop.QuadPart - start.QuadPart) * 1000.0 / frequency.QuadPart); // Get timer result in milliseconds
			std::cout << "Build time: " << build_time << " ms" << std::endl;
			// .. and save!
			m_rt->saveHierarchy(hierarchyCacheFile.getPtr(), m_rtTriangles);
			::printf("Saved hierarchy to %s\n", hierarchyCacheFile.getPtr());
		}
	}
	else
	{
		// nope, bite the bullet and construct it

		LARGE_INTEGER start, stop, frequency;
		QueryPerformanceFrequency(&frequency);
		QueryPerformanceCounter(&start); // Start time stamp		
		
		m_rt->constructHierarchy(m_rtTriangles, m_settings.splitMode);

		QueryPerformanceCounter(&stop); // Stop time stamp

		m_results.build_time = (int)((stop.QuadPart - start.QuadPart) * 1000.0 / frequency.QuadPart); // Get timer result in milliseconds
		std::cout << "Build time: " << m_results.build_time << " ms"<< std::endl;
	}

	// m_rt is complete and mesh data will be constant from now on, so we can gather the emissive triangles
	m_renderer->gatherLightTriangles(m_rt.get());
}



//------------------------------------------------------------------------

void App::downscaleTextures(MeshBase* mesh)
{
	FW_ASSERT(mesh);
	Hash<const Image*, Texture> hash;
	for (int submeshIdx = 0; submeshIdx < mesh->numSubmeshes(); submeshIdx++)
		for (int textureIdx = 0; textureIdx < MeshBase::TextureType_Max; textureIdx++)
		{
		Texture& tex = mesh->material(submeshIdx).textures[textureIdx];
		if (tex.exists())
		{
			const Image* orig = tex.getImage();
			if (!hash.contains(orig))
			{
				Image* scaled = orig->downscale2x();
				hash.add(orig, (scaled) ? Texture(scaled, tex.getID()) : tex);
			}
			tex = hash.get(orig);
		}
		}
}

//------------------------------------------------------------------------

void App::chopBehindPlane(MeshBase* mesh, const Vec4f& pleq)
{
	FW_ASSERT(mesh);
	int posAttrib = mesh->findAttrib(MeshBase::AttribType_Position);
	if (posAttrib == -1)
		return;

	for (int submeshIdx = 0; submeshIdx < mesh->numSubmeshes(); submeshIdx++)
	{
		Array<Vec3i>& inds = mesh->mutableIndices(submeshIdx);
		int triOut = 0;
		for (int triIn = 0; triIn < inds.getSize(); triIn++)
		{
			if (dot(mesh->getVertexAttrib(inds[triIn].x, posAttrib), pleq) >= 0.0f ||
				dot(mesh->getVertexAttrib(inds[triIn].y, posAttrib), pleq) >= 0.0f ||
				dot(mesh->getVertexAttrib(inds[triIn].z, posAttrib), pleq) >= 0.0f)
			{
				inds[triOut++] = inds[triIn];
			}
		}
		inds.resize(triOut);
	}

	mesh->clean();
}


//------------------------------------------------------------------------

void FW::init(std::vector<std::string>& args)
{
	new App(args);
}

//------------------------------------------------------------------------

bool App::fileExists(const String& fn)
{
	FILE* pF = fopen(fn.getPtr(), "rb");
	if (pF != 0)
	{
		fclose(pF);
		return true;
	}
	else
	{
		return false;
	}
}

//------------------------------------------------------------------------

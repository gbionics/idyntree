// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/JointState.h>
#include <iDynTree/Utils.h>
#include <iDynTree/Visualizer.h>

#ifdef IDYNTREE_USES_IRRLICHT
#include "Camera.h"
#include "CameraAnimator.h"
#include "Environment.h"
#include "FrameVisualization.h"
#include "IrrlichtUtils.h"
#include "Label.h"
#include "ModelVisualization.h"
#include "ShapesVisualization.h"
#include "TexturesHandler.h"
#include "VectorsVisualization.h"
#include <irrlicht.h>

#if defined(_WIN32)
#define GLFW_EXPOSE_NATIVE_WIN32
#define GLFW_EXPOSE_NATIVE_WGL
#elif defined(__APPLE__)
#define GLFW_EXPOSE_NATIVE_COCOA
#define GLFW_EXPOSE_NATIVE_NSGL
#elif defined(__linux__)
#define GLFW_EXPOSE_NATIVE_X11
#if defined(IDYNTREE_GLFW_TRY_WAYLAND_FIRST)
#define GLFW_EXPOSE_NATIVE_WAYLAND
#endif
#define GLFW_EXPOSE_NATIVE_GLX
#endif

#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#if defined(_WIN32) && defined(_IRR_COMPILE_WITH_SDL_DEVICE_)
// Required by SetEnvironmentVariableA, _putenv is not unsetting
// correctly the variables
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#if defined(_IRR_COMPILE_WITH_SDL_DEVICE_) || defined(_WIN32) || defined(__APPLE__)
#if (defined(_WIN32) || defined(__APPLE__)) && !defined(_IRR_COMPILE_WITH_SDL_DEVICE_)
#error "On Windows and MacOS it is necessary to use Irrlicht with SDL"
#endif
#define IDYNTREE_USE_GLFW_WINDOW
#endif

#endif

#include "DummyImplementations.h"

#include <cassert>
#include <memory>
#include <unordered_map>

namespace iDynTree
{

ICamera::~ICamera()
{
}

IEnvironment::~IEnvironment()
{
}

IJetsVisualization::~IJetsVisualization()
{
}

IVectorsVisualization::~IVectorsVisualization()
{
}

IFrameVisualization::~IFrameVisualization()
{
}

IShapeVisualization::~IShapeVisualization()
{
}

IModelVisualization::~IModelVisualization()
{
}

ILabel::~ILabel()
{
}

ILight::~ILight()
{
}

ITexture::~ITexture()
{
}

ITexturesHandler::~ITexturesHandler()
{
}

ColorViz::ColorViz()
    : r(1.0)
    , g(1.0)
    , b(1.0)
    , a(1.0)
{
    // default color is white
}

ColorViz::ColorViz(float _r, float _g, float _b, float _a)
    : r(_r)
    , g(_g)
    , b(_b)
    , a(_a)
{
}

ColorViz::ColorViz(const Vector4& rgba)
    : r(rgba(0))
    , g(rgba(1))
    , b(rgba(2))
    , a(rgba(3))
{
}

Vector4 ColorViz::toVector4() const
{
    Vector4 ret;
    ret(0) = r;
    ret(1) = g;
    ret(2) = b;
    ret(3) = a;
    return ret;
}

struct Visualizer::VisualizerPimpl
{
    /**
     * True if init has been called.
     */
    bool m_isInitialized;

    /**
     * Invalid model visualization, useful to return in case of error.
     */
    DummyModelVisualization m_invalidModelViz;

    /**
     * Last FPS measured.
     */
    int lastFPS;

#ifdef IDYNTREE_USES_IRRLICHT

#ifdef IDYNTREE_USE_GLFW_WINDOW
    /**
     * Custom window object
     */
    GLFWwindow* m_window{nullptr};

    static unsigned int m_glfwInstances;

#if defined(_WIN32)
    HWND m_windowId;
#elif defined(__APPLE__)
    id m_windowId;
#elif defined(__linux__)
    void* m_windowId; // Pointer to either wl_surface* or X11 Window
#endif
#endif

    /**
     * Collection of model visualization.
     */
    std::shared_ptr<std::vector<ModelVisualization*>> m_modelViz;

    /**
     * Irrlicht device used by the visualizer.
     */
    irr::IrrlichtDevice* m_irrDevice;

    /**
     * Irrlicht scene manager.
     */
    irr::scene::ISceneManager* m_irrSmgr;

    /**
     * Irrlicht video driver.
     */
    irr::video::IVideoDriver* m_irrDriver;

    /**
     * Irrlicht video data.
     */
    irr::video::SExposedVideoData m_irrVideoData;

    /**
     * Camera used by the visualization.
     */
    Camera m_camera;

    /**
     * Environment used by the visualization.
     */
    Environment m_environment;

    /**
     * Vectors visualization
     */
    VectorsVisualization m_vectors;

    /**
     * Frames visualization
     */
    FrameVisualization m_frames;

    /**
     * Shapes visualization
     */
    ShapeVisualization m_shapes;

    /**
     * Textures handling
     */
    TexturesHandler m_textures;

    /**
     * Dimension of the root frame arrows
     */
    double rootFrameArrowsDimension;

    /**
     * Set of labels
     */
    std::unordered_map<std::string, Label> m_labels;

    /**
     * The scene was started but not ended yet
     */
    bool m_subDrawStarted{false};

    struct ColorPalette
    {
        irr::video::SColorf background = irr::video::SColorf(0.0, 0.4, 0.4, 1.0);
        irr::video::SColor gridColor = irr::video::SColor(100, 0, 0, 255);

        irr::video::SColor xAxis = irr::video::SColor(20, 255, 0, 0);
        irr::video::SColor yAxis = irr::video::SColor(20, 0, 255, 0);
        irr::video::SColor zAxis = irr::video::SColor(20, 0, 0, 255);

        irr::video::SColor vector = irr::video::SColor(1, 1, 0, 0);
    };
    std::unordered_map<std::string, ColorPalette> m_palette;

    void initializePalette()
    {
        irr::u32 alphaLev = 20;
        // vanilla palette (This is the original palette of the visualizer)
        m_palette["vanilla"].background = irr::video::SColorf(0.0, 0.4, 0.4, 1.0);
        m_palette["vanilla"].gridColor = irr::video::SColor(100, 0, 0, 255);
        m_palette["vanilla"].xAxis = irr::video::SColor(alphaLev, 255, 0, 0);
        m_palette["vanilla"].yAxis = irr::video::SColor(alphaLev, 0, 255, 0);
        m_palette["vanilla"].zAxis = irr::video::SColor(alphaLev, 0, 0, 255);
        m_palette["vanilla"].vector = irr::video::SColor(255, 255, 0, 0);

        m_palette["meshcat"].background = irr::video::SColorf(0.42, 0.63, 0.85, 1.0);
        m_palette["meshcat"].gridColor = irr::video::SColor(128, 128, 128, 100);
        m_palette["meshcat"].xAxis = irr::video::SColor(alphaLev, 234, 67, 53);
        m_palette["meshcat"].yAxis = irr::video::SColor(alphaLev, 52, 168, 83);
        m_palette["meshcat"].zAxis = irr::video::SColor(alphaLev, 66, 133, 244);
        m_palette["meshcat"].vector = irr::video::SColor(255, 253, 98, 2);
    }

#ifdef IDYNTREE_USE_GLFW_WINDOW
    void cursorPositionCallback(GLFWwindow* window, double xpos, double ypos)
    {
        if (window != m_window)
        {
            return;
        }

        if (!m_isInitialized)
        {
            return;
        }

        GLint ww, wh;
        glfwGetWindowSize(window, &ww, &wh);

        irr::SEvent::SMouseInput mouseEvent;

        mouseEvent.Event = irr::EMOUSE_INPUT_EVENT::EMIE_MOUSE_MOVED;
        mouseEvent.X = xpos;
        mouseEvent.Y = ypos;

        irr::SEvent event;
        event.EventType = irr::EEVENT_TYPE::EET_MOUSE_INPUT_EVENT;
        event.MouseInput = mouseEvent;

        m_irrDevice->postEventFromUser(event);
    }

    void mouseButtonCallback(GLFWwindow* window, int button, int action, int)
    {
        if (window != m_window)
        {
            return;
        }

        if (!m_isInitialized)
        {
            return;
        }

        irr::SEvent::SMouseInput mouseEvent;

        if (button == GLFW_MOUSE_BUTTON_RIGHT)
        {
            if (action == GLFW_PRESS)
            {
                mouseEvent.Event = irr::EMOUSE_INPUT_EVENT::EMIE_RMOUSE_PRESSED_DOWN;
            } else if (action == GLFW_RELEASE)
            {
                mouseEvent.Event = irr::EMOUSE_INPUT_EVENT::EMIE_RMOUSE_LEFT_UP;
            } else
            {
                return;
            }
        } else if (button == GLFW_MOUSE_BUTTON_LEFT)
        {
            if (action == GLFW_PRESS)
            {
                mouseEvent.Event = irr::EMOUSE_INPUT_EVENT::EMIE_LMOUSE_PRESSED_DOWN;
            } else if (action == GLFW_RELEASE)
            {
                mouseEvent.Event = irr::EMOUSE_INPUT_EVENT::EMIE_LMOUSE_LEFT_UP;
            } else
            {
                return;
            }
        } else if (button == GLFW_MOUSE_BUTTON_MIDDLE)
        {
            if (action == GLFW_PRESS)
            {
                mouseEvent.Event = irr::EMOUSE_INPUT_EVENT::EMIE_MMOUSE_PRESSED_DOWN;
            } else if (action == GLFW_RELEASE)
            {
                mouseEvent.Event = irr::EMOUSE_INPUT_EVENT::EMIE_MMOUSE_LEFT_UP;
            } else
            {
                return;
            }
        } else
        {
            return;
        }

        irr::SEvent event;
        event.EventType = irr::EEVENT_TYPE::EET_MOUSE_INPUT_EVENT;
        event.MouseInput = mouseEvent;

        m_irrDevice->postEventFromUser(event);
    }

    void scrollCallback(GLFWwindow* window, double, double yoffset)
    {
        if (window != m_window)
        {
            return;
        }

        if (!m_isInitialized)
        {
            return;
        }

        irr::SEvent::SMouseInput mouseEvent;

        mouseEvent.Event = irr::EMOUSE_INPUT_EVENT::EMIE_MOUSE_WHEEL;
        mouseEvent.Wheel = yoffset;

        irr::SEvent event;
        event.EventType = irr::EEVENT_TYPE::EET_MOUSE_INPUT_EVENT;
        event.MouseInput = mouseEvent;

        m_irrDevice->postEventFromUser(event);
    }

    static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
    {
        static_cast<Visualizer::VisualizerPimpl*>(glfwGetWindowUserPointer(window))
            ->cursorPositionCallback(window, xpos, ypos);
    }

    static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
    {
        static_cast<Visualizer::VisualizerPimpl*>(glfwGetWindowUserPointer(window))
            ->mouseButtonCallback(window, button, action, mods);
    }

    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
    {
        static_cast<Visualizer::VisualizerPimpl*>(glfwGetWindowUserPointer(window))
            ->scrollCallback(window, xoffset, yoffset);
    }
#endif

#else
    DummyCamera m_camera;
    DummyEnvironment m_environment;
    DummyVectorsVisualization m_invalidVectors;
    DummyFrameVisualization m_invalidFrames;
    DummyTexturesHandler m_invalidTextures;
    DummyShapeVisualization m_invalidShapes;
    DummyLabel m_invalidLabel;
#endif

    VisualizerPimpl()
    {
        m_isInitialized = false;
        lastFPS = -1;

#ifdef IDYNTREE_USES_IRRLICHT
        m_modelViz = std::make_shared<std::vector<ModelVisualization*>>();
        m_modelViz->resize(0);
        m_irrDevice = 0;
        m_irrSmgr = 0;
        m_irrDriver = 0;
#endif
    }
};

#ifdef IDYNTREE_USE_GLFW_WINDOW

unsigned int Visualizer::VisualizerPimpl::m_glfwInstances = 0;

#endif

Visualizer::Visualizer(const Visualizer& /*other*/)
{
    assert(false);
}

Visualizer& Visualizer::operator=(const Visualizer& /*other*/)
{
    assert(false);
    return *this;
}

Visualizer::Visualizer()
    : pimpl(new VisualizerPimpl())
{
}

Visualizer::~Visualizer()
{
    close();

    delete pimpl;
    pimpl = 0;
}

bool Visualizer::init(const VisualizerOptions& visualizerOptions)
{
#ifdef IDYNTREE_USES_IRRLICHT
    if (pimpl->m_isInitialized)
    {
        reportWarning("Visualizer",
                      "init",
                      "Visualier already initialized, call close() to close it to open it again.");
        return false;
    }

    // initialize the color palette
    pimpl->initializePalette();

    irr::SIrrlichtCreationParameters irrDevParams;

#ifdef IDYNTREE_USE_GLFW_WINDOW
    if (pimpl->m_glfwInstances == 0)
    {
        if (!glfwInit())
        {
            reportError("Visualizer", "init", "Unable to initialize GLFW");
            return false;
        }

        glfwWindowHint(GLFW_SAMPLES, 4); // Antialiasing
    }
    pimpl->m_glfwInstances++;

    pimpl->m_window = glfwCreateWindow(visualizerOptions.winWidth,
                                       visualizerOptions.winHeight,
                                       "iDynTree Visualizer",
                                       nullptr,
                                       nullptr);
    if (!pimpl->m_window)
    {
        reportError("Visualizer", "init", "Could not create window");
        return false;
    }

    glfwMakeContextCurrent(pimpl->m_window);
    glfwSwapInterval(1);

#if defined(_WIN32)
    pimpl->m_windowId = glfwGetWin32Window(pimpl->m_window);
    irrDevParams.WindowId = (void*)(pimpl->m_windowId);
#elif defined(__APPLE__)
    pimpl->m_windowId = glfwGetCocoaWindow(pimpl->m_window);
    irrDevParams.WindowId = (void*)(pimpl->m_windowId);
#elif defined(__linux__)

    void* nativeWindow = nullptr;

#ifdef IDYNTREE_GLFW_TRY_WAYLAND_FIRST
    // Try Wayland first
    struct wl_surface* waylandWindow = glfwGetWaylandWindow(pimpl->m_window);
#else
    void* waylandWindow = nullptr;
#endif

    if (waylandWindow)
    {
        nativeWindow = static_cast<void*>(waylandWindow);
    } else
    {
        // Fallback to X11
        Window x11Window = glfwGetX11Window(pimpl->m_window);
        if (x11Window)
            nativeWindow = static_cast<void*>(reinterpret_cast<void*>(x11Window));
    }
    irrDevParams.WindowId = nativeWindow;
#endif

    irrDevParams.DeviceType = irr::EIDT_SDL;
#endif

    irrDevParams.DriverType = irr::video::EDT_OPENGL;
    irrDevParams.WindowSize
        = irr::core::dimension2d<irr::u32>(visualizerOptions.winWidth, visualizerOptions.winHeight);
    irrDevParams.WithAlphaChannel = true;
    irrDevParams.AntiAlias = 4;

    if (visualizerOptions.verbose)
    {
        reportWarning("Visualizer",
                      "init",
                      "verbose flag found, enabling verbose output in Visualizer");
        irrDevParams.LoggingLevel = irr::ELL_DEBUG;
    }

    pimpl->m_irrDevice = 0;

// The Irrlicht SDL device overwrites the value of the SDL_VIDEODRIVER
// environment variable, and this prevents the driver to work correctly.
// For this reason, we save the value before the irr::createDeviceEx call,
// and we restore it (or we unset it) after call
// Workaround for https://github.com/robotology/idyntree/issues/986
#if defined(IDYNTREE_USE_GLFW_WINDOW)
    std::string SDL_VIDEODRIVER_value;
    bool has_SDL_VIDEODRIVER = false;
    if (irrDevParams.DeviceType == irr::EIDT_SDL)
    {
        if (const char* envVal = std::getenv("SDL_VIDEODRIVER"))
        {
            has_SDL_VIDEODRIVER = true;
            SDL_VIDEODRIVER_value = envVal;
        }
    }
#endif

    pimpl->m_irrDevice = irr::createDeviceEx(irrDevParams);

#if defined(IDYNTREE_USE_GLFW_WINDOW)
    if (irrDevParams.DeviceType == irr::EIDT_SDL)
    {
#if defined(_WIN32)
        SetEnvironmentVariableA("SDL_VIDEODRIVER",
                                has_SDL_VIDEODRIVER ? SDL_VIDEODRIVER_value.c_str() : nullptr);
#else
        if (has_SDL_VIDEODRIVER)
        {
            setenv("SDL_VIDEODRIVER", SDL_VIDEODRIVER_value.c_str(), 1);
        } else
        {
            unsetenv("SDL_VIDEODRIVER");
        }
#endif
    }
#endif

    if (pimpl->m_irrDevice == 0)
    {
        reportError("Visualizer", "init", "Impossible to load irrlicht device");
        return false; // could not create selected driver.
    }

    // Checking the irrDevice
    if (pimpl->m_irrDevice == (void*)1)
    {
        reportError("Visualizer",
                    "init",
                    "Impossible to load irrlicht device, createDevice returned a 1 pointer");
        pimpl->m_irrDevice = 0;
        return false; // could not create selected driver.
    }

    pimpl->m_irrSmgr = pimpl->m_irrDevice->getSceneManager();

    // Get video driver
    pimpl->m_irrDriver = pimpl->m_irrDevice->getVideoDriver();

    pimpl->m_irrVideoData = pimpl->m_irrDriver->getExposedVideoData(); // save the current window
                                                                       // settings. This is helpful
                                                                       // in case other Viualizer
                                                                       // objects are created
                                                                       //  since Irrlicht by default
                                                                       //  draws on the last window
                                                                       //  opened.

    // Always visualize the mouse cursor
    pimpl->m_irrDevice->getCursorControl()->setVisible(true);

    // Add environment
    pimpl->rootFrameArrowsDimension = visualizerOptions.rootFrameArrowsDimension;
    pimpl->m_environment.init(pimpl->m_irrSmgr, pimpl->rootFrameArrowsDimension);

    pimpl->m_camera.setIrrlichtCamera(addVizCamera(pimpl->m_irrSmgr));
    pimpl->m_camera.setCameraAnimator(new CameraAnimator(addFrameAxes(pimpl->m_irrSmgr, 0, 0.1),
                                                         visualizerOptions.winWidth,
                                                         visualizerOptions.winHeight));

    pimpl->m_vectors.init(pimpl->m_irrSmgr);

    pimpl->m_frames.init(pimpl->m_irrSmgr, pimpl->m_modelViz);

    pimpl->m_shapes.init(pimpl->m_irrSmgr, pimpl->m_modelViz);

    pimpl->m_textures.init(pimpl->m_irrDriver, pimpl->m_irrSmgr);

#ifdef IDYNTREE_USE_GLFW_WINDOW
    glfwSetWindowUserPointer(pimpl->m_window, pimpl);
    glfwSetCursorPosCallback(pimpl->m_window, VisualizerPimpl::cursor_position_callback);
    glfwSetMouseButtonCallback(pimpl->m_window, VisualizerPimpl::mouse_button_callback);
    glfwSetScrollCallback(pimpl->m_window, VisualizerPimpl::scroll_callback);
#endif

    pimpl->m_isInitialized = true;
    pimpl->lastFPS = -1;

    return true;
#else
    IDYNTREE_UNUSED(visualizerOptions);
    reportError("Visualizer",
                "init",
                "Impossible to use iDynTree::Visualizer, as iDynTree has been compiled without "
                "Irrlicht.");
    return false;
#endif
}

size_t Visualizer::getNrOfVisualizedModels()
{
#ifdef IDYNTREE_USES_IRRLICHT
    return pimpl->m_modelViz->size();
#else
    return 0;
#endif
}

std::string Visualizer::getModelInstanceName(size_t modelInstanceIndex)
{
    if (modelInstanceIndex >= getNrOfVisualizedModels())
    {
        return "";
    }

#ifdef IDYNTREE_USES_IRRLICHT
    return pimpl->m_modelViz->at(modelInstanceIndex)->getInstanceName();
#else
    return "";
#endif
}

int Visualizer::getModelInstanceIndex(const std::string instanceName)
{
#ifdef IDYNTREE_USES_IRRLICHT
    for (size_t mdlInst = 0; mdlInst < getNrOfVisualizedModels(); mdlInst++)
    {
        if (pimpl->m_modelViz->at(mdlInst)->getInstanceName() == instanceName)
        {
            return static_cast<int>(mdlInst);
        }
    }
#else
    IDYNTREE_UNUSED(instanceName);
#endif

    reportError("Visualizer",
                "getModelInstanceIndex",
                "Impossible to find model instance with the specified name");
    return -1;
}

bool Visualizer::addModel(const Model& model, const std::string& instanceName)
{
#ifdef IDYNTREE_USES_IRRLICHT
    if (!this->pimpl->m_isInitialized)
    {
        init();
    }

    if (!this->pimpl->m_isInitialized)
    {
        reportError("Visualizer", "addModel", "Error in initializing Irrlicht device");
        return false;
    }

    ModelVisualization* newModelViz = new ModelVisualization();

    if (!newModelViz->init(model, instanceName, pimpl->m_irrSmgr))
    {
        delete newModelViz;
        return false;
    }

    this->pimpl->m_modelViz->push_back(newModelViz);

    return true;
#else
    IDYNTREE_UNUSED(model);
    IDYNTREE_UNUSED(instanceName);
    reportError("Visualizer",
                "addModel",
                "Impossible to use iDynTree::Visualizer, as iDynTree has been compiled without "
                "Irrlicht.");
    return false;
#endif
}

void Visualizer::draw()
{
#ifdef IDYNTREE_USES_IRRLICHT
    if (!pimpl->m_isInitialized)
    {
        reportError("Visualizer", "draw", "Impossible to run not initialized visualizer");
        return;
    }

    if (!pimpl->m_subDrawStarted)
    {
        int winWidth = width();
        int winHeight = height();

        if (winHeight <= 0)
        {
            return;
        }

#ifdef IDYNTREE_USE_GLFW_WINDOW
        glfwMakeContextCurrent(pimpl->m_window);
#endif

        pimpl->m_irrDriver->beginScene(true,
                                       true,
                                       pimpl->m_environment.m_backgroundColor.toSColor(),
                                       pimpl->m_irrVideoData);

        pimpl->m_irrDriver->setViewPort(irr::core::rect<irr::s32>(0, 0, winWidth, winHeight));

        pimpl->m_irrDriver->OnResize(irr::core::dimension2d<irr::u32>(winWidth, winHeight));

        pimpl->m_textures.draw(pimpl->m_environment, pimpl->m_camera, true);

        pimpl->m_camera.setWindowDimensions(winWidth, winHeight);

        pimpl->m_irrSmgr->drawAll();
    }

    pimpl->m_irrDriver->endScene();
    pimpl->m_subDrawStarted = false;

#ifdef IDYNTREE_USE_GLFW_WINDOW
    glfwSwapBuffers(pimpl->m_window);

    glfwPollEvents();
#endif

    int fps = pimpl->m_irrDriver->getFPS();

    if (pimpl->lastFPS != fps)
    {
        irr::core::stringw str = L"iDynTree Visualizer [";
        str += pimpl->m_irrDriver->getName();
        str += "] FPS:";
        str += fps;
        str += " ";
        irr::core::stringc strc(str);

        pimpl->m_irrDevice->setWindowCaption(str.c_str());
#ifdef IDYNTREE_USE_GLFW_WINDOW
        glfwSetWindowTitle(pimpl->m_window, strc.c_str());
#endif
        pimpl->lastFPS = fps;
    }

#else
    reportError("Visualizer",
                "draw",
                "Impossible to use iDynTree::Visualizer, as iDynTree has been compiled without "
                "Irrlicht.");
#endif
}

void Visualizer::subDraw(int xOffsetFromTopLeft,
                         int yOffsetFromTopLeft,
                         int subImageWidth,
                         int subImageHeight)
{
#ifdef IDYNTREE_USES_IRRLICHT
    if (!pimpl->m_isInitialized)
    {
        reportError("Visualizer", "subDraw", "Impossible to run not initialized visualizer");
        return;
    }

    if (xOffsetFromTopLeft + subImageWidth > width())
    {
        reportError("Visualizer",
                    "subDraw",
                    "The specified draw coordinates are out of bounds. The sum of the "
                    "xOffsetFromTopLeft and width are greater than the window width.");
        return;
    }

    if (yOffsetFromTopLeft + subImageHeight > height())
    {
        reportError("Visualizer",
                    "subDraw",
                    "The specified draw coordinates are out of bounds. The sum of the "
                    "yOffsetFromTopLeft and height are greater than the window height.");
        return;
    }

    if (subImageHeight <= 0)
    {
        return;
    }

    bool clearTextureBuffers = false;
    if (!pimpl->m_subDrawStarted)
    {
#ifdef IDYNTREE_USE_GLFW_WINDOW
        glfwMakeContextCurrent(pimpl->m_window);
#endif
        pimpl->m_irrDriver->beginScene(true,
                                       true,
                                       pimpl->m_environment.m_backgroundColor.toSColor(),
                                       pimpl->m_irrVideoData);
        pimpl->m_subDrawStarted = true;
        clearTextureBuffers = true;
    }

    pimpl->m_textures.draw(pimpl->m_environment, pimpl->m_camera, clearTextureBuffers);

    pimpl->m_camera.setAspectRatio(subImageWidth / (float)subImageHeight);

    int winWidth = width();
    int winHeight = height();
    pimpl->m_irrDriver->OnResize(irr::core::dimension2d<irr::u32>(winWidth, winHeight));
    pimpl->m_irrDriver->setViewPort(
        irr::core::rect<
            irr::s32>(0,
                      0,
                      winWidth,
                      winHeight)); // workaround for
                                   // http://irrlicht.sourceforge.net/forum/viewtopic.php?f=7&t=47004
    pimpl->m_irrDriver->setViewPort(irr::core::rect<irr::s32>(xOffsetFromTopLeft,
                                                              yOffsetFromTopLeft,
                                                              xOffsetFromTopLeft + subImageWidth,
                                                              yOffsetFromTopLeft + subImageHeight));

    pimpl->m_irrSmgr->drawAll();

#else
    reportError("Visualizer",
                "subDraw",
                "Impossible to use iDynTree::Visualizer, as iDynTree has been compiled without "
                "Irrlicht.");
#endif
}

bool Visualizer::drawToFile(const std::string filename)
{
    bool retValue = false;

    if (!pimpl->m_isInitialized)
    {
        reportError("Visualizer",
                    "drawToFile",
                    "Impossible to call drawToFile in a not initialized visualizer");
        return false;
    }

#ifdef IDYNTREE_USES_IRRLICHT
    // Method based on http://www.irrlicht3d.org/wiki/index.php?n=Main.TakingAScreenShot
    irr::video::IImage* const image = pimpl->m_irrDriver->createScreenShot();
    if (image) // should always be true, but you never know
    {
        // write screenshot to file
        if (!pimpl->m_irrDriver->writeImageToFile(image, filename.c_str()))
        {
            std::stringstream ss;
            ss << "Impossible to write image file to " << filename;
            reportError("Visualizer", "drawToFile", ss.str().c_str());
            retValue = false;
        } else
        {
            retValue = true;
        }

        // Don't forget to drop image since we don't need it anymore.
        image->drop();
    } else
    {
        reportError("Visualizer",
                    "drawToFile",
                    "Error in calling irr::video::IVideoDriver::createScreenShot method");
        return false;
    }
#else
    IDYNTREE_UNUSED(filename);
#endif

    return retValue;
}

IModelVisualization& Visualizer::modelViz(const std::string& instanceName)
{
    int idx = getModelInstanceIndex(instanceName);
    if (idx < 0)
    {
        return this->pimpl->m_invalidModelViz;
    }

#ifdef IDYNTREE_USES_IRRLICHT
    return *(this->pimpl->m_modelViz->at(idx));
#else
    return this->pimpl->m_invalidModelViz;
#endif
}

IModelVisualization& Visualizer::modelViz(size_t modelIdx)
{
#ifdef IDYNTREE_USES_IRRLICHT
    return *(this->pimpl->m_modelViz->at(modelIdx));
#else
    return this->pimpl->m_invalidModelViz;
#endif
}

ICamera& Visualizer::camera()
{
    return pimpl->m_camera;
}

IEnvironment& Visualizer::enviroment()
{
    return environment();
}

IEnvironment& Visualizer::environment()
{
    return pimpl->m_environment;
}

IVectorsVisualization& Visualizer::vectors()
{
#ifdef IDYNTREE_USES_IRRLICHT
    if (!this->pimpl->m_isInitialized)
    {
        init();
    }
    return this->pimpl->m_vectors;
#else
    return this->pimpl->m_invalidVectors;
#endif
}

IFrameVisualization& Visualizer::frames()
{
#ifdef IDYNTREE_USES_IRRLICHT
    if (!this->pimpl->m_isInitialized)
    {
        init();
    }
    return this->pimpl->m_frames;
#else
    return this->pimpl->m_invalidFrames;
#endif
}

ITexturesHandler& Visualizer::textures()
{
#ifdef IDYNTREE_USES_IRRLICHT
    return this->pimpl->m_textures;
#else
    return this->pimpl->m_invalidTextures;
#endif
}

IShapeVisualization& Visualizer::shapes()
{
#ifdef IDYNTREE_USES_IRRLICHT
    if (!this->pimpl->m_isInitialized)
    {
        init();
    }
    return this->pimpl->m_shapes;
#else
    return this->pimpl->m_invalidShapes;
#endif
}

ILabel& Visualizer::getLabel(const std::string& labelName)
{
#ifdef IDYNTREE_USES_IRRLICHT
    if (!this->pimpl->m_isInitialized)
    {
        init();
    }
    Label& requestedLabel = this->pimpl->m_labels[labelName];
    if (!requestedLabel.initialized())
    {
        requestedLabel.init(this->pimpl->m_irrSmgr);
    }
    return requestedLabel;
#else
    return this->pimpl->m_invalidLabel;
#endif
}

int Visualizer::width() const
{
#ifdef IDYNTREE_USES_IRRLICHT
    if (!pimpl->m_isInitialized)
    {
        reportError("Visualizer", "width", "Visualizer not initialized.");
        return 0;
    }
#ifdef IDYNTREE_USE_GLFW_WINDOW
    GLint ww, wh;
    glfwGetWindowSize(pimpl->m_window, &ww, &wh);

    return ww;
#else
    auto winDimensions = pimpl->m_irrDriver->getScreenSize();
    return winDimensions.Width;
#endif
#else
    return 0;
#endif
}

int Visualizer::height() const
{
#ifdef IDYNTREE_USES_IRRLICHT
    if (!pimpl->m_isInitialized)
    {
        reportError("Visualizer", "height", "Visualizer not initialized.");
        return 0;
    }

#ifdef IDYNTREE_USE_GLFW_WINDOW
    GLint ww, wh;
    glfwGetWindowSize(pimpl->m_window, &ww, &wh);

    return wh;
#else
    auto winDimensions = pimpl->m_irrDriver->getScreenSize();
    return winDimensions.Height;
#endif
#else
    return 0;
#endif
}

bool Visualizer::run()
{
#ifdef IDYNTREE_USES_IRRLICHT
    if (!pimpl->m_isInitialized)
    {
        reportError("Visualizer", "run", "Impossible to run not initialized visualizer");
        return false;
    }
    bool shouldClose = false;

#ifdef IDYNTREE_USE_GLFW_WINDOW
    shouldClose = glfwWindowShouldClose(pimpl->m_window);
#endif

    return pimpl->m_irrDevice->run() && !shouldClose;
#else
    reportError("Visualizer",
                "run",
                "Impossible to use iDynTree::Visualizer, as iDynTree has been compiled without "
                "Irrlicht.");
    return false;
#endif
}

void Visualizer::close()
{
#ifdef IDYNTREE_USES_IRRLICHT
    if (!pimpl->m_isInitialized)
    {
        return;
    }
#ifdef IDYNTREE_USE_GLFW_WINDOW
    glfwMakeContextCurrent(pimpl->m_window);
#endif

    pimpl->m_vectors.close();
    pimpl->m_frames.close();
    pimpl->m_environment.close();

    pimpl->m_irrDevice->closeDevice();
    pimpl->m_irrDevice->drop();
    pimpl->m_irrDevice = nullptr;
    pimpl->m_isInitialized = false;

    for (size_t mdl = 0; mdl < pimpl->m_modelViz->size(); mdl++)
    {
        if (pimpl->m_modelViz->at(mdl))
        {
            delete pimpl->m_modelViz->at(mdl);
            pimpl->m_modelViz->at(mdl) = nullptr;
        }
    }

    pimpl->m_modelViz->resize(0);

#ifdef IDYNTREE_USE_GLFW_WINDOW
    if (pimpl->m_window)
    {
        glfwMakeContextCurrent(pimpl->m_window);
        glfwDestroyWindow(pimpl->m_window);

        pimpl->m_glfwInstances--;
        if (pimpl->m_glfwInstances == 0)
        {
            glfwTerminate();
        }

        pimpl->m_window = nullptr;
    }
#endif

    return;
#endif
}

bool Visualizer::isWindowActive() const
{
#ifdef IDYNTREE_USES_IRRLICHT
    if (!pimpl->m_isInitialized)
    {
        return false;
    }
    return pimpl->m_irrDevice->isWindowActive();
#else
    return false;
#endif
}

bool Visualizer::setColorPalette(const std::string& name)
{
#ifdef IDYNTREE_USES_IRRLICHT

    if (!pimpl->m_isInitialized)
    {
        reportError("Visualizer",
                    "setColorPalette",
                    "Impossible to set the color palette. Please initialize the visualizer");
        return false;
    }

    const auto colors = pimpl->m_palette.find(name);

    if (colors == pimpl->m_palette.end())
    {
        std::string paletteName;
        for (const auto& tmp : pimpl->m_palette)
            paletteName += " " + tmp.first;

        const std::string error = "The palette named " + name
                                  + " does not exist. The following palette are available"
                                  + paletteName + ".";

        reportError("Visualizer", "setColorPalette", error.c_str());
        return false;
    }

    this->environment().setBackgroundColor(irrlicht2idyntree(colors->second.background));
    this->environment().setFloorGridColor(irrlicht2idyntree(colors->second.gridColor));

    this->vectors().setVectorsColor(irrlicht2idyntree(colors->second.vector));
    this->vectors().setVectorsDefaultColor(irrlicht2idyntree(colors->second.vector));

    // delete the frame in the origin and create a new one
    pimpl->m_environment.m_rootFrameNode->remove();
    pimpl->m_environment.m_rootFrameNode = addFrameAxes(pimpl->m_irrSmgr,
                                                        pimpl->m_environment.m_envNode,
                                                        pimpl->rootFrameArrowsDimension,
                                                        colors->second.xAxis,
                                                        colors->second.yAxis,
                                                        colors->second.zAxis);
    return true;

#else
    reportError("Visualizer",
                "setColorPalette",
                "Impossible to use iDynTree::Visualizer, as iDynTree has been compiled without "
                "Irrlicht.");
    return false;
#endif
}
} // namespace iDynTree

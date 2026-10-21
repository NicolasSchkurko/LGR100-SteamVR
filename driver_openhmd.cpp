//============ Copyright (c) Valve Corporation, All rights reserved. ============

#include <openvr_driver.h>
#include "driverlog.h"

// The headset is driven directly through hidapi: the driver owns the HID
// device, decodes the IMU reports and runs the sensor fusion. There is no
// OpenHMD in between any more.
#include "standalone/lgr100_device.h"

#include <assert.h>

#include <vector>
#include <thread>
#include <chrono>
#include <cstring>
#include <sstream>

#if defined( _WINDOWS )
#include <windows.h>
#endif

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>

using namespace vr;


// Per-eye statistics for ComputeDistortion, used to hunt the intermittent black
// left eye. One instance per eye, so the two eyes can be compared call by call.
struct DistortStats
{
    unsigned long long calls;
    unsigned long long lastCalls;
    unsigned long long oor;
    unsigned long long lastTick;
    float inMinU, inMaxU;
    float inMinV, inMaxV;
    float outMinU, outMaxU;
    float outMinV, outMaxV;
};


// True when a monitor of the headset panel size (1440x960) is present in the
// Windows display topology.
static BOOL CALLBACK findHeadsetMonitor(HMONITOR monitor, HDC, LPRECT, LPARAM data)
{
    MONITORINFO info;
    info.cbSize = sizeof(MONITORINFO);
    if (GetMonitorInfo(monitor, &info))
    {
        int width = info.rcMonitor.right - info.rcMonitor.left;
        int height = info.rcMonitor.bottom - info.rcMonitor.top;
        if (width == 1440 && height == 960)
        {
            *(bool*)data = true;
            return FALSE;
        }
    }
    return TRUE;
}

static bool headsetDisplayPresent()
{
    bool found = false;
    EnumDisplayMonitors(NULL, NULL, findHeadsetMonitor, (LPARAM)&found);
    return found;
}


#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C" 
#else
#error "Unsupported Platform."
#endif

class COpenHMDDeviceDriverController;


// keys for use with the settings API
static const char * const k_pch_Sample_Section = "driver_openhmd";
static const char * const k_pch_Sample_SecondsFromVsyncToPhotons_Float = "secondsFromVsyncToPhotons";
static const char * const k_pch_Sample_DisplayFrequency_Float = "displayFrequency";

HmdQuaternion_t identityquat{ 1, 0, 0, 0};
//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------

class CWatchdogDriver_OpenHMD : public IVRWatchdogProvider
{
public:
    CWatchdogDriver_OpenHMD()
    {
        DriverLog("Created watchdog object\n");
        m_pWatchdogThread = nullptr;
    }
    virtual ~CWatchdogDriver_OpenHMD() {}

    virtual EVRInitError Init( vr::IVRDriverContext *pDriverContext ) ;
    virtual void Cleanup() ;

private:
	std::thread *m_pWatchdogThread;
};

CWatchdogDriver_OpenHMD g_watchdogDriverOpenHMD;


bool g_bExiting = false;

void WatchdogThreadFunction(  )
{
    while ( !g_bExiting )
    {
#if defined( _WINDOWS )
        // on windows send the event when the Y key is pressed.
        if ( (0x01 & GetAsyncKeyState( 'Y' )) != 0 )
        {
            // Y key was pressed.
            vr::VRWatchdogHost()->WatchdogWakeUp(vr::TrackedDeviceClass_HMD);
        }
        std::this_thread::sleep_for( std::chrono::microseconds( 500 ) );
#else
        DriverLog("Watchdog wakeup\n");
        // for the other platforms, just send one every five seconds
        std::this_thread::sleep_for( std::chrono::seconds( 1 ) );
        vr::VRWatchdogHost()->WatchdogWakeUp(vr::TrackedDeviceClass_HMD);
#endif
    }
    DriverLog("Watchdog exit\n");
}

EVRInitError CWatchdogDriver_OpenHMD::Init( vr::IVRDriverContext *pDriverContext )
{
    VR_INIT_WATCHDOG_DRIVER_CONTEXT( pDriverContext );
    InitDriverLog( vr::VRDriverLog() );

    // Watchdog mode on Windows starts a thread that listens for the 'Y' key on the keyboard to
    // be pressed. A real driver should wait for a system button event or something else from the
    // the hardware that signals that the VR system should start up.
    g_bExiting = false;
    
    DriverLog("starting watchdog thread\n");

    m_pWatchdogThread = new std::thread( WatchdogThreadFunction );
    if ( !m_pWatchdogThread )
    {
        DriverLog( "Unable to create watchdog thread\n");
        return VRInitError_Driver_Failed;
    }

    return VRInitError_None;
    }


    void CWatchdogDriver_OpenHMD::Cleanup()
    {
    g_bExiting = true;
    if ( m_pWatchdogThread )
    {
        m_pWatchdogThread->join();
        delete m_pWatchdogThread;
        m_pWatchdogThread = nullptr;
    }

    CleanupDriverLog();
}

// Control hints and types of the null controllers. They used to come from
// OpenHMD's dummy driver; the values are the same, so the input paths the
// controllers expose do not change.
enum ENullControlHint
{
	kNullGeneric = 0, kNullTrigger, kNullTriggerClick, kNullSqueeze,
	kNullMenu, kNullHome, kNullAnalogX, kNullAnalogY, kNullAnalogPress,
	kNullButtonA, kNullButtonB, kNullButtonX, kNullButtonY,
	kNullVolumeUp, kNullVolumeDown, kNullMicMute
};

enum ENullControlType
{
	kNullDigital = 0,
	kNullAnalog = 1
};

class COpenHMDDeviceDriverController : public vr::ITrackedDeviceServerDriver /*, public vr::IVRControllerComponent */ {
public:
    int index;
    bool m_is_left;
    DriverPose_t pose;

    COpenHMDDeviceDriverController(int index, bool bIsLeft) :
		index(index), m_is_left(bIsLeft) {
        DriverLog("construct controller object %d (null device)\n", index);
        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
        pose = { 0 };
    }
    virtual ~COpenHMDDeviceDriverController() {}

    EVRInitError Activate( vr::TrackedDeviceIndex_t unObjectId )
    {
        DriverLog("activate controller %d: %d\n", index, unObjectId);
        m_unObjectId = unObjectId;

        m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer( m_unObjectId );

        const char *controllerModel = m_is_left ? "Left Controller Null Device" : "Right Controller Null Device";

        vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_ModelNumber_String, controllerModel);
        //vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_RenderModelName_String, controllerModel);
	//vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_ModelNumber_String, "1" );
        vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_ControllerType_String, "openhmd_controller" );
        vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_InputProfilePath_String, "{openhmd}/input/openhmd_controller_profile.json" );
        vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_RenderModelName_String, "vr_controller_vive_1_5");


        // return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
        vr::VRProperties()->SetUint64Property( m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2 );

        vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_DeviceClass_Int32, vr::TrackedDeviceClass_Controller);

        // avoid "not fullscreen" warnings from vrmonitor
        vr::VRProperties()->SetBoolProperty( m_ulPropertyContainer, Prop_IsOnDesktop_Bool, false );

	if (m_is_left) {
           DriverLog("Left Controller\n");
           vr::VRProperties()->SetInt32Property( m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_LeftHand);
	   // Set an initial position down and to the left, which will be
	   // used if there's no positional tracking
	   pose.vecPosition[0] = -0.25;
	   pose.vecPosition[1] = -0.5;
	   pose.vecPosition[2] = 0.15;
	} else {
           DriverLog("Right Controller\n");
           vr::VRProperties()->SetInt32Property( m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_RightHand);
	   pose.vecPosition[0] = 0.25;
	   pose.vecPosition[1] = -0.5;
	   pose.vecPosition[2] = 0.15;
	}


        const int control_count = 2;

        const char* controls_fn_str[] = { "generic", "trigger", "trigger_click", "squeeze", "menu", "home",
                "analog-x", "analog-y", "anlog_press", "button-a", "button-b", "button-x", "button-y",
                "volume-up", "volume-down", "mic-mute"};
        const char* controls_type_str[] = {"digital", "analog"};

        int controls_fn[64];
        int controls_types[64];

        controls_fn[0] = kNullButtonA;
        controls_fn[1] = kNullMenu;
        controls_types[0] = kNullAnalog;
        controls_types[1] = kNullDigital;

        for(int i = 0; i < control_count; i++){
          DriverLog("%s (%s)%s\n", controls_fn_str[controls_fn[i]], controls_type_str[controls_types[i]], i == control_count - 1 ? "" : ", ");
          const char *control_map = NULL, *touch_map = NULL;
          EVRScalarUnits analog_type = VRScalarUnits_NormalizedOneSided;

          m_buttons[i] = k_ulInvalidInputComponentHandle;
          m_analogControls[i] = k_ulInvalidInputComponentHandle;
          m_touchControls[i] = k_ulInvalidInputComponentHandle;

          // TODO: inputs match steamapps/common/SteamVR/drivers/oculus/resources/input/touch_profile.json
          // but also support other controllers
          switch (controls_fn[i]) {
            case kNullGeneric:
              control_map = "/input/generic/click";
              break;
            case kNullTrigger:
              control_map = "/input/trigger/value";
              break;
            case kNullTriggerClick:
              control_map = "/input/trigger/click";
              break;
            case kNullSqueeze:
              control_map = "/input/grip/value";
              break;
            case kNullMenu:
              control_map = "/input/system/click";
              break;
            case kNullHome:
                // TODO: this button doesn't have an input in touch_profile.json
              control_map = "/input/system/click";
              break;
            case kNullAnalogX:
              control_map = "/input/joystick/x";
              analog_type = VRScalarUnits_NormalizedTwoSided;
              break;
            case kNullAnalogY:
              control_map = "/input/joystick/y";
              analog_type = VRScalarUnits_NormalizedTwoSided;
              break;
            case kNullAnalogPress:
              control_map = "/input/joystick/click";
              break;
            case kNullButtonA:
              control_map = "/input/a/click";
              touch_map = "/input/a/touch";
              break;
            case kNullButtonB:
              control_map = "/input/b/click";
              touch_map = "/input/b/touch";
              break;
            case kNullButtonX:
              control_map = "/input/x/click";
              touch_map = "/input/x/touch";
              break;
            case kNullButtonY:
              control_map = "/input/y/click";
              touch_map = "/input/y/touch";
              break;

            default:
              break;
          }

          /* We fall through here for generic buttons */
          if (control_map != NULL) {
            if (controls_types[i] == kNullDigital) {
              vr::VRDriverInput()->CreateBooleanComponent( m_ulPropertyContainer, control_map, m_buttons + i);
            }
            else {
              vr::VRDriverInput()->CreateScalarComponent( m_ulPropertyContainer, control_map, m_analogControls + i, VRScalarType_Absolute, analog_type);
            }
          }
	  if (touch_map != NULL) {
              vr::VRDriverInput()->CreateScalarComponent( m_ulPropertyContainer, touch_map, m_touchControls + i, VRScalarType_Absolute, analog_type);
          }
        }

        return VRInitError_None;
    }

    void Deactivate()
    {
        DriverLog("deactivate controller\n");
        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
    }

    void EnterStandby()
    {
                DriverLog("standby controller\n");
    }

    void *GetComponent( const char *pchComponentNameAndVersion )
    {
        DriverLog("get controller component %s | %s ", pchComponentNameAndVersion, /*vr::IVRControllerComponent_Version*/ "<nothing>");
        if (!strcmp(pchComponentNameAndVersion, /*vr::IVRControllerComponent_Version*/ "<nothing>"))
        {
            DriverLog(": yes\n");
            return NULL;//(vr::IVRControllerComponent*)this;

        }

        DriverLog(": no\n");
        return NULL;
    }

    /** debug request from a client */
    void DebugRequest( const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize )
    {
        if( unResponseBufferSize >= 1 )
            pchResponseBuffer[0] = 0;
    }

    DriverPose_t GetPose()
    {
	pose.poseIsValid = true;
	pose.result = TrackingResult_Running_OK;
	pose.deviceIsConnected = true;

	// The null controllers have no tracking of their own: an identity
	// rotation and a fixed position, exactly as OpenHMD's dummy driver did.
	pose.qRotation.x = 0.0f;
	pose.qRotation.y = 0.0f;
	pose.qRotation.z = 0.0f;
	pose.qRotation.w = 1.0f;

	pose.vecPosition[0] = m_is_left ? -0.5f : 0.5f;
	pose.vecPosition[1] = 0.0f;
	pose.vecPosition[2] = 0.0f;

	// DriverLog("get controller %d pose %f %f %f %f, %f %f %f\n", index, quat[0], quat[1], quat[2], quat[3], pos[0], pos[1], pos[2]);

	for (int i = 0; i < 3; i++) {
		pose.vecDriverFromHeadTranslation[i] = 0.0;
		pose.vecWorldFromDriverTranslation[i] = 0.0;
	}

	pose.qDriverFromHeadRotation = identityquat;
	pose.qWorldFromDriverRotation = identityquat;

	return pose;
    }

    void RunFrame() {
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, GetPose(), sizeof( DriverPose_t ) );

        const int control_count = 2;
        float control_state[2] = { 0.1f, 1.0f };

        for (int i = 0; i < control_count; i++) {
          if (m_buttons[i] != k_ulInvalidInputComponentHandle) {
            vr::VRDriverInput()->UpdateBooleanComponent( m_buttons[i], control_state[i] != 0, 0 );
          }
          else if (m_analogControls[i] != k_ulInvalidInputComponentHandle)
            vr::VRDriverInput()->UpdateScalarComponent( m_analogControls[i], control_state[i], 0 );
	  /* If the control is not 0, mark it touched */
          if (m_touchControls[i] != k_ulInvalidInputComponentHandle) {
            vr::VRDriverInput()->UpdateScalarComponent( m_touchControls[i], control_state[i] == 0 ? 0.0 : 1.0, 0 );
          }
        }
    }

    VRControllerState_t controllerstate;

    VRControllerState_t GetControllerState() {
    DriverLog("get controller state\n");
    //return controllerstate;

    controllerstate.unPacketNum = controllerstate.unPacketNum + 1;
    //TODO: buttons

    //TODO: nolo says when a button was pressed a button was also touched. is that so?
    controllerstate.ulButtonTouched |= controllerstate.ulButtonPressed;

    //uint64_t ulChangedTouched = controllerstate.ulButtonTouched ^ controllerstate.ulButtonTouched;
    //uint64_t ulChangedPressed = controllerstate.ulButtonPressed ^ controllerstate.ulButtonPressed;
    return controllerstate;
    }

    bool TriggerHapticPulse( uint32_t unAxisId, uint16_t usPulseDurationMicroseconds ) {
        return false;
    }

    std::string GetSerialNumber() const { 
        DriverLog("get controller serial number %s\n", m_sSerialNumber.c_str());
        return m_sSerialNumber;
    }

private:
    std::string m_sSerialNumber = "Controller serial number " + std::to_string(index);
    std::string m_sModelNumber = "Controller model number " + std::to_string(index);
    vr::TrackedDeviceIndex_t m_unObjectId;
    vr::PropertyContainerHandle_t m_ulPropertyContainer;

    /* Generic button controls */
    vr::VRInputComponentHandle_t m_buttons[64]; /* Maximum components we support */
    /* Analog controls */
    vr::VRInputComponentHandle_t m_analogControls[64]; /* Maximum components we support */
    /* Touch controls */
    vr::VRInputComponentHandle_t m_touchControls[64]; /* Maximum components we support */
};

class COpenHMDDeviceDriver final : public vr::ITrackedDeviceServerDriver, public vr::IVRDisplayComponent
{
public:
    COpenHMDDeviceDriver()
    {
        // The headset is opened directly through hidapi. If it cannot be
        // opened the driver still loads, and reports the reason through the
        // SteamVR log.
        if (!m_device.open())
        {
            DriverLog("failed to open device: %s\n", m_device.lastError());
        }

        // Surface the status of the panel activation. It matters because a
        // failed activation leaves the left panel dark and looks exactly like
        // a rendering bug.
        const char* hmd_status = m_device.lastError();
        if(hmd_status && hmd_status[0])
            DriverLog("driver_openhmd: HMD status: %s\n", hmd_status);

        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
        m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;

        DriverLog( "Using settings values\n" );
        m_flIPD = m_device.ipd();

        {
            std::stringstream buf;
            buf << m_device.product();
            buf << ": ";
            buf << m_device.path();
            m_sSerialNumber = buf.str();
        }

        {
            std::stringstream buf;
            buf << "LG 360 VR: ";
            buf << m_device.product();
            m_sModelNumber = buf.str();
        }

        // Important to pass vendor through. Gaze cursor is only available for "Oculus". So grab the first word.
        m_sVendor = m_device.vendor();
        if (m_sVendor.find(' ') != std::string::npos) {
            m_sVendor = m_sVendor.substr(0, m_sVendor.find(' '));
        }

        m_nWindowX = 1920; //TODO: real window offset
        m_nWindowY = 0;
        m_nWindowWidth = m_device.screenWidth();
        m_nWindowHeight = m_device.screenHeight();
        m_nRenderWidth = m_device.screenWidth();
        m_nRenderHeight = m_device.screenHeight();

        m_flSecondsFromVsyncToPhotons = vr::VRSettings()->GetFloat( k_pch_Sample_Section, k_pch_Sample_SecondsFromVsyncToPhotons_Float );
        m_flDisplayFrequency = vr::VRSettings()->GetFloat( k_pch_Sample_Section, k_pch_Sample_DisplayFrequency_Float );
	// A zero refresh rate makes SteamVR's presentation timing invalid; fall back
	// to a sane value when the setting is missing or unset.
	if(m_flDisplayFrequency <= 0.0f)
		m_flDisplayFrequency = 60.0f;

        DriverLog( "driver_openhmd: Vendor: %s\n", m_sVendor.c_str() );
        DriverLog( "driver_openhmd: Serial Number: %s\n", m_sSerialNumber.c_str() );
        DriverLog( "driver_openhmd: Model Number: %s\n", m_sModelNumber.c_str() );
        DriverLog( "driver_openhmd: Window: %d %d %d %d\n", m_nWindowX, m_nWindowY, m_nWindowWidth, m_nWindowHeight );
        DriverLog( "driver_openhmd: Render Target: %d %d\n", m_nRenderWidth, m_nRenderHeight );
        DriverLog( "driver_openhmd: Seconds from Vsync to Photons: %f\n", m_flSecondsFromVsyncToPhotons );
                                                DriverLog( "driver_openhmd: Display Frequency: %f\n", m_flDisplayFrequency );
        DriverLog( "driver_openhmd: IPD: %f\n", m_flIPD );

        // The headset has no lens calibration: the distortion is the identity.
        float distortion_coeffs[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
        DriverLog("driver_openhmd: Distortion values a=%f b=%f c=%f d=%f\n", distortion_coeffs[0], distortion_coeffs[1], distortion_coeffs[2], distortion_coeffs[3]);

        // Cache everything the eye path needs, once. GetProjectionRaw and
        // ComputeDistortion are called by SteamVR while compositing; taking
        // OpenHMD's update mutex per call used to block them whenever the HID
        // thread stalled, which blacked out one eye.
        // The projection matrices are computed from the panel geometry (the
        // same maths OpenHMD used), so nothing has to be read from a device
        // on the hot path.
        m_device.getEyeProjection(0, m_flProjectionLeft);
        m_device.getEyeProjection(1, m_flProjectionRight);
        {
            float pitch, roll;
            float p[4][4];
            memcpy(p, m_flProjectionLeft, sizeof(p));
            columnMatrixToAngles(&m_flRotationLeft, &pitch, &roll, p);
            memcpy(p, m_flProjectionRight, sizeof(p));
            columnMatrixToAngles(&m_flRotationRight, &pitch, &roll, p);
        }

        m_flViewportScale[0] = m_device.physicalWidth() / 2.0f;
        m_flViewportScale[1] = m_device.physicalHeight();

        const float lens_sep = m_device.lensSeparation();
        m_flLensCenterLeft[1] = m_device.lensVerticalPosition();
        m_flDistortionCoeffs[0] = 0.0f;
        m_flDistortionCoeffs[1] = 0.0f;
        m_flDistortionCoeffs[2] = 0.0f;
        m_flDistortionCoeffs[3] = 1.0f;
        m_flAberration[0] = 1.0f;
        m_flAberration[1] = 1.0f;
        m_flAberration[2] = 1.0f;

        m_flLensCenterLeft[0] = m_flViewportScale[0] - lens_sep / 2.0f;
        m_flLensCenterRight[0] = lens_sep / 2.0f;
        m_flLensCenterRight[1] = m_flLensCenterLeft[1];

        DriverLog("driver_openhmd: cached rotations L=%.1f R=%.1f viewport=(%.4f,%.4f) lensL=(%.4f,%.4f) lensR=(%.4f,%.4f) aberr=(%f,%f,%f)\n",
                  m_flRotationLeft, m_flRotationRight, m_flViewportScale[0], m_flViewportScale[1],
                  m_flLensCenterLeft[0], m_flLensCenterLeft[1], m_flLensCenterRight[0], m_flLensCenterRight[1],
                  m_flAberration[0], m_flAberration[1], m_flAberration[2]);

	/* Sleep for 1 second while activating to let the display connect */
	std::this_thread::sleep_for( std::chrono::seconds(1) );
    }

    virtual ~COpenHMDDeviceDriver()
    {
    }


    EVRInitError Activate( vr::TrackedDeviceIndex_t unObjectId )
    {
        m_unObjectId = unObjectId;
        m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer( m_unObjectId );

        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_ManufacturerName_String, m_sVendor.c_str());
        vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_ModelNumber_String, m_sModelNumber.c_str() );
        vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, Prop_UserIpdMeters_Float, m_flIPD );
        vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, Prop_UserHeadToEyeDepthMeters_Float, 0.f );
        vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, Prop_DisplayFrequency_Float, m_flDisplayFrequency );
        vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, Prop_SecondsFromVsyncToPhotons_Float, m_flSecondsFromVsyncToPhotons );

        // The HMD needs its own input profile. Without it SteamVR looks for a
        // controller type named "openhmd_hmd", does not find it, and every
        // binding load for this device fails ("openhmd_hmd has no configured
        // binding"). The Steam client then retries the binding load in a hot
        // loop (~48 times per minute), the input system never settles, the HMD
        // is never considered ready and SteamVR parks it in standby - which
        // leaves the headset window black.
        vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_ControllerType_String, "openhmd_hmd" );
        vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_InputProfilePath_String, "{openhmd}/input/openhmd_hmd_profile.json" );

        // Reverted: setting Prop_EdidVendorID_Int32 / Prop_EdidProductID_Int32 here
        // did not clear "Found bad mirror window settings" and it left only one eye
        // rendering. The compositor keeps timing out on the panel EDID either way,
        // and the EDID properties made the stereo pair asymmetric.

        //float sep;

        // return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
        vr::VRProperties()->SetUint64Property( m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2 );

        return VRInitError_None;
    }

    void Deactivate()
    {
        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
    }

    void EnterStandby()
    {
    }

    void *GetComponent( const char *pchComponentNameAndVersion )
    {
        if ( !strcmp( pchComponentNameAndVersion, vr::IVRDisplayComponent_Version ) )
        {
            return (vr::IVRDisplayComponent*)this;
        }
        return NULL;
    }

    void PowerOff()
    {
    }

    void DebugRequest( const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize )
    {
        if( unResponseBufferSize >= 1 )
            pchResponseBuffer[0] = 0;
    }

    void GetWindowBounds( int32_t *pnX, int32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight )
    {
        *pnX = m_nWindowX;
        *pnY = m_nWindowY;
        *pnWidth = m_nWindowWidth;
        *pnHeight = m_nWindowHeight;
    }

    bool IsDisplayOnDesktop()
    {
        return true;
    }

    bool IsDisplayRealDisplay()
    {
        return true;
    }

    void GetRecommendedRenderTargetSize( uint32_t *pnWidth, uint32_t *pnHeight )
    {
        *pnWidth = m_nRenderWidth;
        *pnHeight = m_nRenderHeight;
        DriverLog("driver_openhmd: GetRecommendedRenderTargetSize -> %u x %u\n", *pnWidth, *pnHeight);
    }

    void GetEyeOutputViewport( EVREye eEye, uint32_t *pnX, uint32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight )
    {
        *pnY = 0;
        *pnWidth = m_nWindowWidth / 2;
        *pnHeight = m_nWindowHeight;

        if ( eEye == Eye_Left )
        {
            *pnX = 0;
        }
        else
        {
            *pnX = m_nWindowWidth / 2;
        }
        DriverLog("driver_openhmd: GetEyeOutputViewport eye=%d -> x=%u y=%u w=%u h=%u (window %d x %d)\n",
                  (int)eEye, *pnX, *pnY, *pnWidth, *pnHeight, m_nWindowWidth, m_nWindowHeight);
    }

    // flatten 2D indices in a 4x4 matrix explicit so it's easy to see what's happening:
    int f1(int i, int j)
    {
        if (i == 0 && j == 0) return 0;
        if (i == 0 && j == 1) return 1;
        if (i == 0 && j == 2) return 2;
        if (i == 0 && j == 3) return 3;

        if (i == 1 && j == 0) return 4;
        if (i == 1 && j == 1) return 5;
        if (i == 1 && j == 2) return 6;
        if (i == 1 && j == 3) return 7;

        if (i == 2 && j == 0) return 8;
        if (i == 2 && j == 1) return 9;
        if (i == 2 && j == 2) return 10;
        if (i == 2 && j == 3) return 11;

        if (i == 3 && j == 0) return 12;
        if (i == 3 && j == 1) return 13;
        if (i == 3 && j == 2) return 14;
        if (i == 3 && j == 3) return 15;

        return -1;
    }

    int f2(int i, int j)
    {
        if (i == 0 && j == 0) return 0;
        if (i == 0 && j == 1) return 4;
        if (i == 0 && j == 2) return 8;
        if (i == 0 && j == 3) return 12;

        if (i == 1 && j == 0) return 1;
        if (i == 1 && j == 1) return 5;
        if (i == 1 && j == 2) return 9;
        if (i == 1 && j == 3) return 13;

        if (i == 2 && j == 0) return 2;
        if (i == 2 && j == 1) return 6;
        if (i == 2 && j == 2) return 10;
        if (i == 2 && j == 3) return 14;

        if (i == 3 && j == 0) return 3;
        if (i == 3 && j == 1) return 7;
        if (i == 3 && j == 2) return 11;
        if (i == 3 && j == 3) return 15;

        return -1;
    }

    void columnMatrixToAngles(float *yaw, float *pitch, float *roll, float colMatrix[4][4] ) {
        double sinPitch, cosPitch, sinRoll, cosRoll, sinYaw, cosYaw;

        sinPitch = -colMatrix[2][0];
        cosPitch = sqrt(1 - sinPitch*sinPitch);

        if ( abs(cosPitch) > 0.1) 
        {
            sinRoll = colMatrix[2][1] / cosPitch;
            cosRoll = colMatrix[2][2] / cosPitch;
            sinYaw = colMatrix[1][0] / cosPitch;
            cosYaw = colMatrix[0][0] / cosPitch;
        } 
        else 
        {
            sinRoll = -colMatrix[1][2];
            cosRoll = colMatrix[1][1];
            sinYaw = 0;
            cosYaw = 1;
        }

        *yaw   = atan2(sinYaw, cosYaw) * 180 / M_PI;
        *pitch = atan2(sinPitch, cosPitch) * 180 / M_PI;
        *roll  = atan2(sinRoll, cosRoll) * 180 / M_PI;
    } 
    
    typedef union {
        float m[4][4];
        float arr[16];
    } mat4x4f;

    void omat4x4f_mult(const mat4x4f* l, const mat4x4f* r, mat4x4f *o) {
        for(int i = 0; i < 4; i++){
            float a0 = l->m[i][0], a1 = l->m[i][1], a2 = l->m[i][2], a3 = l->m[i][3];
            o->m[i][0] = a0 * r->m[0][0] + a1 * r->m[1][0] + a2 * r->m[2][0] + a3 * r->m[3][0];
            o->m[i][1] = a0 * r->m[0][1] + a1 * r->m[1][1] + a2 * r->m[2][1] + a3 * r->m[3][1];
            o->m[i][2] = a0 * r->m[0][2] + a1 * r->m[1][2] + a2 * r->m[2][2] + a3 * r->m[3][2];
            o->m[i][3] = a0 * r->m[0][3] + a1 * r->m[1][3] + a2 * r->m[2][3] + a3 * r->m[3][3];
        }
    }
    
    void createUnRotation(float angle, mat4x4f *m) {
        memset(m, 0, sizeof(*m));
        m->m[0][0] = 1.0f;
        m->m[1][1] = 1.0f;
        m->m[2][2] = 1.0f;
        m->m[3][3] = 1.0f;
        
        DriverLog("Unrotating for angle %f\n", angle);
        
        if (angle > -5 && angle < 5) {
            return;
        }
        
        else if (angle > 85 && angle < 95) {
            m->m[0][0] = 0.0f; m->m[0][1] = -1.0f; m->m[1][0] = 1.0f; m->m[1][1] = 0.0f;
            m->m[0][1] = 1.0f; m->m[1][0] = -1.0f;
        }

        else if (angle > -95 && angle < -85) {
            m->m[0][0] = 0.0f; m->m[0][1] = -1.0f; m->m[1][0] = 1.0f; m->m[1][1] = 0.0f;
        }
        
        else {
            DriverLog("UNIMPLEMENTED ROTATION!!!\n");
        }
    }

    void GetProjectionRaw( EVREye eEye, float *pfLeft, float *pfRight, float *pfTop, float *pfBottom )
    {
        mat4x4f ohmdprojection;
        float yaw;
        // Matrices and rotations are fixed properties of the panel wiring,
        // cached at construction (also removes the old first-call latch race).
        if (eEye == Eye_Left) {
            memcpy(ohmdprojection.arr, m_flProjectionLeft, sizeof(m_flProjectionLeft));
            yaw = m_flRotationLeft;
        } else {
            memcpy(ohmdprojection.arr, m_flProjectionRight, sizeof(m_flProjectionRight));
            yaw = m_flRotationRight;
        }

        mat4x4f unrotation;
        createUnRotation(yaw, &unrotation);
 
        DriverLog("unrotation\n%f %f %f %f\n%f %f %f %f %f\n%f %f %f %f\n%f %f %f %f\n",
            unrotation.arr[0], unrotation.arr[1], unrotation.arr[2], unrotation.arr[3],
            unrotation.arr[4], unrotation.arr[5], unrotation.arr[6], unrotation.arr[7],
            unrotation.arr[8], unrotation.arr[9], unrotation.arr[10], unrotation.arr[11],
            unrotation.arr[12], unrotation.arr[13], unrotation.arr[14], unrotation.arr[15]);
            
        omat4x4f_mult(&ohmdprojection, &unrotation, &ohmdprojection);
        
        // http://stackoverflow.com/questions/10830293/ddg#12926655
        // get projection matrix from openhmd, convert it into lrtb + near,far with SO formula
        // then divide by near plane distance to get the tangents of the angles from the center plane (tan = opposite side = these values divided by adjacent side = near plane distance)
        // but negate top and bottom. who knows why. there are 3 or so issues for it on github

        // f2 switches row-major and column-major
        float m00 = ohmdprojection.arr[f2(0,0)];
        //float m03 = ohmdprojection[f2(0,3)];
        //float m10 = ohmdprojection[f2(1,3)];
        float m11 = ohmdprojection.arr[f2(1,1)];
        //float m13 = ohmdprojection[f2(1,3)];
        float m23 = ohmdprojection.arr[f2(2,3)];
        float m22 = ohmdprojection.arr[f2(2,2)];
        float m12 = ohmdprojection.arr[f2(1,2)];
        float m02 = ohmdprojection.arr[f2(0,2)];

        float zNear  = m23/(m22-1);
        float zFar   = m23/(m22+1);
        *pfBottom = -   (m12-1)/m11;
        *pfTop    = -   (m12+1)/m11;
        *pfLeft   =     (m02-1)/m00;
        *pfRight  =     (m02+1)/m00;
        
        DriverLog("m 00 %f, 11 %f, 22 %f, 12 %f, 02 %f\n", m00, m11, m23, m22, m12, m02);

        DriverLog("ohmd projection\n%f %f %f %f\n%f %f %f %f %f\n%f %f %f %f\n%f %f %f %f\n",
            ohmdprojection.arr[0], ohmdprojection.arr[1], ohmdprojection.arr[2], ohmdprojection.arr[3],
            ohmdprojection.arr[4], ohmdprojection.arr[5], ohmdprojection.arr[6], ohmdprojection.arr[7],
            ohmdprojection.arr[8], ohmdprojection.arr[9], ohmdprojection.arr[10], ohmdprojection.arr[11],
            ohmdprojection.arr[12], ohmdprojection.arr[13], ohmdprojection.arr[14], ohmdprojection.arr[15]
        );
        
        DriverLog("projectionraw values lrtb, near far: %f %f %f %f | %f %f\n", *pfLeft, *pfRight, *pfTop, *pfBottom, zNear, zFar);
        
        // Per-eye frustum trace (both eyes, first 200 calls each). The aspect must be
        // identical across eyes and must not change with pose. A left/right
        // mismatch, or one eye collapsing to inf/0, is a degenerate frustum and
        // that is what turns an eye black.
        {
            int slot = (eEye == Eye_Left) ? 0 : 1;
            static int counts[2] = {0, 0};
            if(counts[slot] < 200)
            {
                counts[slot]++;
                DriverLog("FRUSTUM %s n%3d: l=%.5f r=%.5f t=%.5f b=%.5f\n",
                          (eEye == Eye_Left) ? "LEFT " : "RIGHT",
                          counts[slot], *pfLeft, *pfRight, *pfTop, *pfBottom);
            }
        }

        //DriverLog("angles %f %f %f\n", yaw, pitch, roll);
    }

    DistortionCoordinates_t ComputeDistortion( EVREye eEye, float fU, float fV )
    {
        float angle = (eEye == Eye_Left ? m_flRotationLeft : m_flRotationRight);

        // Coordinates exactly as SteamVR handed them to us, before the +/-90
        // rotation and the vertical centring below. Kept for the per-eye
        // instrumentation at the end of this function.
        const float inU = fU;
        const float inV = fV;

        //DriverLog("Eye %d before: %f %f\n", eEye, fU, fV);
                
        // Texture scale and per-eye offset for the LGR100 panel, taken from the
        // project's own LGVR4VLC shader, which is calibrated against the real
        // hardware (github.com/bauermaximilian/LG-360-VR-for-PC):
        //
        //   left  eye: offset = (-0.25, -0.0115), scale = (0.95, 0.45), rot = +90
        //   right eye: offset = ( 0.25, -0.0115), scale = (0.95, 0.45), rot = -90
        //
        // The 0.95 x 0.45 scale is what the driver's own distortion maths was
        // missing: it assumed each eye filled its whole viewport, which stretched
        // the image and made it read differently depending on head orientation.
        if (angle > 85 && angle < 95) {
            float cx = fU - 0.5f;
            float cy = fV - 0.5f;
            fU = 0.5f + cy;
            fV = 0.5f - cx;
        } else if (angle > -95 && angle < -85) {
            float cx = fU - 0.5f;
            float cy = fV - 0.5f;
            fU = 0.5f - cy;
            fV = 0.5f + cx;
        }

        // No per-eye offset is applied.
        //
        // The LGVR4VLC shader offsets each eye by -0.25 / +0.25, but that shader
        // samples the *whole* surface and has to pull each half into its own side.
        // SteamVR hands this driver coordinates already normalised per eye, so that
        // offset would only shift the sampling out of [0,1] and black the eye out.
        //
        // LAST KNOWN GOOD: only a small vertical centring was applied here. It was
        // removed while chasing the black left eye (it pushed that eye's top row
        // to 1.0115, outside the surface), and the left eye stayed black anyway,
        // so it was not the cause. Restored to the last build where both eyes
        // rendered; if it is the culprit after all it is a one-line change.
        {
            const float eye_offset_y = -0.0115f;

            fV = (fV - 0.5f) - eye_offset_y + 0.5f;
        }
        
        //DriverLog("Eye %d after: %f %f\n", eEye, fU, fV);
        
        float viewport_scale[2] = { m_flViewportScale[0], m_flViewportScale[1] };
        float distortion_coeffs[4];
        memcpy(distortion_coeffs, m_flDistortionCoeffs, sizeof(m_flDistortionCoeffs));
        float aberr_scale[3];
        memcpy(aberr_scale, m_flAberration, sizeof(m_flAberration));

        float lens_center[2];
        lens_center[0] = (eEye == Eye_Left ? m_flLensCenterLeft[0] : m_flLensCenterRight[0]);
        lens_center[1] = (eEye == Eye_Left ? m_flLensCenterLeft[1] : m_flLensCenterRight[1]);

        // The texture coordinates were rotated above, so the lens centre has to be
        // expressed on the same axes. When the rotation is +/-90 the two axes swap:
        // for the left eye the horizontal centre moves onto the vertical axis.
        // LAST KNOWN GOOD: gated on the left eye's angle only. Extending it to the
        // right eye was tried and did not restore the left eye, so it is back here.
        if(angle > 85 && angle < 95)
        {
            float c = lens_center[0];
            lens_center[0] = lens_center[1];
            lens_center[1] = c;
        }

        // warp_scale is the distance from the lens centre to the far edge of the
        // viewport, which must be measured on the same (rotated) axes as
        // lens_center. Computing it from a single shared value made one eye sample
        // far outside [0,1] and render black.
        float viewport_along[2] = { viewport_scale[0], viewport_scale[1] };
        if(angle > 85 && angle < 95)
        {
            float v = viewport_along[0];
            viewport_along[0] = viewport_along[1];
            viewport_along[1] = v;
        }

        float warp_scale = viewport_along[0] - lens_center[0];
        if(warp_scale < 0.0001f)
            warp_scale = (viewport_along[0] > lens_center[0]) ? (viewport_along[0] - lens_center[0]) : (lens_center[0] - viewport_along[0]);
        if(warp_scale < 0.0001f)
            warp_scale = 1.0f;

        float r[2];
        r[0] = fU * viewport_along[0] - lens_center[0];
        r[1] = fV * viewport_along[1] - lens_center[1];

        r[0] /= warp_scale;
        r[1] /= warp_scale;

        float r_mag = sqrt(r[0] * r[0] + r[1] * r[1]);


        float r_displaced[2];
        r_displaced[0] = r[0] * (distortion_coeffs[3] + distortion_coeffs[2] * r_mag + distortion_coeffs[1] * r_mag * r_mag + distortion_coeffs[0] * r_mag * r_mag * r_mag);

        r_displaced[1] = r[1] * (distortion_coeffs[3] + distortion_coeffs[2] * r_mag + distortion_coeffs[1] * r_mag * r_mag + distortion_coeffs[0] * r_mag * r_mag * r_mag);

        r_displaced[0] *= warp_scale;
        r_displaced[1] *= warp_scale;

        float tc_r[2];
        tc_r[0] = (lens_center[0] + aberr_scale[0] * r_displaced[0]) / viewport_along[0];
        tc_r[1] = (lens_center[1] + aberr_scale[0] * r_displaced[1]) / viewport_along[1];

        float tc_g[2];
        tc_g[0] = (lens_center[0] + aberr_scale[1] * r_displaced[0]) / viewport_along[0];
        tc_g[1] = (lens_center[1] + aberr_scale[1] * r_displaced[1]) / viewport_along[1];

        float tc_b[2];
        tc_b[0] = (lens_center[0] + aberr_scale[2] * r_displaced[0]) / viewport_along[0];
        tc_b[1] = (lens_center[1] + aberr_scale[2] * r_displaced[1]) / viewport_along[1];

        // OpenHMD has no lens calibration for the LGR100: it reports identity
        // distortion (0,0,0,1) and zero aberration. With both degenerate the maths
        // above collapses to the constant lens_center/viewport_scale, i.e. every
        // fragment samples the same texel and the eye renders as a solid block
        // (black). Pass the rotated coordinates through untouched in that case so
        // SteamVR renders the eye undistorted instead of sampling one point.
        // One-shot per eye: report the calibration actually received and whether the
        // passthrough path was taken. The passthrough depends on the aberration
        // coefficients being exactly zero; if they are not, the degenerate maths
        // above runs and can push this eye's sampling outside [0,1], which the
        // compositor renders as solid black.
        if(!m_bDistortLogged[(eEye == Eye_Left) ? 0 : 1])
        {
            m_bDistortLogged[(eEye == Eye_Left) ? 0 : 1] = true;
            bool degenerate = (distortion_coeffs[0] == 0.0f && distortion_coeffs[1] == 0.0f &&
                               distortion_coeffs[2] == 0.0f && distortion_coeffs[3] == 1.0f &&
                               aberr_scale[0] == 0.0f && aberr_scale[1] == 0.0f && aberr_scale[2] == 0.0f);
            DriverLog("CALIB %s: coeffs=(%f,%f,%f,%f) aberr=(%f,%f,%f) passthrough=%d out=(%.4f,%.4f)\n",
                      (eEye == Eye_Left) ? "LEFT " : "RIGHT",
                      distortion_coeffs[0], distortion_coeffs[1], distortion_coeffs[2], distortion_coeffs[3],
                      aberr_scale[0], aberr_scale[1], aberr_scale[2],
                      degenerate ? 1 : 0, tc_r[0], tc_r[1]);
        }

        // OpenHMD reports the LGR100's calibration as identity distortion (0,0,0,1) and
        // aberration (1,1,1). Combined, the maths above already reduces to
        // tc == fU,fV - an exact passthrough - but it is worth making that explicit
        // so the degenerate intermediate values (lens_center divided by
        // viewport_scale) can never be sampled. Note the aberration is ones, not
        // zeros: OpenHMD uses 1.0 to mean "no chromatic shift".
        if((distortion_coeffs[0] == 0.0f && distortion_coeffs[1] == 0.0f &&
            distortion_coeffs[2] == 0.0f && distortion_coeffs[3] == 1.0f &&
            aberr_scale[0] == 1.0f && aberr_scale[1] == 1.0f && aberr_scale[2] == 1.0f) ||
           (distortion_coeffs[0] == 0.0f && distortion_coeffs[1] == 0.0f &&
            distortion_coeffs[2] == 0.0f && distortion_coeffs[3] == 1.0f &&
            aberr_scale[0] == 0.0f && aberr_scale[1] == 0.0f && aberr_scale[2] == 0.0f))
        {
            tc_r[0] = fU;  tc_r[1] = fV;
            tc_g[0] = fU;  tc_g[1] = fV;
            tc_b[0] = fU;  tc_b[1] = fV;

            // Report the extreme texel coordinates this eye will request. Anything
            // outside [0,1] samples black, so this is the direct evidence for
            // whether a half of the panel can render at all.
            if(!m_bDistortLogged[(eEye == Eye_Left) ? 0 : 1])
            {
                m_bDistortLogged[(eEye == Eye_Left) ? 0 : 1] = true;
                DriverLog("distort %s rot=%.1f: u=[%.4f..%.4f] v=[%.4f..%.4f] lens=(%.4f,%.4f)\n",
                          (eEye == Eye_Left) ? "LEFT" : "RIGHT", angle,
                          0.0f, 1.0f, 0.0f, 1.0f, lens_center[0], lens_center[1]);
            }
        }

        //DistortionCoordinates_t coordinates;
        // One-shot diagnostic per eye: sample the distortion at the four corners of the
        // eye viewport on the first rendered frame. If the mapping is wrong (the
        // rotation branch swapping axes the lens maths then assumes are still
        // horizontal/vertical) some corner leaves [0,1] and that half renders black.
        if(!m_bDistortLogged[(eEye == Eye_Left) ? 0 : 1])
        {
            m_bDistortLogged[(eEye == Eye_Left) ? 0 : 1] = true;
            DriverLog("distort %s: rot=%.1f viewport_scale=(%.4f,%.4f) lens_center=(%.4f,%.4f)\n",
                      (eEye == Eye_Left) ? "LEFT" : "RIGHT", angle,
                      viewport_scale[0], viewport_scale[1],
                      lens_center[0], lens_center[1]);
        }

        DistortionCoordinates_t coordinates;
        // Clamp to the render target: anything outside [0,1] samples black, and
        // the small vertical centring below can push the top row slightly past
        // 1.0. Clamping turns a potential black stripe/eye into edge pixels.
        //
        // NOTE for the black-eye hunt: because of this clamp the driver can never
        // hand the compositor an out-of-range coordinate. That rules out "the
        // driver returned a bad texcoord" as the cause of the black left eye and
        // moves the suspicion to the input side (SteamVR asking for a degenerate
        // eye) or to the compositor (dropped submission / mirror window).
        // The counters below capture exactly that, per eye.
        {
            const int slot = (eEye == Eye_Left) ? 0 : 1;
            static DistortStats st[2];
            DistortStats &s = st[slot];

            s.calls++;
            const float tu = tc_r[0];
            const float tv = tc_r[1];

            if (tu < s.outMinU) s.outMinU = tu;
            if (tu > s.outMaxU) s.outMaxU = tu;
            if (tv < s.outMinV) s.outMinV = tv;
            if (tv > s.outMaxV) s.outMaxV = tv;

            if (inU < s.inMinU) s.inMinU = inU;
            if (inU > s.inMaxU) s.inMaxU = inU;
            if (inV < s.inMinV) s.inMinV = inV;
            if (inV > s.inMaxV) s.inMaxV = inV;

            // Out-of-range before the clamp: the vertical centring pushes the top
            // row slightly past 1.0, which is expected and harmless. Anything
            // beyond that is a real bug, so it is logged separately.
            const bool oor = (tu < -0.0001f || tu > 1.0001f || tv < -0.0001f || tv > 1.0001f);
            if (oor)
            {
                s.oor++;
                if (s.oor <= 20)
                {
                    DriverLog("EYEOOR %s n=%llu in=(%.4f,%.4f) out=(%.4f,%.4f) rot=%.1f\n",
                              (eEye == Eye_Left) ? "LEFT " : "RIGHT",
                              s.oor, inU, inV, tu, tv, angle);
                }
            }

            // One summary line per eye per second: the timeline in vrserver.txt
            // can then be correlated with the moment the eye actually goes black.
            const unsigned long long now = (unsigned long long)GetTickCount64();
            if (now - s.lastTick >= 1000)
            {
                s.lastTick = now;
                const unsigned long long perSec = s.calls - s.lastCalls;
                s.lastCalls = s.calls;
                DriverLog("EYESTATS %s calls=%llu/s inU=[%.3f..%.3f] inV=[%.3f..%.3f] outU=[%.3f..%.3f] outV=[%.3f..%.3f] oor=%llu\n",
                          (eEye == Eye_Left) ? "LEFT " : "RIGHT",
                          perSec,
                          s.inMinU, s.inMaxU, s.inMinV, s.inMaxV,
                          s.outMinU, s.outMaxU, s.outMinV, s.outMaxV,
                          s.oor);
                s.inMinU = 1e30f; s.inMaxU = -1e30f;
                s.inMinV = 1e30f; s.inMaxV = -1e30f;
                s.outMinU = 1e30f; s.outMaxU = -1e30f;
                s.outMinV = 1e30f; s.outMaxV = -1e30f;
            }
        }

        coordinates.rfBlue[0] = clamp01(tc_b[0]);
        coordinates.rfBlue[1] = clamp01(tc_b[1]);
        coordinates.rfGreen[0] = clamp01(tc_g[0]);
        coordinates.rfGreen[1] = clamp01(tc_g[1]);
        coordinates.rfRed[0] = clamp01(tc_r[0]);
        coordinates.rfRed[1] = clamp01(tc_r[1]);
        return coordinates;
    }

    static float clamp01(float v)
    {
        return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
    }

    DriverPose_t GetPose()
    {
        DriverPose_t pose = { 0 };
        pose.poseIsValid = true;
        pose.result = TrackingResult_Running_OK;
        pose.deviceIsConnected = true;

        float quat[4];
        m_device.getRotationQuat(quat);

        // Headset buttons. They arrive in HID report ID 2 and are parsed by the
        // LGR100 driver inside OpenHMD, which exposes them through
        // OHMD_CONTROLS_STATE as [OK, BACK]. Nothing read them until now, so
        // the physical buttons of the visor were dead as far as SteamVR was
        // concerned.
        float controls[2] = { 0.0f, 0.0f };
        m_device.getButtons(controls);

        // Proximity sensor (HID report ID 1): 1 when something is in front of
        // it, i.e. the headset is being worn. The visor uses the same sensor
        // to turn the panels off when it is not worn, and SteamVR uses it to
        // decide whether the headset is in use. Logged on every change so the
        // state can be correlated with the standby entries in the log.
        const bool nowPresent = m_device.proximityPresent();
        if (nowPresent != m_bProximityPresent)
        {
            m_bProximityPresent = nowPresent;
            DriverLog("PROXIMITY: %s\n", nowPresent ? "present (visor in use)" : "absent (nothing in front)");
        }

        const bool okDown = controls[0] > 0.5f;
        const bool backDown = controls[1] > 0.5f;

        if (okDown && !m_bButtonOkDown)
        {
            // OK: re-measure the gyro bias and make this direction the new
            // origin. The bias is captured again from the next ~200 samples
            // (about 1 s at 200 Hz), so the visor has to be held still for a
            // moment. This is the on-demand "stabilise" the automatic bias
            // tracking used to do continuously.
            m_device.recalibrateGyro();

            m_recenterRef[0] = quat[0];
            m_recenterRef[1] = quat[1];
            m_recenterRef[2] = quat[2];
            m_recenterRef[3] = quat[3];
            m_bHasRecenter = true;

            DriverLog("BUTTON OK: gyro recalibration requested + recentered to quat %.4f %.4f %.4f %.4f\n",
                      quat[0], quat[1], quat[2], quat[3]);
        }

        if (backDown && !m_bButtonBackDown)
        {
            // BACK: re-send the panel activation commands ("Sleep Disable" +
            // "VR App Start"). That is the software equivalent of the
            // activation sequence: it lights the panels up again when they went
            // to sleep, without unplugging the USB-C.
            const bool result = m_device.activatePanels();
            DriverLog("BUTTON BACK: panel activation commands re-sent (ok=%d)\n", (int)result);
        }

        m_bButtonOkDown = okDown;
        m_bButtonBackDown = backDown;

        float rquat[4];
        if (m_bHasRecenter)
        {
            // q' = conj(ref) * q. The orientation captured at the last recenter
            // becomes "forward" again, so the yaw that drifted since then is
            // cancelled while every movement after it is preserved.
            applyRecenter(m_recenterRef, quat, rquat);
        }
        else
        {
            memcpy(rquat, quat, sizeof(rquat));
        }

        // Report the angular velocity of the head.
        //
        // The pose was always posted with a zero velocity, which makes the
        // device look permanently still: SteamVR decides whether the headset
        // is in use from the reported motion, and a device that never moves
        // gets parked in standby (the mirror window then goes black). The
        // velocity is derived from the rotation between the last two poses.
        {
            const unsigned long long now = (unsigned long long)GetTickCount64();
            float dt = (float)(now - m_lastPoseTick) / 1000.0f;
            if (dt <= 0.0f || dt > 1.0f)
            {
                dt = 1.0f / 90.0f;   // first sample or a long stall: assume 90 Hz
            }
            computeAngularVelocity(m_prevQuat, quat, dt, m_angularVelocity);
            memcpy(m_prevQuat, quat, sizeof(m_prevQuat));
            m_lastPoseTick = now;

            if (now - m_lastVelLogTick >= 2000)
            {
                m_lastVelLogTick = now;
                const float mag = sqrtf(m_angularVelocity[0] * m_angularVelocity[0] +
                                        m_angularVelocity[1] * m_angularVelocity[1] +
                                        m_angularVelocity[2] * m_angularVelocity[2]);
                DriverLog("VELOCITY |w|=%.4f rad/s (%.2f deg/s) w=(%.3f, %.3f, %.3f) dt=%.4f\n",
                          mag, mag * 57.2958f,
                          m_angularVelocity[0], m_angularVelocity[1], m_angularVelocity[2], dt);
            }
        }

        pose.vecAngularVelocity[0] = m_angularVelocity[0];
        pose.vecAngularVelocity[1] = m_angularVelocity[1];
        pose.vecAngularVelocity[2] = m_angularVelocity[2];

        pose.qRotation.x = rquat[0];
        pose.qRotation.y = rquat[1];
        pose.qRotation.z = rquat[2];
        pose.qRotation.w = rquat[3];

        float pos[3];
        m_device.getPosition(pos);
        pose.vecPosition[0] = pos[0];
        pose.vecPosition[1] = pos[1];
        pose.vecPosition[2] = pos[2];

        //printf("%f %f %f %f  %f %f %f\n", quat[0], quat[1], quat[2], quat[3], pos[0], pos[1], pos[2]);
        //fflush(stdout);
        //DriverLog("get hmd pose %f %f %f %f, %f %f %f\n", quat[0], quat[1], quat[2], quat[3], pos[0], pos[1], pos[2]);

        pose.qWorldFromDriverRotation = identityquat;
        pose.qDriverFromHeadRotation = identityquat;

        return pose;
    }


    void RunFrame()
    {
        // In a real driver, this should happen from some pose tracking thread.
        // The RunFrame interval is unspecified and can be very irregular if some other
        // driver blocks it for some periodic task.
        if ( m_unObjectId != vr::k_unTrackedDeviceIndexInvalid )
        {
            vr::VRServerDriverHost()->TrackedDevicePoseUpdated( m_unObjectId, GetPose(), sizeof( DriverPose_t ) );
        }
    }

    // Rotate a quaternion by the inverse of a reference orientation:
    //   out = conj(ref) * q
    // ref and q are (x, y, z, w). The reference is the orientation captured
    // the last time the user recentred, so the view keeps facing the same way
    // as it did at that moment.
    static void applyRecenter(const float ref[4], const float q[4], float out[4])
    {
        const float ax = -ref[0], ay = -ref[1], az = -ref[2], aw = ref[3];
        const float bx = q[0], by = q[1], bz = q[2], bw = q[3];

        out[0] = aw * bx + ax * bw + ay * bz - az * by;
        out[1] = aw * by + ay * bw + az * bx - ax * bz;
        out[2] = aw * bz + az * bw + ax * by - ay * bx;
        out[3] = aw * bw - ax * bx - ay * by - az * bz;
    }

    // Angular velocity (rad/s) from the rotation between two consecutive poses:
    //   dq = cur * conj(prev),   omega = axis * angle / dt
    static void computeAngularVelocity(const float prev[4], const float cur[4], float dt, float out[3])
    {
        const float px = -prev[0], py = -prev[1], pz = -prev[2], pw = prev[3];
        const float cx = cur[0], cy = cur[1], cz = cur[2], cw = cur[3];

        const float dx = cw * px + cx * pw + cy * pz - cz * py;
        const float dy = cw * py + cy * pw + cz * px - cx * pz;
        const float dz = cw * pz + cz * pw + cx * py - cy * px;
        const float dw = cw * pw - cx * px - cy * py - cz * pz;

        const float s = sqrtf(dx * dx + dy * dy + dz * dz);
        if (s < 1e-8f || dt <= 0.0f)
        {
            out[0] = out[1] = out[2] = 0.0f;
            return;
        }

        const float angle = 2.0f * atan2f(s, dw);
        const float k = angle / (s * dt);
        out[0] = dx * k;
        out[1] = dy * k;
        out[2] = dz * k;
    }

    std::string GetSerialNumber() const { return m_sSerialNumber; }

private:
    Lgr100Device m_device;

    vr::TrackedDeviceIndex_t m_unObjectId;
    vr::PropertyContainerHandle_t m_ulPropertyContainer;

    std::string m_sVendor;
    std::string m_sSerialNumber;
    std::string m_sModelNumber;

    int32_t m_nWindowX;
    int32_t m_nWindowY;
    int32_t m_nWindowWidth;
    int32_t m_nWindowHeight;
    int32_t m_nRenderWidth;
    int32_t m_nRenderHeight;
    float m_flSecondsFromVsyncToPhotons;
    float m_flDisplayFrequency;
    float m_flIPD;
    
    float m_flRotationLeft = 0.0f;
    float m_flRotationRight = 0.0f;
    float m_flProjectionLeft[16];
    float m_flProjectionRight[16];
    float m_flViewportScale[2];
    float m_flLensCenterLeft[2];
    float m_flLensCenterRight[2];
    float m_flDistortionCoeffs[4];
    float m_flAberration[3];
    bool m_bDistortLogged[2] = {false, false};

    // Headset button state (OK / BACK) and the recenter reference.
    bool m_bButtonOkDown = false;
    bool m_bButtonBackDown = false;
    bool m_bProximityPresent = false;
    bool m_bHasRecenter = false;
    float m_recenterRef[4] = { 0.0f, 0.0f, 0.0f, 1.0f };

    // Previous pose and clock, used to derive the angular velocity.
    float m_prevQuat[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
    float m_angularVelocity[3] = { 0.0f, 0.0f, 0.0f };
    unsigned long long m_lastPoseTick = 0;
    unsigned long long m_lastVelLogTick = 0;
};

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CServerDriver_OpenHMD: public IServerTrackedDeviceProvider
{
public:
    CServerDriver_OpenHMD()
        : m_OpenHMDDeviceDriver( NULL )
    {
    }
    virtual ~CServerDriver_OpenHMD() {}

    virtual EVRInitError Init( vr::IVRDriverContext *pDriverContext ) ;
    virtual void Cleanup() ;
    virtual const char * const *GetInterfaceVersions() { return vr::k_InterfaceVersions; }
    virtual void RunFrame() ;

    // The LGR100 is presented as a *desktop mirror window* on the panel, not as a
    // direct-mode HMD. When SteamVR puts the device in standby it stops
    // rendering the mirror window altogether, and the panel goes **completely
    // black** while the visor is still perfectly connected and tracking.
    //
    // The log showed this happening over and over (several times per minute),
    // which is exactly the "intermittent black" the project has been chasing:
    //
    //   20:47:09  0 - entering standby
    //   20:52:38  0 - leaving standby
    //   20:53:00  0 - entering standby
    //   ...
    //
    // The visor has no real power management to do (the panels are kept alive by
    // the HID "Sleep Disable" command), so there is nothing to gain from letting
    // SteamVR idle it. Blocking standby keeps the compositor presenting.
    virtual bool ShouldBlockStandbyMode()  { return true; }
    virtual void EnterStandby()  {}
    virtual void LeaveStandby()  {}

private:
    COpenHMDDeviceDriver *m_OpenHMDDeviceDriver;
    COpenHMDDeviceDriverController *m_OpenHMDDeviceDriverControllerL;
    COpenHMDDeviceDriverController *m_OpenHMDDeviceDriverControllerR;
};

CServerDriver_OpenHMD g_serverDriverOpenHMD;

EVRInitError CServerDriver_OpenHMD::Init( vr::IVRDriverContext *pDriverContext )
{
    VR_INIT_SERVER_DRIVER_CONTEXT( pDriverContext );
    InitDriverLog( vr::VRDriverLog() );

    // The headset is opened by the driver itself (hidapi + sensor fusion).
    // The two controllers are null devices, exactly as OpenHMD's dummy driver
    // provided them.
    m_OpenHMDDeviceDriver = new COpenHMDDeviceDriver();

    // Adding the HMD device makes SteamVR start the compositor, which needs the
    // LG panel to already be part of the display topology. On a cold start the
    // panel can take a few seconds to enumerate; without this wait the
    // compositor fails with "Invalid adapter output" and the first launch
    // reports that no headset was detected. Bounded, well inside the loader
    // watchdog.
    for(int wait = 0; wait < 20 && !headsetDisplayPresent(); wait++)
    {
        DriverLog("openhmd init: headset display not present yet, waiting (%d/20)\n", wait + 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    vr::VRServerDriverHost()->TrackedDeviceAdded( m_OpenHMDDeviceDriver->GetSerialNumber().c_str(), vr::TrackedDeviceClass_HMD, m_OpenHMDDeviceDriver );
    DriverLog("openhmd init: HMD tracked device added\n");

    DriverLog("openhmd init: opening left controller\n");
    m_OpenHMDDeviceDriverControllerL = new COpenHMDDeviceDriverController(0, true);
    vr::VRServerDriverHost()->TrackedDeviceAdded( m_OpenHMDDeviceDriverControllerL->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_OpenHMDDeviceDriverControllerL );

    DriverLog("openhmd init: opening right controller\n");
    m_OpenHMDDeviceDriverControllerR = new COpenHMDDeviceDriverController(1, false);
    vr::VRServerDriverHost()->TrackedDeviceAdded(  m_OpenHMDDeviceDriverControllerR->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_OpenHMDDeviceDriverControllerR );

    DriverLog("openhmd init: all tracked devices added\n");
    return VRInitError_None;
}

void CServerDriver_OpenHMD::Cleanup()
{
    CleanupDriverLog();
    delete m_OpenHMDDeviceDriver;
    m_OpenHMDDeviceDriver = NULL;

    if (m_OpenHMDDeviceDriverControllerL)
      delete m_OpenHMDDeviceDriverControllerL;
    if (m_OpenHMDDeviceDriverControllerR)
      delete m_OpenHMDDeviceDriverControllerR;
}


void CServerDriver_OpenHMD::RunFrame()
{
    if ( m_OpenHMDDeviceDriver )
        m_OpenHMDDeviceDriver->RunFrame();

    if (m_OpenHMDDeviceDriverControllerL)
        m_OpenHMDDeviceDriverControllerL->RunFrame();
    if (m_OpenHMDDeviceDriverControllerR)
        m_OpenHMDDeviceDriverControllerR->RunFrame();
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
HMD_DLL_EXPORT void *HmdDriverFactory( const char *pInterfaceName, int *pReturnCode )
{
    if( 0 == strcmp( IServerTrackedDeviceProvider_Version, pInterfaceName ) )
    {
        return &g_serverDriverOpenHMD;
    }
    if( 0 == strcmp( IVRWatchdogProvider_Version, pInterfaceName ) )
    {
        return &g_watchdogDriverOpenHMD;
    }

    DriverLog("no interface %s\n", pInterfaceName);

    if( pReturnCode )
        *pReturnCode = VRInitError_Init_InterfaceNotFound;

    return NULL;
}


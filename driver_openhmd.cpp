//============ Copyright (c) Valve Corporation, All rights reserved. ============

#include "ohmd_config.h" // which hmds and trackers to use

#include <openvr_driver.h>
#include "driverlog.h"

#include <assert.h>

#include <vector>
#include <thread>
#include <chrono>
#include <cstring>
#include <sstream>
#include <atomic>

#if defined( _WINDOWS )
#include <windows.h>
#endif

#include <openhmd.h>
#include "riftd_shm.h"

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <math.h>
#include <stdio.h>

using namespace vr;


#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C" 
#else
#error "Unsupported Platform."
#endif

ohmd_context* ctx;
// rotation-only devices report position 0; lift the tracking origin so the head is not at floor level
float g_flHeadHeight = 1.6f;

// driver_openhmd.backend = "riftd": poses, inputs and display parameters come from the riftd
// shared memory (riftd_shm.h) instead of OpenHMD in this process.
bool g_riftd = false;
riftd_shm g_snap;
std::atomic<bool> g_snapFresh(false);
// guards g_snap: written by the server RunFrame, read by device GetPose and RunFrame, which can run on other threads
#if defined( _WINDOWS )
SRWLOCK g_snapLock = SRWLOCK_INIT;
#else
#define AcquireSRWLockShared(l)
#define ReleaseSRWLockShared(l)
#define AcquireSRWLockExclusive(l)
#define ReleaseSRWLockExclusive(l)
int g_snapLock;
#endif

static riftd_device SnapDevice(int idx)
{
    AcquireSRWLockShared(&g_snapLock);
    riftd_device d = g_snap.devices[idx];
    ReleaseSRWLockShared(&g_snapLock);
    return d;
}

#if defined( _WINDOWS )
class RiftdClient {
public:
    RiftdClient()
    {
        LARGE_INTEGER f;
        QueryPerformanceFrequency(&f);
        m_freq = f.QuadPart;
    }

    bool Open(int64_t deadline)
    {
        for (;;) {
            if (Ready())
                return true;
            if (Now() >= deadline)
                return false;
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
    }

    bool Ready()
    {
        if (!m_shm) {
            m_map = OpenFileMappingA(FILE_MAP_READ, FALSE, RIFTD_SHM_NAME);
            if (m_map)
                m_shm = (const riftd_shm *)MapViewOfFile(m_map, FILE_MAP_READ, 0, 0, sizeof(riftd_shm));
            if (m_map && !m_shm) {
                CloseHandle(m_map);
                m_map = NULL;
            }
        }
        static riftd_shm first;
        if (!(m_shm && Header(m_shm) && m_shm->state == RIFTD_STATE_READY && Snapshot(&first)))
            return false;
        AcquireSRWLockExclusive(&g_snapLock);
        g_snap = first;
        ReleaseSRWLockExclusive(&g_snapLock);
        return true;
    }

    static bool Header(const riftd_shm *s)
    {
        return s->magic == RIFTD_SHM_MAGIC && s->version == RIFTD_SHM_VERSION && s->size == sizeof(riftd_shm);
    }

    bool Snapshot(riftd_shm *out)
    {
        for (int tries = 0; tries < 1000; tries++) {
            uint32_t seq = m_shm->seq;
            if (seq & 1) {
                YieldProcessor();
                continue;
            }
            MemoryBarrier();
            memcpy(out, (const void *)m_shm, sizeof(*out));
            MemoryBarrier();
            if (m_shm->seq == seq)
                return true;
        }
        return false;
    }

    // Snapshot plus a check that riftd is still writing. A read that keeps colliding with the
    // writer (riftd preempted mid-write) keeps the previous snapshot.
    bool Update()
    {
        static riftd_shm next;
        if (!m_shm) {
            // riftd was not running at Init; look for it once a second
            if (Now() - m_lastOpenTry < m_freq)
                return false;
            m_lastOpenTry = Now();
            if (!Ready())
                return false;
        }
        if (Snapshot(&next)) {
            // a stopped riftd or a different shm layout counts as riftd gone
            if (!Header(&next))
                return false;
            AcquireSRWLockExclusive(&g_snapLock);
            g_snap = next;
            ReleaseSRWLockExclusive(&g_snapLock);
        }
        return g_snap.state == RIFTD_STATE_READY && Now() - g_snap.heartbeat_qpc < m_freq / 2;
    }

    int64_t Now()
    {
        LARGE_INTEGER t;
        QueryPerformanceCounter(&t);
        return t.QuadPart;
    }

    double SampleAge(const riftd_device &d) { return (double)(Now() - d.sample_qpc) / m_freq; }

    int64_t Freq() { return m_freq; }

private:
    HANDLE m_map = NULL;
    const riftd_shm *m_shm = NULL;
    int64_t m_freq = 1;
    int64_t m_lastOpenTry = 0;
};
RiftdClient g_riftdClient;

// Last ready riftd state (display and device descriptions), so vrserver can add the devices when it
// starts before riftd: %LOCALAPPDATA%\openhmd\riftd-devices.bin
static std::string RiftdCachePath()
{
    char dir[MAX_PATH];
    DWORD n = GetEnvironmentVariableA("LOCALAPPDATA", dir, sizeof(dir));
    if (n == 0 || n >= sizeof(dir))
        return "";
    std::string path = std::string(dir) + "\\openhmd";
    CreateDirectoryA(path.c_str(), NULL);
    return path + "\\riftd-devices.bin";
}

static void SaveRiftdCache(const riftd_shm &s)
{
    std::string path = RiftdCachePath();
    FILE *f = path.empty() ? NULL : fopen(path.c_str(), "wb");
    if (!f)
        return;
    fwrite(&s, sizeof(s), 1, f);
    fclose(f);
}

static bool LoadRiftdCache(riftd_shm *s)
{
    std::string path = RiftdCachePath();
    FILE *f = path.empty() ? NULL : fopen(path.c_str(), "rb");
    if (!f)
        return false;
    bool ok = fread(s, sizeof(*s), 1, f) == 1 && s->magic == RIFTD_SHM_MAGIC && s->version == RIFTD_SHM_VERSION &&
        s->size == sizeof(*s) && s->display.h_resolution > 0;
    fclose(f);
    return ok;
}
#endif

static int64_t QpcNow()
{
#if defined( _WINDOWS )
    LARGE_INTEGER t;
    QueryPerformanceCounter(&t);
    return t.QuadPart;
#else
    return 0;
#endif
}

// Driver Init must return well within vrserver's watchdog (about 20 s) or SteamVR enters safe mode;
// waits for riftd and for the HMD display end at this deadline.
int64_t g_initDeadline = 0;

static void FillDisplayFromOpenHMD(ohmd_device *hmd, riftd_display *d)
{
    ohmd_device_geti(hmd, OHMD_SCREEN_HORIZONTAL_RESOLUTION, &d->h_resolution);
    ohmd_device_geti(hmd, OHMD_SCREEN_VERTICAL_RESOLUTION, &d->v_resolution);
    ohmd_device_getf(hmd, OHMD_SCREEN_HORIZONTAL_SIZE, &d->h_size);
    ohmd_device_getf(hmd, OHMD_SCREEN_VERTICAL_SIZE, &d->v_size);
    ohmd_device_getf(hmd, OHMD_LENS_HORIZONTAL_SEPARATION, &d->lens_separation);
    ohmd_device_getf(hmd, OHMD_LENS_VERTICAL_POSITION, &d->lens_v_position);
    ohmd_device_getf(hmd, OHMD_EYE_IPD, &d->ipd);
    ohmd_device_getf(hmd, OHMD_UNIVERSAL_DISTORTION_K, d->universal_distortion_k);
    ohmd_device_getf(hmd, OHMD_UNIVERSAL_ABERRATION_K, d->universal_aberration_k);
    ohmd_device_getf(hmd, OHMD_LEFT_EYE_GL_PROJECTION_MATRIX, d->left_projection);
    ohmd_device_getf(hmd, OHMD_RIGHT_EYE_GL_PROJECTION_MATRIX, d->right_projection);
}

static void FillDeviceFromOpenHMD(int idx, ohmd_device *dev, riftd_device *d)
{
    int hints[64] = { 0 }, types[64] = { 0 }, n = 0;
    strncpy(d->vendor, ohmd_list_gets(ctx, idx, OHMD_VENDOR), sizeof(d->vendor) - 1);
    strncpy(d->product, ohmd_list_gets(ctx, idx, OHMD_PRODUCT), sizeof(d->product) - 1);
    strncpy(d->path, ohmd_list_gets(ctx, idx, OHMD_PATH), sizeof(d->path) - 1);
    ohmd_list_geti(ctx, idx, OHMD_DEVICE_FLAGS, &d->ohmd_flags);
    ohmd_device_geti(dev, OHMD_CONTROL_COUNT, &n);
    if (n > RIFTD_MAX_CONTROLS)
        n = RIFTD_MAX_CONTROLS;
    ohmd_device_geti(dev, OHMD_CONTROLS_HINTS, hints);
    ohmd_device_geti(dev, OHMD_CONTROLS_TYPES, types);
    d->control_count = n;
    memcpy(d->control_hints, hints, n * sizeof(int));
    memcpy(d->control_types, types, n * sizeof(int));
}

// The compositor extrapolates every pose it renders, so one non-finite value reaches its view matrices.
// Normalizes the rotation and drops implausible velocities; false if rotation or position is unusable.
static bool SanitizePose(DriverPose_t &pose)
{
    HmdQuaternion_t &q = pose.qRotation;
    double n = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (!(n > 0.5 && n < 2.0))
        return false;
    q.w /= n; q.x /= n; q.y /= n; q.z /= n;
    bool vel_ok = true;
    for (int i = 0; i < 3; i++) {
        if (!isfinite(pose.vecPosition[i]))
            return false;
        vel_ok = vel_ok && fabs(pose.vecVelocity[i]) < 20.0 && fabs(pose.vecAngularVelocity[i]) < 60.0;
    }
    if (!vel_ok)
        for (int i = 0; i < 3; i++)
            pose.vecVelocity[i] = pose.vecAngularVelocity[i] = 0.0;
    return true;
}

// Fills the tracking fields of pose from a riftd device. Returns true when it has an optical fix.
static bool RiftdPose(int idx, DriverPose_t &pose)
{
    static DriverPose_t lastGood[RIFTD_NUM_DEVICES];
    static bool haveGood[RIFTD_NUM_DEVICES];
    const riftd_device d = SnapDevice(idx);
    bool is_hmd = idx == RIFTD_DEV_HMD;
    bool valid = g_snapFresh && (d.flags & RIFTD_DEVICE_POSE_VALID);
    bool optical = valid && (d.flags & RIFTD_DEVICE_OPTICAL);

    pose.deviceIsConnected = g_snapFresh && (d.flags & RIFTD_DEVICE_CONNECTED);
    pose.poseIsValid = valid;
    // IMU-only head rotation stays Running_OK: OutOfRange on the HMD fades the view to the grid
    if (!valid)
        pose.result = TrackingResult_Running_OutOfRange;
    else if (optical || is_hmd)
        pose.result = TrackingResult_Running_OK;
    else
        pose.result = TrackingResult_Running_OutOfRange;

    pose.qRotation.x = d.orientation[0];
    pose.qRotation.y = d.orientation[1];
    pose.qRotation.z = d.orientation[2];
    pose.qRotation.w = d.orientation[3];
    for (int i = 0; i < 3; i++) {
        pose.vecPosition[i] = d.position[i];
        bool vel = valid && (d.flags & RIFTD_DEVICE_VELOCITY);
        pose.vecVelocity[i] = vel ? d.linear_velocity[i] : 0.0;
        pose.vecAngularVelocity[i] = vel ? d.angular_velocity[i] : 0.0;
        pose.vecAcceleration[i] = 0.0;
        pose.vecAngularAcceleration[i] = 0.0;
    }
    if (pose.qRotation.w == 0 && pose.qRotation.x == 0 && pose.qRotation.y == 0 && pose.qRotation.z == 0)
        pose.qRotation.w = 1;
#if defined( _WINDOWS )
    double age = g_riftdClient.SampleAge(d);
    pose.poseTimeOffset = age > 0.1 ? -0.1 : (age < 0 ? 0 : -age);
#endif
    if (SanitizePose(pose)) {
        lastGood[idx] = pose;
        haveGood[idx] = true;
        return optical;
    }
    static std::atomic<unsigned> bad(0);
    if (bad++ % 1000 == 0)
        DriverLog("driver_openhmd: device %d non-finite pose from riftd, holding last good pose (%u)\n", idx, bad.load());
    bool connected = pose.deviceIsConnected;
    if (haveGood[idx])
        pose = lastGood[idx];
    else {
        pose.qRotation = { 1, 0, 0, 0 };
        for (int i = 0; i < 3; i++)
            pose.vecPosition[i] = 0.0;
    }
    for (int i = 0; i < 3; i++)
        pose.vecVelocity[i] = pose.vecAngularVelocity[i] = 0.0;
    pose.deviceIsConnected = connected;
    pose.poseIsValid = haveGood[idx];
    pose.result = haveGood[idx] && is_hmd ? TrackingResult_Running_OK : TrackingResult_Running_OutOfRange;
    return false;
}

class COpenHMDDeviceDriverController;


// gets float values from the device and prints them
void print_infof(ohmd_device* hmd, const char* name, int len, ohmd_float_value val)
{
    float f[20];
    assert (len <= 20);

    ohmd_device_getf(hmd, val, f);
    printf("%-25s", name);
    for(int i = 0; i < len; i++)
        printf("%f ", f[i]);
    printf("\n");
}

// gets int values from the device and prints them
void print_infoi(ohmd_device* hmd, const char* name, int len, ohmd_int_value val)
{
    int iv[20];
    assert (len <= 20);

    ohmd_device_geti(hmd, val, iv);
    printf("%-25s", name);
    for(int i = 0; i < len; i++)
        printf("%d ", iv[i]);
    printf("\n");
}

// keys for use with the settings API
static const char * const k_pch_Sample_Section = "driver_openhmd";
static const char * const k_pch_Sample_SecondsFromVsyncToPhotons_Float = "secondsFromVsyncToPhotons";
static const char * const k_pch_Sample_DisplayFrequency_Float = "displayFrequency";

HmdQuaternion_t identityquat{ 1, 0, 0, 0};

// riftd Touch grip offset, right controller frame (+x right, +y up, -z forward); the left controller mirrors
// x, yaw and roll. Defaults fit riftd's Touch LED model onto SteamVR's oculus_cv1_controller render model.
struct TouchOffset { double t[3]; double pitch, yaw, roll; };
static TouchOffset g_touchOffset = { { 0.007, 0.031, -0.07 }, 39.4, 0, 0 };

static float SettingFloat(const char *key, float def, float limit)
{
    vr::EVRSettingsError err = vr::VRSettingsError_None;
    float v = vr::VRSettings()->GetFloat(k_pch_Sample_Section, key, &err);
    return err == vr::VRSettingsError_None && fabsf(v) <= limit ? v : def;
}

static void ReadTouchOffset()
{
    TouchOffset o;
    o.t[0] = SettingFloat("touchOffsetX", 0.007f, 0.5f);
    o.t[1] = SettingFloat("touchOffsetY", 0.031f, 0.5f);
    o.t[2] = SettingFloat("touchOffsetZ", -0.07f, 0.5f);
    o.pitch = SettingFloat("touchPitch", 39.4f, 360.0f);
    o.yaw = SettingFloat("touchYaw", 0.0f, 360.0f);
    o.roll = SettingFloat("touchRoll", 0.0f, 360.0f);
    if (memcmp(&o, &g_touchOffset, sizeof(o)) != 0)
        DriverLog("driver_openhmd: touch offset %.3f %.3f %.3f m, pitch %.1f yaw %.1f roll %.1f deg\n",
                  o.t[0], o.t[1], o.t[2], o.pitch, o.yaw, o.roll);
    AcquireSRWLockExclusive(&g_snapLock);
    g_touchOffset = o;
    ReleaseSRWLockExclusive(&g_snapLock);
}

static HmdQuaternion_t AxisAngle(double x, double y, double z, double deg)
{
    double h = deg * M_PI / 360, s = sin(h);
    return { cos(h), x * s, y * s, z * s };
}

static HmdQuaternion_t QMul(const HmdQuaternion_t &a, const HmdQuaternion_t &b)
{
    return { a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
             a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
             a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
             a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w };
}
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

class COpenHMDDeviceDriverController : public vr::ITrackedDeviceServerDriver /*, public vr::IVRControllerComponent */ {
public:
    int index;
    ohmd_device* device;
    int device_idx;
    int device_flags;
    DriverPose_t pose;

    bool m_is_oculus = false;
    // static device info; with the riftd backend device is NULL and device_idx is a riftd_device_index
    riftd_device m_info = {};
    bool m_hadOptical = false;
    double m_defaultPosition[3] = { 0, 0, 0 };

    COpenHMDDeviceDriverController(int index, ohmd_device* _device, int _device_idx) :
		index(index), device(_device), device_idx(_device_idx) {
        DriverLog("construct controller object %d (%s device %d)\n", index, g_riftd ? "riftd" : "OpenHMD", device_idx);
        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
        pose = { 0 };

        if (g_riftd)
            m_info = g_snap.devices[device_idx];
        else
            FillDeviceFromOpenHMD(device_idx, device, &m_info);

        if (strcmp(m_info.vendor, "Oculus VR, Inc.") == 0) {
            m_is_oculus = true;
            DriverLog("detected oculus controllers, using oculus input profile");
        }
    }
    virtual ~COpenHMDDeviceDriverController() {}

    EVRInitError Activate( vr::TrackedDeviceIndex_t unObjectId )
    {
        DriverLog("activate controller %d: %d\n", index, unObjectId);
        m_unObjectId = unObjectId;

        if (this->device == NULL && !g_riftd) {
          return VRInitError_Init_InterfaceNotFound;
        }

        m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer( m_unObjectId );

        const char *controllerModel = m_info.product;

	device_flags = m_info.ohmd_flags;

        vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_ModelNumber_String, controllerModel);
        //vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_RenderModelName_String, controllerModel);
	//vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_ModelNumber_String, "1" );
        if (m_is_oculus) {
            // steamapps/common/SteamVR/drivers/oculus/resources/input/touch_profile.json -> "controller_type": "oculus_touch"
            vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_ControllerType_String, "oculus_touch" );
            vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_InputProfilePath_String, "{oculus}/input/touch_profile.json" );

            if (device_flags & OHMD_DEVICE_FLAGS_LEFT_CONTROLLER) {
                // steamapps/common/SteamVR/resources/rendermodels/oculus_cv1_controller_left
                vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_cv1_controller_left");
            } else if (device_flags & OHMD_DEVICE_FLAGS_RIGHT_CONTROLLER) {
                // steamapps/common/SteamVR/resources/rendermodels/oculus_cv1_controller_right
                vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_cv1_controller_right");
            }
        } else {
            vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_ControllerType_String, "openhmd_controller" );
            vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_InputProfilePath_String, "{openhmd}/input/openhmd_controller_profile.json" );
            vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_RenderModelName_String, "vr_controller_vive_1_5");
        }


        // return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
        vr::VRProperties()->SetUint64Property( m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2 );

        vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_DeviceClass_Int32, vr::TrackedDeviceClass_Controller);

        // avoid "not fullscreen" warnings from vrmonitor
        vr::VRProperties()->SetBoolProperty( m_ulPropertyContainer, Prop_IsOnDesktop_Bool, false );

	if (device_flags & OHMD_DEVICE_FLAGS_LEFT_CONTROLLER) {
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
	for (int i = 0; i < 3; i++)
	    m_defaultPosition[i] = pose.vecPosition[i];


        int control_count = m_info.control_count;

        const char* controls_fn_str[] = { "generic", "trigger", "trigger_click", "squeeze", "menu", "home",
                "analog-x", "analog-y", "anlog_press", "button-a", "button-b", "button-x", "button-y",
                "volume-up", "volume-down", "mic-mute"};
        const char* controls_type_str[] = {"digital", "analog"};

        const int32_t *controls_fn = m_info.control_hints;
        const int32_t *controls_types = m_info.control_types;

        for(int i = 0; i < control_count; i++){
          DriverLog("%s (%s)%s\n", controls_fn_str[controls_fn[i]], controls_type_str[controls_types[i]], i == control_count - 1 ? "" : ", ");
          const char *control_map = NULL, *touch_map = NULL;
          EVRScalarUnits analog_type = VRScalarUnits_NormalizedOneSided;

          m_buttons[i] = k_ulInvalidInputComponentHandle;
          m_analogControls[i] = k_ulInvalidInputComponentHandle;
          m_touchControls[i] = k_ulInvalidInputComponentHandle;
          m_touchCap[i] = -1;

          // TODO: inputs match steamapps/common/SteamVR/drivers/oculus/resources/input/touch_profile.json
          // but also support other controllers
          switch (controls_fn[i]) {
            case OHMD_GENERIC:
              control_map = "/input/generic/click";
              break;
            case OHMD_TRIGGER:
              control_map = "/input/trigger/value";
              break;
            case OHMD_TRIGGER_CLICK:
              control_map = "/input/trigger/click";
              break;
            case OHMD_SQUEEZE:
              control_map = "/input/grip/value";
              break;
            case OHMD_MENU:
              control_map = "/input/system/click";
              break;
            case OHMD_HOME:
                // TODO: this button doesn't have an input in touch_profile.json
              control_map = "/input/system/click";
              break;
            case OHMD_ANALOG_X:
              control_map = "/input/joystick/x";
              analog_type = VRScalarUnits_NormalizedTwoSided;
              break;
            case OHMD_ANALOG_Y:
              control_map = "/input/joystick/y";
              analog_type = VRScalarUnits_NormalizedTwoSided;
              break;
            case OHMD_ANALOG_PRESS:
              control_map = "/input/joystick/click";
              break;
            case OHMD_BUTTON_A:
              control_map = "/input/a/click";
              touch_map = "/input/a/touch";
              m_touchCap[i] = 0;
              break;
            case OHMD_BUTTON_B:
              control_map = "/input/b/click";
              touch_map = "/input/b/touch";
              m_touchCap[i] = 1;
              break;
            case OHMD_BUTTON_X:
              control_map = "/input/x/click";
              touch_map = "/input/x/touch";
              m_touchCap[i] = 0;
              break;
            case OHMD_BUTTON_Y:
              control_map = "/input/y/click";
              touch_map = "/input/y/touch";
              m_touchCap[i] = 1;
              break;

            default:
              break;
          }

          /* We fall through here for generic buttons */
          if (control_map != NULL) {
            if (controls_types[i] == OHMD_DIGITAL) {
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

        if (g_riftd && m_is_oculus) {
          const char *extra[3] = { "/input/thumbrest/touch", "/input/joystick/touch", "/input/trigger/touch" };
          for (int k = 0; k < 3; k++)
            vr::VRDriverInput()->CreateScalarComponent( m_ulPropertyContainer, extra[k], m_extraTouch + k, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
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
    void DebugRequest( const char *, char *pchResponseBuffer, uint32_t unResponseBufferSize )
    {
        if( unResponseBufferSize >= 1 )
            pchResponseBuffer[0] = 0;
    }

    DriverPose_t GetPose()
    {
	if (g_riftd) {
		if (RiftdPose(device_idx, pose))
			m_hadOptical = true;
		// until the cameras have seen it, hold the controller at the rotation-only default spot
		if (!m_hadOptical)
			for (int i = 0; i < 3; i++)
				pose.vecPosition[i] = m_defaultPosition[i];
	} else {
	pose.poseIsValid = true;
	pose.result = TrackingResult_Running_OK;
	pose.deviceIsConnected = true;
	}

	if (!g_riftd && (device_flags & OHMD_DEVICE_FLAGS_ROTATIONAL_TRACKING)) {
		float quat[4];
		ohmd_device_getf(device, OHMD_ROTATION_QUAT, quat);
		pose.qRotation.x = quat[0];
		pose.qRotation.y = quat[1];
		pose.qRotation.z = quat[2];
		pose.qRotation.w = quat[3];
	}

	if (!g_riftd && (device_flags & OHMD_DEVICE_FLAGS_POSITIONAL_TRACKING)) {
		float pos[3];
		ohmd_device_getf(device, OHMD_POSITION_VECTOR, pos);
		pose.vecPosition[0] = pos[0];
		pose.vecPosition[1] = pos[1];
		pose.vecPosition[2] = pos[2];
	}

	// DriverLog("get controller %d pose %f %f %f %f, %f %f %f\n", index, quat[0], quat[1], quat[2], quat[3], pos[0], pos[1], pos[2]);

	for (int i = 0; i < 3; i++) {
		pose.vecDriverFromHeadTranslation[i] = 0.0;
		pose.vecWorldFromDriverTranslation[i] = 0.0;
	}
	pose.vecWorldFromDriverTranslation[1] = (g_riftd && m_hadOptical) ? 0.0 : g_flHeadHeight;

	// This fixed offset only suits upstream OpenHMD's 3DoF Touch rotation; riftd reports the Touch LED
	// model frame and takes the tunable g_touchOffset.
	if (m_is_oculus && !g_riftd) {
		const HmdQuaternion_t oculusOffsetQ  = { 0.966, 0.259, 0, 0};

		pose.qDriverFromHeadRotation = oculusOffsetQ;
		pose.qWorldFromDriverRotation = identityquat;

		pose.vecDriverFromHeadTranslation[2] = 0.08;
	}
	else if (m_is_oculus) {
		AcquireSRWLockShared(&g_snapLock);
		const TouchOffset o = g_touchOffset;
		ReleaseSRWLockShared(&g_snapLock);
		double m = device_idx == RIFTD_DEV_LEFT ? -1 : 1;
		pose.qDriverFromHeadRotation = QMul(QMul(AxisAngle(0, 1, 0, m * o.yaw), AxisAngle(1, 0, 0, o.pitch)), AxisAngle(0, 0, 1, m * o.roll));
		pose.qWorldFromDriverRotation = identityquat;
		pose.vecDriverFromHeadTranslation[0] = m * o.t[0];
		pose.vecDriverFromHeadTranslation[1] = o.t[1];
		pose.vecDriverFromHeadTranslation[2] = o.t[2];
	}
	else {
		pose.qDriverFromHeadRotation = identityquat;
		pose.qWorldFromDriverRotation = identityquat;
	}

	return pose;
    }

    void RunFrame() {
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, GetPose(), sizeof( DriverPose_t ) );

        int control_count = m_info.control_count;
        float control_state[256] = { 0 };
        float touch_state[5] = { 0 };

        if (g_riftd) {
          riftd_device snap = SnapDevice(device_idx);
          if (control_count > RIFTD_MAX_CONTROLS)
            control_count = RIFTD_MAX_CONTROLS;
          memcpy(control_state, snap.controls, control_count * sizeof(float));
          memcpy(touch_state, snap.touch, sizeof(touch_state));
        } else
          ohmd_device_getf(device, OHMD_CONTROLS_STATE, control_state);

        for (int i = 0; i < control_count; i++) {
          if (m_buttons[i] != k_ulInvalidInputComponentHandle) {
            vr::VRDriverInput()->UpdateBooleanComponent( m_buttons[i], control_state[i] != 0, 0 );
          }
          else if (m_analogControls[i] != k_ulInvalidInputComponentHandle)
            vr::VRDriverInput()->UpdateScalarComponent( m_analogControls[i], control_state[i], 0 );
          if (m_touchControls[i] != k_ulInvalidInputComponentHandle) {
            float touched = (g_riftd && m_touchCap[i] >= 0) ? touch_state[m_touchCap[i]] : (control_state[i] == 0 ? 0.0f : 1.0f);
            vr::VRDriverInput()->UpdateScalarComponent( m_touchControls[i], touched, 0 );
          }
        }
        if (g_riftd) {
          for (int k = 0; k < 3; k++)
            if (m_extraTouch[k] != k_ulInvalidInputComponentHandle)
              vr::VRDriverInput()->UpdateScalarComponent( m_extraTouch[k], touch_state[2 + k], 0 );
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

    bool TriggerHapticPulse( uint32_t, uint16_t ) {
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
    int m_touchCap[64]; /* index into riftd_device.touch for m_touchControls[i], -1 if none */
    vr::VRInputComponentHandle_t m_extraTouch[3] = { k_ulInvalidInputComponentHandle, k_ulInvalidInputComponentHandle, k_ulInvalidInputComponentHandle }; /* thumb rest, stick, trigger */
};

class COpenHMDDeviceDriver final : public vr::ITrackedDeviceServerDriver, public vr::IVRDisplayComponent
{
public:
    COpenHMDDeviceDriver(int hmddisplay_idx, int hmdtracker_idx)
    {
        std::string product, path, vendor;

        if (g_riftd) {
            const riftd_device &d = g_snap.devices[RIFTD_DEV_HMD];
            m_display = g_snap.display;
            product = d.product;
            path = d.path;
            vendor = d.vendor;
        } else {
        hmd = ohmd_list_open_device(ctx, hmddisplay_idx);
        if (hmdtracker_idx != -1 && hmdtracker_idx != hmddisplay_idx)
	    hmdtracker = ohmd_list_open_device(ctx, hmdtracker_idx);
	else
	    hmdtracker = NULL;

        if(!hmd){
            DriverLog("failed to open device: %s\n", ohmd_ctx_get_error(ctx));
        }

        FillDisplayFromOpenHMD(hmd, &m_display);
        product = ohmd_list_gets(ctx, hmddisplay_idx, OHMD_PRODUCT);
        path = ohmd_list_gets(ctx, hmddisplay_idx, OHMD_PATH);
        vendor = ohmd_list_gets(ctx, hmddisplay_idx, OHMD_VENDOR);
        }

        /*
        print_infof(hmd, "hsize:",            1, OHMD_SCREEN_HORIZONTAL_SIZE);
        print_infof(hmd, "vsize:",            1, OHMD_SCREEN_VERTICAL_SIZE);
        print_infof(hmd, "lens separation:",  1, OHMD_LENS_HORIZONTAL_SEPARATION);
        print_infof(hmd, "lens vcenter:",     1, OHMD_LENS_VERTICAL_POSITION);
        print_infof(hmd, "left eye fov:",     1, OHMD_LEFT_EYE_FOV);
        print_infof(hmd, "right eye fov:",    1, OHMD_RIGHT_EYE_FOV);
        print_infof(hmd, "left eye aspect:",  1, OHMD_LEFT_EYE_ASPECT_RATIO);
        print_infof(hmd, "right eye aspect:", 1, OHMD_RIGHT_EYE_ASPECT_RATIO);
        print_infof(hmd, "distortion k:",     6, OHMD_DISTORTION_K);

        print_infoi(hmd, "digital button count:", 1, OHMD_BUTTON_COUNT);
        */

        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
        m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;

        DriverLog( "Using settings values\n" );
        m_flIPD = m_display.ipd;

        m_sSerialNumber = product + ": " + path;
        m_sModelNumber = "OpenHMD: " + product;

        // Important to pass vendor through. Gaze cursor is only available for "Oculus". So grab the first word.
        char const* vendor_override = getenv("OHMD_VENDOR_OVERRIDE");
        if (vendor_override) {
            m_sVendor = vendor_override;
        } else {
            m_sVendor = vendor;
            if (m_sVendor.find(' ') != std::string::npos) {
                m_sVendor = m_sVendor.substr(0, m_sVendor.find(' '));
            }
        }

        // windowX/windowY: desktop position of the HMD panel in extended mode
        vr::EVRSettingsError serr = vr::VRSettingsError_None;
        m_nWindowX = vr::VRSettings()->GetInt32( k_pch_Sample_Section, "windowX", &serr );
        if (serr != vr::VRSettingsError_None) m_nWindowX = 1920;
        serr = vr::VRSettingsError_None;
        m_nWindowY = vr::VRSettings()->GetInt32( k_pch_Sample_Section, "windowY", &serr );
        if (serr != vr::VRSettingsError_None) m_nWindowY = 0;
        m_nWindowWidth = m_nRenderWidth = m_display.h_resolution;
        m_nWindowHeight = m_nRenderHeight = m_display.v_resolution;
        //m_nRenderWidth /= 2;
        //m_nRenderHeight /= 2;

        m_flSecondsFromVsyncToPhotons = vr::VRSettings()->GetFloat( k_pch_Sample_Section, k_pch_Sample_SecondsFromVsyncToPhotons_Float );
        //TODO: find actual frequency somehow (from openhmd?)
        m_flDisplayFrequency = vr::VRSettings()->GetFloat( k_pch_Sample_Section, k_pch_Sample_DisplayFrequency_Float );

        DriverLog( "driver_openhmd: Vendor: %s\n", m_sVendor.c_str() );
        DriverLog( "driver_openhmd: Serial Number: %s\n", m_sSerialNumber.c_str() );
        DriverLog( "driver_openhmd: Model Number: %s\n", m_sModelNumber.c_str() );
        DriverLog( "driver_openhmd: Window: %d %d %d %d\n", m_nWindowX, m_nWindowY, m_nWindowWidth, m_nWindowHeight );
        DriverLog( "driver_openhmd: Render Target: %d %d\n", m_nRenderWidth, m_nRenderHeight );
        DriverLog( "driver_openhmd: Seconds from Vsync to Photons: %f\n", m_flSecondsFromVsyncToPhotons );
                                                DriverLog( "driver_openhmd: Display Frequency: %f\n", m_flDisplayFrequency );
        DriverLog( "driver_openhmd: IPD: %f\n", m_flIPD );

        const float *distortion_coeffs = m_display.universal_distortion_k;
        DriverLog("driver_openhmd: Distortion values a=%f b=%f c=%f d=%f\n", distortion_coeffs[0], distortion_coeffs[1], distortion_coeffs[2], distortion_coeffs[3]);

#if defined( _WINDOWS )
        // The CV1 panel only appears as a desktop display once the HMD is opened; the compositor
        // fails with InvalidOutputDesktop if it starts before Windows has attached it. A ready
        // riftd has already waited for it, so the riftd backend only checks once.
        bool found = false;
        int64_t waitStart = QpcNow();
        int tries;
        for (tries = 0; !found; tries++) {
            DISPLAY_DEVICEA dd = { sizeof(dd) };
            for (DWORD i = 0; EnumDisplayDevicesA(NULL, i, &dd, 0); i++, dd.cb = sizeof(dd)) {
                DEVMODEA dm = {};
                dm.dmSize = sizeof(dm);
                if ((dd.StateFlags & DISPLAY_DEVICE_ATTACHED_TO_DESKTOP) &&
                    EnumDisplaySettingsA(dd.DeviceName, ENUM_CURRENT_SETTINGS, &dm) &&
                    dm.dmPosition.x == m_nWindowX && dm.dmPosition.y == m_nWindowY &&
                    (int32_t)dm.dmPelsWidth == m_nWindowWidth && (int32_t)dm.dmPelsHeight == m_nWindowHeight) {
                    DriverLog("driver_openhmd: HMD display %s attached at %d Hz after %d ms\n", dd.DeviceName, (int)dm.dmDisplayFrequency, tries * 250);
                    found = true;
                    break;
                }
            }
            if (found || g_riftd || QpcNow() >= g_initDeadline)
                break;
            std::this_thread::sleep_for( std::chrono::milliseconds(250) );
        }
        if (!found)
            DriverLog("driver_openhmd: no %dx%d display at %d,%d after %.1f s; check windowX/windowY\n", m_nWindowWidth, m_nWindowHeight,
                      m_nWindowX, m_nWindowY, (double)(QpcNow() - waitStart) / g_riftdClient.Freq());
        else if (tries > 0)
            // the compositor fails with InvalidOutputDesktop on a panel that attached a moment ago
            std::this_thread::sleep_for( std::chrono::seconds(1) );
#endif
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

        //float sep;

        // return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
        vr::VRProperties()->SetUint64Property( m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2 );

        // A calibrated universe keeps vrmonitor from launching Room Setup at every start. Standing
        // and seated are the raw tracking space; the bounds are hidden.
        static const vr::ETrackedDeviceProperty Prop_DriverProvidedChaperoneJson_String = (vr::ETrackedDeviceProperty)2095;
        vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_DriverProvidedChaperoneJson_String,
            "{\"jsonid\":\"chaperone_info\",\"version\":5,\"time\":\"2026-11-20T00:00:00\",\"universes\":[{\"universeID\":\"2\","
            "\"play_area\":[4.0,4.0],\"collision_bounds\":[[[-2,0,-2],[-2,2.4,-2],[2,2.4,-2],[2,0,-2]],[[2,0,-2],[2,2.4,-2],[2,2.4,2],[2,0,2]],"
            "[[2,0,2],[2,2.4,2],[-2,2.4,2],[-2,0,2]],[[-2,0,2],[-2,2.4,2],[-2,2.4,-2],[-2,0,-2]]],"
            "\"seated\":{\"translation\":[0,0,0],\"yaw\":0},\"standing\":{\"translation\":[0,0,0],\"yaw\":0}}]}" );
        vr::VRProperties()->SetBoolProperty( m_ulPropertyContainer, Prop_DriverProvidedChaperoneVisibility_Bool, false );

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

    void DebugRequest( const char *, char *pchResponseBuffer, uint32_t unResponseBufferSize )
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

        if ( fabs(cosPitch) > 0.1) 
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

        *yaw   = (float)(atan2(sinYaw, cosYaw) * 180 / M_PI);
        *pitch = (float)(atan2(sinPitch, cosPitch) * 180 / M_PI);
        *roll  = (float)(atan2(sinRoll, cosRoll) * 180 / M_PI);
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
        memcpy(ohmdprojection.arr, eEye == Eye_Left ? m_display.left_projection : m_display.right_projection, sizeof(ohmdprojection.arr));

        float yaw, pitch, roll;
        float p[4][4];
        memcpy(p, ohmdprojection.arr, 16 * sizeof(float));
        columnMatrixToAngles(&yaw, &pitch, &roll, p);
        
        if (eEye == Eye_Left) {
            rotation_left = yaw;
        } else {
            rotation_right = yaw;
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
        
        //DriverLog("angles %f %f %f\n", yaw, pitch, roll);
    }

    DistortionCoordinates_t ComputeDistortion( EVREye eEye, float fU, float fV )
    {
        float angle = (eEye == Eye_Left ? rotation_left : rotation_right);
        
        //DriverLog("Eye %d before: %f %f\n", eEye, fU, fV);
                
        if (angle > -5 && angle < 5) {
        } else if (angle > 85 && angle < 95) {
            float tmp = fV;
            fV = 1.f - fU;
            fU = tmp;
        } else if (angle > -95 && angle < -85) {
            float tmp = fV;
            fV = fU;
            fU = 1.f - tmp;
        } else {
            float x = 0 * fU + -1 * fV;
            float y = -1 * fU + 0 * 0 * fV;
            fU = x;
            fV = y;
        }
        
        //DriverLog("Eye %d after: %f %f\n", eEye, fU, fV);
        
        float viewport_scale[2];
        const float *distortion_coeffs = m_display.universal_distortion_k;
        const float *aberr_scale = m_display.universal_aberration_k;
        float sep = m_display.lens_separation;
        float left_lens_center[2];
        float right_lens_center[2];
        //viewport is half the screen
        viewport_scale[0] = m_display.h_size / 2.0f;
        viewport_scale[1] = m_display.v_size;
        //calculate lens centers (assuming the eye separation is the distance betweenteh lense centers)
        left_lens_center[1] = m_display.lens_v_position;
        right_lens_center[1] = m_display.lens_v_position;
        left_lens_center[0] = viewport_scale[0] - sep/2.0f;
        right_lens_center[0] = sep/2.0f;
        //asume calibration was for lens view to which ever edge of screen is further away from lens center
        float warp_scale = (left_lens_center[0] > right_lens_center[0]) ? left_lens_center[0] : right_lens_center[0];

        float lens_center[2];
        lens_center[0] = (eEye == Eye_Left ? left_lens_center[0] : right_lens_center[0]);
        lens_center[1] = (eEye == Eye_Left ? left_lens_center[1] : right_lens_center[1]);

        float r[2];
        r[0] = fU * viewport_scale[0] - lens_center[0];
        r[1] = fV * viewport_scale[1] - lens_center[1];

        r[0] /= warp_scale;
        r[1] /= warp_scale;

        float r_mag = sqrt(r[0] * r[0] + r[1] * r[1]);


        float r_displaced[2];
        r_displaced[0] = r[0] * (distortion_coeffs[3] + distortion_coeffs[2] * r_mag + distortion_coeffs[1] * r_mag * r_mag + distortion_coeffs[0] * r_mag * r_mag * r_mag);

        r_displaced[1] = r[1] * (distortion_coeffs[3] + distortion_coeffs[2] * r_mag + distortion_coeffs[1] * r_mag * r_mag + distortion_coeffs[0] * r_mag * r_mag * r_mag);

        r_displaced[0] *= warp_scale;
        r_displaced[1] *= warp_scale;

        float tc_r[2];
        tc_r[0] = (lens_center[0] + aberr_scale[0] * r_displaced[0]) / viewport_scale[0];
        tc_r[1] = (lens_center[1] + aberr_scale[0] * r_displaced[1]) / viewport_scale[1];

        float tc_g[2];
        tc_g[0] = (lens_center[0] + aberr_scale[1] * r_displaced[0]) / viewport_scale[0];
        tc_g[1] = (lens_center[1] + aberr_scale[1] * r_displaced[1]) / viewport_scale[1];

        float tc_b[2];
        tc_b[0] = (lens_center[0] + aberr_scale[2] * r_displaced[0]) / viewport_scale[0];
        tc_b[1] = (lens_center[1] + aberr_scale[2] * r_displaced[1]) / viewport_scale[1];

        //DriverLog("Distort %f %f -> %f %f; %f %f %f %f\n", fU, fV, tc_b[0], tc_b[1], distortion_coeffs[0], distortion_coeffs[1], distortion_coeffs[2], distortion_coeffs[3]);

        DistortionCoordinates_t coordinates;
        coordinates.rfBlue[0] = tc_b[0];
        coordinates.rfBlue[1] = tc_b[1];
        coordinates.rfGreen[0] = tc_g[0];
        coordinates.rfGreen[1] = tc_g[1];
        coordinates.rfRed[0] = tc_r[0];
        coordinates.rfRed[1] = tc_r[1];
        return coordinates;
    }

    DriverPose_t GetPose()
    {
        DriverPose_t pose = { 0 };
        if (g_riftd) {
            if (RiftdPose(RIFTD_DEV_HMD, pose))
                m_hadOptical = true;
            pose.qWorldFromDriverRotation = identityquat;
            pose.qDriverFromHeadRotation = identityquat;
            // headHeight only lifts the rotation-only fallback position; optical positions are used as tracked
            pose.vecWorldFromDriverTranslation[1] = m_hadOptical ? 0.0 : g_flHeadHeight;
            return pose;
        }
        pose.poseIsValid = true;
        pose.result = TrackingResult_Running_OK;
        pose.deviceIsConnected = true;

        ohmd_device* d = hmdtracker ? hmdtracker : hmd;

        float quat[4];
        ohmd_device_getf(d, OHMD_ROTATION_QUAT, quat);
        pose.qRotation.x = quat[0];
        pose.qRotation.y = quat[1];
        pose.qRotation.z = quat[2];
        pose.qRotation.w = quat[3];

        float pos[3];
        ohmd_device_getf(d, OHMD_POSITION_VECTOR, pos);
        pose.vecPosition[0] = pos[0];
        pose.vecPosition[1] = pos[1];
        pose.vecPosition[2] = pos[2];

        //printf("%f %f %f %f  %f %f %f\n", quat[0], quat[1], quat[2], quat[3], pos[0], pos[1], pos[2]);
        //fflush(stdout);
        //DriverLog("get hmd pose %f %f %f %f, %f %f %f\n", quat[0], quat[1], quat[2], quat[3], pos[0], pos[1], pos[2]);

        pose.qWorldFromDriverRotation = identityquat;
        pose.qDriverFromHeadRotation = identityquat;
        pose.vecWorldFromDriverTranslation[1] = g_flHeadHeight;

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

    std::string GetSerialNumber() const { return m_sSerialNumber; }

private:
    ohmd_device* hmd = NULL;
    ohmd_device* hmdtracker = NULL;
    riftd_display m_display = {};
    bool m_hadOptical = false;

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
    
    float rotation_left = 0.0;
    float rotation_right = 0.0;
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
    virtual bool ShouldBlockStandbyMode()  { return false; }
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
#if defined( _WINDOWS )
    g_initDeadline = QpcNow() + 12 * g_riftdClient.Freq();
#endif

    vr::EVRSettingsError herr = vr::VRSettingsError_None;
    float head_height = vr::VRSettings()->GetFloat( k_pch_Sample_Section, "headHeight", &herr );
    if (herr == vr::VRSettingsError_None)
        g_flHeadHeight = head_height;
    DriverLog("driver_openhmd: head height %f\n", g_flHeadHeight);
    ReadTouchOffset();

    char backend[64] = "";
    herr = vr::VRSettingsError_None;
    vr::VRSettings()->GetString( k_pch_Sample_Section, "backend", backend, sizeof(backend), &herr );
    if (strcmp(backend, "riftd") == 0) {
#if defined( _WINDOWS )
        herr = vr::VRSettingsError_None;
        int32_t timeout = vr::VRSettings()->GetInt32( k_pch_Sample_Section, "riftdTimeout", &herr );
        if (herr != vr::VRSettingsError_None || timeout < 0)
            timeout = 10;
        if (timeout > 12)
            timeout = 12;
        DriverLog("driver_openhmd: backend riftd, waiting up to %d s for %s\n", timeout, RIFTD_SHM_NAME);
        int64_t deadline = QpcNow() + timeout * g_riftdClient.Freq();
        if (g_riftdClient.Open(deadline < g_initDeadline ? deadline : g_initDeadline)) {
            g_snapFresh = true;
            DriverLog("driver_openhmd: riftd pid %u ready, %u sensors\n", g_snap.pid, g_snap.n_sensors);
        } else if (LoadRiftdCache(&g_snap)) {
            // Failing Init here would make SteamVR blame the driver; the devices stay disconnected until riftd is ready
            g_snapFresh = false;
            DriverLog("driver_openhmd: riftd not ready after %d s; adding the devices it last reported, disconnected until it is ready\n", timeout);
        } else {
            DriverLog("driver_openhmd: riftd not ready after %d s and no %s; start build\\arm64\\tracker\\riftd.exe or set driver_openhmd.backend to openhmd\n",
                      timeout, RiftdCachePath().c_str());
            return VRInitError_Init_HmdNotFound;
        }
        g_riftd = true;

        m_OpenHMDDeviceDriver = new COpenHMDDeviceDriver(-1, -1);
        vr::VRServerDriverHost()->TrackedDeviceAdded( m_OpenHMDDeviceDriver->GetSerialNumber().c_str(), vr::TrackedDeviceClass_HMD, m_OpenHMDDeviceDriver );
        if (g_snap.devices[RIFTD_DEV_LEFT].flags & RIFTD_DEVICE_PRESENT) {
            m_OpenHMDDeviceDriverControllerL = new COpenHMDDeviceDriverController(0, NULL, RIFTD_DEV_LEFT);
            vr::VRServerDriverHost()->TrackedDeviceAdded( m_OpenHMDDeviceDriverControllerL->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_OpenHMDDeviceDriverControllerL );
        }
        if (g_snap.devices[RIFTD_DEV_RIGHT].flags & RIFTD_DEVICE_PRESENT) {
            m_OpenHMDDeviceDriverControllerR = new COpenHMDDeviceDriverController(1, NULL, RIFTD_DEV_RIGHT);
            vr::VRServerDriverHost()->TrackedDeviceAdded( m_OpenHMDDeviceDriverControllerR->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_OpenHMDDeviceDriverControllerR );
        }
        return VRInitError_None;
#else
        DriverLog("driver_openhmd: backend riftd is Windows only\n");
        return VRInitError_Init_HmdNotFound;
#endif
    }

    ctx = ohmd_ctx_create();
    int num_devices = ohmd_ctx_probe(ctx);
    if(num_devices < 0){
        DriverLog("failed to probe devices: %s\n", ohmd_ctx_get_error(ctx));
    }

    int hmddisplay_idx = get_configvalues()[0];
    int hmdtracker_idx = get_configvalues()[1];
    int lcontroller_idx = get_configvalues()[2];
    int rcontroller_idx = get_configvalues()[3];

    for(int i = 0; i < num_devices; i++){
        DriverLog("device %d\n", i);
        DriverLog("  vendor:  %s\n", ohmd_list_gets(ctx, i, OHMD_VENDOR));
        DriverLog("  product: %s\n", ohmd_list_gets(ctx, i, OHMD_PRODUCT));
        DriverLog("  path:    %s\n\n", ohmd_list_gets(ctx, i, OHMD_PATH));

        int device_class = 0, device_flags = 0;

        ohmd_list_geti(ctx, i, OHMD_DEVICE_CLASS, &device_class);
        ohmd_list_geti(ctx, i, OHMD_DEVICE_FLAGS, &device_flags);

	switch (device_class) {
		case OHMD_DEVICE_CLASS_HMD:
			if (hmddisplay_idx == -1)
				hmddisplay_idx = i;
			break;
		case OHMD_DEVICE_CLASS_CONTROLLER:
			if (lcontroller_idx == -1 && (device_flags & OHMD_DEVICE_FLAGS_LEFT_CONTROLLER))
				lcontroller_idx = i;
			else if (rcontroller_idx == -1 && (device_flags & OHMD_DEVICE_FLAGS_RIGHT_CONTROLLER))
				rcontroller_idx = i;
			break;
		case OHMD_DEVICE_CLASS_GENERIC_TRACKER:
			if (hmdtracker_idx == -1 && !(device_flags & (OHMD_DEVICE_FLAGS_LEFT_CONTROLLER|OHMD_DEVICE_FLAGS_RIGHT_CONTROLLER)))
				hmdtracker_idx = i;
			else if (lcontroller_idx == -1 && (device_flags & OHMD_DEVICE_FLAGS_LEFT_CONTROLLER))
				lcontroller_idx = i;
			else if (rcontroller_idx == -1 && (device_flags & OHMD_DEVICE_FLAGS_RIGHT_CONTROLLER))
				rcontroller_idx = i;
			break;
		default:
			break;
	}
    }

    if (hmdtracker_idx == -1)
    	hmdtracker_idx = hmddisplay_idx;

    DriverLog("Using HMD Display %d, HMD Tracker %d, Left Controller %d, Right Controller %d\n", hmddisplay_idx, hmdtracker_idx, lcontroller_idx, rcontroller_idx);

    m_OpenHMDDeviceDriver = new COpenHMDDeviceDriver(hmddisplay_idx, hmdtracker_idx);
    vr::VRServerDriverHost()->TrackedDeviceAdded( m_OpenHMDDeviceDriver->GetSerialNumber().c_str(), vr::TrackedDeviceClass_HMD, m_OpenHMDDeviceDriver );

    if (lcontroller_idx >= 0) {
	ohmd_device* lcontroller = ohmd_list_open_device(ctx, lcontroller_idx);
	if (lcontroller)
		m_OpenHMDDeviceDriverControllerL = new COpenHMDDeviceDriverController(0, lcontroller, lcontroller_idx);
	if (m_OpenHMDDeviceDriverControllerL)
		vr::VRServerDriverHost()->TrackedDeviceAdded( m_OpenHMDDeviceDriverControllerL->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_OpenHMDDeviceDriverControllerL );
    }

    if (rcontroller_idx >= 0) {
	ohmd_device *rcontroller = ohmd_list_open_device(ctx, rcontroller_idx);
	if (rcontroller)
		m_OpenHMDDeviceDriverControllerR = new COpenHMDDeviceDriverController(1, rcontroller, rcontroller_idx);
	if (m_OpenHMDDeviceDriverControllerR)
		vr::VRServerDriverHost()->TrackedDeviceAdded(  m_OpenHMDDeviceDriverControllerR->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_OpenHMDDeviceDriverControllerR );
    }

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

    if (ctx)
        ohmd_ctx_destroy (ctx);
}


void CServerDriver_OpenHMD::RunFrame()
{
#if defined( _WINDOWS )
    if (g_riftd) {
        static bool cached = false;
        static int64_t lastSettings = 0;
        bool fresh = g_riftdClient.Update();
        // drivers get no settings-changed event; polling lets vrset changes apply live
        if (QpcNow() - lastSettings > g_riftdClient.Freq()) {
            ReadTouchOffset();
            lastSettings = QpcNow();
        }
        if (fresh != g_snapFresh)
            DriverLog("driver_openhmd: riftd %s\n", fresh ? "updating" : "stopped updating; devices marked invalid");
        if (fresh && !cached) {
            SaveRiftdCache(g_snap);
            cached = true;
        }
        g_snapFresh = fresh;
    } else
#endif
    ohmd_ctx_update(ctx);

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

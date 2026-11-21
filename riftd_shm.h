// Copyright 2026, Michael Conard.
// SPDX-License-Identifier: BSL-1.0
//
// Shared memory published by riftd (oculus-on-arm, service/riftd.c) and read by this
// driver when driver_openhmd.backend is "riftd". Windows only.
//
// Writer: increments seq to an odd value, writes, increments it to an even value.
// Reader: reads seq (retry while odd), copies, rereads seq, retries if it changed.

#ifndef RIFTD_SHM_H
#define RIFTD_SHM_H

#include <stddef.h>
#include <stdint.h>

#define RIFTD_SHM_NAME "Local\\riftd-shm"
#define RIFTD_SHM_MAGIC 0x44544652u /* "RFTD" */
#define RIFTD_SHM_VERSION 4

#define RIFTD_MAX_CONTROLS 16
#define RIFTD_MAX_SENSORS 4
/* Holding the right Touch Oculus button this long makes riftd set forward and center from the
 * headset; the driver only sends shorter presses to SteamVR. */
#define RIFTD_RECENTER_HOLD_S 1.0

enum riftd_device_index {
	RIFTD_DEV_HMD = 0,
	RIFTD_DEV_LEFT = 1,
	RIFTD_DEV_RIGHT = 2,
	RIFTD_NUM_DEVICES = 3
};

enum riftd_service_state {
	RIFTD_STATE_STARTING = 0,
	RIFTD_STATE_READY = 1,    /* devices open, HMD display powered */
	RIFTD_STATE_STOPPING = 2
};

/* riftd_device.flags */
#define RIFTD_DEVICE_PRESENT 0x01   /* opened through OpenHMD */
#define RIFTD_DEVICE_CONNECTED 0x02 /* reporting IMU data */
#define RIFTD_DEVICE_POSE_VALID 0x04
#define RIFTD_DEVICE_OPTICAL 0x08   /* position from a camera observation within 0.5 s */
#define RIFTD_DEVICE_VELOCITY 0x10  /* linear and angular velocity are set */

typedef struct riftd_device {
	uint32_t flags;
	int32_t ohmd_flags;           /* OHMD_DEVICE_FLAGS */
	int64_t sample_qpc;           /* QueryPerformanceCounter when the pose was read */
	uint64_t imu_updates;         /* advances while the device reports */

	float position[3];            /* m, OpenHMD tracking space */
	float orientation[4];         /* x, y, z, w */
	float linear_velocity[3];     /* m/s */
	float angular_velocity[3];    /* rad/s, device frame (DriverPose_t convention) */

	int32_t control_count;
	int32_t control_hints[RIFTD_MAX_CONTROLS];  /* ohmd_control_hint */
	int32_t control_types[RIFTD_MAX_CONTROLS];  /* ohmd_control_type */
	float controls[RIFTD_MAX_CONTROLS];         /* OHMD_CONTROLS_STATE */
	float touch[5];               /* capacitive A/X, B/Y, thumb rest, stick, trigger */

	char vendor[64];
	char product[64];
	char path[128];
} riftd_device;

typedef struct riftd_display {
	int32_t h_resolution;
	int32_t v_resolution;
	float h_size;                 /* m */
	float v_size;
	float lens_separation;
	float lens_v_position;
	float ipd;
	float left_fov;               /* OHMD_LEFT_EYE_FOV */
	float right_fov;
	float left_aspect;
	float right_aspect;
	float distortion_k[6];        /* OHMD_DISTORTION_K */
	float universal_distortion_k[4];
	float universal_aberration_k[3];
	float left_projection[16];    /* OHMD_LEFT_EYE_GL_PROJECTION_MATRIX */
	float right_projection[16];
} riftd_display;

typedef struct riftd_sensor {
	char serial[33];
	uint8_t have_pose;
	uint8_t reserved[2];
	float fps;
} riftd_sensor;

typedef struct riftd_shm {
	uint32_t magic;
	uint32_t version;
	uint32_t size;                /* sizeof(riftd_shm) */
	uint32_t pid;
	volatile uint32_t seq;
	volatile uint32_t state;      /* riftd_service_state */
	int64_t qpc_frequency;
	int64_t heartbeat_qpc;        /* last write */

	riftd_display display;
	riftd_device devices[RIFTD_NUM_DEVICES];

	uint32_t n_sensors;
	riftd_sensor sensors[RIFTD_MAX_SENSORS];

	/* Room offset as x, y, z (m) and yaw (rad), the transform from the stored sensor frame to the
	 * published tracking space (rift-room-config.json). The tracker applies room_applied, the one
	 * loaded at start; riftd applies the rest to the published poses. */
	float room_offset[4];
	float room_applied[4];
	uint32_t room_seq;            /* last riftd_room_request.seq handled */
	int32_t room_result;          /* 0 stored, -1 not stored */

	int32_t proximity;            /* CV1 proximity sensor, raw; -1 without one */
	uint32_t hmd_worn;            /* proximity over 3 */
	char hmd_serial[32];          /* the headset's USB serial number, "" if unknown */
} riftd_shm;

/* Room offset request, written by a client (riftsetup), polled by riftd: write offset, then
 * increment seq. riftd stores it and reports room_seq and room_result. */
#define RIFTD_ROOM_NAME "Local\\riftd-room"

typedef struct riftd_room_request {
	volatile uint32_t seq;
	float offset[4];
} riftd_room_request;

/* Touch vibration requests, written by the driver, polled by riftd. Per controller (index
 * RIFTD_DEV_LEFT - 1, RIFTD_DEV_RIGHT - 1): increment seq to an odd value, write the fields,
 * increment it to an even value. riftd applies each new even seq; amplitude 0 stops, duration 0 is
 * one pulse (OpenVR's legacy TriggerHapticPulse). */
#define RIFTD_HAPTIC_NAME "Local\\riftd-haptic"

typedef struct riftd_haptic_pulse {
	volatile uint32_t seq;
	float amplitude;              /* 0..1 */
	float frequency;              /* Hz; the CV1 Touch vibrates at 160 or 320 Hz */
	float duration;               /* s */
} riftd_haptic_pulse;

typedef struct riftd_haptic_request {
	riftd_haptic_pulse hand[2];
} riftd_haptic_request;

#define RIFTD_SHM_ASSERT(name, cond) typedef char riftd_shm_assert_##name[(cond) ? 1 : -1]
RIFTD_SHM_ASSERT(haptic, sizeof(riftd_haptic_request) == 32);
RIFTD_SHM_ASSERT(size, sizeof(riftd_shm) == 2168);
RIFTD_SHM_ASSERT(devices, offsetof(riftd_shm, devices) == 264);
RIFTD_SHM_ASSERT(sensors, offsetof(riftd_shm, sensors) == 1924);

#endif

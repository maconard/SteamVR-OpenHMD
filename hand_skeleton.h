// Copyright 2026, Michael Conard.
// SPDX-License-Identifier: BSL-1.0
//
// Estimated Touch hand skeleton for /input/skeleton/left|right. The reference poses come from
// SteamVR's own animation files, read at runtime: drivers/oculus/resources/anims/touch_poses_right.glb
// (hand on the controller) and resources/anims/hand_right_poses.glb (open hand, fist).

#ifndef HAND_SKELETON_H
#define HAND_SKELETON_H

#include <openvr_driver.h>
#include <stdint.h>
#include <string>
#include <vector>

static const uint32_t HAND_BONES = 31;

struct HandInput {
    float trigger;      // 0..1
    float grip;         // 0..1
    float joyX, joyY;   // -1..1
    bool triggerTouch;
    bool aTouch, bTouch; // A/X and B/Y
    bool aClick, bClick;
    bool restTouch, stickTouch;
};

// Parses both .glb files; false with *err set if either lacks the frames used here.
bool LoadHandPoses(const std::vector<uint8_t> &touchPoses, const std::vector<uint8_t> &handPoses, std::string *err);

class HandSkeleton {
public:
    // dt in seconds smooths the switches between touch states.
    void Update(const HandInput &in, float dt, bool left, vr::VRBoneTransform_t *withController,
                vr::VRBoneTransform_t *withoutController);

    // Hand closed on the controller: trigger and grip pulled, thumb on the thumb rest.
    static void GripLimit(bool left, vr::VRBoneTransform_t *out);

private:
    float m_thumb[8] = { 1, 0, 0, 0, 0, 0, 0, 0 };
    float m_point = 0;
};

#endif

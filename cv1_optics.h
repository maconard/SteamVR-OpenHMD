// Copyright 2026, Michael Conard.
// Copyright 2020-2025, Collabora, Ltd. (Monado rift_driver.c)
// Copyright 2025-2026, Beyley Cardellio (Monado rift_driver.c, u_distortion_mesh_rift.c)
// Copyright 2026, NVIDIA CORPORATION. (Monado rift_driver.c)
// SPDX-License-Identifier: BSL-1.0
//
// Rift CV1 stereo view: projection tangents, render target size and distortion per eye, from one
// model so the projection and the distortion agree. Panel coordinates are meters within one eye's
// half of the panel, x right, y down from the top edge; the lens center is lensFromTop below it.
//
// Lens models (driver_openhmd.distortion):
//   libovr    LibOVR's CV1 lens profile, a Catmull-Rom magnification curve over the squared tan angle,
//             as ported by Monado (src/xrt/auxiliary/util/u_distortion_mesh_rift.c and
//             src/xrt/drivers/rift/rift_driver.c), with the FOV the Oculus OpenXR runtime reports
//   thaytan   OpenHMD polynomial, coefficients from thaytan's fork (hand-measured 2021)
//   upstream  OpenHMD polynomial, coefficients from OpenHMD master (hand-measured 2017)

#ifndef CV1_OPTICS_H
#define CV1_OPTICS_H

#include <math.h>
#include <string.h>

// LibOVR CV1 lens profile
static const float kMetersPerTan = 0.0438f;
static const float kChroma[4] = { -0.008f, -0.005f, 0.015f, 0.005f };
static const int kN = 11;
static const float kK[kN] = { 1.0f, 1.0312999f, 1.0698f, 1.1155f, 1.173f, 1.2460001f, 1.336f, 1.457f, 1.630f, 1.900f, 2.3599999f };

struct Cv1Optics {
    enum Model { LIBOVR, THAYTAN, UPSTREAM };

    Model model = LIBOVR;
    float panelW = 0.11934f, panelH = 0.0663f; // whole panel, m
    float lensSep = 0.054f;                    // between lens centers
    float lensFromTop = 0.03084f;
    float eyeToScreen = 0.03962f;
    int eyePxW = 1080, eyePxH = 1200;

    // per-eye tangents for the left eye; the right eye mirrors outer and inner
    float tanOuter = 0, tanInner = 0, tanUp = 0, tanDown = 0;

    static bool Parse(const char *name, Model *m)
    {
        if (!strcmp(name, "libovr")) *m = LIBOVR;
        else if (!strcmp(name, "thaytan")) *m = THAYTAN;
        else if (!strcmp(name, "upstream")) *m = UPSTREAM;
        else return false;
        return true;
    }

    static const char *Name(Model m)
    {
        return m == LIBOVR ? "libovr" : m == THAYTAN ? "thaytan" : "upstream";
    }

    void Init()
    {
        if (model == LIBOVR) {
            tanOuter = tanf(0.76754993f);
            tanInner = tanf(0.6208969f);
            tanUp = tanf(0.7269826f);
            tanDown = tanf(0.8378981f);
        } else {
            float lcx = EyeW() - lensSep / 2;
            tanOuter = lcx / eyeToScreen;
            tanInner = (EyeW() - lcx) / eyeToScreen;
            tanUp = lensFromTop / eyeToScreen;
            tanDown = (panelH - lensFromTop) / eyeToScreen;
        }
    }

    float EyeW() const { return panelW / 2; }

    // OpenVR raw projection. IVRSystem::GetProjectionMatrix puts the image's top edge at the
    // "bottom" tangent, so bottom is the upward extent and top the negated downward one.
    void Projection(bool left, float *l, float *r, float *t, float *b) const
    {
        *l = -(left ? tanOuter : tanInner);
        *r = left ? tanInner : tanOuter;
        *t = -tanDown;
        *b = tanUp;
    }

    // one render target pixel per panel pixel at the lens center
    void RenderSize(unsigned *w, unsigned *h) const
    {
        if (model == LIBOVR) {
            *w = (unsigned)ceilf(eyePxW / EyeW() * kMetersPerTan * (tanOuter + tanInner));
            *h = (unsigned)ceilf(eyePxH / panelH * kMetersPerTan * (tanUp + tanDown));
        } else {
            const float *k = Poly();
            *w = (unsigned)ceilf(eyePxW / k[3]);
            *h = (unsigned)ceilf(eyePxH / k[3]);
        }
    }

    // u, v: 0..1 across the eye's viewport from the top left; out: red, green, blue texture uv
    void Distort(bool left, float u, float v, float out[3][2]) const
    {
        float lcx = left ? EyeW() - lensSep / 2 : lensSep / 2;
        float dx = u * EyeW() - lcx, dy = v * panelH - lensFromTop;
        float l = -(left ? tanOuter : tanInner), r = left ? tanInner : tanOuter, scale[3], tx, ty;

        if (model == LIBOVR) {
            tx = dx / kMetersPerTan;
            ty = dy / kMetersPerTan;
            float rho = tx * tx + ty * ty, m = Magnification(rho);
            scale[0] = m * (1 + kChroma[0] + rho * kChroma[1]);
            scale[1] = m;
            scale[2] = m * (1 + kChroma[2] + rho * kChroma[3]);
        } else {
            // OpenHMD's shader: radius normalized to the farther lens-to-edge distance
            const float *k = Poly(), *ab = Aberration();
            float ws = EyeW() - lensSep / 2;
            float rm = sqrtf(dx * dx + dy * dy) / ws;
            float f = k[3] + k[2] * rm + k[1] * rm * rm + k[0] * rm * rm * rm;
            tx = dx / eyeToScreen;
            ty = dy / eyeToScreen;
            for (int i = 0; i < 3; i++)
                scale[i] = f * ab[i];
        }
        for (int i = 0; i < 3; i++) {
            out[i][0] = (tx * scale[i] - l) / (r - l);
            out[i][1] = (ty * scale[i] + tanUp) / (tanUp + tanDown);
        }
    }

private:
    const float *Poly() const
    {
        static const float thaytan[4] = { 0.269f, -0.25f, 0.178f, 0.803f };
        static const float upstream[4] = { 0.098f, 0.324f, -0.241f, 0.819f };
        return model == UPSTREAM ? upstream : thaytan;
    }

    const float *Aberration() const
    {
        static const float thaytan[3] = { 0.9992107f, 1.0f, 1.0120361f };
        static const float upstream[3] = { 0.9952420f, 1.0f, 1.0008074f };
        return model == UPSTREAM ? upstream : thaytan;
    }

    static float SlopeAfter(int j)
    {
        if (j == 0) return kK[1] - kK[0];
        if (j <= kN - 3) return 0.5f * (kK[j + 1] - kK[j - 1]);
        if (j == kN - 2) return 0.5f * (kK[kN - 1] - kK[kN - 2]);
        return kK[kN - 1] - kK[kN - 2];
    }

    static float SlopeBefore(int j)
    {
        if (j <= kN - 3) return SlopeAfter(j);
        if (j == kN - 2) return 0.5f * (kK[kN - 1] - kK[kN - 3]);
        return kK[kN - 1] - kK[kN - 2];
    }

    // max_r is 1, so the curve position is the squared tan radius times (kN - 1)
    static float Magnification(float rho)
    {
        float s = (kN - 1) * rho;
        int j = (int)floorf(s);
        j = j < 0 ? 0 : j > kN - 1 ? kN - 1 : j;
        float t = s - j, p0 = kK[j], m0 = SlopeAfter(j), p1, m1;
        if (j == kN - 1) {
            p1 = p0 + m0;
            m1 = m0;
        } else {
            p1 = kK[j + 1];
            m1 = SlopeBefore(j + 1);
        }
        float t2 = t * t, t3 = t2 * t;
        return (2 * t3 - 3 * t2 + 1) * p0 + (t3 - 2 * t2 + t) * m0 + (-2 * t3 + 3 * t2) * p1 + (t3 - t2) * m1;
    }
};

#endif

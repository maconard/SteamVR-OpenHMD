// Copyright 2026, Michael Conard.
// SPDX-License-Identifier: BSL-1.0

#include "hand_skeleton.h"

#include <algorithm>
#include <array>
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace {

// Minimal JSON reader for the glTF chunk of a .glb file.
struct Json {
    enum Type { Null, Bool, Num, Str, Arr, Obj };
    Type type = Null;
    double num = 0;
    std::string str;
    std::vector<Json> items;
    std::vector<std::string> keys; // Obj: name of items[i]

    const Json &operator[](const char *k) const
    {
        if (type == Obj)
            for (size_t i = 0; i < keys.size(); i++)
                if (keys[i] == k)
                    return items[i];
        return None();
    }
    const Json &At(int i) const { return type == Arr && i >= 0 && (size_t)i < items.size() ? items[i] : None(); }
    size_t Size() const { return type == Arr ? items.size() : 0; }
    int Int(int def) const { return type == Num && num >= 0 && num < 1e9 ? (int)num : def; }
    static const Json &None()
    {
        static const Json n;
        return n;
    }
};

class JsonParser {
public:
    explicit JsonParser(const std::string &s) : m_p(s.c_str()), m_end(s.c_str() + s.size()) {}
    bool Parse(Json &out)
    {
        if (!Value(out, 0))
            return false;
        Ws();
        return m_p == m_end;
    }

private:
    const char *m_p, *m_end;

    void Ws()
    {
        while (m_p < m_end && (*m_p == ' ' || *m_p == '\t' || *m_p == '\n' || *m_p == '\r'))
            m_p++;
    }
    bool Lit(const char *s)
    {
        size_t n = strlen(s);
        if ((size_t)(m_end - m_p) < n || memcmp(m_p, s, n) != 0)
            return false;
        m_p += n;
        return true;
    }
    bool Expect(char c)
    {
        Ws();
        if (m_p >= m_end || *m_p != c)
            return false;
        m_p++;
        return true;
    }
    bool String(std::string &s)
    {
        if (!Expect('"'))
            return false;
        while (m_p < m_end && *m_p != '"') {
            char c = *m_p++;
            if (c == '\\') {
                if (m_p >= m_end)
                    return false;
                c = *m_p++;
                if (c == 'u') {
                    if (m_end - m_p < 4)
                        return false;
                    m_p += 4;
                    c = '?';
                } else if (c == 'n' || c == 't' || c == 'r' || c == 'b' || c == 'f')
                    c = ' ';
            }
            s += c;
        }
        return Expect('"');
    }
    bool Value(Json &v, int depth)
    {
        if (depth > 64)
            return false;
        Ws();
        if (m_p >= m_end)
            return false;
        if (*m_p == '{' || *m_p == '[') {
            bool obj = *m_p++ == '{';
            char close = obj ? '}' : ']';
            v.type = obj ? Json::Obj : Json::Arr;
            Ws();
            if (m_p < m_end && *m_p == close) {
                m_p++;
                return true;
            }
            for (;;) {
                if (obj) {
                    v.keys.emplace_back();
                    if (!String(v.keys.back()) || !Expect(':'))
                        return false;
                }
                v.items.emplace_back();
                if (!Value(v.items.back(), depth + 1))
                    return false;
                Ws();
                if (m_p < m_end && *m_p == ',') {
                    m_p++;
                    continue;
                }
                return Expect(close);
            }
        }
        if (*m_p == '"') {
            v.type = Json::Str;
            return String(v.str);
        }
        if (Lit("true")) {
            v.type = Json::Bool;
            v.num = 1;
            return true;
        }
        if (Lit("false")) {
            v.type = Json::Bool;
            return true;
        }
        if (Lit("null"))
            return true;
        char *e = NULL;
        v.num = strtod(m_p, &e); // the string is NUL-terminated
        if (e == m_p)
            return false;
        v.type = Json::Num;
        m_p = e;
        return true;
    }
};

struct Bone {
    float t[3];
    float q[4]; // w, x, y, z
};
typedef std::array<Bone, HAND_BONES> Hand;

// OpenVR hand skeleton bone order; the .glb node names carry a "REF:" prefix
const char *const k_boneNames[HAND_BONES] = {
    "Root", "wrist_r",
    "finger_thumb_0_r", "finger_thumb_1_r", "finger_thumb_2_r", "finger_thumb_r_end",
    "finger_index_meta_r", "finger_index_0_r", "finger_index_1_r", "finger_index_2_r", "finger_index_r_end",
    "finger_middle_meta_r", "finger_middle_0_r", "finger_middle_1_r", "finger_middle_2_r", "finger_middle_r_end",
    "finger_ring_meta_r", "finger_ring_0_r", "finger_ring_1_r", "finger_ring_2_r", "finger_ring_r_end",
    "finger_pinky_meta_r", "finger_pinky_0_r", "finger_pinky_1_r", "finger_pinky_2_r", "finger_pinky_r_end",
    "finger_thumb_r_aux", "finger_index_r_aux", "finger_middle_r_aux", "finger_ring_r_aux", "finger_pinky_r_aux",
};
const int k_parent[HAND_BONES] = { -1, 0, 1, 2, 3, 4, 1, 6, 7, 8, 9, 1, 11, 12, 13, 14, 1, 16, 17, 18, 19,
                                   1, 21, 22, 23, 24, 0, 0, 0, 0, 0 };
// aux bones 26..30 hold the root-space transform of each finger's distal bone
const int k_distal[5] = { 4, 9, 14, 19, 24 };

enum { THUMB_FIRST = 2, THUMB_LAST = 5, INDEX_FIRST = 6, INDEX_LAST = 10, GRIP_FIRST = 11, GRIP_LAST = 25 };

// touch_poses_right.glb frames, identified by where the fingertips sit on SteamVR's CV1 Touch render model
enum {
    TF_REST = 0,          // thumb raised, index on the trigger, other fingers around the grip
    TF_A_TOUCH = 5, TF_A_CLICK = 6, TF_AB_TOUCH = 7, TF_B_CLICK = 8, TF_B_TOUCH = 9,
    TF_THUMBREST = 12,
    TF_STICK = 14,        // 3x3 grid 14..22: rows center, back, forward; columns center, right, left
    TF_TRIGGER_PULLED = 24,
    TF_POINT = 25,
    TF_COUNT
};
// hand_right_poses.glb
enum { HF_OPEN = 0, HF_FIST = 1, HF_COUNT };

enum ThumbSlot { TS_NONE, TS_A_TOUCH, TS_A_CLICK, TS_B_TOUCH, TS_B_CLICK, TS_AB, TS_REST, TS_STICK, TS_COUNT };
const int k_slotFrame[TS_COUNT] = { TF_REST, TF_A_TOUCH, TF_A_CLICK, TF_B_TOUCH, TF_B_CLICK, TF_AB_TOUCH, TF_THUMBREST, TF_STICK };

const float k_switchTime = 0.08f; // s for a full switch between thumb or index states

std::vector<Hand> g_touch, g_hand;

float Dot4(const float *a, const float *b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]; }

void Normalize(float *q)
{
    float n = sqrtf(Dot4(q, q));
    if (n < 1e-6f) {
        q[0] = 1;
        q[1] = q[2] = q[3] = 0;
        return;
    }
    for (int i = 0; i < 4; i++)
        q[i] /= n;
}

void QMul(float *o, const float *a, const float *b)
{
    float r[4] = {
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    };
    memcpy(o, r, sizeof(r));
}

void QRot(float *o, const float *q, const float *v)
{
    float p[4] = { 0, v[0], v[1], v[2] }, c[4] = { q[0], -q[1], -q[2], -q[3] }, r[4];
    QMul(r, q, p);
    QMul(r, r, c);
    o[0] = r[1];
    o[1] = r[2];
    o[2] = r[3];
}

// Weighted average of poses: Add each with its weight, then Finish.
struct HandSum {
    Hand h;
    float total = 0;

    void Add(const Hand &src, float w)
    {
        if (w <= 0)
            return;
        for (uint32_t b = 0; b < HAND_BONES; b++) {
            Bone &d = h[b];
            const Bone &s = src[b];
            if (total == 0) {
                for (int i = 0; i < 3; i++)
                    d.t[i] = s.t[i] * w;
                for (int i = 0; i < 4; i++)
                    d.q[i] = s.q[i] * w;
                continue;
            }
            float sign = Dot4(d.q, s.q) < 0 ? -w : w;
            for (int i = 0; i < 3; i++)
                d.t[i] += s.t[i] * w;
            for (int i = 0; i < 4; i++)
                d.q[i] += s.q[i] * sign;
        }
        total += w;
    }
    bool Finish()
    {
        if (total <= 0)
            return false;
        for (Bone &d : h) {
            for (int i = 0; i < 3; i++)
                d.t[i] /= total;
            Normalize(d.q);
        }
        return true;
    }
};

// Bones first..last of h move toward target by w.
void Blend(Hand &h, const Hand &target, int first, int last, float w)
{
    if (w <= 0)
        return;
    if (w > 1)
        w = 1;
    for (int b = first; b <= last; b++) {
        Bone &d = h[b];
        const Bone &s = target[b];
        float sign = Dot4(d.q, s.q) < 0 ? -1.0f : 1.0f;
        for (int i = 0; i < 3; i++)
            d.t[i] += (s.t[i] - d.t[i]) * w;
        for (int i = 0; i < 4; i++)
            d.q[i] = d.q[i] * (1 - w) + s.q[i] * sign * w;
        Normalize(d.q);
    }
}

void FixAux(Hand &h)
{
    Bone m[HAND_BONES];
    m[0] = h[0];
    for (int b = 1; b <= GRIP_LAST; b++) {
        const Bone &p = m[k_parent[b]];
        const Bone &l = h[b];
        QRot(m[b].t, p.q, l.t);
        for (int i = 0; i < 3; i++)
            m[b].t[i] += p.t[i];
        QMul(m[b].q, p.q, l.q);
    }
    // relative to the root bone, the aux bones' parent
    float rinv[4] = { h[0].q[0], -h[0].q[1], -h[0].q[2], -h[0].q[3] };
    for (int k = 0; k < 5; k++) {
        const Bone &d = m[k_distal[k]];
        Bone &a = h[26 + k];
        float rel[3] = { d.t[0] - h[0].t[0], d.t[1] - h[0].t[1], d.t[2] - h[0].t[2] };
        QRot(a.t, rinv, rel);
        QMul(a.q, rinv, d.q);
    }
}

// Right hand to left: mirror x. Bones below the wrist also turn 180 deg about their own x axis, as in
// SteamVR's vr_glove_left_skeleton.
void MirrorToLeft(Hand &h)
{
    for (uint32_t b = 0; b < HAND_BONES; b++) {
        Bone &d = h[b];
        float w = d.q[0], x = d.q[1], y = d.q[2], z = d.q[3];
        if (b <= 1) {
            d.t[0] = -d.t[0];
            d.q[2] = -y;
            d.q[3] = -z;
        } else if (k_parent[b] <= 1) {
            d.t[0] = -d.t[0];
            d.q[0] = -x;
            d.q[1] = w;
            d.q[2] = -z;
            d.q[3] = y;
        } else {
            for (int i = 0; i < 3; i++)
                d.t[i] = -d.t[i];
        }
    }
}

void Export(Hand h, bool left, vr::VRBoneTransform_t *out)
{
    if (left)
        MirrorToLeft(h);
    for (uint32_t b = 0; b < HAND_BONES; b++) {
        const Bone &s = h[b];
        out[b].position.v[0] = s.t[0];
        out[b].position.v[1] = s.t[1];
        out[b].position.v[2] = s.t[2];
        out[b].position.v[3] = 1;
        out[b].orientation.w = s.q[0];
        out[b].orientation.x = s.q[1];
        out[b].orientation.y = s.q[2];
        out[b].orientation.z = s.q[3];
    }
}

uint32_t Rd32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

bool ReadAccessor(const Json &g, const uint8_t *bin, size_t binLen, int idx, int comps, std::vector<float> &out)
{
    const Json &a = g["accessors"].At(idx);
    const char *type = comps == 4 ? "VEC4" : "VEC3";
    if (a["componentType"].Int(0) != 5126 || a["type"].str != type)
        return false;
    const Json &bv = g["bufferViews"].At(a["bufferView"].Int(-1));
    int count = a["count"].Int(0), stride = bv["byteStride"].Int(0);
    int viewOff = bv["byteOffset"].Int(0), accOff = a["byteOffset"].Int(0);
    if (bv.type != Json::Obj || bv["buffer"].Int(0) != 0 || count <= 0 || viewOff < 0 || accOff < 0)
        return false;
    size_t elem = (size_t)comps * 4, step = stride > 0 ? (size_t)stride : elem;
    size_t off = (size_t)viewOff + (size_t)accOff;
    if (step < elem || off > binLen || ((size_t)count - 1) * step + elem > binLen - off)
        return false;
    out.resize((size_t)count * comps);
    for (int i = 0; i < count; i++)
        memcpy(&out[(size_t)i * comps], bin + off + (size_t)i * step, elem);
    return true;
}

bool LoadGlb(const std::vector<uint8_t> &file, const char *what, std::vector<Hand> &frames, std::string *err)
{
    *err = std::string(what) + ": ";
    const uint8_t *d = file.data();
    if (file.size() < 28 || Rd32(d) != 0x46546C67 || Rd32(d + 4) != 2 || Rd32(d + 16) != 0x4E4F534A) {
        *err += "not a glTF 2 binary";
        return false;
    }
    size_t jsonLen = Rd32(d + 12);
    size_t binHdr = 20 + ((jsonLen + 3) & ~(size_t)3);
    if (jsonLen > file.size() - 20 || binHdr + 8 > file.size() || Rd32(d + binHdr + 4) != 0x004E4942 ||
        Rd32(d + binHdr) > file.size() - binHdr - 8) {
        *err += "bad chunks";
        return false;
    }
    const uint8_t *bin = d + binHdr + 8;
    size_t binLen = Rd32(d + binHdr);

    Json g;
    std::string text((const char *)d + 20, jsonLen);
    if (!JsonParser(text).Parse(g)) {
        *err += "bad JSON";
        return false;
    }

    const Json &nodes = g["nodes"];
    int node[HAND_BONES];
    for (uint32_t b = 0; b < HAND_BONES; b++) {
        node[b] = -1;
        for (size_t n = 0; n < nodes.Size(); n++) {
            const std::string &name = nodes.At((int)n)["name"].str;
            size_t colon = name.rfind(':');
            if (name.compare(colon == std::string::npos ? 0 : colon + 1, std::string::npos, k_boneNames[b]) == 0)
                node[b] = (int)n;
        }
    }

    const Json &anim = g["animations"].At(0);
    std::vector<float> tr[HAND_BONES], rot[HAND_BONES];
    const Json &channels = anim["channels"];
    for (size_t c = 0; c < channels.Size(); c++) {
        const Json &ch = channels.At((int)c);
        int n = ch["target"]["node"].Int(-1);
        const std::string &path = ch["target"]["path"].str;
        for (uint32_t b = 1; b < HAND_BONES; b++) {
            if (node[b] != n || (path != "translation" && path != "rotation"))
                continue;
            bool isRot = path == "rotation";
            int out = anim["samplers"].At(ch["sampler"].Int(-1))["output"].Int(-1);
            if (!ReadAccessor(g, bin, binLen, out, isRot ? 4 : 3, isRot ? rot[b] : tr[b])) {
                *err += std::string("bad keyframes for ") + k_boneNames[b];
                return false;
            }
        }
    }

    size_t count = (size_t)-1;
    for (uint32_t b = 1; b < HAND_BONES; b++) {
        if (tr[b].empty()) {
            *err += std::string("no keyframes for ") + k_boneNames[b];
            return false;
        }
        count = std::min(count, tr[b].size() / 3);
    }
    // the finger end bones have no rotation keyframes; they keep the node's rotation
    for (uint32_t b = 1; b < HAND_BONES; b++) {
        if (!rot[b].empty()) {
            count = std::min(count, rot[b].size() / 4);
            continue;
        }
        const Json &r = nodes.At(node[b])["rotation"];
        float q[4] = { 0, 0, 0, 1 };
        if (r.Size() == 4)
            for (int i = 0; i < 4; i++)
                q[i] = (float)r.At(i).num;
        for (size_t f = 0; f < count; f++)
            rot[b].insert(rot[b].end(), q, q + 4);
    }

    frames.resize(count);
    for (size_t f = 0; f < count; f++) {
        Hand &h = frames[f];
        h[0] = { { 0, 0, 0 }, { 1, 0, 0, 0 } };
        for (uint32_t b = 1; b < HAND_BONES; b++) {
            const float *t = &tr[b][f * 3], *q = &rot[b][f * 4]; // glTF quaternion order x, y, z, w
            Bone &o = h[b];
            if (k_parent[b] == 0) {
                // the files face +z; OpenVR's root bone is turned 180 deg about y from that
                o = { { -t[0], t[1], -t[2] }, { -q[1], q[2], q[3], -q[0] } };
            } else
                o = { { t[0], t[1], t[2] }, { q[3], q[0], q[1], q[2] } };
            Normalize(o.q);
        }
    }
    err->clear();
    return true;
}

// Right-hand pose on the controller.
Hand OnController(const float *thumb, float point, const HandInput &in)
{
    HandSum sum;
    for (int s = 0; s < TS_COUNT; s++) {
        if (s != TS_STICK) {
            sum.Add(g_touch[k_slotFrame[s]], thumb[s]);
            continue;
        }
        // bilinear over the stick grid
        float wx[3] = { 1 - fabsf(in.joyX), in.joyX > 0 ? in.joyX : 0, in.joyX < 0 ? -in.joyX : 0 };
        float wy[3] = { 1 - fabsf(in.joyY), in.joyY < 0 ? -in.joyY : 0, in.joyY > 0 ? in.joyY : 0 };
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                sum.Add(g_touch[TF_STICK + r * 3 + c], thumb[s] * wy[r] * wx[c]);
    }
    Hand h = sum.Finish() ? sum.h : g_touch[TF_REST];
    Blend(h, g_touch[TF_TRIGGER_PULLED], INDEX_FIRST, INDEX_LAST, in.trigger);
    Blend(h, g_touch[TF_POINT], INDEX_FIRST, INDEX_LAST, point);
    Blend(h, g_hand[HF_OPEN], GRIP_FIRST, GRIP_LAST, 1 - in.grip);
    FixAux(h);
    return h;
}

// Right-hand pose as if no controller were held: the same touches close the hand into a fist.
Hand Free(const float *thumb, float point, const HandInput &in)
{
    Hand h = g_hand[HF_OPEN];
    Blend(h, g_hand[HF_FIST], THUMB_FIRST, THUMB_LAST, 1 - thumb[TS_NONE]);
    Blend(h, g_hand[HF_FIST], INDEX_FIRST, INDEX_LAST, (1 - point) * (0.35f + 0.65f * in.trigger));
    Blend(h, g_hand[HF_FIST], GRIP_FIRST, GRIP_LAST, in.grip);
    FixAux(h);
    return h;
}

ThumbSlot ThumbTarget(const HandInput &in)
{
    if (in.aClick)
        return TS_A_CLICK;
    if (in.bClick)
        return TS_B_CLICK;
    if (in.stickTouch)
        return TS_STICK;
    if (in.aTouch && in.bTouch)
        return TS_AB;
    if (in.aTouch)
        return TS_A_TOUCH;
    if (in.bTouch)
        return TS_B_TOUCH;
    if (in.restTouch)
        return TS_REST;
    return TS_NONE;
}

float Clamp(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

HandInput Sanitize(HandInput in, bool left)
{
    in.trigger = Clamp(in.trigger, 0, 1);
    in.grip = Clamp(in.grip, 0, 1);
    // the left hand is the mirrored right hand, so its stick x is mirrored too
    in.joyX = Clamp(left ? -in.joyX : in.joyX, -1, 1);
    in.joyY = Clamp(in.joyY, -1, 1);
    return in;
}

} // namespace

bool LoadHandPoses(const std::vector<uint8_t> &touchPoses, const std::vector<uint8_t> &handPoses, std::string *err)
{
    std::vector<Hand> touch, hand;
    if (!LoadGlb(touchPoses, "touch_poses_right.glb", touch, err) || !LoadGlb(handPoses, "hand_right_poses.glb", hand, err))
        return false;
    if (touch.size() < TF_COUNT || hand.size() < HF_COUNT) {
        *err = "too few frames in the pose files";
        return false;
    }
    g_touch.swap(touch);
    g_hand.swap(hand);
    return true;
}

void HandSkeleton::Update(const HandInput &raw, float dt, bool left, vr::VRBoneTransform_t *withController,
                          vr::VRBoneTransform_t *withoutController)
{
    HandInput in = Sanitize(raw, left);
    float step = Clamp(dt, 0, 0.1f) / k_switchTime;

    ThumbSlot target = ThumbTarget(in);
    float total = 0;
    for (int s = 0; s < TS_COUNT; s++) {
        m_thumb[s] += Clamp((s == target ? 1.0f : 0.0f) - m_thumb[s], -step, step);
        total += m_thumb[s];
    }
    for (int s = 0; s < TS_COUNT; s++)
        m_thumb[s] = total > 0 ? m_thumb[s] / total : (s == TS_NONE ? 1.0f : 0.0f);

    float pointTarget = !in.triggerTouch && in.trigger < 0.05f ? 1.0f : 0.0f;
    m_point += Clamp(pointTarget - m_point, -step, step);

    Export(OnController(m_thumb, m_point, in), left, withController);
    Export(Free(m_thumb, m_point, in), left, withoutController);
}

void HandSkeleton::GripLimit(bool left, vr::VRBoneTransform_t *out)
{
    float thumb[TS_COUNT] = {};
    thumb[TS_REST] = 1;
    HandInput in = {};
    in.trigger = 1;
    in.grip = 1;
    Export(OnController(thumb, 0, in), left, out);
}

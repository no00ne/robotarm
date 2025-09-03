#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "LeapC.h"

/* make sure M_PI exists */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

/* ---------- axis map: arm = (Lz, Lx, Ly) ; mm -> m ---------- */
static inline void map_mm_to_m(const LEAP_VECTOR *L, double out_arm[3]) {
    out_arm[0] = -(double)L->z * 0.001;  /* X_arm = -Z_leap */
    out_arm[1] = -(double)L->x * 0.001;  /* Y_arm = -X_leap */
    out_arm[2] =  (double)L->y * 0.001;  /* Z_arm = Y_leap */
}

/* Convert Leap quaternion -> rotation matrix (row-major)
   Leap quaternion type: LEAP_QUATERNION has x,y,z,w
*/
static inline void quat_to_mat3(const LEAP_QUATERNION *q, double R[3][3]) {
    double x=q->x, y=q->y, z=q->z, w=q->w;
    double xx=x*x, yy=y*y, zz=z*z;
    double xy=x*y, xz=x*z, yz=y*z;
    double wx=w*x, wy=w*y, wz=w*z;

    R[0][0] = 1.0 - 2.0*(yy+zz);
    R[0][1] = 2.0*(xy - wz);
    R[0][2] = 2.0*(xz + wy);

    R[1][0] = 2.0*(xy + wz);
    R[1][1] = 1.0 - 2.0*(xx+zz);
    R[1][2] = 2.0*(yz - wx);

    R[2][0] = 2.0*(xz - wy);
    R[2][1] = 2.0*(yz + wx);
    R[2][2] = 1.0 - 2.0*(xx+yy);
}

/* Remap orientation from Leap frame to Arm frame by axis permutation:
   M = [[0,0,1],
        [1,0,0],
        [0,1,0]]
   R_arm = M * R_leap * M^T
*/
static inline void remap_rotation_to_arm(const LEAP_QUATERNION *q_leap, double R_arm[3][3]) {
    double RL[3][3];
    quat_to_mat3(q_leap, RL);

    double M[3][3]  = {{ 0, 0,-1},
                       {-1, 0, 0},
                       { 0, 1, 0}};
    double MT[3][3] = {{ 0,-1, 0},
                       { 0, 0, 1},
                       {-1, 0, 0}};
    double T[3][3];
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            T[i][j] = 0.0;
            for (int k=0;k<3;k++) T[i][j] += M[i][k]*RL[k][j];
        }
    }
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            R_arm[i][j] = 0.0;
            for (int k=0;k<3;k++) R_arm[i][j] += T[i][k]*MT[k][j];
        }
    }
}

/* Rotation matrix (row-major) -> ZYX Euler angles (rx,ry,rz) in radians.
   rx: roll (around X), ry: pitch (around Y), rz: yaw (around Z).
*/
static inline void rotmat_to_euler_zyx(const double R[3][3], double *rx, double *ry, double *rz) {
    double r11=R[0][0], r12=R[0][1], r13=R[0][2];
    double r21=R[1][0], r22=R[1][1], r23=R[1][2];
    double r31=R[2][0], r32=R[2][1], r33=R[2][2];

    double sy = -r31;
    if (sy <= -1.0) *ry = -M_PI_2;
    else if (sy >= 1.0) *ry = M_PI_2;
    else *ry = asin(sy);

    *rx = atan2(r32, r33);
    *rz = atan2(r21, r11);
}

/* distance squared to origin in Leap mm */
static inline double dist2_origin_mm(const LEAP_VECTOR *p) {
    double x = p->x, y = p->y, z = p->z;
    return x*x + y*y + z*z;
}

/* ---------------- global state ---------------- */
static LEAP_CONNECTION g_conn = NULL;
static int g_tracking_active = 0;
static int64_t g_last_hand_id = -1;
static double g_origin_arm[3] = {0.0,0.0,0.0};

/* init / shutdown */
static PyObject* py_init(PyObject* self, PyObject* args, PyObject* kwargs) {
    int optimize_hmd = 0;
    static char *kwlist[] = {"optimize_hmd", NULL};
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "|p", kwlist, &optimize_hmd)) return NULL;

    if (g_conn) Py_RETURN_NONE;
    eLeapRS rs = LeapCreateConnection(NULL, &g_conn);
    if (rs != eLeapRS_Success) {
        return PyErr_Format(PyExc_RuntimeError, "LeapCreateConnection failed: %d", rs);
    }
    rs = LeapOpenConnection(g_conn);
    if (rs != eLeapRS_Success) {
        LeapCloseConnection(g_conn);
        g_conn = NULL;
        return PyErr_Format(PyExc_RuntimeError, "LeapOpenConnection failed: %d", rs);
    }

    if (optimize_hmd) {
        uint64_t setFlags = (uint64_t)eLeapPolicyFlag_OptimizeHMD;
        uint64_t clearFlags = 0;
        LeapSetPolicyFlags(g_conn, setFlags, clearFlags);
    }

    g_tracking_active = 0;
    g_last_hand_id = -1;
    Py_RETURN_NONE;
}

static PyObject* py_shutdown(PyObject* self, PyObject* args) {
    if (g_conn) { LeapCloseConnection(g_conn); g_conn = NULL; }
    g_tracking_active = 0;
    g_last_hand_id = -1;
    Py_RETURN_NONE;
}

/* next_event(timeout_ms=0) -> dict or None
   Minimal protocol:
    - start: { "event":"start", "frame_id":..., "hand_id":..., "X":0,"Y":0,"Z":0, "RX":..., "RY":..., "RZ":... }
    - data:  { "event":"data",  "frame_id":..., "hand_id":..., "X":..., "Y":..., "Z":..., "RX":..., "RY":..., "RZ":..., "pinch_strength":... }
    - end:   { "event":"end" }
    - connected/disconnected
*/
static PyObject* py_next_event(PyObject* self, PyObject* args, PyObject* kwargs) {
    int timeout_ms = 0;
    static char *kwlist[] = {"timeout_ms", NULL};
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "|i", kwlist, &timeout_ms)) return NULL;
    if (!g_conn) Py_RETURN_NONE;

    LEAP_CONNECTION_MESSAGE msg;
    eLeapRS rs;

    /* 首次等待（尊重 timeout）*/
    rs = LeapPollConnection(g_conn, (uint32_t)timeout_ms, &msg);
    if (rs == eLeapRS_Timeout) Py_RETURN_NONE;
    if (rs != eLeapRS_Success) return PyErr_Format(PyExc_RuntimeError, "LeapPollConnection failed: %d", rs);

    /* 立刻处理连接/断连事件（优先） */
    if (msg.type == eLeapEventType_Connection) {
        PyObject* d = PyDict_New();
        PyDict_SetItemString(d, "event", PyUnicode_FromString("connected"));
        return d;
    }
    if (msg.type == eLeapEventType_ConnectionLost) {
        g_tracking_active = 0;
        g_last_hand_id = -1;
        PyObject* d = PyDict_New();
        PyDict_SetItemString(d, "event", PyUnicode_FromString("disconnected"));
        return d;
    }

    /* 我们要排空队列并保留“最新含手的 tracking 帧”。同时记录是否看到了空帧（nHands==0）。 */
    int have_latest = 0;
    int saw_empty_tracking = 0;

    unsigned long long latest_frame_id = 0;
    int latest_hand_id = -1;
    LEAP_VECTOR latest_palm_pos = {0};
    LEAP_QUATERNION latest_palm_orient = {0};
    float latest_pinch_strength = 0.0f;

    /* 先处理首次得到的 msg（若为 tracking，则根据 nHands 处理） */
    if (msg.type == eLeapEventType_Tracking && msg.tracking_event) {
        const LEAP_TRACKING_EVENT *te = msg.tracking_event;
        if (te->nHands == 0) {
            saw_empty_tracking = 1;
        } else {
            /* 选最近手并拷贝为 latest */
            int best = -1; double best_d2 = 1e300;
            for (uint32_t i=0;i<te->nHands;i++){
                double dx = te->pHands[i].palm.position.x;
                double dy = te->pHands[i].palm.position.y;
                double dz = te->pHands[i].palm.position.z;
                double d2 = dx*dx + dy*dy + dz*dz;
                if (d2 < best_d2) { best_d2 = d2; best = (int)i; }
            }
            if (best >= 0) {
                have_latest = 1;
                latest_frame_id = te->tracking_frame_id;
                latest_hand_id = (int)te->pHands[best].id;
                latest_palm_pos = te->pHands[best].palm.position;
                latest_palm_orient = te->pHands[best].palm.orientation;
                latest_pinch_strength = te->pHands[best].pinch_strength;
            }
        }
    }

    /* 排空剩余消息（非阻塞），更新 latest 或 saw_empty_tracking；遇到连接/断连立即返回 */
    while (1) {
        rs = LeapPollConnection(g_conn, 0, &msg); /* non-blocking */
        if (rs == eLeapRS_Timeout) break;
        if (rs != eLeapRS_Success) break;

        if (msg.type == eLeapEventType_Connection) {
            PyObject* d = PyDict_New();
            PyDict_SetItemString(d, "event", PyUnicode_FromString("connected"));
            return d;
        }
        if (msg.type == eLeapEventType_ConnectionLost) {
            g_tracking_active = 0;
            g_last_hand_id = -1;
            PyObject* d = PyDict_New();
            PyDict_SetItemString(d, "event", PyUnicode_FromString("disconnected"));
            return d;
        }

        if (msg.type == eLeapEventType_Tracking && msg.tracking_event) {
            const LEAP_TRACKING_EVENT *te = msg.tracking_event;
            if (te->nHands == 0) {
                saw_empty_tracking = 1;
            } else {
                int best = -1; double best_d2 = 1e300;
                for (uint32_t i=0;i<te->nHands;i++){
                    double dx = te->pHands[i].palm.position.x;
                    double dy = te->pHands[i].palm.position.y;
                    double dz = te->pHands[i].palm.position.z;
                    double d2 = dx*dx + dy*dy + dz*dz;
                    if (d2 < best_d2) { best_d2 = d2; best = (int)i; }
                }
                if (best >= 0) {
                    have_latest = 1;
                    latest_frame_id = te->tracking_frame_id;
                    latest_hand_id = (int)te->pHands[best].id;
                    latest_palm_pos = te->pHands[best].palm.position;
                    latest_palm_orient = te->pHands[best].palm.orientation;
                    latest_pinch_strength = te->pHands[best].pinch_strength;
                }
            }
        }
        /* 其他事件忽略 */
    }

    /* 如果没有任何含手的帧，但在排空过程中看到了空帧且之前处于跟踪状态 -> 返回 end 事件 */
    if (!have_latest && saw_empty_tracking && g_tracking_active) {
        g_tracking_active = 0;
        g_last_hand_id = -1;
        PyObject* d = PyDict_New();
        PyDict_SetItemString(d, "event", PyUnicode_FromString("end"));
        return d;
    }

    /* 如果有最新含手的帧，按 start/data 逻辑返回（与之前实现一致）*/
    if (have_latest) {
        double palm_arm[3];
        map_mm_to_m(&latest_palm_pos, palm_arm);

        double R_arm[3][3];
        remap_rotation_to_arm(&latest_palm_orient, R_arm);
        double rx_rad, ry_rad, rz_rad;
        rotmat_to_euler_zyx(R_arm, &rx_rad, &ry_rad, &rz_rad);

        double pinch_strength = (double)latest_pinch_strength;

        if (!g_tracking_active || (int64_t)latest_hand_id != g_last_hand_id) {
            /* start */
            g_tracking_active = 1;
            g_last_hand_id = latest_hand_id;
            g_origin_arm[0] = palm_arm[0];
            g_origin_arm[1] = palm_arm[1];
            g_origin_arm[2] = palm_arm[2];

            PyObject* d = PyDict_New();
            PyDict_SetItemString(d,"event", PyUnicode_FromString("start"));
            PyDict_SetItemString(d,"frame_id", PyLong_FromUnsignedLongLong(latest_frame_id));
            PyDict_SetItemString(d,"hand_id", PyLong_FromLong((long)latest_hand_id));
            PyDict_SetItemString(d,"X", PyLong_FromLongLong(0));
            PyDict_SetItemString(d,"Y", PyLong_FromLongLong(0));
            PyDict_SetItemString(d,"Z", PyLong_FromLongLong(0));
            long long RX_mdeg = (long long)lround(rx_rad*(180.0/M_PI)*1000.0);
            long long RY_mdeg = (long long)lround(ry_rad*(180.0/M_PI)*1000.0);
            long long RZ_mdeg = (long long)lround(rz_rad*(180.0/M_PI)*1000.0);
            PyDict_SetItemString(d,"RX", PyLong_FromLongLong(RX_mdeg));
            PyDict_SetItemString(d,"RY", PyLong_FromLongLong(RY_mdeg));
            PyDict_SetItemString(d,"RZ", PyLong_FromLongLong(RZ_mdeg));
            PyDict_SetItemString(d,"pinch_strength", PyFloat_FromDouble(pinch_strength));
            return d;
        } else {
            /* data */
            double rel_m[3] = { palm_arm[0]-g_origin_arm[0], palm_arm[1]-g_origin_arm[1], palm_arm[2]-g_origin_arm[2] };
            long long X_um = (long long)llround(rel_m[0] * 1e6);
            long long Y_um = (long long)llround(rel_m[1] * 1e6);
            long long Z_um = (long long)llround(rel_m[2] * 1e6);

            long long RX_mdeg = (long long)llround(rx_rad*(180.0/M_PI)*1000.0);
            long long RY_mdeg = (long long)llround(ry_rad*(180.0/M_PI)*1000.0);
            long long RZ_mdeg = (long long)llround(rz_rad*(180.0/M_PI)*1000.0);

            PyObject* d = PyDict_New();
            PyDict_SetItemString(d,"event", PyUnicode_FromString("data"));
            PyDict_SetItemString(d,"frame_id", PyLong_FromUnsignedLongLong(latest_frame_id));
            PyDict_SetItemString(d,"hand_id", PyLong_FromLong((long)latest_hand_id));
            PyDict_SetItemString(d,"X", PyLong_FromLongLong(X_um));
            PyDict_SetItemString(d,"Y", PyLong_FromLongLong(Y_um));
            PyDict_SetItemString(d,"Z", PyLong_FromLongLong(Z_um));
            PyDict_SetItemString(d,"RX", PyLong_FromLongLong(RX_mdeg));
            PyDict_SetItemString(d,"RY", PyLong_FromLongLong(RY_mdeg));
            PyDict_SetItemString(d,"RZ", PyLong_FromLongLong(RZ_mdeg));
            PyDict_SetItemString(d,"pinch_strength", PyFloat_FromDouble(pinch_strength));
            return d;
        }
    }

    /* 否则什么也没有 */
    Py_RETURN_NONE;
}


/* module def */
static PyMethodDef Methods[] = {
    {"init", (PyCFunction)py_init, METH_VARARGS|METH_KEYWORDS, "Initialize LeapC (optimize_hmd optional bool)"},
    {"next_event", (PyCFunction)py_next_event, METH_VARARGS|METH_KEYWORDS, "Poll next event (timeout_ms optional)"},
    {"shutdown", (PyCFunction)py_shutdown, METH_NOARGS, "Shutdown Leap connection"},
    {NULL, NULL, 0, NULL}
};

static struct PyModuleDef ModuleDef = {
    PyModuleDef_HEAD_INIT, "leap_ext", "LeapC minimal extension (relative XYZ μm, RX/RY/RZ millideg, pinch_strength)", -1, Methods
};

PyMODINIT_FUNC PyInit_leap_ext(void) {
    return PyModule_Create(&ModuleDef);
}

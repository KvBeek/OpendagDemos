// File: Assets/Scripts/HeadCoupledPerspective.cs
using UnityEngine;

/// <summary>
/// Head-Coupled Perspective met schaal-kalibratie, gains, deadzone en stabiele smoothing.
/// Gebruik dit als je tracker niet in meters is of te "snappy" voelt.
/// </summary>
[RequireComponent(typeof(Camera))]
public class HeadCoupledPerspective : MonoBehaviour
{
    [Header("Tracking")]
    public Transform headTransform;                 // ruwe tracker transform
    public Vector3 headLocalOffset = Vector3.zero;  // offset naar midpoint tussen ogen
    [Tooltip("Vermenigvuldig tracker-units naar meters. 0.01 als tracker=cm, 1.0 als tracker=meters.")]
    public float headUnitsToMeters = 1.0f;

    [Header("Screen Size")]
    public Transform screenTransform;               // centrum schermvlak
    [Tooltip("In centimeter invullen = aan. In meter invullen = uit.")]
    public bool screenSizeInCentimeters = true;
    public float screenWidth = 31.5f;               // cm of m
    public float screenHeight = 19.7f;

    [Header("Behavior")]
    [Tooltip("Flip als de cyan normaal niet naar de speler wijst.")]
    public bool flipScreenNormal = false;

    [Tooltip("XY parallax gain. <1 langzamer, >1 sterker.")]
    public float xyGain = 0.6f;
    [Tooltip("Z (diepte) gain. <1 dempt, >1 versterkt.")]
    public float zGain = 0.4f;
    [Tooltip("Bewegingsdrempel in meter. Kleine jitter wordt genegeerd.")]
    public float deadzone = 0.01f;

    [Tooltip("Smoothing half-life (s). 0=uit, 0.08-0.2 is typisch.")]
    public float smoothHalfLife = 0.12f;
    [Tooltip("Clamp verplaatsing/s in meter om spikes te temmen.")]
    public float maxHeadSpeed = 2.0f;

    [Header("Clip Planes")]
    public float nearClip = 0.02f;
    public float farClip = 50f;
    public bool autoClampNearToD = true;

    [Header("Debug")]
    public bool showRuntimeDebug = true;
    public bool drawGizmos = true;

    private Camera _cam;
    private Vector3 _smoothedHeadMeters;   // in meter-ruimte
    private Vector3 _prevHeadMeters;       // voor snelheid
    private float _lastD, _l, _r, _b, _t;

    // Kalibratie
    private Vector3 _calibRawHead;         // ruwe head bij calibratie
    private bool _hasCalib;

    void Awake()
    {
        _cam = GetComponent<Camera>();
        var h = GetHeadWorldMeters();
        _smoothedHeadMeters = h;
        _prevHeadMeters = h;
    }

    void LateUpdate()
    {
        if (headTransform == null || screenTransform == null) return;

        // 1) Head in meters
        Vector3 headMeters = GetHeadWorldMeters();

        // 2) Max-speed clamp
        float dt = Mathf.Max(Time.deltaTime, 1e-4f);
        Vector3 delta = headMeters - _prevHeadMeters;
        float speed = delta.magnitude / dt;
        if (speed > maxHeadSpeed)
        {
            float ratio = maxHeadSpeed / Mathf.Max(speed, 1e-4f);
            headMeters = _prevHeadMeters + delta * ratio;
        }
        _prevHeadMeters = headMeters;

        // 3) Deadzone (rond schermcentrum projecteren)
        Vector3 headRel = headMeters - screenTransform.position;
        if (headRel.magnitude < deadzone)
        {
            headMeters = screenTransform.position + headRel.normalized * 0f; // nul
        }

        // 4) Gains: demp XY en Z apart
        Vector3 local = WorldToScreenBasis(headMeters) - WorldToScreenBasis(screenTransform.position); // in scherm-basis
        local = new Vector3(local.x * xyGain, local.y * xyGain, local.z * zGain);
        headMeters = ScreenBasisToWorld(local) + screenTransform.position;

        // 5) Exponentiële smoothing met half-life
        if (smoothHalfLife > 0f)
        {
            float alpha = 1f - Mathf.Exp(-Mathf.Log(2f) * dt / Mathf.Max(smoothHalfLife, 1e-4f));
            _smoothedHeadMeters = Vector3.Lerp(_smoothedHeadMeters, headMeters, alpha);
        }
        else
        {
            _smoothedHeadMeters = headMeters;
        }

        // 6) Schermhoeken + basis
        GetScreenCorners(out Vector3 pa, out Vector3 pb, out Vector3 pc);
        Vector3 vr = (pb - pa).normalized;
        Vector3 vu = (pc - pa).normalized;
        Vector3 vn = Vector3.Cross(vr, vu).normalized;

        // 7) Normaal richting speler afdwingen
        if (flipScreenNormal || Vector3.Dot(vn, (_smoothedHeadMeters - screenTransform.position)) < 0f)
            vn = -vn;

        // 8) Kooima
        float d = Vector3.Dot(pa - _smoothedHeadMeters, vn);
        _lastD = d;

        float n = Mathf.Max(nearClip, 0.005f);
        float f = Mathf.Max(farClip, n + 0.01f);
        if (autoClampNearToD && d <= n * 1.05f) d = n * 1.05f;

        _l = Vector3.Dot(vr, (pa - _smoothedHeadMeters)) * n / d;
        _r = Vector3.Dot(vr, (pb - _smoothedHeadMeters)) * n / d;
        _b = Vector3.Dot(vu, (pa - _smoothedHeadMeters)) * n / d;
        _t = Vector3.Dot(vu, (pc - _smoothedHeadMeters)) * n / d;

        Matrix4x4 proj = PerspectiveOffCenter(_l, _r, _b, _t, n, f);
        proj = GL.GetGPUProjectionMatrix(proj, false);

        Matrix4x4 view = BuildViewMatrix(_smoothedHeadMeters, vr, vu, vn);

        _cam.worldToCameraMatrix = view;
        _cam.projectionMatrix = proj;
    }

    // ------- Helpers -------

    Vector3 GetHeadWorldMeters()
    {
        // Let op: schaal ruwe tracker-units -> meters
        Vector3 raw = headTransform.TransformPoint(headLocalOffset);
        if (_hasCalib)
        {
            // schaal is al in headUnitsToMeters geschreven door CalibrateDistanceMeters()
        }
        return screenTransform.position + (raw - screenTransform.position) * headUnitsToMeters;
    }

    void GetScreenCorners(out Vector3 pa, out Vector3 pb, out Vector3 pc)
    {
        float w = screenWidth;
        float h = screenHeight;
        if (screenSizeInCentimeters) { w *= 0.01f; h *= 0.01f; }

        Vector3 right = screenTransform.right * (w * 0.5f);
        Vector3 up    = screenTransform.up    * (h * 0.5f);
        Vector3 c     = screenTransform.position;

        pa = c - right - up; // LL
        pb = c + right - up; // LR
        pc = c - right + up; // UL
    }

    // Zet wereldpos om naar scherm-basis-coördinaten
    Vector3 WorldToScreenBasis(Vector3 p)
    {
        Vector3 c = screenTransform.position;
        Vector3 vr = screenTransform.right;
        Vector3 vu = screenTransform.up;
        Vector3 vn = Vector3.Cross(vr, vu).normalized;
        return new Vector3(Vector3.Dot(p - c, vr), Vector3.Dot(p - c, vu), Vector3.Dot(p - c, vn));
    }

    Vector3 ScreenBasisToWorld(Vector3 v)
    {
        Vector3 vr = screenTransform.right;
        Vector3 vu = screenTransform.up;
        Vector3 vn = Vector3.Cross(vr, vu).normalized;
        if (flipScreenNormal) vn = -vn;
        return screenTransform.position + vr * v.x + vu * v.y + vn * v.z;
    }

    static Matrix4x4 PerspectiveOffCenter(float l, float r, float b, float t, float n, float f)
    {
        float x = 2f * n / (r - l);
        float y = 2f * n / (t - b);
        float a = (r + l) / (r - l);
        float e = (t + b) / (t - b);
        float c = -(f + n) / (f - n);
        float d = -(2f * f * n) / (f - n);

        Matrix4x4 m = new Matrix4x4();
        m[0,0]=x; m[0,1]=0; m[0,2]=a; m[0,3]=0;
        m[1,0]=0; m[1,1]=y; m[1,2]=e; m[1,3]=0;
        m[2,0]=0; m[2,1]=0; m[2,2]=c; m[2,3]=d;
        m[3,0]=0; m[3,1]=0; m[3,2]=-1; m[3,3]=0;
        return m;
    }

    static Matrix4x4 BuildViewMatrix(Vector3 pe, Vector3 vr, Vector3 vu, Vector3 vn)
    {
        Matrix4x4 r = Matrix4x4.identity;
        r.SetColumn(0, new Vector4(vr.x, vr.y, vr.z, 0f));
        r.SetColumn(1, new Vector4(vu.x, vu.y, vu.z, 0f));
        r.SetColumn(2, new Vector4(vn.x, vn.y, vn.z, 0f));
        r.SetColumn(3, new Vector4(0f, 0f, 0f, 1f));
        return r.transpose * Matrix4x4.Translate(-pe);
    }

    // ------- Kalibratie -------

    /// <summary>
    /// Sla huidige ruwe head-positie op voor kalibratie.
    /// </summary>
    public void BeginCalibration()
    {
        _calibRawHead = headTransform.TransformPoint(headLocalOffset);
        _hasCalib = true;
    }

    /// <summary>
    /// Vul de echte afstand oog→scherm in meters. Berekent headUnitsToMeters.
    /// Gebruik: kijk recht naar het midden, klik BeginCalibration(), meet afstand (bijv. 0.55), roep CalibrateDistanceMeters(0.55f).
    /// </summary>
    public void CalibrateDistanceMeters(float trueDistanceMeters)
    {
        Vector3 c = screenTransform.position;
        Vector3 vr = screenTransform.right;
        Vector3 vu = screenTransform.up;
        Vector3 vn = Vector3.Cross(vr, vu).normalized;
        if (flipScreenNormal) vn = -vn;

        float rawD = Vector3.Dot(c - _calibRawHead, vn); // in tracker-units
        if (Mathf.Abs(rawD) < 1e-4f) rawD = 1e-4f;
        headUnitsToMeters = trueDistanceMeters / rawD;   // schaal zodat d klopt
    }

    // ------- Debug UI / Gizmos -------

    void OnGUI()
    {
        if (!showRuntimeDebug) return;
        const float pad = 8f;
        GUI.Box(new Rect(pad, pad, 360f, 150f), "HCP Debug");
        GUILayout.BeginArea(new Rect(pad + 8f, pad + 24f, 344f, 120f));
        GUILayout.Label($"d (eye→screen): {_lastD:0.000} m");
        GUILayout.Label($"l/r/b/t @ near: {_l:0.000} / {_r:0.000} / {_b:0.000} / {_t:0.000}");
        GUILayout.Label($"headUnitsToMeters: {headUnitsToMeters:0.0000}");
        GUILayout.Label($"xyGain:{xyGain:0.00}  zGain:{zGain:0.00}  deadzone:{deadzone:0.000}  halfLife:{smoothHalfLife:0.000}s");
        if (GUI.Button(new Rect(pad + 8f, 120f, 150f, 20f), "Begin Calibration")) BeginCalibration();
        if (GUI.Button(new Rect(pad + 170f, 120f, 180f, 20f), "Set 0.55 m & Solve"))
        {
            CalibrateDistanceMeters(0.55f); // voorbeeld
        }
        GUILayout.EndArea();
    }

    void OnDrawGizmos()
    {
        if (!drawGizmos || screenTransform == null) return;
        GetScreenCorners(out Vector3 pa, out Vector3 pb, out Vector3 pc);
        Vector3 pd = pb + (pc - pa);

        Gizmos.color = Color.green;
        Gizmos.DrawLine(pa, pb);
        Gizmos.DrawLine(pb, pd);
        Gizmos.DrawLine(pd, pc);
        Gizmos.DrawLine(pc, pa);

        Vector3 vr = (pb - pa).normalized;
        Vector3 vu = (pc - pa).normalized;
        Vector3 vn = Vector3.Cross(vr, vu).normalized;
        if (flipScreenNormal) vn = -vn;

        Gizmos.color = Color.cyan;
        Gizmos.DrawRay(screenTransform.position, vn * 0.2f);

        if (headTransform != null)
        {
            Vector3 peRaw = headTransform.TransformPoint(headLocalOffset);
            Vector3 pe = screenTransform.position + (peRaw - screenTransform.position) * headUnitsToMeters;
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(pe, 0.01f);
            Gizmos.DrawLine(pe, screenTransform.position);
        }
    }
}

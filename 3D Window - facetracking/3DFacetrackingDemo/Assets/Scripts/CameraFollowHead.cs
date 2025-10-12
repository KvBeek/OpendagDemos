// File: Assets/Scripts/CameraFollowHeadScaled.cs
using UnityEngine;

public class CameraFollowHead : MonoBehaviour
{
    [Header("Follow")]
    [SerializeField] private Transform target;

    [Header("Scale")]
    [SerializeField] private float multiplier = 1.5f;      // >1 versterkt, <1 dempt
    [SerializeField] private Transform pivotTransform;     // laat leeg om Start-positie te gebruiken
    [SerializeField] private Vector3 axisMask = Vector3.one; // zet bv. Z=0 om Z te negeren

    private Vector3 pivotPosition;

    void Start()
    {
        pivotPosition = pivotTransform ? pivotTransform.position : (target ? target.position : transform.position);
    }

    void Update()
    {
        if (!target) return;

        Vector3 pivot = pivotTransform ? pivotTransform.position : pivotPosition;
        Vector3 delta = Vector3.Scale(target.position - pivot, axisMask);
        transform.position = pivot + delta * multiplier;
    }

    /// <summary>Reset pivot naar huidige target-positie.</summary>
    public void RecenterPivot()
    {
        pivotPosition = target ? target.position : transform.position;
    }

    /// <summary>Stel multiplier live in.</summary>
    public void SetMultiplier(float m) => multiplier = m;
}

using UnityEngine;
using UnityEngine.InputSystem;

namespace Teleoperation.Robots.G1
{
    [DisallowMultipleComponent]
    public sealed class G1TargetMouseDrag : MonoBehaviour
    {
        [SerializeField, Min(10f)] private float pickRadiusPixels = 75f;
        private Camera activeCamera;
        private Plane dragPlane;
        private Vector3 offset;
        private bool dragging;

        private void Update()
        {
            var pointer = Pointer.current;
            if (pointer == null)
                return;
            if (pointer.press.wasPressedThisFrame)
                Begin(pointer.position.ReadValue());
            if (dragging && pointer.press.isPressed)
                Drag(pointer.position.ReadValue());
            if (pointer.press.wasReleasedThisFrame)
                dragging = false;
        }

        private void Begin(Vector2 screenPosition)
        {
            activeCamera = Camera.main;
            if (activeCamera == null)
                return;
            var projected = activeCamera.WorldToScreenPoint(transform.position);
            if (projected.z <= 0f || Vector2.Distance(screenPosition, projected) > pickRadiusPixels)
                return;

            var ray = activeCamera.ScreenPointToRay(screenPosition);
            dragPlane = new Plane(-activeCamera.transform.forward, transform.position);
            if (!dragPlane.Raycast(ray, out var distance))
                return;
            offset = transform.position - ray.GetPoint(distance);
            dragging = true;
        }

        private void Drag(Vector2 screenPosition)
        {
            var ray = activeCamera.ScreenPointToRay(screenPosition);
            if (dragPlane.Raycast(ray, out var distance))
                transform.position = ray.GetPoint(distance) + offset;
        }
    }
}

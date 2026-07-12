using UnityEngine;
using UnityEngine.InputSystem;

namespace Teleoperation.Robots.UR5e
{
    [DisallowMultipleComponent]
    public sealed class UR5eTargetMouseDrag : MonoBehaviour
    {
        [SerializeField, Min(10f)] private float pickRadiusPixels = 70f;

        private Camera activeCamera;
        private Plane dragPlane;
        private Vector3 grabOffset;
        private bool dragging;

        private void Awake()
        {
            activeCamera = Camera.main;
        }

        private void Update()
        {
            var pointer = Pointer.current;
            if (pointer == null)
                return;

            if (pointer.press.wasPressedThisFrame)
                TryBeginDrag(pointer.position.ReadValue());

            if (dragging && pointer.press.isPressed)
                ContinueDrag(pointer.position.ReadValue());

            if (pointer.press.wasReleasedThisFrame)
                dragging = false;
        }

        private void TryBeginDrag(Vector2 screenPosition)
        {
            activeCamera = FindViewingCamera(screenPosition);
            if (activeCamera == null)
                return;

            var targetScreenPosition = activeCamera.WorldToScreenPoint(transform.position);
            if (targetScreenPosition.z <= 0f ||
                Vector2.Distance(screenPosition, targetScreenPosition) > pickRadiusPixels)
            {
                return;
            }

            var ray = activeCamera.ScreenPointToRay(screenPosition);
            dragPlane = new Plane(-activeCamera.transform.forward, transform.position);
            if (!dragPlane.Raycast(ray, out var enter))
                return;

            grabOffset = transform.position - ray.GetPoint(enter);
            dragging = true;
        }

        private void ContinueDrag(Vector2 screenPosition)
        {
            var ray = activeCamera.ScreenPointToRay(screenPosition);
            if (dragPlane.Raycast(ray, out var enter))
                transform.position = ray.GetPoint(enter) + grabOffset;
        }

        private Camera FindViewingCamera(Vector2 screenPosition)
        {
            if (activeCamera != null &&
                activeCamera.isActiveAndEnabled &&
                activeCamera.pixelRect.Contains(screenPosition))
            {
                return activeCamera;
            }

            if (Camera.main != null && Camera.main.pixelRect.Contains(screenPosition))
                return Camera.main;

            foreach (var candidate in Camera.allCameras)
            {
                if (candidate.isActiveAndEnabled && candidate.pixelRect.Contains(screenPosition))
                    return candidate;
            }

            return null;
        }
    }
}

using UnityEngine;
using UnityEngine.InputSystem;

namespace Teleoperation
{
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Camera))]
    public sealed class FlyCameraController : MonoBehaviour
    {
        [SerializeField, Min(0.1f)] private float moveSpeed = 1.5f;
        [SerializeField, Min(1f)] private float boostMultiplier = 3f;
        [SerializeField, Min(0.01f)] private float lookSensitivity = 0.12f;

        private float yaw;
        private float pitch;

        private void Awake()
        {
            var angles = transform.eulerAngles;
            yaw = angles.y;
            pitch = NormalizeAngle(angles.x);
        }

        private void Update()
        {
            var keyboard = Keyboard.current;
            var mouse = Mouse.current;
            if (keyboard == null || mouse == null)
                return;

            var looking = mouse.rightButton.isPressed;
            if (looking)
            {
                var delta = mouse.delta.ReadValue();
                yaw += delta.x * lookSensitivity;
                pitch = Mathf.Clamp(pitch - delta.y * lookSensitivity, -85f, 85f);
                transform.rotation = Quaternion.Euler(pitch, yaw, 0f);
            }

            var input = Vector3.zero;
            if (keyboard.wKey.isPressed) input.z += 1f;
            if (keyboard.sKey.isPressed) input.z -= 1f;
            if (keyboard.dKey.isPressed) input.x += 1f;
            if (keyboard.aKey.isPressed) input.x -= 1f;
            if (keyboard.eKey.isPressed) input.y += 1f;
            if (keyboard.qKey.isPressed) input.y -= 1f;

            if (input.sqrMagnitude > 1f)
                input.Normalize();

            var speed = moveSpeed;
            if (keyboard.leftShiftKey.isPressed || keyboard.rightShiftKey.isPressed)
                speed *= boostMultiplier;

            transform.position += transform.TransformDirection(input) * (speed * Time.unscaledDeltaTime);
        }

        private static float NormalizeAngle(float angle)
        {
            return angle > 180f ? angle - 360f : angle;
        }
    }
}


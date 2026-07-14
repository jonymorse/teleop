using UnityEngine;
using UnityEngine.InputSystem;

namespace Teleoperation.Robots.G1
{
    [DisallowMultipleComponent]
    public sealed class G1DesktopCameraNavigation : MonoBehaviour
    {
        [SerializeField, Min(0.1f)] private float moveSpeed = 1.5f;
        [SerializeField, Min(1f)] private float boostMultiplier = 3f;
        [SerializeField, Min(0.01f)] private float lookSensitivity = 0.12f;
        [SerializeField, Min(0.1f)] private float minimumSpeed = 0.25f;
        [SerializeField, Min(0.1f)] private float maximumSpeed = 8f;

        private float yaw;
        private float pitch;
        private bool looking;

        private void Awake()
        {
            var euler = transform.eulerAngles;
            yaw = euler.y;
            pitch = NormalizeAngle(euler.x);
        }

        private void Update()
        {
            var keyboard = Keyboard.current;
            var mouse = Mouse.current;
            if (keyboard == null || mouse == null)
                return;

            if (mouse.rightButton.wasPressedThisFrame)
            {
                looking = true;
                Cursor.lockState = CursorLockMode.Locked;
                Cursor.visible = false;
            }
            if (mouse.rightButton.wasReleasedThisFrame)
            {
                looking = false;
                Cursor.lockState = CursorLockMode.None;
                Cursor.visible = true;
            }

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

            var scroll = mouse.scroll.ReadValue().y;
            if (!Mathf.Approximately(scroll, 0f))
                moveSpeed = Mathf.Clamp(moveSpeed + Mathf.Sign(scroll) * 0.25f, minimumSpeed, maximumSpeed);

            if (input.sqrMagnitude > 1f)
                input.Normalize();
            var speed = moveSpeed * (keyboard.leftShiftKey.isPressed ? boostMultiplier : 1f);
            var worldDirection = transform.right * input.x + transform.forward * input.z + Vector3.up * input.y;
            transform.position += worldDirection * (speed * Time.deltaTime);
        }

        private void OnDisable()
        {
            if (!looking)
                return;
            Cursor.lockState = CursorLockMode.None;
            Cursor.visible = true;
        }

        private void OnGUI()
        {
            GUI.Box(new Rect(Screen.width - 355f, 18f, 337f, 66f), string.Empty);
            GUI.Label(new Rect(Screen.width - 342f, 27f, 315f, 22f),
                "NAVIGATION  RMB look | WASD move | Q/E down/up");
            GUI.Label(new Rect(Screen.width - 342f, 51f, 315f, 22f),
                $"Shift boost | Wheel speed: {moveSpeed:F2} m/s");
        }

        private static float NormalizeAngle(float angle)
        {
            return angle > 180f ? angle - 360f : angle;
        }
    }

    internal static class G1DesktopCameraNavigationInstaller
    {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void Install()
        {
            if (UnityEngine.SceneManagement.SceneManager.GetActiveScene().name != "G1ImportTest")
                return;
            var camera = Camera.main;
            if (camera != null && camera.GetComponent<G1DesktopCameraNavigation>() == null)
                camera.gameObject.AddComponent<G1DesktopCameraNavigation>();
        }
    }
}

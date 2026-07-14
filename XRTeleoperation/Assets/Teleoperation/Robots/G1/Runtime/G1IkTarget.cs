using UnityEngine;

namespace Teleoperation.Robots.G1
{
    public enum G1IkStatus
    {
        Solving,
        Reached,
        Unreachable,
        Disabled
    }

    [DisallowMultipleComponent]
    public sealed class G1IkTarget : MonoBehaviour
    {
        private Renderer targetRenderer;
        private G1IkStatus status;

        public G1IkStatus Status => status;

        private void Awake()
        {
            targetRenderer = GetComponentInChildren<Renderer>();
        }

        public void SetStatus(G1IkStatus next)
        {
            status = next;
            if (targetRenderer == null)
                return;

            targetRenderer.material.color = next switch
            {
                G1IkStatus.Reached => new Color(0.15f, 1f, 0.35f),
                G1IkStatus.Unreachable => new Color(1f, 0.15f, 0.12f),
                G1IkStatus.Disabled => new Color(0.45f, 0.45f, 0.45f),
                _ => new Color(1f, 0.65f, 0.08f)
            };
        }
    }
}

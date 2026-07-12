using UnityEngine;

namespace Teleoperation.Robots.UR5e
{
    public enum IkTargetStatus
    {
        Solving,
        Reachable,
        Unreachable,
        Disabled
    }

    [DisallowMultipleComponent]
    public sealed class UR5eIkTarget : MonoBehaviour
    {
        [SerializeField] private Color solvingColor = new(1f, 0.65f, 0.05f, 1f);
        [SerializeField] private Color reachableColor = new(0.1f, 0.9f, 0.25f, 1f);
        [SerializeField] private Color unreachableColor = new(0.95f, 0.1f, 0.1f, 1f);
        [SerializeField] private Color disabledColor = new(0.4f, 0.4f, 0.4f, 1f);

        private Renderer[] renderers;
        private MaterialPropertyBlock propertyBlock;
        private IkTargetStatus status;

        public IkTargetStatus Status => status;

        private void Awake()
        {
            renderers = GetComponentsInChildren<Renderer>(true);
            propertyBlock = new MaterialPropertyBlock();
            SetStatus(IkTargetStatus.Solving);
        }

        public void SetStatus(IkTargetStatus newStatus)
        {
            status = newStatus;
            if (renderers == null)
                renderers = GetComponentsInChildren<Renderer>(true);
            propertyBlock ??= new MaterialPropertyBlock();

            var color = newStatus switch
            {
                IkTargetStatus.Reachable => reachableColor,
                IkTargetStatus.Unreachable => unreachableColor,
                IkTargetStatus.Disabled => disabledColor,
                _ => solvingColor
            };

            foreach (var targetRenderer in renderers)
            {
                targetRenderer.GetPropertyBlock(propertyBlock);
                propertyBlock.SetColor("_BaseColor", color);
                propertyBlock.SetColor("_Color", color);
                targetRenderer.SetPropertyBlock(propertyBlock);
            }
        }
    }
}


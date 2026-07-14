using UnityEngine;

namespace Teleoperation.Robots.G1
{
    public enum G1GraspFeedback
    {
        None, Near, Contact, Ready, Held, Slipping
    }

    [DisallowMultipleComponent]
    [RequireComponent(typeof(Rigidbody))]
    public sealed class G1Graspable : MonoBehaviour
    {
        [SerializeField] private string displayName = "Manipulation Object";
        private Renderer[] renderers;
        private MaterialPropertyBlock[] propertyBlocks;
        private Color[] baseColors;

        public string DisplayName => displayName;
        public void SetDisplayName(string value) => displayName = value;

        private void Awake()
        {
            renderers = GetComponentsInChildren<Renderer>(true);
            propertyBlocks = new MaterialPropertyBlock[renderers.Length];
            baseColors = new Color[renderers.Length];
            for (var i = 0; i < renderers.Length; i++)
            {
                propertyBlocks[i] = new MaterialPropertyBlock();
                var material = renderers[i].sharedMaterial;
                baseColors[i] = material != null && material.HasProperty("_BaseColor")
                    ? material.GetColor("_BaseColor")
                    : material != null && material.HasProperty("_Color")
                        ? material.GetColor("_Color") : Color.white;
            }
        }

        public void SetFeedback(G1GraspFeedback feedback)
        {
            if (renderers == null)
                return;
            var signal = feedback == G1GraspFeedback.Near ? new Color(1f, 0.82f, 0.12f)
                : feedback == G1GraspFeedback.Contact ? new Color(1f, 0.42f, 0.08f)
                : feedback == G1GraspFeedback.Ready ? new Color(0.15f, 1f, 0.3f)
                : feedback == G1GraspFeedback.Held ? new Color(0.1f, 0.75f, 1f)
                : feedback == G1GraspFeedback.Slipping ? new Color(1f, 0.12f, 0.08f)
                : Color.white;
            var amount = feedback == G1GraspFeedback.None ? 0f : 0.55f;
            for (var i = 0; i < renderers.Length; i++)
            {
                var renderer = renderers[i];
                var block = propertyBlocks[i];
                renderer.GetPropertyBlock(block);
                var color = Color.Lerp(baseColors[i], signal, amount);
                block.SetColor("_BaseColor", color);
                block.SetColor("_Color", color);
                renderer.SetPropertyBlock(block);
            }
        }
    }
}

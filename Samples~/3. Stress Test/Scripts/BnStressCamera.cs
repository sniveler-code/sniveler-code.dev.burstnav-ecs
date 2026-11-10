using UnityEngine;

namespace BurstNav.Samples.StressTest
{
    public sealed class BnStressCamera : MonoBehaviour
    {
        public int CellsX = 80;

        public int CellsY = 24;

        public float CellSize = 2f;

        public Vector2 Origin = Vector2.zero;

        public float Margin = 3f;

        private Camera _camera;

        private void OnEnable()
        {
            _camera = GetComponent<Camera>();
        }

        private void LateUpdate()
        {
            if (_camera == null)
            {
                return;
            }

            var width = Mathf.Max(CellsX, 1) * Mathf.Max(CellSize, 0.01f);
            var height = Mathf.Max(CellsY, 1) * Mathf.Max(CellSize, 0.01f);
            var aspect = Mathf.Max(_camera.aspect, 0.1f);

            _camera.orthographic = true;
            _camera.orthographicSize = Mathf.Max(height * 0.5f + Margin, (width * 0.5f + Margin) / aspect);
            transform.position = new Vector3(Origin.x + width * 0.5f, Origin.y + height * 0.5f, -10f);
            transform.rotation = Quaternion.identity;
        }
    }
}

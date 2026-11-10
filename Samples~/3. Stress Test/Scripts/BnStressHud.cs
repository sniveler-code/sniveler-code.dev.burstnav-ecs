using Unity.Entities;
using UnityEngine;

namespace BurstNav.Samples.StressTest
{
    public sealed class BnStressHud : MonoBehaviour
    {
        public int FontSize = 16;

        public Color TextColor = Color.white;

        private EntityQuery _agents;

        private World _queryWorld;

        private bool _hasQuery;

        private int _agentCount;

        private float _fps;

        private float _nextSample;

        private GUIStyle _style;

        private void OnDisable()
        {
            ReleaseQuery();
        }

        private void Update()
        {
            if (Time.unscaledTime < _nextSample)
            {
                return;
            }

            _nextSample = Time.unscaledTime + 0.25f;
            _fps = Time.smoothDeltaTime > 0f ? 1f / Time.smoothDeltaTime : 0f;

            var world = World.DefaultGameObjectInjectionWorld;
            if (world == null || !world.IsCreated)
            {
                return;
            }

            EnsureQuery(world);
            _agentCount = _agents.CalculateEntityCount();
        }

        private void OnGUI()
        {
            _style ??= new GUIStyle(GUI.skin.label)
            {
                fontSize = FontSize,
                normal = { textColor = TextColor },
            };

            GUI.Label(
                new Rect(12f, 12f, 640f, 24f),
                $"agents {_agentCount} | {_fps:F0} fps | {Time.smoothDeltaTime * 1000f:F1} ms",
                _style);
        }

        private void EnsureQuery(World world)
        {
            if (_hasQuery && _queryWorld == world)
            {
                return;
            }

            ReleaseQuery();
            _agents = world.EntityManager.CreateEntityQuery(ComponentType.ReadOnly<BnAgentSteering>(), ComponentType.ReadOnly<BnStressRoute>());
            _queryWorld = world;
            _hasQuery = true;
        }

        private void ReleaseQuery()
        {
            if (!_hasQuery)
            {
                return;
            }

            if (_queryWorld != null && _queryWorld.IsCreated)
            {
                _agents.Dispose();
            }

            _agents = default;
            _queryWorld = null;
            _hasQuery = false;
        }
    }
}

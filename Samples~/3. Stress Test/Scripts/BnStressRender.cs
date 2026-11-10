using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

namespace BurstNav.Samples.StressTest
{
    public sealed class BnStressRender : MonoBehaviour
    {
        public float Scale = 1.2f;

        public Color Tint = new Color(0.25f, 0.8f, 1f, 1f);

        private const int BatchSize = 1023;

        private EntityQuery _agents;

        private World _queryWorld;

        private bool _hasQuery;

        private Mesh _mesh;

        private Material _material;

        private Matrix4x4[] _batch;

        private RenderParams _params;

        private Bounds _bounds;

        private void OnEnable()
        {
            _mesh = CreateQuad();
            _material = CreateMaterial();
            _batch = new Matrix4x4[BatchSize];
            _params = new RenderParams(_material)
            {
                shadowCastingMode = ShadowCastingMode.Off,
                receiveShadows = false,
            };
            _bounds = new Bounds(Vector3.zero, new Vector3(100000f, 100000f, 100000f));
        }

        private void OnDisable()
        {
            ReleaseQuery();

            if (_mesh != null)
            {
                Destroy(_mesh);
                _mesh = null;
            }

            if (_material != null)
            {
                Destroy(_material);
                _material = null;
            }
        }

        private void LateUpdate()
        {
            var world = World.DefaultGameObjectInjectionWorld;
            if (world == null || !world.IsCreated)
            {
                return;
            }

            EnsureQuery(world);
            if (_agents.IsEmptyIgnoreFilter)
            {
                return;
            }

            using var states = _agents.ToComponentDataArray<BnAgentSteering>(Allocator.Temp);
            _params.worldBounds = _bounds;

            var count = 0;
            for (var index = 0; index < states.Length; index++)
            {
                var position = states[index].Position;
                _batch[count++] = Matrix4x4.TRS(
                    new Vector3(position.x, position.y, position.z),
                    Quaternion.identity,
                    new Vector3(Scale, Scale, Scale));

                if (count == BatchSize)
                {
                    Graphics.RenderMeshInstanced(_params, _mesh, 0, _batch, count);
                    count = 0;
                }
            }

            if (count > 0)
            {
                Graphics.RenderMeshInstanced(_params, _mesh, 0, _batch, count);
            }
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

        private static Mesh CreateQuad()
        {
            var mesh = new Mesh { name = "BnStressAgentQuad" };
            mesh.SetVertices(new[]
            {
                new Vector3(-0.5f, -0.5f, 0f),
                new Vector3(0.5f, -0.5f, 0f),
                new Vector3(0.5f, 0.5f, 0f),
                new Vector3(-0.5f, 0.5f, 0f),
            });
            mesh.SetUVs(0, new[]
            {
                new Vector2(0f, 0f),
                new Vector2(1f, 0f),
                new Vector2(1f, 1f),
                new Vector2(0f, 1f),
            });
            mesh.SetTriangles(new[] { 0, 1, 2, 0, 2, 3 }, 0);
            mesh.RecalculateBounds();
            return mesh;
        }

        private Material CreateMaterial()
        {
            var shader = Shader.Find("Universal Render Pipeline/Unlit")
                         ?? Shader.Find("Unlit/Color")
                         ?? Shader.Find("Standard");
            var material = new Material(shader) { enableInstancing = true };

            if (material.HasProperty("_BaseColor"))
            {
                material.SetColor("_BaseColor", Tint);
            }
            else if (material.HasProperty("_Color"))
            {
                material.SetColor("_Color", Tint);
            }

            return material;
        }
    }
}

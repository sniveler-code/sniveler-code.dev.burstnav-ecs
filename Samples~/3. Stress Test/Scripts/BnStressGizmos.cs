using Unity.Mathematics;
using UnityEngine;

namespace BurstNav.Samples.StressTest
{
    public sealed class BnStressGizmos : MonoBehaviour
    {
        [Min(1)]
        public int CellsX = 80;

        [Min(1)]
        public int CellsY = 24;

        [Min(0.01f)]
        public float CellSize = 2f;

        public Vector2 Origin = Vector2.zero;

        public Color WallColor = new Color(0.16f, 0.18f, 0.22f, 1f);

        public Color SpawnColor = new Color(0.2f, 0.9f, 0.4f, 0.35f);

        public Color GoalColor = new Color(1f, 0.55f, 0.2f, 0.35f);

        private void OnDrawGizmos()
        {
            var size = math.max(CellSize, 0.01f);
            var cellsX = math.max(CellsX, 1);
            var cellsY = math.max(CellsY, 1);

            Gizmos.color = WallColor;
            for (var y = 0; y < cellsY; y++)
            {
                for (var x = 0; x < cellsX; x++)
                {
                    if (!BnStressField.IsBlocked(x, y, cellsX, cellsY))
                    {
                        continue;
                    }

                    var center = new Vector3(Origin.x + (x + 0.5f) * size, Origin.y + (y + 0.5f) * size, 0f);
                    Gizmos.DrawCube(center, new Vector3(size * 0.96f, size * 0.96f, 0.1f));
                }
            }

            DrawBand(0, BnStressField.SpawnWidth, cellsX, cellsY, size, SpawnColor);
            DrawBand(cellsX - BnStressField.GoalWidth, cellsX, cellsX, cellsY, size, GoalColor);
            DrawOutline(cellsX, cellsY, size);
        }

        private void DrawBand(int fromX, int toX, int cellsX, int cellsY, float size, Color color)
        {
            Gizmos.color = color;
            var height = cellsY * size;

            for (var x = math.max(fromX, 0); x < math.min(toX, cellsX); x++)
            {
                var left = Origin.x + x * size;
                Gizmos.DrawCube(
                    new Vector3(left + size * 0.5f, Origin.y + height * 0.5f, 0.05f),
                    new Vector3(size, height, 0.02f));
            }
        }

        private void DrawOutline(int cellsX, int cellsY, float size)
        {
            Gizmos.color = new Color(0.35f, 0.4f, 0.45f, 1f);
            var min = new Vector3(Origin.x, Origin.y, 0f);
            var max = min + new Vector3(cellsX * size, cellsY * size, 0f);
            Gizmos.DrawLine(min, new Vector3(max.x, min.y, 0f));
            Gizmos.DrawLine(new Vector3(max.x, min.y, 0f), max);
            Gizmos.DrawLine(max, new Vector3(min.x, max.y, 0f));
            Gizmos.DrawLine(new Vector3(min.x, max.y, 0f), min);
        }
    }
}

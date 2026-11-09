using System.Runtime.InteropServices;
using Unity.Entities;
using Unity.Mathematics;

namespace BurstNav
{
    [StructLayout(LayoutKind.Sequential)]
    public struct BnLinkData
    {
        public float2 PortalLeft;

        public float2 PortalRight;

        public int Target;

        public float Cost;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct BnPolygon
    {
        public float2 Center;

        public int VertexOffset;

        public int VertexCount;

        public int NeighborOffset;

        public int NeighborCount;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct BnMeshGraphBlob
    {
        public BlobArray<float2> Vertices;

        public BlobArray<BnPolygon> Polygons;

        public BlobArray<int> NeighborPolygons;

        public BlobArray<BnLinkData> Links;

        public BlobArray<int> PolyLinkOffsets;

        public BlobArray<int> PolyLinkCounts;

        public int PolygonCount => Polygons.Length;

        public int LinkCount => Links.Length;

        public int VertexCount => Vertices.Length;

        public float2 GetPolygonCenter(int polygon) => Polygons[polygon].Center;

        public int GetPolyLinkOffset(int polygon) => PolyLinkOffsets[polygon];

        public int GetPolyLinkCount(int polygon) => PolyLinkCounts[polygon];

        public BnLinkData GetPolyLink(int polygon, int index) =>
            Links[PolyLinkOffsets[polygon] + index];

        public int GetNeighborCount(int polygon) => Polygons[polygon].NeighborCount;

        public int GetNeighbor(int polygon, int index) =>
            NeighborPolygons[Polygons[polygon].NeighborOffset + index];

        public int GetVertexCount(int polygon) => Polygons[polygon].VertexCount;

        public float2 GetVertex(int polygon, int index) =>
            Vertices[Polygons[polygon].VertexOffset + index];
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct BnBlobMeshGraph : IComponentData
    {
        public BlobAssetReference<BnMeshGraphBlob> Value;
    }
}

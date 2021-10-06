using HuePat.VoxIR.Util.Geometry;
using OpenTK.Mathematics;
using System.Collections.Generic;

namespace HuePat.VoxIR.Datasets.Util.Octree {
    class SerialSplitCell : Cell {
        public SerialSplitCell(
                List<(Mesh, GroundTruthInfo)> groundTruthMeshes) :
                    base(groundTruthMeshes) {
        }

        public SerialSplitCell(
                Vector3d min,
                Vector3d max,
                Cell parent,
                List<(Triangle, GroundTruthInfo)> triangles) :
                    base(
                        min,
                        max,
                        parent,
                        triangles) {
        }

        protected override List<(Triangle, GroundTruthInfo)>[] SplitInBuckets(
                List<(Triangle, GroundTruthInfo)> triangles) {

            List<(Triangle, GroundTruthInfo)>[] geometryBuckets = InitializeTriangleBuckets();

            foreach ((Triangle, GroundTruthInfo) triangle in triangles) {
                for (int i = 0; i < 8; i++) {
                    if (Intersects(
                            childrenMin[i], 
                            childrenMax[i], 
                            triangle.Item1)) {
                        geometryBuckets[i].Add(triangle);
                    }
                }
            }

            return geometryBuckets;
        }

        protected override List<Cell> SplitInCells(
                List<(Triangle, GroundTruthInfo)>[] triangleBuckets) {

            List<Cell> children = new List<Cell>();

            for (int i = 0; i < 8; i++) {
                children.Add(
                    new SerialSplitCell(
                        childrenMin[i],
                        childrenMax[i],
                        this,
                        triangleBuckets[i]));
            }

            return children;
        }
    }
}
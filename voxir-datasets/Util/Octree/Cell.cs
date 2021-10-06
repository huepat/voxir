using HuePat.VoxIR.Util.Geometry;
using OpenTK.Mathematics;
using System.Collections.Generic;
using System.Linq;

namespace HuePat.VoxIR.Datasets.Util.Octree {
    abstract class Cell : AABox {

        private int clumpingDepth;
        private int triangleCount;
        protected Vector3d[] childrenMin;
        protected Vector3d[] childrenMax;
        private List<(Triangle, GroundTruthInfo)> triangles;

        public bool IsRoot { get; private set; }
        public bool IsLeaf { get; private set; }
        public Cell Parent { get; private set; }
        public IReadOnlyList<Cell> Children { get; private set; }

        public IReadOnlyList<(Triangle, GroundTruthInfo)> Triangles {
            get {

                if (IsLeaf) {
                    return triangles;
                }

                return Children
                    .SelectMany(child => child.Triangles)
                    .ToList();
            }
        }

        public Cell(
                List<(Mesh, GroundTruthInfo)> groundTruthMeshes) :
                    base(FromContainedGeometries(
                        groundTruthMeshes
                            .Select(groundTruthMesh => groundTruthMesh.Item1)
                            .ToList())) {

            List<(Triangle, GroundTruthInfo)> triangles;

            triangles = UnwrapGroundTruthMeshes(groundTruthMeshes);

            IsRoot = true;
            clumpingDepth = 0;
            triangleCount = triangles.Count;

            Process(triangles);
        }

        protected Cell(
                Vector3d min,
                Vector3d max,
                Cell parent,
                List<(Triangle, GroundTruthInfo)> triangles) : 
                    base(
                        min, 
                        max) {

            IsRoot = false;
            triangleCount = triangles.Count;
            Parent = parent;
            clumpingDepth = parent.clumpingDepth;

            UpdateClumpingDepth();

            if (clumpingDepth < GroundTruth.Parameters.OCTREE_CLUMPING_DEATH_THRESHOLD) {
                Process(triangles);
            }
            else {
                IsLeaf = true;
                this.triangles = triangles;
            }
        }

        private List<(Triangle, GroundTruthInfo)> UnwrapGroundTruthMeshes(
                List<(Mesh, GroundTruthInfo)> groundTruthMeshes) {

            return groundTruthMeshes
                .SelectMany(groundTruthMesh => groundTruthMesh.Item1
                    .Select(face => (
                        face.Geometry,
                        groundTruthMesh.Item2)))
                .ToList();
        }

        private void UpdateClumpingDepth() {

            if (triangleCount == Parent.triangleCount) {
                clumpingDepth++;
            }
            else {
                clumpingDepth = 0;
            }
        }

        private void Process(
                List<(Triangle, GroundTruthInfo)> triangles) {

            if (triangles.Count > GroundTruth.Parameters.OCTREE_MAX_OCCUPATION) {
                IsLeaf = false;
                Split(triangles);
            }
            else {
                IsLeaf = true;
                this.triangles = triangles;
            }
        }

        private void Split(
                List<(Triangle, GroundTruthInfo)> triangles) {

            List<(Triangle, GroundTruthInfo)>[] triangleBuckets;

            InitializeChildrenMinMaxPoints();
            triangleBuckets = SplitInBuckets(triangles);
            Children = SplitInCells(triangleBuckets);
        }

        private void InitializeChildrenMinMaxPoints() {

            Vector3d s = (Max - Min) / 2;

            childrenMin = new Vector3d[] {
                Min,
                new Vector3d(Min.X + s.X, Min.Y, Min.Z),
                new Vector3d(Min.X + s.X, Min.Y + s.Y, Min.Z),
                new Vector3d(Min.X, Min.Y + s.Y, Min.Z),
                new Vector3d(Min.X, Min.Y, Min.Z + s.Z),
                new Vector3d(Min.X + s.X, Min.Y, Min.Z + s.Z),
                Min + s,
                new Vector3d(Min.X, Min.Y + s.Y, Min.Z + s.Z)
            };

            childrenMax = new Vector3d[] {
                Min + s,
                new Vector3d(Max.X, Min.Y + s.Y, Min.Z + s.Z),
                new Vector3d(Max.X, Max.Y, Min.Z + s.Z),
                new Vector3d(Min.X + s.X, Max.Y, Min.Z + s.Z),
                new Vector3d(Min.X + s.X, Min.Y + s.Y, Max.Z),
                new Vector3d(Max.X, Min.Y + s.Y, Max.Z),
                Max,
                new Vector3d(Min.X + s.X, Max.Y, Max.Z)
            };
        }

        protected abstract List<(Triangle, GroundTruthInfo)>[] SplitInBuckets(
            List<(Triangle, GroundTruthInfo)> geometries);

        protected abstract List<Cell> SplitInCells(
            List<(Triangle, GroundTruthInfo)>[] geometryBuckets);

        protected List<(Triangle, GroundTruthInfo)>[] InitializeTriangleBuckets() {

            List<(Triangle, GroundTruthInfo)>[] triangleBuckets 
                = new List<(Triangle, GroundTruthInfo)>[8];

            for (int i = 0; i < 8; i++) {
                triangleBuckets[i] = new List<(Triangle, GroundTruthInfo)>();
            }

            return triangleBuckets;
        }

        protected bool Intersects(
                Vector3d min,
                Vector3d max,
                Triangle triangle) {

            AABox bBox = triangle.BBox;
            Vector3d bBoxMin = bBox.Min;
            Vector3d bBoxMax = bBox.Max;

            return
                (min.X <= bBoxMax.X && max.X >= bBoxMin.X) &&
                (min.Y <= bBoxMax.Y && max.Y >= bBoxMin.Y) &&
                (min.Z <= bBoxMax.Z && max.Z >= bBoxMin.Z);
        }
    }
}
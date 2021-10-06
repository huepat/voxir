using HuePat.VoxIR.Util.Geometry;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace HuePat.VoxIR.Datasets.Util.Octree {
    class ParallelSplitCell : Cell {
        private readonly object LOCK = new object();

        public ParallelSplitCell(
                List<(Mesh, GroundTruthInfo)> groundTruthMeshes) :
                    base(groundTruthMeshes) {
        }

        protected override List<(Triangle, GroundTruthInfo)>[] SplitInBuckets(
                List<(Triangle, GroundTruthInfo)> triangles) {

            List<(Triangle, GroundTruthInfo)>[] geometryBuckets = InitializeTriangleBuckets();

            Parallel.ForEach(
                Partitioner.Create(
                    0, 
                    triangles.Count),
                InitializeTriangleBuckets,
                (partition, loopState, partitionBuckets) => {

                    for (int i = partition.Item1; i < partition.Item2; i++) {
                        for (int j = 0; j < 8; j++) {
                            if (Intersects(
                                    childrenMin[j], 
                                    childrenMax[j], 
                                    triangles[i].Item1)) {
                                partitionBuckets[j].Add(triangles[i]);
                            }
                        }
                    }

                    return partitionBuckets;
                },
                partitionBuckets => {

                    for (int i = 0; i < 8; i++) {
                        lock (LOCK) {
                            geometryBuckets[i].AddRange(partitionBuckets[i]);
                        }
                    }
                });
            return geometryBuckets;
        }

        protected override List<Cell> SplitInCells(
                List<(Triangle, GroundTruthInfo)>[] triangleBuckets) {

            List<Cell> children = new List<Cell> {
                null, null, null, null,
                null, null, null, null
            };

            Parallel.For(
                0,
                8,
                i => {

                    Cell child = new SerialSplitCell(
                        childrenMin[i],
                        childrenMax[i],
                        this,
                        triangleBuckets[i]);

                    lock (LOCK) {
                        children[i] = child;
                    }
                });

            return children;
        }
    }
}
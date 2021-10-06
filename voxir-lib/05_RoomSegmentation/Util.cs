using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace HuePat.VoxIR.RoomSegmentation {
    static class Util {
        public static (int, int)[] GetPerpendicularDirections(
                this (int, int) direction) {

            if (direction.IsDirectionDiagonal()) {

                return new (int, int)[] {
                    (
                        direction.Item1 * -1,
                        direction.Item2
                    ),
                    (
                        direction.Item1,
                        direction.Item2 * -1
                    )
                };
            }

            return new (int, int)[] {
                (
                    direction.Item1 == 0 ? 1 : 0,
                    direction.Item2 == 0 ? 1 : 0
                ),
                (
                    direction.Item1 == 0 ? -1 : 0,
                    direction.Item2 == 0 ? -1 : 0
                )
            };
        }

        public static Dictionary<int, int> GetMergeMapping(
                int[,,] segmentationGrid,
                List<int> transitions,
                Func<int, int, bool> mergeDecisionCallback,
                Func<(int, int, int), IEnumerable<(int, int, int)>> neighbourhoodCallback) {

            object @lock = new object();
            HashSet<(int, int)> mergeIds = new HashSet<(int, int)>();

            Parallel.ForEach(
                Partitioner.Create(
                    0, 
                    transitions.Count),
                () => new HashSet<(int, int)>(),
                (partition, loopState, localMergeIds) => {

                    for (int j = partition.Item1; j < partition.Item2; j++) {

                        if (transitions[j] == segmentationGrid.GetLength(2)) {
                            continue;
                        }

                        UpdateMergeIds(
                            transitions[j],
                            segmentationGrid,
                            localMergeIds,
                            mergeDecisionCallback,
                            neighbourhoodCallback);
                    }

                    return localMergeIds;
                },
                localMergeIds => {
                    lock (@lock) {
                        mergeIds.AddRange(localMergeIds);
                    }
                });

            return GetMergeMapping(
                mergeIds,
                cluster => cluster.First());
        }

        public static (int, int) GetUnambiguousOrder(
                int value1,
                int value2) {

            return value1 > value2 ?
                (value2, value1) :
                (value1, value2);
        }

        public static Dictionary<int, int> GetMergeMapping(
                HashSet<(int, int)> mergeIds,
                Func<HashSet<int>, int> clusterDestinationSelectionCallback) {

            Dictionary<int, int> clusterMapping = GetClusterMapping(mergeIds);

            Dictionary<int, HashSet<int>> clusters = GetClusters(clusterMapping);

            return GetMergeMapping(
                clusters,
                clusterDestinationSelectionCallback);
        }

        private static void UpdateMergeIds(
                int c,
                int[,,] segmentationGrid,
                HashSet<(int, int)> mergeIds,
                Func<int, int, bool> mergeDecisionCallback,
                Func<(int, int, int), IEnumerable<(int, int, int)>> neighbourhoodCallback) {

            int i, r;
            int segmentId, segmentId2;

            for (i = 0; i < segmentationGrid.GetLength(0); i++) {
                for (r = 0; r < segmentationGrid.GetLength(1); r++) {

                    segmentId = segmentationGrid[i, r, c];
                    if (segmentId == 0) {
                        continue;
                    }

                    foreach ((int, int, int) voxel in neighbourhoodCallback((i, r, c))) {

                        if (voxel.Item1 < 0
                                || voxel.Item2 < 0
                                || voxel.Item3 < 0
                                || voxel.Item1 >= segmentationGrid.GetLength(0)
                                || voxel.Item2 >= segmentationGrid.GetLength(1)
                                || voxel.Item3 >= segmentationGrid.GetLength(2)) {
                            continue;
                        }

                        segmentId2 = segmentationGrid[
                            voxel.Item1,
                            voxel.Item2,
                            voxel.Item3];

                        if (segmentId2 != 0
                                && segmentId2 != segmentId
                                && mergeDecisionCallback(segmentId, segmentId2)) {

                            mergeIds.Add(
                                GetUnambiguousOrder(
                                    segmentId,
                                    segmentId2));
                        }
                    }
                }
            }
        }

        private static Dictionary<int, int> GetClusterMapping(
                HashSet<(int, int)> mergeIds) {

            int clusterCount = 0;
            int clusterIndex1, clusterIndex2;
            int destinationIndex, sourceIndex;
            Dictionary<int, int> clusterMapping = new Dictionary<int, int>();

            foreach ((int, int) mergeId in mergeIds) {

                clusterIndex1 = -1;
                clusterIndex2 = -1;

                if (clusterMapping.ContainsKey(mergeId.Item1)) {
                    clusterIndex1 = clusterMapping[mergeId.Item1];
                }

                if (clusterMapping.ContainsKey(mergeId.Item2)) {
                    clusterIndex2 = clusterMapping[mergeId.Item2];
                }

                if (clusterIndex1 == -1 && clusterIndex2 == -1) {
                    clusterMapping.Add(mergeId.Item1, clusterCount);
                    clusterMapping.Add(mergeId.Item2, clusterCount);
                    clusterCount++;
                }
                else if (clusterIndex1 == -1) {
                    clusterMapping.Add(
                        mergeId.Item1,
                        clusterMapping[mergeId.Item2]);
                }
                else if (clusterIndex2 == -1) {
                    clusterMapping.Add(
                        mergeId.Item2,
                        clusterMapping[mergeId.Item1]);
                }
                else {
                    if (clusterIndex1 == clusterIndex2) {
                        continue;
                    }
                    if (clusterIndex1 < clusterIndex2) {
                        destinationIndex = clusterIndex1;
                        sourceIndex = clusterIndex2;
                    }
                    else {
                        destinationIndex = clusterIndex2;
                        sourceIndex = clusterIndex1;
                    }
                    foreach (int segmentIndex in clusterMapping.Keys.ToList()) {
                        if (clusterMapping[segmentIndex] == sourceIndex) {
                            clusterMapping[segmentIndex] = destinationIndex;
                        }
                    }
                }
            }

            return clusterMapping;
        }

        private static Dictionary<int, HashSet<int>> GetClusters(
                Dictionary<int, int> clusterMapping) {

            Dictionary<int, HashSet<int>> clusters = new Dictionary<int, HashSet<int>>();

            foreach (int segmentIndex in clusterMapping.Keys) {
                clusters.BucketAdd(
                    clusterMapping[segmentIndex],
                    segmentIndex);
            }

            return clusters;
        }

        private static Dictionary<int, int> GetMergeMapping(
                Dictionary<int, HashSet<int>> clusters,
                Func<HashSet<int>, int> clusterDestinationSelectionCallback) {

            int destinationIndex;
            Dictionary<int, int> mergeMapping = new Dictionary<int, int>();

            foreach (int clusterIndex in clusters.Keys) {

                destinationIndex = clusterDestinationSelectionCallback(
                    clusters[clusterIndex]);

                foreach (int segmentIndex in clusters[clusterIndex]) {

                    if (segmentIndex != destinationIndex) {
                        mergeMapping.Add(
                            segmentIndex,
                            destinationIndex);
                    }
                }
            }

            return mergeMapping;
        }
    }
}
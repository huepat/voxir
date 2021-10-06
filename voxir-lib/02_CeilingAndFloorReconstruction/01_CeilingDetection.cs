using HuePat.VoxIR.Util.Grid;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace HuePat.VoxIR.CeilingAndFloorReconstruction {
    public static class CeilingDetection {
        public static void TraceWallThroughCeilings(
                byte[,,] normalGrid,
                out long[] floorIncidence) {

            object @lock = new object();
            long[] _floorIncidence = new long[normalGrid.GetLength(0)];

            Parallel.ForEach(
                Partitioner.Create(
                    0, 
                    normalGrid.GetLength(2)),
                () => new long[normalGrid.GetLength(0)],
                (partition, loopState, localFloorIncidence) => {

                    int i, r, c;

                    for (c = partition.Item1; c < partition.Item2; c++) {
                        for (r = 0; r < normalGrid.GetLength(1); r++) {
                            for (i = normalGrid.GetLength(0) - 1; i > 0; i--) {

                                if (normalGrid[i, r, c] == NormalGridValues.NORMAL_UP) {
                                    localFloorIncidence[i]++;
                                }

                                if (normalGrid[i - 1, r, c] == NormalGridValues.NORMAL_DOWN) {

                                    if (normalGrid[i, r, c] == NormalGridValues.NORMAL_HORIZONTAL) {
                                        normalGrid[i - 1, r, c] = NormalGridValues.NORMAL_HORIZONTAL;
                                    }
                                    else if (normalGrid[i, r, c] == NormalGridValues.NORMAL_UP) {
                                        normalGrid[i, r, c] = NormalGridValues.NORMAL_HORIZONTAL;
                                        normalGrid[i - 1, r, c] = NormalGridValues.NORMAL_HORIZONTAL;
                                    }
                                }
                            }
                        }
                    }

                    return localFloorIncidence;
                },
                localFloorIncidence => {

                    lock (@lock) {
                        for (int i = normalGrid.GetLength(0) - 1; i > 0; i--) {
                            _floorIncidence[i] += localFloorIncidence[i];
                        }
                    }
                });

            floorIncidence = _floorIncidence;
        }

        public static List<VoxelSegment> SegmentCeilings(
                double resolution,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            int di, i, i2, dr, r, r2, dc, c, c2;
            int minArea = Parameters
                .ROOM_MIN_AREA
                .GetAreaInVoxels(resolution);
            bool[,,] isSegmented = new bool[
                normalGrid.GetLength(0),
                normalGrid.GetLength(1),
                normalGrid.GetLength(2)];
            Queue<(int, int, int)> candidates = new Queue<(int, int, int)>();
            List<(int, int, int)> voxels;
            List<VoxelSegment> segments = new List<VoxelSegment>();

            for (i = 0; i < normalGrid.GetLength(0); i++) {
                for (r = 0; r < normalGrid.GetLength(1); r++) {
                    for (c = 0; c < normalGrid.GetLength(2); c++) {

                        if (isSegmented[i, r, c]
                                || normalGrid[i, r, c] != NormalGridValues.NORMAL_DOWN) {
                            continue;
                        }

                        candidates.Enqueue((i, r, c));
                        voxels = new List<(int, int, int)>();

                        do {
                            (int, int, int) candidate = candidates.Dequeue();
                            if (isSegmented[
                                    candidate.Item1,
                                    candidate.Item2,
                                    candidate.Item3]) {
                                continue;
                            }

                            isSegmented[
                                candidate.Item1,
                                candidate.Item2,
                                candidate.Item3] = true;
                            voxels.Add(candidate);

                            for (di = -1; di <= 1; di++) {
                                for (dr = -1; dr <= 1; dr++) {
                                    for (dc = -1; dc <= 1; dc++) {

                                        if (di.Abs() + dr.Abs() + dc.Abs() != 1) {
                                            continue;
                                        }

                                        i2 = candidate.Item1 + di;
                                        r2 = candidate.Item2 + dr;
                                        c2 = candidate.Item3 + dc;

                                        if (i2 >= 0 && r2 >= 0 && c2 >= 0
                                                && i2 < normalGrid.GetLength(0)
                                                && r2 < normalGrid.GetLength(1)
                                                && c2 < normalGrid.GetLength(2)
                                                && !isSegmented[i2, r2, c2]
                                                && normalGrid[i2, r2, c2] == NormalGridValues.NORMAL_DOWN) {
                                            candidates.Enqueue((i2, r2, c2));
                                        }
                                    }
                                }
                            }
                        } while (candidates.Count > 0);

                        if (voxels
                                .Select(voxel => (voxel.Item2, voxel.Item3))
                                .Distinct()
                                .Count() < minArea) {
                            continue;
                        }

                        GridBBox3D bBox = new GridBBox3D();
                        foreach ((int, int, int) voxel in voxels) {

                            bBox.Add(voxel);
                            reconstructionGrid[
                                    voxel.Item1, 
                                    voxel.Item2, 
                                    voxel.Item3] 
                                = VoxelState.CreateVoxelState(
                                    segments.Count,
                                    VoxelClassValues.CEILING);
                        }
                        segments.Add(
                            new VoxelSegment(
                                bBox, 
                                voxels));
                    }
                }
            }
            return segments;
        }
    }
}
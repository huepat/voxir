using HuePat.VoxIR.Util.Grid;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace HuePat.VoxIR.RoomSegmentation {
    public static class RampSpaceDetection {
        public static bool[,,] InitializeWallGrid(
                double resolution,
                int[,,][] reconstructionGrid) {

            int wallGridMinWallHeight = Parameters
                .WALL_GRID_MIN_WALL_HEIGHT
                .GetDistanceInVoxels(resolution);
            bool[,,] wallGrid = new bool[
                reconstructionGrid.GetLength(0),
                reconstructionGrid.GetLength(1),
                reconstructionGrid.GetLength(2)];

            Parallel.For(
                0,
                reconstructionGrid.GetLength(2),
                c => {

                    int i, i2, r;
                    int stopIndex;
                    int wallCount;
                    int[] voxelState;
                    int[] voxelClassValues;

                    for (i = 0; i < reconstructionGrid.GetLength(0); i++) {
                        for (r = 0; r < reconstructionGrid.GetLength(1); r++) {

                            voxelState = reconstructionGrid[i, r, c];
                            if (voxelState == null) {
                                continue;
                            }

                            voxelClassValues = voxelState.GetVoxelClassValues(0);
                            if (!voxelClassValues.Contains(VoxelClassValues.WALL)
                                    || !voxelClassValues.Contains(VoxelClassValues.CEILING)) {
                                continue;
                            }

                            wallCount = GetVerticalWallCount(
                                i,
                                r,
                                c,
                                reconstructionGrid,
                                out stopIndex);

                            if (wallCount < wallGridMinWallHeight) {
                                continue;
                            }
                            for (i2 = i + 1; i2 < stopIndex; i2++) {
                                wallGrid[i2, r, c] = true;
                            }
                        }
                    }
                });

            return wallGrid;
        }

        public static int[,,] InitializeSlopeGrid(
                double resolution,
                bool[,,] wallGrid,
                int[,,][] reconstructionGrid) {

            int slopeDeterminationRadius = Parameters
                .SLOPE_DETERMINATION_RADIUS
                .GetDistanceInVoxels(resolution);
            int slopeDeterminationRadiusDiagonal = Parameters
                .SLOPE_DETERMINATION_RADIUS
                .GetDistanceInVoxels(
                    resolution.GetVoxelSizeDiagonal());
            int maxRampHeightOffset = (int)(Parameters.SLOPE_DETERMINATION_RADIUS
                        * Math.Tan(Parameters.MAX_RAMP_SLOPE_DEGREES.DegreeToRadian())
                    / resolution
                ).Ceil();
            int[,,] slopeGrid = new int[
                reconstructionGrid.GetLength(0),
                reconstructionGrid.GetLength(1),
                reconstructionGrid.GetLength(2)];

            Parallel.For(
                0,
                reconstructionGrid.GetLength(0),
                i => {
                    InitializeSlopeGridSection(
                        i,
                        maxRampHeightOffset,
                        slopeDeterminationRadius,
                        slopeDeterminationRadiusDiagonal,
                        wallGrid,
                        slopeGrid,
                        reconstructionGrid);
                });

            return slopeGrid;
        }

        public static bool[,,] InitializeRampSpaceGrid(
                double resolution,
                bool[,,] wallGrid,
                int[,,] slopeGrid,
                int[,,][] reconstructionGrid,
                out bool rampsSpacesDetected) {

            bool[,,] rampSpaceGrid;
            List<List<(int, int, int)>> rampSegments;

            rampsSpacesDetected = false;

            rampSegments = GetRampSegments(
                resolution,
                wallGrid,
                slopeGrid,
                reconstructionGrid);

            if (rampSegments.Count == 0) {
                return new bool[
                    reconstructionGrid.GetLength(0),
                    reconstructionGrid.GetLength(1),
                    reconstructionGrid.GetLength(2)];
            }

            rampsSpacesDetected = true;

            rampSpaceGrid = InitializeRampSpaceGrid(
                reconstructionGrid,
                rampSegments);

            RefineRampSpaceGrid(
                resolution,
                ref rampSpaceGrid,
                reconstructionGrid);

            return rampSpaceGrid;
        }

        public static void RefineHorizontalRampSurfaces(
                double resolution,
                bool[,,] wallGrid,
                bool[,,] rampSpaceGrid,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            bool isFloor;
            bool isVerticalOrientationDetermined;
            int i, r, c;
            int minArea = Parameters
                .HORIZONTAL_SURFACE_REFINEMENT_MIN_AREA
                .GetAreaInVoxels(resolution);
            bool[,,] removeCandidateGrid;
            bool[,,] isSegmented = new bool[
                reconstructionGrid.GetLength(0),
                reconstructionGrid.GetLength(1),
                reconstructionGrid.GetLength(2)];
            List<(int, int, int)> segment;

            removeCandidateGrid = FindRemoveCandidateVoxels(
                wallGrid,
                normalGrid,
                reconstructionGrid);

            for (i = 0; i < reconstructionGrid.GetLength(0); i++) {
                for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                    for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                        if (!CheckSegmentationSeed(
                                i,
                                r,
                                c,
                                isSegmented,
                                removeCandidateGrid,
                                reconstructionGrid,
                                out isVerticalOrientationDetermined,
                                out isFloor)) {
                            continue;
                        }

                        segment = Segment(
                            isVerticalOrientationDetermined,
                            isFloor,
                            i,
                            r,
                            c,
                            resolution,
                            isSegmented,
                            removeCandidateGrid,
                            reconstructionGrid);

                        if (!CheckSegmentArea(
                                minArea,
                                segment)) {
                            continue;
                        }

                        if (!CheckSegmentRampRatio(
                                rampSpaceGrid,
                                reconstructionGrid,
                                segment)) {
                            continue;
                        }

                        RemoveSegment(
                            rampSpaceGrid,
                            normalGrid,
                            reconstructionGrid,
                            segment);
                    }
                }
            }

            RefineCeilingSurfaces(
                normalGrid,
                reconstructionGrid);
        }

        private static int GetVerticalWallCount(
                int i,
                int r,
                int c,
                int[,,][] reconstructionGrid,
                out int stopIndex) {

            int i2 = i;
            int wallCount = 0;
            int[] voxelState;
            int[] voxelClassValues;

            while (true) {

                i2++;
                if (i2 >= reconstructionGrid.GetLength(0)) {
                    wallCount = 0;
                    break;
                }

                voxelState = reconstructionGrid[i2, r, c];
                if (voxelState == null) {
                    continue;
                }

                voxelClassValues = voxelState.GetVoxelClassValues(0);
                if (voxelClassValues.Contains(VoxelClassValues.FLOOR)) {
                    break;
                }

                if (voxelClassValues.Contains(VoxelClassValues.WALL)) {
                    wallCount++;
                }
            }

            stopIndex = i2;

            return wallCount;
        }

        private static void InitializeSlopeGridSection(
                int i,
                int maxRampHeightOffset,
                int slopeDeterminationRadius,
                int slopeDeterminationRadiusDiagonal,
                bool[,,] wallGrid,
                int[,,] slopeGrid,
                int[,,][] reconstructionGrid) {

            int r, c;
            int[] voxelState;

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                    slopeGrid[i, r, c] = -1;

                    voxelState = reconstructionGrid[i, r, c];
                    if (voxelState == null) {
                        continue;
                    }

                    if (!voxelState
                            .GetVoxelClassValues(0)
                            .Contains(VoxelClassValues.FLOOR)) {
                        continue;
                    }

                    InitializeSlopeGridVoxel(
                        i,
                        r,
                        c,
                        maxRampHeightOffset,
                        slopeDeterminationRadius,
                        slopeDeterminationRadiusDiagonal,
                        wallGrid,
                        slopeGrid,
                        reconstructionGrid);
                }
            }
        }

        private static void InitializeSlopeGridVoxel(
                int i,
                int r,
                int c,
                int maxRampHeightOffset,
                int slopeDeterminationRadius,
                int slopeDeterminationRadiusDiagonal,
                bool[,,] wallGrid,
                int[,,] slopeGrid,
                int[,,][] reconstructionGrid) {

            int dr, dc, maxD;
            int slope;

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    if (dr == 0 && dc == 0) {
                        continue;
                    }

                    maxD = (dr, dc).IsDirectionDiagonal() ?
                        slopeDeterminationRadiusDiagonal :
                        slopeDeterminationRadius;

                    if (IntersectsWall(
                            i,
                            r,
                            c,
                            dr,
                            dc,
                            maxD,
                            wallGrid,
                            reconstructionGrid)) {
                        continue;
                    }

                    slope = GetSlopeValue(
                            i,
                            r,
                            c,
                            dr,
                            dc,
                            maxD,
                            maxRampHeightOffset,
                            reconstructionGrid);

                    if (slope > slopeGrid[i, r, c]) {
                        slopeGrid[i, r, c] = slope;
                    }
                }
            }
        }

        private static bool IntersectsWall(
                int i,
                int r,
                int c,
                int dr,
                int dc,
                int maxD,
                bool[,,] wallGrid,
                int[,,][] reconstructionGrid) {

            int r2, c2, d;

            for (d = 1; d <= maxD; d++) {

                r2 = r + d * dr;
                c2 = c + d * dc;

                if (r2 < 0 || c2 < 0
                        || r2 >= reconstructionGrid.GetLength(1)
                        || c2 >= reconstructionGrid.GetLength(2)
                        || wallGrid[i, r2, c2]) {
                    return true;
                }
            }

            return false;
        }

        private static int GetSlopeValue(
                int i,
                int r, 
                int c,
                int dr,
                int dc,
                int maxD,
                int maxRampHeightOffset,
                int[,,][] reconstructionGrid) {

            int di, i2, r2, c2, j;
            int[] voxelState;

            r2 = r + maxD * dr;
            c2 = c + maxD * dc;

            for (di = 0; di <= maxRampHeightOffset; di++) {
                for (j = -1; j <= 1; j += 2) {

                    i2 = i + j * di;
                    if (i2 < 0
                            || i2 >= reconstructionGrid.GetLength(0)) {
                        continue;
                    }

                    voxelState = reconstructionGrid[i2, r2, c2];
                    if (voxelState == null) {
                        continue;
                    }

                    if (voxelState
                            .GetVoxelClassValues(0)
                            .Contains(VoxelClassValues.FLOOR)) {
                        return di.Abs();
                    }
                }
            }
            return 0;
        }

        private static List<List<(int, int, int)>> GetRampSegments(
                double resolution,
                bool[,,] wallGrid,
                int[,,] slopeGrid,
                int[,,][] reconstructionGrid) {

            int[,,] rampSegmentationGrid;
            List<int> transitions;
            Dictionary<int, int> mergeMapping;
            Dictionary<int, List<(int, int, int)>> rampSegments;

            rampSegmentationGrid = SegmentRamps(
                resolution,
                wallGrid,
                slopeGrid,
                reconstructionGrid,
                out transitions);

            mergeMapping = Util.GetMergeMapping(
                rampSegmentationGrid,
                transitions,
                (segmentId1, segmentId2) => true,
                (voxel) => GetVoxelNeighboursForSegmentMerging(voxel));

            rampSegments = GetMergedSegments(
                rampSegmentationGrid,
                mergeMapping);

            return GetRampSegmentsInUnambiguousOrder(
                resolution,
                slopeGrid,
                rampSegments);
        }

        private static int[,,] SegmentRamps(
                double resolution,
                bool[,,] wallGrid,
                int[,,] slopeGrid,
                int[,,][] reconstructionGrid,
                out List<int> transitions) {

            int segmentCount = 0;
            int minRampHeightOffset = (int)(Parameters.SLOPE_DETERMINATION_RADIUS
                        * Math.Tan(Parameters.MIN_RAMP_SLOPE_DEGREES.DegreeToRadian())
                    / resolution
                ).Ceil();
            object @lock = new object();
            int[,,] rampSegmentationGrid = new int[
                slopeGrid.GetLength(0),
                slopeGrid.GetLength(1),
                slopeGrid.GetLength(2)];
            List<int> _transitions = new List<int>();

            Parallel.ForEach(
                Partitioner.Create(
                    0, 
                    slopeGrid.GetLength(2)),
                (partition, loopState) => {

                    bool cut;
                    int di, i, i2, i3, dr, r, r2, dc, c, c2;
                    int segmentIndex;
                    int[] voxelState;
                    Queue<(int, int, int)> candidates = new Queue<(int, int, int)>();

                    lock (@lock) {
                        _transitions.Add(partition.Item2);
                    }

                    for (c = partition.Item1; c < partition.Item2; c++) {
                        for (i = 1; i < slopeGrid.GetLength(0); i++) {
                            for (r = 0; r < slopeGrid.GetLength(1); r++) {

                                if (rampSegmentationGrid[i, r, c] != 0
                                        || slopeGrid[i, r, c] < minRampHeightOffset
                                        || wallGrid[i - 1, r, c]) {
                                    continue;
                                }

                                lock (@lock) {
                                    segmentCount++;
                                    segmentIndex = segmentCount;
                                }

                                candidates.Enqueue((i, r, c));
                                do {

                                    (int, int, int) candidate = candidates.Dequeue();
                                    if (rampSegmentationGrid[
                                            candidate.Item1,
                                            candidate.Item2,
                                            candidate.Item3] != 0) {
                                        continue;
                                    }

                                    rampSegmentationGrid[
                                        candidate.Item1,
                                        candidate.Item2,
                                        candidate.Item3] = segmentIndex;

                                    for (di = -1; di <= 1; di++) {
                                        for (dr = -1; dr <= 1; dr++) {
                                            for (dc = -1; dc <= 1; dc++) {

                                                i2 = candidate.Item1 + di;
                                                r2 = candidate.Item2 + dr;
                                                c2 = candidate.Item3 + dc;
                                                if (i2 < 0 || r2 < 0 || c2 < partition.Item1
                                                        || i2 >= slopeGrid.GetLength(0)
                                                        || r2 >= slopeGrid.GetLength(1)
                                                        || c2 >= partition.Item2
                                                        || rampSegmentationGrid[i2, r2, c2] != 0
                                                        || slopeGrid[i2, r2, c2] < minRampHeightOffset) {
                                                    continue;
                                                }

                                                cut = false;
                                                i3 = i2 - 1;

                                                while (true) {

                                                    if (i3 < 0) {
                                                        break;
                                                    }

                                                    if (wallGrid[i3, r2, c2]) {
                                                        cut = true;
                                                        break;
                                                    }

                                                    voxelState = reconstructionGrid[i3, r2, c2];
                                                    if (voxelState == null) {
                                                        break;
                                                    }

                                                    if (voxelState
                                                            .GetVoxelClassValues(0)
                                                            .Contains(VoxelClassValues.FLOOR)) {
                                                        i3--;
                                                        continue;
                                                    }

                                                    break;
                                                }
                                                if (!cut) {
                                                    candidates.Enqueue((i2, r2, c2));
                                                }
                                            }
                                        }
                                    }
                                } while (candidates.Count > 0);
                            }
                        }
                    }
                });

            transitions = _transitions;

            return rampSegmentationGrid;
        }

        private static List<(int, int, int)> GetVoxelNeighboursForSegmentMerging(
                (int, int, int) voxel) {

            int di, dr, c;
            List<(int, int, int)> neighbours = new List<(int, int, int)>();

            c = voxel.Item3 - 1;
            for (di = -1; di <= 1; di++) {
                for (dr = -1; dr <= 1; dr++) {
                    neighbours.Add((
                        voxel.Item1 + di,
                        voxel.Item2 + dr,
                        c
                    ));
                }
            }
            return neighbours;
        }

        private static Dictionary<int, List<(int, int, int)>> GetMergedSegments(
                int[,,] segmentationGrid,
                Dictionary<int, int> mergeIds) {

            object @lock = new object();
            Dictionary<int, List<(int, int, int)>> segments = new Dictionary<int, List<(int, int, int)>>();

            Parallel.ForEach(
                Partitioner.Create(
                    0, 
                    segmentationGrid.GetLength(2)),
                () => new Dictionary<int, List<(int, int, int)>>(),
                (partition, loopState, localSegments) => {
                    for (int c = partition.Item1; c < partition.Item2; c++) {
                        UpdateMergedSegments(
                            c,
                            segmentationGrid,
                            mergeIds,
                            localSegments);
                    }
                    return localSegments;
                },
                (localSegments) => {
                    lock (@lock) {
                        segments.BucketAdd(localSegments);
                    }
                });
            return segments;
        }

        private static void UpdateMergedSegments(
                int c,
                int[,,] segmentationGrid,
                Dictionary<int, int> mergeIds,
                Dictionary<int, List<(int, int, int)>> segments) {

            int i, r;
            int segmentId;

            for (i = 0; i < segmentationGrid.GetLength(0); i++) {
                for (r = 0; r < segmentationGrid.GetLength(1); r++) {
                    segmentId = segmentationGrid[i, r, c];
                    if (segmentId != 0) {
                        segments.BucketAdd(
                            mergeIds.ContainsKey(segmentId) ?
                                mergeIds[segmentId] :
                                segmentId,
                            (i, r, c));
                    }
                }
            }
        }

        private static List<List<(int, int, int)>> GetRampSegmentsInUnambiguousOrder(
                double resolution,
                int[,,] slopeGrid,
                Dictionary<int, List<(int, int, int)>> rampSegments) {

            int rampMinHeight = Parameters
                .RAMP_MIN_HEIGHT
                .GetDistanceInVoxels(resolution);

            return rampSegments
                .Values
                .AsParallel()
                .Where(segment => {
                    int minI = int.MaxValue;
                    int maxI = int.MinValue;
                    foreach ((int, int, int) voxel in segment) {
                        if (voxel.Item1 > maxI) {
                            maxI = voxel.Item1;
                        }
                        if (voxel.Item1 < minI) {
                            minI = voxel.Item1;
                        }
                    }
                    return maxI - minI + 1 >= rampMinHeight;
                })
                .OrderBy(segment => segment
                    .Select(voxel => GetIndex1D(
                        slopeGrid.GetLength(1),
                        slopeGrid.GetLength(2),
                        voxel))
                    .Min())
                .ToList();
        }

        private static int GetIndex1D(
                int sizeR,
                int sizeC,
                (int, int, int) voxel) {
            return voxel.Item1 * sizeR * sizeC
                + voxel.Item2 * sizeC
                + voxel.Item3;
        }

        private static bool[,,] InitializeRampSpaceGrid(
                int[,,][] reconstructionGrid,
                List<List<(int, int, int)>> rampSegments) {

            bool[,,] rampSpaceGrid = new bool[
                reconstructionGrid.GetLength(0),
                reconstructionGrid.GetLength(1),
                reconstructionGrid.GetLength(2)];

            Parallel.For(
                0,
                rampSegments.Count,
                j => {

                    foreach ((int, int, int) voxel in rampSegments[j]) {
                        AddRampSpaceColumn(
                            voxel.Item1 - 1,
                            voxel,
                            rampSpaceGrid,
                            reconstructionGrid);
                    }
                });

            return rampSpaceGrid;
        }

        private static void AddRampSpaceColumn(
                int i,
                (int, int, int) voxel,
                bool[,,] rampSpaceGrid,
                int[,,][] reconstructionGrid) {

            int[] voxelState;
            int[] voxelClassValues;

            while (i >= 0) {

                voxelState = reconstructionGrid[
                    i,
                    voxel.Item2,
                    voxel.Item3];
                if (voxelState == null) {
                    break;
                }

                voxelClassValues = voxelState.GetVoxelClassValues(0);
                if (!voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                        && !voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                    break;
                }

                rampSpaceGrid[
                    i--,
                    voxel.Item2,
                    voxel.Item3] = true;
            }
        }

        private static void RefineRampSpaceGrid(
                double resolution,
                ref bool[,,] rampSpaceGrid,
                int[,,][] reconstructionGrid) {

            RemoveNegligibleRampSpaceSegments(
                resolution,
                rampSpaceGrid);

            rampSpaceGrid = ApplyMorphologicalClosing(
                resolution,
                rampSpaceGrid,
                reconstructionGrid);

            ExtendRampSpaceVerticallyUntilFloor(
                rampSpaceGrid,
                reconstructionGrid);
        }

        private static void RemoveNegligibleRampSpaceSegments(
                double resolution,
                bool[,,] rampSpaceGrid) {

            int[,,] segmentationGrid;
            List<int> transitions;
            Dictionary<int, List<(int, int, int)>> rampSpaceSegments;

            segmentationGrid = SegmentRampSpace(
                rampSpaceGrid,
                out transitions);

            Dictionary<int, int> mergeMapping = Util.GetMergeMapping(
                segmentationGrid,
                transitions,
                (segmentId1, segmentId2) => true,
                (voxel) => new (int, int, int)[] {
                    (
                        voxel.Item1,
                        voxel.Item2,
                        voxel.Item3 - 1
                    )
                });

            rampSpaceSegments = GetMergedSegments(
                    segmentationGrid,
                    mergeMapping);

            RemoveNegligibleRampSpaceSegments(
                resolution,
                rampSpaceGrid,
                rampSpaceSegments);
        }

        private static int[,,] SegmentRampSpace(
                bool[,,] rampSpaceGrid,
                out List<int> transitions) {

            int segmentCount = 0;
            object @lock = new object();
            int[,,] segmentationGrid = new int[
                rampSpaceGrid.GetLength(0),
                rampSpaceGrid.GetLength(1),
                rampSpaceGrid.GetLength(2)];
            List<int> _transitions = new List<int>();

            Parallel.ForEach(
                Partitioner.Create(
                    0, 
                    rampSpaceGrid.GetLength(2)),
                (partition, loopState) => {

                    int di, i, i2, dr, r, r2, dc, c, c2;
                    int segmentIndex;
                    Queue<(int, int, int)> candidates = new Queue<(int, int, int)>();

                    lock (@lock) {
                        _transitions.Add(partition.Item2);
                    }

                    for (c = partition.Item1; c < partition.Item2; c++) {
                        for (i = 0; i < rampSpaceGrid.GetLength(0); i++) {
                            for (r = 0; r < rampSpaceGrid.GetLength(1); r++) {

                                if (segmentationGrid[i, r, c] != 0
                                        || !rampSpaceGrid[i, r, c]) {
                                    continue;
                                }

                                lock (@lock) {
                                    segmentCount++;
                                    segmentIndex = segmentCount;
                                }

                                candidates.Enqueue((i, r, c));
                                do {
                                    (int, int, int) candidate = candidates.Dequeue();
                                    if (segmentationGrid[
                                            candidate.Item1,
                                            candidate.Item2,
                                            candidate.Item3] != 0) {
                                        continue;
                                    }
                                    segmentationGrid[
                                        candidate.Item1,
                                        candidate.Item2,
                                        candidate.Item3] = segmentIndex;

                                    for (di = -1; di <= 1; di++) {
                                        for (dr = -1; dr <= 1; dr++) {
                                            for (dc = -1; dc <= 1; dc++) {

                                                if (di.Abs() + dr.Abs() + dc.Abs() != 1) {
                                                    continue;
                                                }

                                                i2 = candidate.Item1 + di;
                                                r2 = candidate.Item2 + dr;
                                                c2 = candidate.Item3 + dc;
                                                if (i2 >= 0 && r2 >= 0 && c2 >= partition.Item1
                                                        && i2 < rampSpaceGrid.GetLength(0)
                                                        && r2 < rampSpaceGrid.GetLength(1)
                                                        && c2 < partition.Item2
                                                        && segmentationGrid[i2, r2, c2] == 0
                                                        && rampSpaceGrid[i2, r2, c2]) {
                                                    candidates.Enqueue((i2, r2, c2));
                                                }
                                            }
                                        }
                                    }
                                } while (candidates.Count > 0);
                            }
                        }
                    }
                });

            transitions = _transitions;

            return segmentationGrid;
        }

        private static void RemoveNegligibleRampSpaceSegments(
                double resolution,
                bool[,,] rampSpaceGrid,
                Dictionary<int, List<(int, int, int)>> rampSpaceSegments) {

            int roomMinArea = Parameters
                .ROOM_MIN_AREA
                .GetAreaInVoxels(resolution);

            rampSpaceSegments
                .Values
                .AsParallel()
                .Where(segment => segment.GetArea(rampSpaceGrid) < roomMinArea)
                .ForEach(segment => {
                    foreach ((int, int, int) voxel in segment) {
                        rampSpaceGrid[
                            voxel.Item1,
                            voxel.Item2,
                            voxel.Item3] = false;
                    }
                });
        }

        private static int GetArea(
                this List<(int, int, int)> segment,
                bool[,,] grid) {

            int i, area = 0;

            foreach ((int, int, int) voxel in segment) {
                i = voxel.Item1 + 1;
                if (i == grid.GetLength(0)
                        || !grid[
                            i,
                            voxel.Item2,
                            voxel.Item3]) {
                    area++;
                }
            }

            return area;
        }

        private static bool[,,] ApplyMorphologicalClosing(
                double resolution,
                bool[,,] rampSpaceGrid,
                int[,,][] reconstructionGrid) {

            int rampSpaceClosingRadius = Parameters
                .RAMP_SPACE_CLOSING_RADIUS
                .GetDistanceInVoxels(resolution);
            bool[,,] kernel;

            kernel = InitializeSphereKernel(rampSpaceClosingRadius);

            rampSpaceGrid = DilateRampSpace(
                rampSpaceGrid,
                kernel,
                reconstructionGrid);

            rampSpaceGrid = ErodeRampSpace(
                rampSpaceGrid,
                kernel,
                reconstructionGrid);

            return rampSpaceGrid;
        }

        private static bool[,,] InitializeSphereKernel(
                int radius) {

            int i, r, c;
            double center = radius - 1;
            double threshold = ((double)radius).Squared();
            bool[,,] kernel = new bool[
                2 * radius - 1,
                2 * radius - 1,
                2 * radius - 1];

            for (i = 0; i < kernel.GetLength(0); i++) {
                for (r = 0; r < kernel.GetLength(1); r++) {
                    for (c = 0; c < kernel.GetLength(2); c++) {
                        if ((center - i).Squared()
                                + (center - r).Squared()
                                + (center - c).Squared() <= threshold) {
                            kernel[i, r, c] = true;
                        }
                    }
                }
            }

            return kernel;
        }

        private static bool[,,] DilateRampSpace(
                bool[,,] grid,
                bool[,,] kernel,
                int[,,][] reconstructionGrid) {

            int s_i = (kernel.GetLength(0) - 1) / 2;
            int s_r = (kernel.GetLength(1) - 1) / 2;
            int s_c = (kernel.GetLength(2) - 1) / 2;
            bool[,,] result = new bool[
                grid.GetLength(0),
                grid.GetLength(1),
                grid.GetLength(2)];

            Parallel.For(
                s_i,
                grid.GetLength(0) - s_i,
                i => {

                    int i2, di, r, r2, dr, c, c2, dc;
                    int[] voxelState;
                    int[] voxelClassValues;

                    for (r = s_r; r < grid.GetLength(1) - s_r; r++) {
                        for (c = s_c; c < grid.GetLength(2) - s_c; c++) {

                            if (!grid[i, r, c]) {
                                continue;
                            }

                            for (di = -s_i; di <= s_i; di++) {
                                for (dr = -s_r; dr <= s_r; dr++) {
                                    for (dc = -s_c; dc <= s_c; dc++) {

                                        if (!kernel[
                                                di + s_i,
                                                dr + s_r,
                                                dc + s_r]) {
                                            continue;
                                        }

                                        i2 = i + di;
                                        r2 = r + dr;
                                        c2 = c + dc;

                                        voxelState = reconstructionGrid[i2, r2, c2];
                                        if (voxelState == null) {
                                            continue;
                                        }

                                        voxelClassValues = voxelState.GetVoxelClassValues(0);
                                        if (voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                                                || voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                                            result[i2, r2, c2] = true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                });

            return result;
        }

        private static bool[,,] ErodeRampSpace(
                bool[,,] grid,
                bool[,,] kernel,
                int[,,][] reconstructionGrid) {

            int s_i = (kernel.GetLength(0) - 1) / 2;
            int s_r = (kernel.GetLength(1) - 1) / 2;
            int s_c = (kernel.GetLength(2) - 1) / 2;
            bool[,,] result = new bool[
                grid.GetLength(0),
                grid.GetLength(1),
                grid.GetLength(2)];

            Parallel.For(
                s_i,
                grid.GetLength(0) - s_i,
                i => {

                    bool erode;
                    int i2, di, r, r2, dr, c, c2, dc;
                    int[] voxelState;
                    int[] voxelClassValues;

                    for (r = s_r; r < grid.GetLength(1) - s_r; r++) {
                        for (c = s_c; c < grid.GetLength(2) - s_c; c++) {

                            if (!grid[i, r, c]) {
                                continue;
                            }

                            erode = false;

                            for (di = -s_i; di <= s_i; di++) {
                                for (dr = -s_r; dr <= s_r; dr++) {
                                    for (dc = -s_c; dc <= s_c; dc++) {

                                        if (!kernel[
                                                di + s_i,
                                                dr + s_r,
                                                dc + s_r]) {
                                            continue;
                                        }

                                        i2 = i + di;
                                        r2 = r + dr;
                                        c2 = c + dc;
                                        if (grid[i2, r2, c2]) {
                                            continue;
                                        }

                                        voxelState = reconstructionGrid[i2, r2, c2];
                                        if (voxelState == null) {
                                            continue;
                                        }

                                        voxelClassValues = voxelState.GetVoxelClassValues(0);
                                        if (!voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                                                && !voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                                            continue;
                                        }

                                        erode = true;
                                        break;
                                    }
                                    if (erode) {
                                        break;
                                    }
                                }
                            }
                            if (!erode) {
                                result[i, r, c] = true;
                            }
                        }
                    }
                });
            return result;
        }

        private static void ExtendRampSpaceVerticallyUntilFloor(
                bool[,,] rampSpaceGrid,
                int[,,][] reconstructionGrid) {

            Parallel.For(
                0,
                 rampSpaceGrid.GetLength(2),
                 c => {

                     int i, i2, r;
                     int[] voxelState;
                     int[] voxelClassValues;

                     for (i = 0; i < rampSpaceGrid.GetLength(0) - 1; i++) {
                         for (r = 0; r < rampSpaceGrid.GetLength(1); r++) {

                             i2 = i + 1;
                             if (!rampSpaceGrid[i, r, c]
                                     || rampSpaceGrid[i2, r, c]) {
                                 continue;
                             }

                             voxelState = reconstructionGrid[i2, r, c];
                             if (voxelState == null) {
                                 continue;
                             }

                             voxelClassValues = voxelState.GetVoxelClassValues(0);
                             if (voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                                     || voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                                 rampSpaceGrid[i2, r, c] = true;
                             }
                         }
                     }
                 });
        }

        private static bool[,,] FindRemoveCandidateVoxels(
                bool[,,] wallGrid,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            bool[,,] removeCandidateGrid = new bool[
                reconstructionGrid.GetLength(0),
                reconstructionGrid.GetLength(1),
                reconstructionGrid.GetLength(2)];

            Parallel.For(
                0,
                reconstructionGrid.GetLength(2),
                c => {

                    int i, r;

                    for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                        for (i = 0; i < reconstructionGrid.GetLength(0); i++) {
                            if (IsRemoveCanidate(
                                    i,
                                    r,
                                    c,
                                    wallGrid,
                                    normalGrid,
                                    reconstructionGrid)) {
                                removeCandidateGrid[i, r, c] = true;
                            }
                        }
                    }
                });

            return removeCandidateGrid;
        }

        private static bool IsRemoveCanidate(
                int i,
                int r,
                int c,
                bool[,,] wallGrid,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            int i2;
            int[] voxelState;
            int[] voxelClassValues;

            voxelState = reconstructionGrid[i, r, c];
            if (voxelState == null) {
                return false;
            }

            voxelClassValues = voxelState.GetVoxelClassValues(0);
            if (normalGrid[i, r, c] != NormalGridValues.EMPTY
                    || (!voxelClassValues.Contains(VoxelClassValues.FLOOR)
                        && !voxelClassValues.Contains(VoxelClassValues.CEILING))) {
                return false;
            }

            if (voxelClassValues.Contains(VoxelClassValues.FLOOR)
                    && voxelClassValues.Contains(VoxelClassValues.WALL)) {
                i2 = i - 1;
                if (i2 >= 0 && wallGrid[i2, r, c]) {
                    return false;
                }
            }

            if (voxelClassValues.Contains(VoxelClassValues.CEILING)
                    && voxelClassValues.Contains(VoxelClassValues.WALL)) {
                i2 = i - 1;
                if (i2 >= 0 && wallGrid[i2, r, c]) {
                    return false;
                }
            }

            if (voxelClassValues.Contains(VoxelClassValues.FLOOR)
                    && IsValidHorizontalSurface(
                        i,
                        r,
                        c,
                        reconstructionGrid)) {
                return false;
            }

            return true;
        }

        private static bool IsValidHorizontalSurface(
                int i,
                int r,
                int c,
                int[,,][] reconstructionGrid) {

            int i2 = i - 1;
            int[] voxelState;
            int[] voxelClassValues;

            while (true) {

                if (i2 < 0) {
                    break;
                }

                voxelState = reconstructionGrid[i2, r, c];
                if (voxelState == null) {
                    break;
                }


                voxelClassValues = voxelState.GetVoxelClassValues(0);
                if (voxelClassValues.Contains(VoxelClassValues.FLOOR)) {
                    i2--;
                    continue;
                }

                if (!voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)) {
                    if (voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                        return true;
                    }
                    break;
                }
                i2--;
            }

            return false;
        }

        private static bool CheckSegmentationSeed(
                int i,
                int r,
                int c,
                bool[,,] isSegmented,
                bool[,,] removeCandidateGrid,
                int[,,][] reconstructionGrid,
                out bool isVerticalOrientationDetermined,
                out bool isFloor) {

            int[] voxelState;
            int[] voxelClassValues;

            isVerticalOrientationDetermined = isFloor = false;
            if (!removeCandidateGrid[i, r, c]
                    || isSegmented[i, r, c]) {
                return false;
            }

            voxelState = reconstructionGrid[i, r, c];
            if (voxelState == null) {
                return false;
            }

            voxelClassValues = voxelState.GetVoxelClassValues(0);
            if (!voxelClassValues.Contains(VoxelClassValues.FLOOR)
                    || !voxelClassValues.Contains(VoxelClassValues.CEILING)) {
                
                isVerticalOrientationDetermined = true;

                if (voxelClassValues.Contains(VoxelClassValues.FLOOR)) {
                    isFloor = true;
                }
                else if (voxelClassValues.Contains(VoxelClassValues.CEILING)) {
                    isFloor = false;
                }
                else {
                    return false;
                }
            }

            return true;
        }

        private static List<(int, int, int)> Segment(
                bool isVerticalOrientationDetermined,
                bool isFloor,
                int i,
                int r,
                int c,
                double resolution,
                bool[,,] isSegmented,
                bool[,,] removeCandidateGrid,
                int[,,][] reconstructionGrid) {

            int di, i2, dr, r2, dc, c2;
            int maxHeightDifference = Parameters
                .FLOOR_MAX_HEIGHT_DIFFERENCE
                .GetDistanceInVoxels(resolution);
            List<(int, int, int)> segment = new List<(int, int, int)>();
            Queue<(int, int, int)> candidates = new Queue<(int, int, int)>();

            candidates.Enqueue((i, r, c));
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

                segment.Add(candidate);

                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        r2 = candidate.Item2 + dr;
                        c2 = candidate.Item3 + dc;
                        if (r2 < 0 || c2 < 0
                                || r2 >= reconstructionGrid.GetLength(1)
                                || c2 >= reconstructionGrid.GetLength(2)) {
                            continue;
                        }

                        if (!isSegmented[candidate.Item1, r2, c2]
                                && removeCandidateGrid[candidate.Item1, r2, c2]) {
                            if (CheckSegmentationCandidate(
                                    true,
                                    ref isVerticalOrientationDetermined,
                                    ref isFloor,
                                    reconstructionGrid[candidate.Item1, r2, c2])) {
                                candidates.Enqueue((candidate.Item1, r2, c2));
                            }
                            continue;
                        }

                        for (di = 1; di <= maxHeightDifference; di++) {

                            i2 = candidate.Item1 + di;
                            if (i2 >= reconstructionGrid.GetLength(0)
                                    || isSegmented[i2, r2, c2]) {
                                break;
                            }

                            if (!isSegmented[i2, r2, c2]
                                    && removeCandidateGrid[i2, r2, c2]) {
                                if (CheckSegmentationCandidate(
                                        false,
                                        ref isVerticalOrientationDetermined,
                                        ref isFloor,
                                        reconstructionGrid[i2, r2, c2])) {
                                    candidates.Enqueue((i2, r2, c2));
                                    break;
                                }
                                continue;
                            }

                            i2 = candidate.Item1 - di;
                            if (i2 < 0
                                    || isSegmented[i2, r2, c2]) {
                                break;
                            }

                            if (!isSegmented[i2, r2, c2]
                                    && removeCandidateGrid[i2, r2, c2]) {
                                if (CheckSegmentationCandidate(
                                        false,
                                        ref isVerticalOrientationDetermined,
                                        ref isFloor,
                                        reconstructionGrid[i2, r2, c2])) {
                                    candidates.Enqueue((i2, r2, c2));
                                    break;
                                }
                                continue;
                            }
                        }
                    }
                }
            } while (candidates.Count > 0);

            return segment;
        }

        private static bool CheckSegmentationCandidate(
                bool isHorizontalNeighbour,
                ref bool isVerticalOrientationDetermined,
                ref bool isFloor,
                int[] voxelState) {

            int[] voxelClassValues;

            if (voxelState == null) {
                return false;
            }

            voxelClassValues = voxelState.GetVoxelClassValues(0);

            if (isVerticalOrientationDetermined) {
                if (!voxelClassValues.Contains(VoxelClassValues.FLOOR)
                        || !voxelClassValues.Contains(VoxelClassValues.CEILING)) {

                    isVerticalOrientationDetermined = true;

                    if (voxelClassValues.Contains(VoxelClassValues.FLOOR)) {
                        isFloor = true;
                    }
                    else if (voxelClassValues.Contains(VoxelClassValues.CEILING)) {
                        isFloor = false;
                    }
                    else {
                        if (isHorizontalNeighbour) {
                            isVerticalOrientationDetermined = false;
                        }
                        return false;
                    }
                }
            }
            else {
                if (isFloor
                        && !voxelClassValues.Contains(VoxelClassValues.FLOOR)) {
                    return false;
                }
                if (!isFloor
                        && !voxelClassValues.Contains(VoxelClassValues.CEILING)) {
                    return false;
                }
            }

            return true;
        }

        private static bool CheckSegmentArea(
                int minArea,
                List<(int, int, int)> segment) {

            int dr, r, dc, c;
            int counter = 0;
            int counter2;
            HashSet<(int, int)> positions = new HashSet<(int, int)>();

            foreach ((int, int, int) voxel in segment) {
                positions.Add((
                    voxel.Item2,
                    voxel.Item3));
            }

            foreach ((int, int) position in positions) {

                counter2 = 0;

                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (dr.Abs() == dc.Abs()) {
                            continue;
                        }

                        r = position.Item1 + dr;
                        c = position.Item2 + dc;
                        if (positions.Contains((r, c))) {
                            counter2++;
                        }
                    }
                }

                if (counter2 >= 3) {
                    counter++;
                    if (counter >= minArea) {
                        return true;
                    }
                }
            }

            return false;
        }

        private static bool CheckSegmentRampRatio(
                bool[,,] rampSpaceGrid,
                int[,,][] reconstructionGrid,
                List<(int, int, int)> segment) {

            int i;
            int counter = 0;
            int[] deltas;
            int[] voxelState;

            foreach ((int, int, int) voxel in segment) {

                voxelState = reconstructionGrid[
                    voxel.Item1,
                    voxel.Item2,
                    voxel.Item3];
                if (voxelState == null) {
                    continue;
                }

                deltas = GetDeltas(voxelState);
                if (deltas.Length == 0) {
                    continue;
                }

                foreach (int d in deltas) {
                    i = voxel.Item1 + d;
                    if (i >= 0
                            && i < reconstructionGrid.GetLength(0)
                            && rampSpaceGrid[
                                i,
                                voxel.Item2,
                                voxel.Item3]) {
                        counter++;
                        break;
                    }
                }
            }

            return (double)counter / segment.Count
                >= Parameters.HORIZONTAL_SURFACE_REFINEMENT_MIN_RAMP_RATIO;
        }

        private static int[] GetDeltas(
                int[] voxelState) {

            int[] voxelClassValues = voxelState.GetVoxelClassValues(0);

            if (voxelClassValues.Contains(VoxelClassValues.FLOOR)
                    && voxelClassValues.Contains(VoxelClassValues.CEILING)) {
                return new int[] { -1, 1 };
            }
            
            if (voxelClassValues.Contains(VoxelClassValues.FLOOR)) {
                return new int[] { -1 };
            }
            
            if (voxelClassValues.Contains(VoxelClassValues.CEILING)) {
                return new int[] { 1 };
            }

            return new int[0];
        }

        private static void RemoveSegment(
                bool[,,] rampSpaceGrid,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid,
                List<(int, int, int)> segment) {

            int[] voxelState;
            int[] deltas;

            foreach ((int, int, int) voxel in segment) {

                rampSpaceGrid[
                    voxel.Item1,
                    voxel.Item2,
                    voxel.Item3] = true;

                voxelState = reconstructionGrid[
                    voxel.Item1,
                    voxel.Item2,
                    voxel.Item3];
                if (voxelState == null) {
                    continue;
                }

                deltas = GetDeltas(voxelState);
                if (deltas.Length == 0) {
                    continue;
                }

                reconstructionGrid[
                        voxel.Item1,
                        voxel.Item2,
                        voxel.Item3]
                    = VoxelState.CreateVoxelState(
                        0,
                        normalGrid[
                                voxel.Item1,
                                voxel.Item2,
                                voxel.Item3] == NormalGridValues.EMPTY ?
                            VoxelClassValues.EMPTY_INTERIOR :
                            VoxelClassValues.INTERIOR_OBJECT);

                RefineRampSpace(
                    voxel,
                    deltas,
                    rampSpaceGrid,
                    reconstructionGrid);
            }
        }

        private static void RefineRampSpace(
                (int, int, int) voxel,
                int[] deltas,
                bool[,,] rampSpaceGrid,
                int[,,][] reconstructionGrid) {

            int i;
            int[] voxelState;
            int[] voxelClassValues;

            foreach (int d in deltas) {

                i = voxel.Item1 + d;

                while (true) {

                    if (i < 0
                            || i >= reconstructionGrid.GetLength(0)
                            || rampSpaceGrid[
                                i,
                                voxel.Item2,
                                voxel.Item3]) {
                        break;
                    }

                    voxelState = reconstructionGrid[
                        i,
                        voxel.Item2,
                        voxel.Item3];
                    if (voxelState == null) {
                        break;
                    }

                    voxelClassValues = voxelState.GetVoxelClassValues(0);
                    if (!voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                            && !voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                        break;
                    }

                    rampSpaceGrid[
                        i,
                        voxel.Item2,
                        voxel.Item3] = true;

                    i += d;
                }
            }
        }

        private static void RefineCeilingSurfaces(
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            Parallel.For(
                0,
                reconstructionGrid.GetLength(2),
                c => {

                    bool outOfCeiling;
                    int i, i2, r;
                    int[] voxelClassValues;
                    int[] voxelState;

                    for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                        for (i = 0; i < reconstructionGrid.GetLength(0); i++) {

                            voxelState = reconstructionGrid[i, r, c];
                            if (voxelState == null) {
                                continue;
                            }

                            voxelClassValues = voxelState.GetVoxelClassValues(0);
                            if (!voxelClassValues.Contains(VoxelClassValues.CEILING)
                                    || voxelClassValues.Contains(VoxelClassValues.WALL)) {
                                continue;
                            }

                            i2 = i + 1;
                            outOfCeiling = false;

                            while (true) {

                                if (i2 >= reconstructionGrid.GetLength(0)) {
                                    break;
                                }

                                voxelState = reconstructionGrid[i2, r, c];
                                if (voxelState == null) {
                                    outOfCeiling = true;
                                    reconstructionGrid[i2, r, c] = VoxelState.CreateVoxelState(
                                        0,
                                        normalGrid[i2, r, c] == NormalGridValues.EMPTY ?
                                            VoxelClassValues.EMPTY_INTERIOR :
                                            VoxelClassValues.INTERIOR_OBJECT);
                                    i2++;
                                    continue;
                                }

                                voxelClassValues = voxelState.GetVoxelClassValues(0);
                                if (voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                                        || voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                                    outOfCeiling = true;
                                    i2++;
                                    continue;
                                }

                                if (outOfCeiling
                                        && voxelClassValues.Contains(VoxelClassValues.CEILING)) {

                                    reconstructionGrid[i2, r, c] = VoxelState.CreateVoxelState(
                                        0,
                                        new int[] { 
                                            VoxelClassValues.FLOOR,
                                            VoxelClassValues.CEILING
                                        });
                                }
                                break;
                            }
                        }
                    }
                });
        }
    }
}
using HuePat.VoxIR.Util.Grid;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace HuePat.VoxIR.NormalGridDetermination {

    public static class NormalGridDetermination {
        public const int UPPER_OUTSIDE_ID = -1;
        public const int LOWER_OUTSIDE_ID = -2;

        public static byte[,,] RedetermineNormalGridValues(
                double resolution,
                byte[,,] normalGrid) {

            bool[,,] occupancyGrid = InitializeOccupancyGrid(normalGrid);

            return DetermineNormalGrid(
                resolution,
                occupancyGrid);
        }

        public static byte[,,] DetermineNormalGrid(
                double resolution,
                bool[,,] occupancyGrid) {

            byte[,,] normalGrid = new byte[
                occupancyGrid.GetLength(0),
                occupancyGrid.GetLength(1),
                occupancyGrid.GetLength(2)];
            int[,,] verticalDistanceAboveGrid;
            int[,,] normalVerticalSegmentationGrid;
            HashSet<int> normalUpSegmentIds;
            HashSet<int> normalDownSegmentIds;
            VerticalAdjacencyGraph verticalAdjacencyGraph;
            Dictionary<int, List<(int, int, int)>> normalVerticalSegments;

            InitializeNormalHorizontalVoxels(
                resolution,
                occupancyGrid,
                normalGrid);

            RefineNormalHorizontalVoxels(
                resolution,
                occupancyGrid,
                normalGrid);

            verticalDistanceAboveGrid = GetVerticalDistanceAboveGrid(
                occupancyGrid,
                normalGrid);

            normalVerticalSegmentationGrid = SegmentNormalVerticalVoxels(
                resolution,
                occupancyGrid,
                normalGrid,
                verticalDistanceAboveGrid,
                out normalVerticalSegments);

            verticalAdjacencyGraph = GetVerticalAdjacencyGraph(
                normalGrid,
                normalVerticalSegmentationGrid);

            ClassifyNormalVerticalSegments(
                resolution,
                normalGrid,
                normalVerticalSegmentationGrid,
                verticalAdjacencyGraph,
                normalVerticalSegments,
                out normalUpSegmentIds,
                out normalDownSegmentIds);

            UpdateNormalGrid(
                normalGrid,
                normalVerticalSegmentationGrid,
                normalUpSegmentIds,
                normalDownSegmentIds);

            RefineNormalGrid(normalGrid);

            return normalGrid;
        }

        private static bool[,,] InitializeOccupancyGrid(
                byte[,,] normalGrid) {

            bool[,,] occupancyGrid = new bool[
                normalGrid.GetLength(0),
                normalGrid.GetLength(1),
                normalGrid.GetLength(2)];

            Parallel.For(
                0,
                normalGrid.GetLength(0),
                i => {

                    int r, c;

                    for (r = 0; r < normalGrid.GetLength(1); r++) {
                        for (c = 0; c < normalGrid.GetLength(2); c++) {

                            if (normalGrid[i, r, c] != NormalGridValues.EMPTY) {
                                occupancyGrid[i, r, c] = true;
                            }
                        }
                    }
                });

            return occupancyGrid;
        }

        private static void InitializeNormalHorizontalVoxels(
                double resolution,
                bool[,,] occupancyGrid,
                byte[,,] normalGrid) {

            int normalHorizontalMinVoxelStackHeight = Parameters
                .NORMAL_GRID_DETERMINATION_NORMAL_HORIZONTAL_MIN_VOXEL_STACK_HEIGHT
                .GetDistanceInVoxels(resolution);

            Parallel.For(
                0,
                occupancyGrid.GetLength(2),
                c => {

                    int i, i2, i3, r;

                    for (r = 0; r < occupancyGrid.GetLength(1); r++) {
                        for (i = 0; i < occupancyGrid.GetLength(0); i++) {

                            if (!occupancyGrid[i, r, c]) {
                                continue;
                            }

                            i2 = i;
                            do {
                                i2++;
                                if (i2 >= occupancyGrid.GetLength(0) - 1
                                        || !occupancyGrid[i2, r, c]) {
                                    break;
                                }
                            } while (true);

                            if (i2 - i + 1 >= normalHorizontalMinVoxelStackHeight) {
                                for (i3 = i; i3 <= i2; i3++) {
                                    normalGrid[i3, r, c] = NormalGridValues.NORMAL_HORIZONTAL;
                                }
                            }

                            i = i2;
                        }
                    }
                });
        }

        private static void RefineNormalHorizontalVoxels(
                double resolution,
                bool[,,] occupancyGrid,
                byte[,,] normalGrid) {

            AssignNegligibleNormalVerticalCandidatePositionSegmentsToNormalHorizontalVoxels(
                resolution,
                occupancyGrid,
                normalGrid);

            VerticallyGrowNormalHorizontalVoxelsColumns(
                occupancyGrid,
                normalGrid);
        }

        private static void AssignNegligibleNormalVerticalCandidatePositionSegmentsToNormalHorizontalVoxels(
                double resolution,
                bool[,,] occupancyGrid,
                byte[,,] normalGrid) {

            int maxRefinementDistance = Parameters
                .NORMAL_GRID_DETERMINATION_NORMAL_HORIZONTAL_REFINEMENT_MAX_DISTANCE
                .GetDistanceInVoxels(resolution);
            int maxRefinementArea = Parameters
                .NORMAL_GRID_DETERMINATION_NORMAL_HORIZONTAL_REFINEMENT_MAX_AREA
                .GetAreaInVoxels(resolution);

            Parallel.For(
                0,
                normalGrid.GetLength(0),
                i => {

                    int r, c;
                    bool[,] isSegmented = new bool[
                        normalGrid.GetLength(1),
                        normalGrid.GetLength(2)];
                    HashSet<(int, int)> normalHorizontalContactPositions;
                    List<(int, int)> normalVerticalCandidateSegment;

                    for (r = 0; r < normalGrid.GetLength(1); r++) {
                        for (c = 0; c < normalGrid.GetLength(2); c++) {

                            if (!occupancyGrid[i, r, c]
                                    || normalGrid[i, r, c] == NormalGridValues.NORMAL_HORIZONTAL) {
                                continue;
                            }

                            normalVerticalCandidateSegment = SegmentNormalVerticalCandidatePositions(
                                i,
                                r,
                                c,
                                isSegmented,
                                occupancyGrid,
                                normalGrid,
                                out normalHorizontalContactPositions);

                            if (CanAssignNormalVerticalCandidateSegmentToNormalHorizontalVoxels(
                                    maxRefinementDistance,
                                    maxRefinementArea,
                                    normalVerticalCandidateSegment,
                                    normalHorizontalContactPositions)) {

                                foreach ((int, int) normalVerticalCandidatePosition in normalVerticalCandidateSegment) {
                                    normalGrid[
                                        i,
                                        normalVerticalCandidatePosition.Item1,
                                        normalVerticalCandidatePosition.Item2] = NormalGridValues.NORMAL_HORIZONTAL;
                                }
                            }
                        }
                    }
                });
        }

        private static List<(int, int)> SegmentNormalVerticalCandidatePositions(
                int i,
                int r,
                int c,
                bool[,] isSegmented,
                bool[,,] occupancyGrid,
                byte[,,] normalGrid,
                out HashSet<(int, int)> normalHorizontalContactPositions) {

            int dr, r2, dc, c2;
            Queue<(int, int)> candidates = new Queue<(int, int)>();
            List<(int, int)> segment = new List<(int, int)>();

            normalHorizontalContactPositions = new HashSet<(int, int)>();

            candidates.Enqueue((r, c));

            do {
                (int, int) candidate = candidates.Dequeue();

                if (isSegmented[
                        candidate.Item1,
                        candidate.Item2]) {
                    continue;
                }

                segment.Add(candidate);
                isSegmented[
                    candidate.Item1,
                    candidate.Item2] = true;

                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        r2 = candidate.Item1 + dr;
                        c2 = candidate.Item2 + dc;

                        if (r2 < 0 || c2 < 0
                                || r2 >= normalGrid.GetLength(1)
                                || c2 >= normalGrid.GetLength(2)) {
                            continue;
                        }

                        if (normalGrid[i, r2, c2] == NormalGridValues.NORMAL_HORIZONTAL) {
                            normalHorizontalContactPositions.Add((r2, c2));
                            continue;
                        }

                        if (dr.Abs() != dc.Abs()
                                &&!isSegmented[r2, c2]
                                && occupancyGrid[i, r2, c2]
                                && normalGrid[i, r2, c2] == NormalGridValues.EMPTY) {

                            candidates.Enqueue((r2, c2));
                        }
                    }
                }

            } while (candidates.Count > 0);

            return segment;
        }

        private static bool CanAssignNormalVerticalCandidateSegmentToNormalHorizontalVoxels(
                int maxRefinementDistance,
                int maxRefinementArea,
                List<(int, int)> normalVerticalCandidateSegment,
                HashSet<(int, int)> normalHorizontalContactPositions) {

            if (normalHorizontalContactPositions.Count == 0
                    || normalVerticalCandidateSegment.Count >= maxRefinementArea) {
                return false;
            }

            return !normalVerticalCandidateSegment
                .Any(normalVerticalCandidatePosition => normalHorizontalContactPositions
                    .Min(normalHorizontalContactPosition => PixelUtils.GetDistance(
                        normalVerticalCandidatePosition,
                        normalHorizontalContactPosition)) > maxRefinementDistance);
        }

        private static void VerticallyGrowNormalHorizontalVoxelsColumns(
                bool[,,] occupancyGrid,
                byte[,,] normalGrid) {

            Parallel.For(
                0,
                normalGrid.GetLength(2),
                c => {

                    int di, i, i2, r;

                    for (i = 0; i < normalGrid.GetLength(0); i++) {
                        for (r = 0; r < normalGrid.GetLength(1); r++) {

                            if (normalGrid[i, r, c] != NormalGridValues.NORMAL_HORIZONTAL) {
                                continue;
                            }

                            for (di = -1; di <= 1; di += 2) {

                                i2 = i;
                                do {

                                    i2 += di;
                                    if (i2 < 0
                                        || i2 >= normalGrid.GetLength(0)) {
                                        break;
                                    }

                                    if (normalGrid[i2, r, c] == NormalGridValues.NORMAL_HORIZONTAL
                                            || !occupancyGrid[i2, r, c]) {
                                        break;
                                    }

                                    normalGrid[i2, r, c] = NormalGridValues.NORMAL_HORIZONTAL;

                                } while (true);
                            }
                        }
                    }
                });
        }

        private static int[,,] GetVerticalDistanceAboveGrid(
                bool[,,] occupancyGrid,
                byte[,,] normalGrid) {

            int[,,] verticalDistanceAboveGrid = new int[
                normalGrid.GetLength(0),
                normalGrid.GetLength(1),
                normalGrid.GetLength(2)];

            Parallel.For(
                0,
                normalGrid.GetLength(2),
                c => {

                    bool hasLeftSurface;
                    int i, i2, r;
                    int verticalDistanceAbove;

                    for (r = 0; r < normalGrid.GetLength(1); r++) {
                        for (i = normalGrid.GetLength(0) - 1; i >= 0; i--) {

                            if (!occupancyGrid[i, r, c]
                                    || normalGrid[i, r, c] == NormalGridValues.NORMAL_HORIZONTAL) {
                                continue;
                            }

                            i2 = i;
                            verticalDistanceAbove = 0;
                            hasLeftSurface = false;

                            do {

                                i2--;
                                if (i2 < 0) {
                                    break;
                                }

                                if (occupancyGrid[i2, r, c]) {
                                    if (hasLeftSurface) {
                                        verticalDistanceAbove = i2;
                                        break;
                                    }
                                }
                                else {
                                    hasLeftSurface = true;
                                }

                            } while (true);

                            i2 = i;
                            verticalDistanceAboveGrid[i, r, c] = verticalDistanceAbove;

                            do {

                                i2--;
                                if (i2 < 0) {
                                    break;
                                }

                                if (!occupancyGrid[i2, r, c]) {
                                    break;
                                }

                                verticalDistanceAboveGrid[i2, r, c] = verticalDistanceAbove;

                            } while (true);

                            i = i2;
                        }
                    }
                });

            return verticalDistanceAboveGrid;
        }

        private static int[,,] SegmentNormalVerticalVoxels(
                double resolution,
                bool[,,] occupancyGrid,
                byte[,,] normalGrid,
                int[,,] verticalDistanceAboveGrid,
                out Dictionary<int, List<(int, int, int)>> normalVerticalSegments) {

            int i, r, c;
            int segmentId = 0;
            int minSegmentArea = Parameters
                .NORMAL_GRID_DETERMINATION_NORMAL_VERTICAL_SEGMENT_MIN_AREA
                .GetAreaInVoxels(resolution);
            int maxHeightDifference = Parameters
                .FLOOR_MAX_HEIGHT_DIFFERENCE
                .GetDistanceInVoxels(resolution);
            int[,,] normalVerticalSegmentationGrid = new int[
                normalGrid.GetLength(0),
                normalGrid.GetLength(1),
                normalGrid.GetLength(2)];
            HashSet<(int, int)> positions;

            normalVerticalSegments = new Dictionary<int, List<(int, int, int)>>();

            for (i = 0; i < normalGrid.GetLength(0); i++) {
                for (r = 0; r < normalGrid.GetLength(1); r++) {
                    for (c = 0; c < normalGrid.GetLength(2); c++) {

                        if (!occupancyGrid[i, r, c]
                                || normalGrid[i, r, c] == NormalGridValues.NORMAL_HORIZONTAL
                                || normalVerticalSegmentationGrid[i, r, c] != 0) {
                            continue;
                        }

                        List<(int, int, int)> segment = GrowNormalVerticalSegment(
                            ++segmentId,
                            (i, r, c),
                            occupancyGrid,
                            normalGrid,
                            verticalDistanceAboveGrid,
                            normalVerticalSegmentationGrid,
                            out positions);

                        if (positions.Count >= minSegmentArea
                                || IsSameLevelWithNeighbourhood(
                                    segmentId,
                                    maxHeightDifference,
                                    verticalDistanceAboveGrid,
                                    normalVerticalSegmentationGrid,
                                    segment)) {

                            normalVerticalSegments.Add(
                                segmentId,
                                segment);
                        }
                        else {
                            foreach ((int, int, int) voxel in segment) {

                                normalVerticalSegmentationGrid[
                                    voxel.Item1,
                                    voxel.Item2,
                                    voxel.Item3] = 0;

                                normalGrid[
                                    voxel.Item1,
                                    voxel.Item2,
                                    voxel.Item3] = NormalGridValues.NORMAL_HORIZONTAL;
                            }
                        }
                    }
                }
            }

            return normalVerticalSegmentationGrid;
        }

        private static List<(int, int, int)> GrowNormalVerticalSegment(
                int segmentId,
                (int, int, int) voxel,
                bool[,,] occupancyGrid,
                byte[,,] normalGrid,
                int[,,] verticalDistanceAboveGrid,
                int[,,] normalVerticalSegmentationGrid,
                out HashSet<(int, int)> positions) {

            bool isTopmostLayer;
            int di, i, dr, r, dc, c;
            int verticalDistanceAbove;
            (int, int, int) candidate;
            List<(int, int, int)> segment = new List<(int, int, int)>();
            Queue<(int, int, int)> candidates = new Queue<(int, int, int)>();

            positions = new HashSet<(int, int)>();

            candidates.Enqueue(voxel);

            do {

                candidate = candidates.Dequeue();
                if (normalVerticalSegmentationGrid[
                        candidate.Item1,
                        candidate.Item2,
                        candidate.Item3] != 0) {
                    continue;
                }

                positions.Add((
                    candidate.Item2,
                    candidate.Item3));

                segment.Add((
                    candidate.Item1,
                    candidate.Item2,
                    candidate.Item3));

                normalVerticalSegmentationGrid[
                    candidate.Item1,
                    candidate.Item2,
                    candidate.Item3] = segmentId;

                verticalDistanceAbove = verticalDistanceAboveGrid[
                    candidate.Item1,
                    candidate.Item2,
                    candidate.Item3];

                isTopmostLayer = verticalDistanceAbove == 0;

                for (di = -1; di <= 1; di++) {
                    for (dr = -1; dr <= 1; dr++) {
                        for (dc = -1; dc <= 1; dc++) {

                            if (di.Abs() + dr.Abs() + dc.Abs() != 1) {
                                continue;
                            }

                            i = candidate.Item1 + di;
                            r = candidate.Item2 + dr;
                            c = candidate.Item3 + dc;
                            if (i < 0 || r < 0 || c < 0
                                    || i >= occupancyGrid.GetLength(0)
                                    || r >= occupancyGrid.GetLength(1)
                                    || c >= occupancyGrid.GetLength(2)) {
                                continue;
                            }

                            if (occupancyGrid[i, r, c]
                                    && normalGrid[i, r, c] == NormalGridValues.EMPTY
                                    && ((isTopmostLayer
                                            && verticalDistanceAboveGrid[i, r, c] == 0)
                                        || (!isTopmostLayer
                                            && verticalDistanceAboveGrid[i, r, c] != 0
                                            && (verticalDistanceAboveGrid[i, r, c] - verticalDistanceAbove).Abs() <= 1))) {

                                candidates.Enqueue((i, r, c));
                            }
                        }
                    }
                }
            } while (candidates.Count > 0);

            return segment;
        }

        private static bool IsSameLevelWithNeighbourhood(
                int segmentId,
                int maxHeightDifference,
                int[,,] verticalDistanceAboveGrid,
                int[,,] normalVerticalSegmentationGrid,
                List<(int, int, int)> segment) {

            bool stop = false;
            bool foundBorder;
            bool foundSameSegmentNeighbour;
            bool foundOtherSegmentNeighbour;
            int d, di, i, dr, r, dc, c;
            int borderCount = 0;
            int sameLevelBorderCount = 0;

            foreach ((int, int, int) voxel in segment) {

                foundBorder = foundOtherSegmentNeighbour = false;

                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (dr.Abs() != 1 
                                && dc.Abs() != 1) {
                            continue;
                        }

                        r = voxel.Item2 + dr;
                        c = voxel.Item3 + dc;

                        if (r < 0 || c < 0
                                || r >= verticalDistanceAboveGrid.GetLength(1)
                                || c >= verticalDistanceAboveGrid.GetLength(2)) {
                            continue;
                        }

                        stop = false;
                        foundSameSegmentNeighbour = false;

                        for (di = 0; di <= maxHeightDifference; di++) {
                            for (d = -1; d <= 1; d += 2) {

                                i = voxel.Item1 + d * di;

                                if (i < 0
                                        || i >= verticalDistanceAboveGrid.GetLength(0)) {
                                    continue;
                                }

                                if (normalVerticalSegmentationGrid[i, r, c] == segmentId) {
                                    stop = true;
                                    foundSameSegmentNeighbour = true;
                                    break;
                                }

                                if (verticalDistanceAboveGrid[i, r, c] != 0) {
                                    stop = true;
                                    foundOtherSegmentNeighbour = true;
                                    break;
                                }

                                if (di == 0) {
                                    continue;
                                }
                            }

                            if (stop) {
                                break;
                            }
                        }

                        if (!foundSameSegmentNeighbour) {
                            foundBorder = true;
                        }

                        if (foundBorder && foundOtherSegmentNeighbour) {
                            stop = true;
                        }
                        else {
                            stop = false;
                        }
                    }

                    if (stop) {
                        break;
                    }
                }

                if (foundBorder) {
                    borderCount++;
                }
                if (foundOtherSegmentNeighbour) {
                    sameLevelBorderCount++;
                }
            }

            return (double)sameLevelBorderCount / borderCount 
                >= Parameters.NORMAL_GRID_DETERMINATION_MIN_BORDER_FILL_RATIO;
        }

        private static VerticalAdjacencyGraph GetVerticalAdjacencyGraph(
                byte[,,] normalGrid,
                int[,,] normalVerticalSegmentationGrid) {

            object @lock = new object();
            VerticalAdjacencyGraph verticalAdjacencyGraph = new VerticalAdjacencyGraph();

            Parallel.ForEach(
                Partitioner.Create(
                    0,
                    normalVerticalSegmentationGrid.GetLength(2)),
                () => new VerticalAdjacencyGraph(),
                (partition, loopState, localVerticalAdjacencyGraph) => {

                    for (int c = partition.Item1; c < partition.Item2; c++) {

                        UpdateVerticalAdjacencyGraph(
                            c,
                            normalGrid,
                            normalVerticalSegmentationGrid,
                            localVerticalAdjacencyGraph);
                    }

                    return localVerticalAdjacencyGraph;
                },
                localVerticalAdjacencyGraph => {

                    lock (@lock) {
                        verticalAdjacencyGraph.Add(localVerticalAdjacencyGraph);
                    }
                });

            return verticalAdjacencyGraph;
        }

        private static void UpdateVerticalAdjacencyGraph(
                int c,
                byte[,,] normalGrid,
                int[,,] normalVerticalSegmentationGrid,
                VerticalAdjacencyGraph verticalAdjacencyGraph) {

            bool foundFirst;
            int i, i2, r;
            int segmentId, segmentId2;

            for (r = 0; r < normalVerticalSegmentationGrid.GetLength(1); r++) {

                foundFirst = false;

                for (i = normalVerticalSegmentationGrid.GetLength(0) - 1; i >= 0; i--) {

                    segmentId = normalVerticalSegmentationGrid[i, r, c];

                    if (segmentId == 0) {
                        continue;
                    }

                    if (!foundFirst) {

                        foundFirst = true;
                        verticalAdjacencyGraph.Add(
                            LOWER_OUTSIDE_ID,
                            segmentId,
                            normalVerticalSegmentationGrid.GetLength(0) - 1 - i);
                    }

                    i2 = i;

                    do {

                        i2--;
                        if (i2 < 0) {

                            verticalAdjacencyGraph.Add(
                                segmentId,
                                UPPER_OUTSIDE_ID,
                                i);

                            break;
                        }

                        if (normalGrid[i2, r, c] == NormalGridValues.NORMAL_HORIZONTAL) {
                            break;
                        }

                        segmentId2 = normalVerticalSegmentationGrid[i2, r, c];

                        if (segmentId2 == 0
                                || segmentId2 == segmentId) {
                            continue;
                        }

                        verticalAdjacencyGraph.Add(
                            segmentId,
                            segmentId2,
                            i - i2);

                        break;

                    } while (true);

                    i = i2;
                }
            }
        }

        private static void ClassifyNormalVerticalSegments(
                double resolution,
                byte[,,] normalGrid,
                int[,,] normalVerticalSegmentationGrid,
                VerticalAdjacencyGraph verticalAdjacencyGraph,
                Dictionary<int, List<(int, int, int)>> normalVerticalSegments,
                out HashSet<int> normalUpSegmentIds,
                out HashSet<int> normalDownSegmentIds) {

            int minRoomHeight = Parameters
                .NORMAL_GRID_DETERMINATION_MIN_ROOM_HEIGHT
                .GetDistanceInVoxels(resolution);
            int borderFillRatioDeterminationHeightOffset = Parameters
                .NORMAL_GRID_DETERMINATION_BORDER_FILL_RATION_DETERMINATION_HEIGHT_OFFSET
                .GetDistanceInVoxels(resolution);
            int minFloorInitializationArea = Parameters
                .NORMAL_GRID_DETERMINATION_FLOOR_INITIALIZATION_MIN_AREA
                .GetAreaInVoxels(resolution);
            HashSet<int> floorLayerSegmentIds;
            HashSet<int> ceilingLayerSegmentIds;

            normalUpSegmentIds = new HashSet<int>();
            normalDownSegmentIds = new HashSet<int>();

            floorLayerSegmentIds = InitializeLowestFloorLayerSegmentIds(
                minFloorInitializationArea,
                verticalAdjacencyGraph);

            do {
                normalUpSegmentIds.AddRange(floorLayerSegmentIds);

                ceilingLayerSegmentIds = verticalAdjacencyGraph
                    .GetMainSegmentIdsAbove(
                        floorLayerSegmentIds,
                        minRoomHeight)
                    .ToHashSet();

                normalDownSegmentIds.AddRange(ceilingLayerSegmentIds);

                ReassessLastLayerCeilings(
                    minRoomHeight,
                    borderFillRatioDeterminationHeightOffset,
                    normalGrid,
                    normalVerticalSegmentationGrid,
                    verticalAdjacencyGraph,
                    floorLayerSegmentIds,
                    ceilingLayerSegmentIds,
                    normalUpSegmentIds,
                    normalDownSegmentIds,
                    normalVerticalSegments);

                ClassifyNormalVerticalSegmentsInbetweenCeilingAndFloor(
                    minRoomHeight,
                    borderFillRatioDeterminationHeightOffset,
                    normalGrid,
                    normalVerticalSegmentationGrid,
                    verticalAdjacencyGraph,
                    floorLayerSegmentIds,
                    ceilingLayerSegmentIds,
                    normalUpSegmentIds,
                    normalDownSegmentIds,
                    normalVerticalSegments);

                floorLayerSegmentIds = GetNextFloorLayerSegmentIds(
                    minRoomHeight,
                    verticalAdjacencyGraph,
                    ceilingLayerSegmentIds,
                    normalUpSegmentIds);

            } while (floorLayerSegmentIds.Count > 0);

            CompleteNormalVerticalSegmentClassification(
                normalGrid,
                normalVerticalSegmentationGrid,
                verticalAdjacencyGraph,
                normalUpSegmentIds,
                normalDownSegmentIds);
        }

        private static HashSet<int> InitializeLowestFloorLayerSegmentIds(
                int minFloorInitializationArea,
                VerticalAdjacencyGraph verticalAdjacencyGraph) {

            int[] falseFloorLayerSegmentIds;
            HashSet<int> floorLayerSegmentIds;

            floorLayerSegmentIds = verticalAdjacencyGraph
                .GetMainSegmentIdsAbove(LOWER_OUTSIDE_ID)
                .AsParallel()
                .Where(segmentId => verticalAdjacencyGraph.GetMaxCoverage(segmentId) >= minFloorInitializationArea)
                .Where(segmentId => !verticalAdjacencyGraph
                        .GetAbove(segmentId)
                        .Contains(UPPER_OUTSIDE_ID)
                    || verticalAdjacencyGraph.GetCoverage(
                            LOWER_OUTSIDE_ID,
                            segmentId)
                        < verticalAdjacencyGraph.GetCoverage(
                            segmentId,
                            UPPER_OUTSIDE_ID))
                .ToHashSet();

            falseFloorLayerSegmentIds = floorLayerSegmentIds
                .AsParallel()
                .Where(floorLayerSegmentId => verticalAdjacencyGraph
                    .GetBelow(floorLayerSegmentId)
                    .Where(segmentIdBelow => floorLayerSegmentIds.Contains(segmentIdBelow))
                    .Sum(segmentIdBelow => verticalAdjacencyGraph.GetCoverage(
                            segmentIdBelow,
                            floorLayerSegmentId))
                        >= verticalAdjacencyGraph.GetCoverage(
                            LOWER_OUTSIDE_ID,
                            floorLayerSegmentId))
                .ToArray();

            floorLayerSegmentIds.Remove(falseFloorLayerSegmentIds);

            return floorLayerSegmentIds;
        }

        private static void ReassessLastLayerCeilings(
                int minRoomHeight,
                int borderFillRatioDeterminationHeightOffset,
                byte[,,] normalGrid,
                int[,,] normalVerticalSegmentationGrid,
                VerticalAdjacencyGraph verticalAdjacencyGraph,
                HashSet<int> floorLayerSegmentIds,
                HashSet<int> ceilingLayerSegmentIds,
                HashSet<int> normalUpSegmentIds,
                HashSet<int> normalDownSegmentIds,
                Dictionary<int, List<(int, int, int)>> normalVerticalSegments) {

            foreach (int ceilingSegmentId in ceilingLayerSegmentIds) {

                foreach (int segmentIdBelow in verticalAdjacencyGraph.GetMainSegmentIdsBelow(ceilingSegmentId)) {

                    if (normalDownSegmentIds.Contains(segmentIdBelow)) {

                        normalDownSegmentIds.Remove(segmentIdBelow);

                        if (CouldSegmentBeNormalUp(
                                verticalAdjacencyGraph.GetDistance(
                                    segmentIdBelow,
                                    ceilingSegmentId) >= minRoomHeight,
                                borderFillRatioDeterminationHeightOffset,
                                segmentIdBelow,
                                normalGrid,
                                normalVerticalSegmentationGrid,
                                normalVerticalSegments[segmentIdBelow])) {

                            normalUpSegmentIds.Add(segmentIdBelow);
                            floorLayerSegmentIds.Add(segmentIdBelow);
                        }
                    }
                }
            }
        }

        private static bool CouldSegmentBeNormalUp(
                bool heightCondition,
                int borderFillRatioDeterminationHeightOffset,
                int segmentId,
                byte[,,] normalGrid,
                int[,,] normalVerticalSegmentationGrid,
                List<(int, int, int)> normalVerticalSegment) {

            double borderFillRatioBelow;
            double borderFillRatioAbove;

            GetBorderFillRatios(
                borderFillRatioDeterminationHeightOffset,
                segmentId,
                normalGrid,
                normalVerticalSegmentationGrid,
                normalVerticalSegment,
                out borderFillRatioBelow,
                out borderFillRatioAbove);

            if (!((borderFillRatioBelow < Parameters.NORMAL_GRID_DETERMINATION_MIN_BORDER_FILL_RATIO
                        && borderFillRatioAbove < Parameters.NORMAL_GRID_DETERMINATION_MIN_BORDER_FILL_RATIO)
                    || (borderFillRatioBelow >= Parameters.NORMAL_GRID_DETERMINATION_MIN_BORDER_FILL_RATIO
                        && borderFillRatioAbove >= Parameters.NORMAL_GRID_DETERMINATION_MIN_BORDER_FILL_RATIO))) {

                if (borderFillRatioBelow >= Parameters.NORMAL_GRID_DETERMINATION_MIN_BORDER_FILL_RATIO) {
                    return true;
                }

                return false;
            }

            return heightCondition;
        }

        private static void GetBorderFillRatios(
                int heightOffset,
                int segmentId,
                byte[,,] normalGrid,
                int[,,] normalVerticalSegmentationGrid,
                List<(int, int, int)> normalVerticalSegment,
                out double borderFillRatioBelow,
                out double borderFillRatioAbove) {

            int borderCounter = 0;
            int borderFillCounterBelow = 0;
            int borderFillCounterAbove = 0;
            List<(int, int)> borderDirections;

            foreach ((int, int, int) voxel in normalVerticalSegment) {

                if (!IsBorder(
                        segmentId,
                        voxel,
                        normalGrid,
                        normalVerticalSegmentationGrid,
                        out borderDirections)) {

                    continue;
                }

                borderCounter++;

                UpdateBorderFillCounter(
                    heightOffset,
                    -1,
                    ref borderFillCounterAbove,
                    voxel,
                    normalGrid,
                    borderDirections);

                UpdateBorderFillCounter(
                    heightOffset,
                    1,
                    ref borderFillCounterBelow,
                    voxel,
                    normalGrid,
                    borderDirections);
            }

            borderFillRatioBelow = (double)borderFillCounterBelow / borderCounter;
            borderFillRatioAbove = (double)borderFillCounterAbove / borderCounter;
        }

        private static bool IsBorder(
                int segmentId,
                (int, int, int) voxel,
                byte[,,] normalGrid,
                int[,,] normalVerticalSegmentationGrid,
                out List<(int, int)> borderDirections) {

            bool isBorder = false;
            int dr, r, dc, c;
            
            borderDirections = new List<(int, int)>();

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    r = voxel.Item2 + dr;
                    c = voxel.Item3 + dc;

                    if (r < 0 || c < 0
                            || r >= normalGrid.GetLength(1)
                            || c >= normalGrid.GetLength(2)
                            || normalVerticalSegmentationGrid[
                                voxel.Item1,
                                r,
                                c] == segmentId) {
                        continue;
                    }

                    isBorder = true;
                    borderDirections.Add((dr, dc));
                }
            }

            return isBorder;
        }

        private static void UpdateBorderFillCounter(
                int heightOffset,
                int direction,
                ref int borderFillCounter,
                (int, int, int) voxel,
                byte[,,] normalGrid,
                List<(int, int)> borderDirections) {

            int i = voxel.Item1 + direction * heightOffset;

            if (i < 0
                    || i >= normalGrid.GetLength(0)) {
                return;
            }

            if (normalGrid[
                    i,
                    voxel.Item2,
                    voxel.Item3] == NormalGridValues.NORMAL_HORIZONTAL) {

                borderFillCounter++;
                return;
            }

            foreach ((int, int) borderDirection in borderDirections) {

                if (normalGrid[
                        i,
                        voxel.Item2 + borderDirection.Item1,
                        voxel.Item3 + borderDirection.Item2] == NormalGridValues.NORMAL_HORIZONTAL) {

                    borderFillCounter++;
                    return;
                }
            }
        }

        private static void ClassifyNormalVerticalSegmentsInbetweenCeilingAndFloor(
                int minRoomHeight,
                int borderFillRatioDeterminationHeightOffset,
                byte[,,] normalGrid,
                int[,,] normalVerticalSegmentationGrid,
                VerticalAdjacencyGraph verticalAdjacencyGraph,
                HashSet<int> floorLayerSegmentIds,
                HashSet<int> ceilingLayerSegmentIds,
                HashSet<int> normalUpSegmentIds,
                HashSet<int> normalDownSegmentIds,
                Dictionary<int, List<(int, int, int)>> normalVerticalSegments) {

            foreach (int floorLayerSegmentId in floorLayerSegmentIds) {
                foreach (int segmentIdAboveFloor in verticalAdjacencyGraph.GetAbove(floorLayerSegmentId)) {
                    
                    if (normalUpSegmentIds.Contains(segmentIdAboveFloor)
                            || normalDownSegmentIds.Contains(segmentIdAboveFloor)
                            || !verticalAdjacencyGraph
                                .GetAbove(segmentIdAboveFloor)
                                .Any(upperSegmentId => ceilingLayerSegmentIds.Contains(upperSegmentId))) {
                        continue;
                    }

                    if (CouldSegmentBeNormalUp(
                            verticalAdjacencyGraph.GetDistance(
                                floorLayerSegmentId,
                                segmentIdAboveFloor) < minRoomHeight,
                            borderFillRatioDeterminationHeightOffset,
                            segmentIdAboveFloor,
                            normalGrid,
                            normalVerticalSegmentationGrid,
                            normalVerticalSegments[segmentIdAboveFloor])) {

                        normalUpSegmentIds.Add(segmentIdAboveFloor);
                    }
                    else {
                        normalDownSegmentIds.Add(segmentIdAboveFloor);
                    }
                }
            }
        }

        private static HashSet<int> GetNextFloorLayerSegmentIds(
                int minRoomHeight,
                VerticalAdjacencyGraph verticalAdjacencyGraph,
                HashSet<int> ceilingLayerSegmentIds,
                HashSet<int> normalUpSegmentIds) {

            HashSet<int> majorSegmentIdsAbove;
            HashSet<int> segmentIdsAbove;
            HashSet<int> floorLayerSegmentIds = new HashSet<int>();

            foreach (int ceilingLayerSegmentId in ceilingLayerSegmentIds) {

                segmentIdsAbove = verticalAdjacencyGraph
                    .GetMainSegmentIdsAbove(ceilingLayerSegmentId)
                    .Where(segmentIdAbove => segmentIdAbove != UPPER_OUTSIDE_ID)
                    .ToHashSet();

                majorSegmentIdsAbove = segmentIdsAbove
                    .WhereMax(segmentIdAbove => verticalAdjacencyGraph.GetCoverage(
                        ceilingLayerSegmentId,
                        segmentIdAbove))
                    .ToHashSet();

                if (majorSegmentIdsAbove.Any(majorSegmentIdAbove => verticalAdjacencyGraph.GetDistance(
                        ceilingLayerSegmentId,
                        majorSegmentIdAbove) >= minRoomHeight)) {

                    normalUpSegmentIds.Add(ceilingLayerSegmentId);
                    floorLayerSegmentIds.Add(ceilingLayerSegmentId);
                }
                else {
                    foreach (int segmentIdAbove in segmentIdsAbove) {

                        if (verticalAdjacencyGraph.GetDistance(
                                    ceilingLayerSegmentId,
                                    segmentIdAbove) < minRoomHeight
                                && verticalAdjacencyGraph
                                    .GetAbove(segmentIdAbove)
                                    .WhereMax(segmentIdAboveAbove => verticalAdjacencyGraph.GetCoverage(
                                        segmentIdAbove,
                                        segmentIdAboveAbove))
                                    .Where(majorSegmentIdAboveAbove => majorSegmentIdAboveAbove != UPPER_OUTSIDE_ID)
                                    .Any()) {

                            normalUpSegmentIds.Add(segmentIdAbove);
                            floorLayerSegmentIds.Add(segmentIdAbove);
                        }
                    }
                }
            }

            return floorLayerSegmentIds;
        }

        private static void CompleteNormalVerticalSegmentClassification(
                byte[,,] normalGrid,
                int[,,] normalVerticalSegmentationGrid,
                VerticalAdjacencyGraph verticalAdjacencyGraph,
                HashSet<int> normalUpSegmentIds,
                HashSet<int> normalDownSegmentIds) {

            bool assignToNormalUp;
            int i, r, c;
            int segmentId;
            int normalUpContactCount;
            int normalDownContactCount;
            int normalHorizontalContactCount;
            bool[,,] isSegmented = new bool[
                normalVerticalSegmentationGrid.GetLength(0),
                normalVerticalSegmentationGrid.GetLength(1),
                normalVerticalSegmentationGrid.GetLength(2)];
            HashSet<int> ceilingCandidateSegmentIds;

            ceilingCandidateSegmentIds = verticalAdjacencyGraph
                .GetMainSegmentIdsBelow(UPPER_OUTSIDE_ID)
                .AsParallel()
                .Where(ceilingCandidateSegmentId => !verticalAdjacencyGraph
                    .GetBelow(ceilingCandidateSegmentId)
                    .WhereMax(segmentIdBelow => verticalAdjacencyGraph.GetCoverage(
                        segmentIdBelow,
                        ceilingCandidateSegmentId))
                    .Where(maxCoverageSegmentIdBelow => maxCoverageSegmentIdBelow == LOWER_OUTSIDE_ID)
                    .Any())
                .ToHashSet();


            for (i = 0; i < normalVerticalSegmentationGrid.GetLength(0); i++) {
                for (r = 0; r < normalVerticalSegmentationGrid.GetLength(1); r++) {
                    for (c = 0; c < normalVerticalSegmentationGrid.GetLength(2); c++) {

                        segmentId = normalVerticalSegmentationGrid[i, r, c];
                        if (segmentId == 0
                                || normalUpSegmentIds.Contains(segmentId)
                                || normalDownSegmentIds.Contains(segmentId)) {
                            continue;
                        }

                        if (ceilingCandidateSegmentIds.Contains(segmentId)) {

                            normalDownSegmentIds.Add(segmentId);
                            continue;
                        }

                        CountContacts(
                            segmentId,
                            (i, r, c),
                            isSegmented,
                            normalGrid,
                            normalVerticalSegmentationGrid,
                            normalUpSegmentIds,
                            normalDownSegmentIds,
                            out normalUpContactCount,
                            out normalDownContactCount,
                            out normalHorizontalContactCount);

                        if ((normalUpContactCount == 0 
                                    && normalDownContactCount == 0)
                                || (normalHorizontalContactCount > normalUpContactCount
                                    && normalHorizontalContactCount > normalDownContactCount)) {
                            continue;
                        }
                        else if(normalUpContactCount >= normalDownContactCount) {
                            assignToNormalUp = true;
                        }
                        else {
                            assignToNormalUp = false;
                        }

                        (assignToNormalUp ?
                                normalUpSegmentIds :
                                normalDownSegmentIds)
                            .Add(segmentId);
                    }
                }
            }
        }

        private static void CountContacts(
                int segmentId,
                (int, int, int) voxel,
                bool[,,] isSegmented,
                byte[,,] normalGrid,
                int[,,] normalVerticalSegmentationGrid,
                HashSet<int> normalUpSegmentIds,
                HashSet<int> normalDownSegmentIds,
                out int normalUpContactCount,
                out int normalDownContactCount,
                out int normalHorizontalContactCount) {

            int dr, r, dc, c, di, i;
            int candidateSegmentId;
            Queue<(int, int, int)> candidates = new Queue<(int, int, int)>();

            normalUpContactCount = 0;
            normalDownContactCount = 0;
            normalHorizontalContactCount = 0;

            candidates.Enqueue(voxel);

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

                for (di = -1; di <= 1; di++) {
                    for (dr = -1; dr <= 1; dr++) {
                        for (dc = -1; dc <= 1; dc++) {

                            if (di.Abs() + dr.Abs() + dc.Abs() != 1) {
                                continue;
                            }

                            i = candidate.Item1 + di;
                            r = candidate.Item2 + dr;
                            c = candidate.Item3 + dc;

                            if (i < 0 || r < 0 || c < 0
                                    || i >= normalVerticalSegmentationGrid.GetLength(0)
                                    || r >= normalVerticalSegmentationGrid.GetLength(1)
                                    || c >= normalVerticalSegmentationGrid.GetLength(2)) {
                                continue;
                            }

                            if (normalGrid[i, r, c] == NormalGridValues.NORMAL_HORIZONTAL) {
                                normalHorizontalContactCount++;
                                continue;
                            }

                            candidateSegmentId = normalVerticalSegmentationGrid[i, r, c];

                            if (candidateSegmentId != segmentId) {
                                if (candidateSegmentId != 0) {

                                    if (normalUpSegmentIds.Contains(candidateSegmentId)) {
                                        normalUpContactCount++;
                                    }
                                    if (normalDownSegmentIds.Contains(candidateSegmentId)) {
                                        normalDownContactCount++;
                                    }
                                }
                            }
                            else if (!isSegmented[i, r, c]) {
                                candidates.Enqueue((i, r, c));
                            }
                        }
                    }
                }

            } while (candidates.Count > 0);
        }

        private static void UpdateNormalGrid(
            byte[,,] normalGrid,
            int[,,] normalVerticalSegmentationGrid,
            HashSet<int> normalUpSegmentIds,
            HashSet<int> normalDownSegmentIds) {

            Parallel.For(
                0,
                normalGrid.GetLength(0),
                i => {

                    int r, c;
                    int segmentId;

                    for (r = 0; r < normalGrid.GetLength(1); r++) {
                        for (c = 0; c < normalGrid.GetLength(2); c++) {

                            segmentId = normalVerticalSegmentationGrid[i, r, c];
                            if (segmentId == 0) {
                                continue;
                            }

                            if (normalUpSegmentIds.Contains(segmentId)
                                    && normalDownSegmentIds.Contains(segmentId)) {

                               normalGrid[i, r, c] = NormalGridValues.NORMAL_UP_AND_DOWN;
                            }
                            else if (normalUpSegmentIds.Contains(segmentId)) {
                                normalGrid[i, r, c] = NormalGridValues.NORMAL_UP;
                            }
                            else if (normalDownSegmentIds.Contains(segmentId)) {
                                normalGrid[i, r, c] = NormalGridValues.NORMAL_DOWN;
                            }
                            else {
                                normalGrid[i, r, c] = NormalGridValues.NORMAL_HORIZONTAL;
                            }
                        }
                    }

                });
        }

        private static void RefineNormalGrid(
                byte[,,] normalGrid) {

            SwitchNormalVerticalPillarsToNormalHorizontal(normalGrid);
            RemoveIsolatedNormalUpVoxels(normalGrid);
        }

        private static void SwitchNormalVerticalPillarsToNormalHorizontal(
                byte[,,] normalGrid) {

            Parallel.For(
                0,
                normalGrid.GetLength(2),
                c => {

                    byte normalGridValue;
                    int i, i2, i3, r;

                    for (r = 0; r < normalGrid.GetLength(1); r++) {
                        for (i = 0; i < normalGrid.GetLength(0); i++) {

                            normalGridValue = normalGrid[i, r, c];

                            if (normalGridValue == NormalGridValues.EMPTY
                                    || normalGridValue == NormalGridValues.NORMAL_HORIZONTAL) {

                                continue;
                            }

                            i2 = i;
                            do {
                                i2++;
                            } while (i2 < normalGrid.GetLength(0)
                                && normalGrid[i2, r, c] == normalGridValue);

                            i2--;

                            if (i2 - i <= (normalGridValue == NormalGridValues.NORMAL_DOWN ? 1 : 0)) {
                                continue;
                            }

                            for (i3 = i + 1; i3 <= i2; i3++) {

                                if (!IsPartOfHorizontalPlane(
                                        i3,
                                        r,
                                        c,
                                        normalGrid)) {

                                    normalGrid[i3, r, c] = NormalGridValues.NORMAL_HORIZONTAL;
                                }
                            }

                            if (normalGridValue == NormalGridValues.NORMAL_UP) {

                                if (!IsPartOfHorizontalPlane(
                                        i2,
                                        r,
                                        c,
                                        normalGrid)) {

                                    normalGrid[i2, r, c] = NormalGridValues.NORMAL_HORIZONTAL;
                                }
                            }
                            else if (normalGridValue == NormalGridValues.NORMAL_DOWN) {

                                if (!IsPartOfHorizontalPlane(
                                        i,
                                        r,
                                        c,
                                        normalGrid)) {

                                    normalGrid[i, r, c] = NormalGridValues.NORMAL_HORIZONTAL;
                                }
                            }
                            else { // NORMAL_UP_AND_DOWN
                                normalGrid[i, r, c] = NormalGridValues.NORMAL_UP;
                                normalGrid[i2, r, c] = NormalGridValues.NORMAL_DOWN;
                            }
                        }
                    }
                });
        }

        private static bool IsPartOfHorizontalPlane(
                int i,
                int r,
                int c,
                byte[,,] normalGrid) {

            int dr, r2, dc, c2;

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    if (dr.Abs() + dc.Abs() != 1) {
                        continue;
                    }

                    r2 = r + dr;
                    c2 = c + dc;

                    if (r2 < 0 || c2 < 0
                            || r2 >= normalGrid.GetLength(1)
                            || c2 >= normalGrid.GetLength(2)
                            || normalGrid[i, r2, c2] == NormalGridValues.EMPTY) {

                        return false;
                    }
                }
            }

            return true;
        }

        private static void RemoveIsolatedNormalUpVoxels(
                byte[,,] normalGrid) {

            Parallel.For(
                0,
                normalGrid.GetLength(0),
                i => {

                    bool hasNeighbours;
                    int dr, r, r2, dc, c, c2;

                    for (r = 0; r < normalGrid.GetLength(1); r++) {
                        for (c = 0; c < normalGrid.GetLength(2); c++) {

                            if (normalGrid[i, r, c] != NormalGridValues.NORMAL_UP) {
                                continue;
                            }

                            hasNeighbours = false;

                            for (dr = -1; dr <= 1; dr++) {
                                for (dc = -1; dc <= 1; dc++) {

                                    if (dr.Abs() + dc.Abs() != 1) {
                                        continue;
                                    }

                                    r2 = r + dr;
                                    c2 = c + dc;

                                    if (r2 >= 0 && c2 >= 0
                                            && r2 < normalGrid.GetLength(1)
                                            && c2 < normalGrid.GetLength(2)
                                            && normalGrid[i, r2, c2] == NormalGridValues.NORMAL_UP) {

                                        hasNeighbours = true;
                                        break;
                                    }
                                }

                                if (hasNeighbours) {
                                    break;
                                }
                            }

                            if (!hasNeighbours) {
                                normalGrid[i, r, c] = NormalGridValues.NORMAL_HORIZONTAL;
                            }
                        }
                    }
                });
        }
    }
}
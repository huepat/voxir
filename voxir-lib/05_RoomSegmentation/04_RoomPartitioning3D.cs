using HuePat.VoxIR.Util.Grid;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace HuePat.VoxIR.RoomSegmentation {
    public static class RoomPartitioning3D {
        public static bool[,,] InitializeInteriorSpaceGrid(
                int[,,][] reconstructionGrid) {

            bool[,,] interiorSpaceGrid = new bool[
                reconstructionGrid.GetLength(0),
                reconstructionGrid.GetLength(1),
                reconstructionGrid.GetLength(2)];

            Parallel.For(
                0,
                reconstructionGrid.GetLength(0),
                i => InitializeInteriorSpaceGridSection(
                    i,
                    interiorSpaceGrid,
                    reconstructionGrid)); ;

            return interiorSpaceGrid;
        }

        public static int[,,] RepartitionInteriorSpace(
                double resolution,
                bool[,,] wallGrid,
                bool[,,] rampSpaceGrid,
                bool[,,] interiorSpaceGrid,
                int[,,][] reconstructionGrid,
                out int transitionSpaceCount,
                out HashSet<int> rampSpaceIds) {

            bool[,,] roomSpaceGrid;
            bool[,,] transitionSpaceGrid;
            bool[,,] bottleNeckSpaceGrid;

            RepartitionInteriorSpaceInHorizontal2DSections(
                resolution,
                wallGrid,
                interiorSpaceGrid,
                out roomSpaceGrid,
                out transitionSpaceGrid,
                out bottleNeckSpaceGrid);

            Aggregate2DSpacePartitioningSlicesVertically(
                resolution,
                interiorSpaceGrid,
                roomSpaceGrid,
                transitionSpaceGrid,
                bottleNeckSpaceGrid);

            RefineSpacePartitioningByConsideringRampSpaces(
                resolution,
                interiorSpaceGrid,
                wallGrid,
                rampSpaceGrid,
                roomSpaceGrid,
                transitionSpaceGrid,
                reconstructionGrid);

            return SegmentSpacePartitioningGrid(
                roomSpaceGrid,
                transitionSpaceGrid,
                rampSpaceGrid,
                out transitionSpaceCount,
                out rampSpaceIds);
        }

        public static void RefineSpacePartitioning(
                int transitionSpaceCount,
                double resolution,
                bool[,,] wallGrid,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                HashSet<int> rampSpaceIds) {

            bool hasGrown;
            bool hasNewTransitionSpaces = false;
            bool[,,] roomInteriorContactSurfaceGrid;
            HashSet<int> rampTransitionSpaceIds;
            HashSet<int> unmergeableRampTransitionSpaceIds;
            Dictionary<int, int> spacePartitioningSegmentAreas;
            Dictionary<int, int> rampTransitionSpaceAreas;
            Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency;

            GrowSpacePartitioningSegmentsOverUnpartitionedInteriorVoxels(
                wallGrid,
                normalGrid,
                spacePartitioningGrid,
                reconstructionGrid,
                out roomInteriorContactSurfaceGrid);

            GetSpacePartitioningSegmentAreasAndAdjacency(
                spacePartitioningGrid,
                reconstructionGrid,
                out spacePartitioningSegmentAreas,
                out spacePartitioningSegmentAdjacency);

            MergeSpacePartitioningSegments(
                transitionSpaceCount,
                resolution,
                normalGrid,
                spacePartitioningGrid,
                reconstructionGrid,
                spacePartitioningSegmentAreas,
                spacePartitioningSegmentAdjacency,
                out HashSet<int> newTransitionSpaceIds);

            transitionSpaceCount += newTransitionSpaceIds.Count;

            RemoveDeadEndTransitionSpaces(
                spacePartitioningGrid,
                null,
                spacePartitioningSegmentAreas,
                spacePartitioningSegmentAdjacency);

            GrowRampSpacesOverAdjacentTransitionSpacesFromWithin(
                spacePartitioningGrid,
                rampSpaceIds,
                spacePartitioningSegmentAdjacency);

            FindMissingTransitionSpaces(
                ref transitionSpaceCount,
                resolution,
                roomInteriorContactSurfaceGrid,
                normalGrid,
                spacePartitioningGrid,
                reconstructionGrid,
                spacePartitioningSegmentAreas,
                spacePartitioningSegmentAdjacency);

            RemoveDeadEndTransitionSpacesAndSmallDeadEndRooms(
                resolution,
                normalGrid,
                spacePartitioningGrid,
                reconstructionGrid,
                spacePartitioningSegmentAreas,
                spacePartitioningSegmentAdjacency);

            RefineMultiRoomTransitionSpaces(
                ref transitionSpaceCount,
                normalGrid,
                spacePartitioningGrid,
                reconstructionGrid,
                spacePartitioningSegmentAreas,
                spacePartitioningSegmentAdjacency);

            rampTransitionSpaceIds = GetRampTransitionSpaceIds(
                rampSpaceIds,
                newTransitionSpaceIds,
                spacePartitioningSegmentAdjacency);

            RefineTransitionSpaces(
                true,
                resolution,
                normalGrid,
                spacePartitioningGrid,
                reconstructionGrid,
                rampTransitionSpaceIds,
                spacePartitioningSegmentAreas,
                spacePartitioningSegmentAdjacency,
                out unmergeableRampTransitionSpaceIds,
                out rampTransitionSpaceAreas);

            MergeSmallRampSpaceNeighboursToRampSpace(
                resolution,
                spacePartitioningGrid,
                rampSpaceIds,
                rampTransitionSpaceIds,
                unmergeableRampTransitionSpaceIds,
                spacePartitioningSegmentAreas,
                rampTransitionSpaceAreas,
                spacePartitioningSegmentAdjacency);

            RefineRoomsVertically(
                resolution,
                normalGrid,
                spacePartitioningGrid,
                reconstructionGrid,
                rampSpaceIds);

            hasGrown = GrowSpacePartitioningSegmentsOverUnpartitionedInteriorVoxels(
                wallGrid,
                normalGrid,
                spacePartitioningGrid,
                reconstructionGrid,
                out _);

            if (hasGrown) {
                hasNewTransitionSpaces = FindMissingTransitionSpaces(
                    ref transitionSpaceCount,
                    resolution,
                    roomInteriorContactSurfaceGrid,
                    normalGrid,
                    spacePartitioningGrid,
                    reconstructionGrid,
                    spacePartitioningSegmentAreas,
                    spacePartitioningSegmentAdjacency);
            }

            if (RefineRooms(
                    resolution,
                    normalGrid,
                    spacePartitioningGrid,
                    reconstructionGrid,
                    rampSpaceIds,
                    spacePartitioningSegmentAreas,
                    spacePartitioningSegmentAdjacency)) {
                hasNewTransitionSpaces = true;
            }

            if (RefineMultiRoomTransitionSpaces(
                    ref transitionSpaceCount,
                    normalGrid,
                    spacePartitioningGrid,
                    reconstructionGrid,
                    spacePartitioningSegmentAreas,
                    spacePartitioningSegmentAdjacency)) {
                hasNewTransitionSpaces = true;
            }

            if (!hasNewTransitionSpaces) {
                return;
            }

            MergeAdjacentTransitionSpaces(
                ref transitionSpaceCount,
                spacePartitioningGrid,
                rampTransitionSpaceIds,
                spacePartitioningSegmentAreas,
                spacePartitioningSegmentAdjacency);

            RefineTransitionSpaces(
                false,
                resolution,
                normalGrid,
                spacePartitioningGrid,
                reconstructionGrid,
                rampTransitionSpaceIds,
                spacePartitioningSegmentAreas,
                spacePartitioningSegmentAdjacency,
                out _,
                out _);

            AssignHorizontalTransitionSpaceSectionSegmentsToSingleAdjacentRoom(
                normalGrid,
                spacePartitioningGrid,
                reconstructionGrid);

            AssignVerticallyAdjacentWallOpeningVoxelsToTransitionSpaces(
                normalGrid,
                spacePartitioningGrid,
                reconstructionGrid);
        }

        public static void ApplySpacePartitioningToReconstructionGrid(
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid) {

            Parallel.For(
                0,
                reconstructionGrid.GetLength(2),
                c => {

                    ApplySpacePartitioningSegmentIdsToInteriorSpace(
                        c,
                        spacePartitioningGrid,
                        reconstructionGrid);

                    TransferSpacePartitioningFromInteriorToCeilingsAndFloors(
                        c,
                        reconstructionGrid);
                });

            Parallel.For(
                0,
                reconstructionGrid.GetLength(0),
                i => {

                    TransferSpacePartitioningToWalls(
                        i,
                        reconstructionGrid);

                    RemoveRoomlessVoxelsExceptForCeilings(
                        i,
                        reconstructionGrid);
                });

            Parallel.For(
                0,
                reconstructionGrid.GetLength(2),
                c => {

                    ResolveRoomlessCeilingVoxels(
                        c,
                        reconstructionGrid);

                    RemoveFalseWallLabelsFromFloorsAndCeilings(
                        c,
                        reconstructionGrid);
                });
        }

        private static void InitializeInteriorSpaceGridSection(
                int i,
                bool[,,] interiorSpaceGrid,
                int[,,][] reconstructionGrid) {

            int r, c;
            int[] voxelState;
            int[] voxelClassValues;

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                    voxelState = reconstructionGrid[i, r, c];
                    if (voxelState == null) {
                        continue;
                    }

                    voxelClassValues = voxelState.GetVoxelClassValues(0);
                    if (voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                            || voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                        interiorSpaceGrid[i, r, c] = true;
                    }
                }
            }
        }

        private static void RepartitionInteriorSpaceInHorizontal2DSections(
                double resolution,
                bool[,,] wallGrid,
                bool[,,] interiorSpaceGrid,
                out bool[,,] roomSpaceGrid,
                out bool[,,] transitionSpaceGrid,
                out bool[,,] bottleNeckSpaceGrid) {

            bool[,,] _roomSpaceGrid = new bool[
                interiorSpaceGrid.GetLength(0),
                interiorSpaceGrid.GetLength(1),
                interiorSpaceGrid.GetLength(2)];
            bool[,,] _transitionSpaceGrid = new bool[
                interiorSpaceGrid.GetLength(0),
                interiorSpaceGrid.GetLength(1),
                interiorSpaceGrid.GetLength(2)];
            bool[,,] _bottleNeckSpaceGrid = new bool[
                interiorSpaceGrid.GetLength(0),
                interiorSpaceGrid.GetLength(1),
                interiorSpaceGrid.GetLength(2)];

            Parallel.For(
                0,
                interiorSpaceGrid.GetLength(0),
                i => {

                    bool[,] interiorSpaceGridSection;
                    bool[,] bottleNeckSpaceGridSection;
                    int[,] spacePartitioningGrid2D;

                    interiorSpaceGridSection = GetInteriorSpaceGridSection(
                        i,
                        interiorSpaceGrid);

                    RoomPartitioning2D
                        .RepartitionInteriorSpace(
                            resolution,
                            interiorSpaceGridSection,
                            out bottleNeckSpaceGridSection,
                            out spacePartitioningGrid2D);

                    SplitSpacePartitioningGrid2D(
                        i,
                        bottleNeckSpaceGridSection,
                        spacePartitioningGrid2D,
                        wallGrid,
                        interiorSpaceGrid,
                        _roomSpaceGrid,
                        _transitionSpaceGrid,
                        _bottleNeckSpaceGrid);
                });

            roomSpaceGrid = _roomSpaceGrid;
            transitionSpaceGrid = _transitionSpaceGrid;
            bottleNeckSpaceGrid = _bottleNeckSpaceGrid;
        }

        private static bool[,] GetInteriorSpaceGridSection(
                int i,
                bool[,,] interiorSpaceGrid) {

            int r, c;
            bool[,] interiorSpaceGridSection = new bool[
                interiorSpaceGrid.GetLength(1),
                interiorSpaceGrid.GetLength(2)];

            for (r = 0; r < interiorSpaceGrid.GetLength(1); r++) {
                for (c = 0; c < interiorSpaceGrid.GetLength(2); c++) {

                    if (interiorSpaceGrid[i, r, c]) {
                        interiorSpaceGridSection[r, c] = true;
                    }
                }
            }

            return interiorSpaceGridSection;
        }

        private static void SplitSpacePartitioningGrid2D(
                int i,
                bool[,] bottleNeckSpaceGridSection,
                int[,] spacePartitioningGrid2D,
                bool[,,] wallGrid,
                bool[,,] interiorSpaceGrid,
                bool[,,] roomSpaceGrid,
                bool[,,] transitionSpaceGrid,
                bool[,,] bottleNeckSpaceGrid) {

            int r, c;

            for (r = 0; r < spacePartitioningGrid2D.GetLength(0); r++) {
                for (c = 0; c < spacePartitioningGrid2D.GetLength(1); c++) {

                    bottleNeckSpaceGrid[i, r, c] = bottleNeckSpaceGridSection[r, c];

                    if (spacePartitioningGrid2D[r, c] > 0) {
                        roomSpaceGrid[i, r, c] = true;
                    }
                    else if (spacePartitioningGrid2D[r, c] < 0) {
                        transitionSpaceGrid[i, r, c] = true;
                    }

                    if (interiorSpaceGrid[i, r, c]
                            && wallGrid[i, r, c]) {

                        roomSpaceGrid[i, r, c] = false;
                        transitionSpaceGrid[i, r, c] = true;
                    }
                }
            }
        }

        private static void Aggregate2DSpacePartitioningSlicesVertically(
                double resolution,
                bool[,,] interiorSpaceGrid,
                bool[,,] roomSpaceGrid,
                bool[,,] transitionSpaceGrid,
                bool[,,] bottleNeckSpaceGrid) {

            int roomSpaceMinHeight = (int)(Parameters.ROOM_SPACE_MIN_HEIGHT / resolution).Round();
            int transitionSpaceMinHeight = (int)(Parameters.TRANSITION_SPACE_MIN_HEIGHT / resolution).Round();
            bool[,,] isProcessed = new bool[
                interiorSpaceGrid.GetLength(0),
                interiorSpaceGrid.GetLength(1),
                interiorSpaceGrid.GetLength(2)];

            Parallel.For(
                0,
                roomSpaceGrid.GetLength(2),
                c => {

                    AggregateTransitionSpaceVerticallyByNumberOfOccurence(
                        c,
                        transitionSpaceMinHeight,
                        isProcessed,
                        interiorSpaceGrid,
                        roomSpaceGrid,
                        transitionSpaceGrid,
                        bottleNeckSpaceGrid);

                    AggregateVerticallyByNumberOfUninterruptedOccurence(
                        c,
                        roomSpaceMinHeight,
                        roomSpaceGrid);

                    AggregateVerticallyByNumberOfUninterruptedOccurence(
                        c,
                        transitionSpaceMinHeight,
                        transitionSpaceGrid);
                });
        }

        private static void AggregateTransitionSpaceVerticallyByNumberOfOccurence(
                int c,
                int transitionSpaceMinHeight,
                bool[,,] isProcessed,
                bool[,,] interiorSpaceGrid,
                bool[,,] roomSpaceGrid,
                bool[,,] transitionSpaceGrid,
                bool[,,] bottleNeckSpaceGrid) {

            int transitionSpaceVoxelCount;
            int i, i2, i3, r;

            for (i = 0; i < roomSpaceGrid.GetLength(0); i++) {
                for (r = 0; r < roomSpaceGrid.GetLength(1); r++) {

                    if (!interiorSpaceGrid[i, r, c]
                            || isProcessed[i, r, c]) {
                        continue;
                    }

                    i2 = GetInteriorSpaceLowerHeightBound(
                        i,
                        r,
                        c,
                        interiorSpaceGrid);

                    if (i2 - i < transitionSpaceMinHeight) {
                        for (i3 = i; i3 < i2; i3++) {
                            isProcessed[i3, r, c] = true;
                        }
                    }
                    else {

                        transitionSpaceVoxelCount = GetTransitionSpaceVoxelCount(
                            i,
                            i2,
                            r,
                            c,
                            transitionSpaceGrid);

                        if (transitionSpaceVoxelCount < transitionSpaceMinHeight) {
                            for (i3 = i; i3 < i2; i3++) {
                                isProcessed[i3, r, c] = true;
                                transitionSpaceGrid[i3, r, c] = false;
                            }
                        }
                        else {
                            for (i3 = i; i3 < i2; i3++) {
                                roomSpaceGrid[i3, r, c] = false;
                                isProcessed[i3, r, c] = true;
                                if (!bottleNeckSpaceGrid[i3, r, c]) {
                                    transitionSpaceGrid[i3, r, c] = true;
                                }
                            }
                        }
                    }
                }
            }
        }

        private static int GetInteriorSpaceLowerHeightBound(
                int i,
                int r,
                int c,
                bool[,,] interiorSpaceGrid) {

            int i2 = i;

            do {
                i2++;
            } while (
                i2 < interiorSpaceGrid.GetLength(0)
                    && interiorSpaceGrid[i2, r, c]);

            return i2;
        }

        private static int GetTransitionSpaceVoxelCount(
                int startI,
                int stopI,
                int r,
                int c,
                bool[,,] transitionSpaceGrid) {

            int i;
            int transitionSpaceVoxelCount = 0;

            for (i = startI; i < stopI; i++) {
                if (transitionSpaceGrid[i, r, c]) {
                    transitionSpaceVoxelCount++;
                }
            }

            return transitionSpaceVoxelCount;
        }

        private static void AggregateVerticallyByNumberOfUninterruptedOccurence(
                int c,
                int minColumnHeight,
                bool[,,] targetGrid) {

            bool isColumn = false;
            int i, i2, r;
            int columnStartIndex = -1;

            for (r = 0; r < targetGrid.GetLength(1); r++) {
                for (i = 0; i < targetGrid.GetLength(0); i++) {

                    if (targetGrid[i, r, c]) {
                        if (!isColumn) {
                            isColumn = true;
                            columnStartIndex = i;
                        }
                        continue;
                    }

                    if (isColumn) {
                        isColumn = false;
                        if (i - columnStartIndex < minColumnHeight) {
                            for (i2 = i - 1; i2 >= columnStartIndex; i2--) {
                                targetGrid[i2, r, c] = false;
                            }
                        }
                    }
                }
                if (isColumn) {

                    isColumn = false;
                    if (i - columnStartIndex < minColumnHeight) {
                        for (i2 = i - 1; i2 >= columnStartIndex; i2--) {
                            targetGrid[i2, r, c] = false;
                        }
                    }
                }
            }
        }

        private static void RefineSpacePartitioningByConsideringRampSpaces(
                double resolution,
                bool[,,] interiorSpaceGrid,
                bool[,,] wallGrid,
                bool[,,] rampSpaceGrid,
                bool[,,] roomSpaceGrid,
                bool[,,] transitionSpaceGrid,
                int[,,][] reconstructionGrid) {

            if (!EnforceRampSpaceBeingRoomSpace(
                    interiorSpaceGrid,
                    rampSpaceGrid,
                    roomSpaceGrid,
                    transitionSpaceGrid)) {
                return;
            }

            RefineRampSpaceNeighbouringRooms(
                resolution,
                interiorSpaceGrid,
                wallGrid,
                rampSpaceGrid,
                roomSpaceGrid,
                transitionSpaceGrid,
                reconstructionGrid);

            CreateTransitionSpacesBetweenRampSpacesAndNeighbouringRooms(
                interiorSpaceGrid,
                rampSpaceGrid,
                roomSpaceGrid,
                transitionSpaceGrid,
                reconstructionGrid);
        }

        private static bool EnforceRampSpaceBeingRoomSpace(
                bool[,,] interiorSpaceGrid,
                bool[,,] rampSpaceGrid,
                bool[,,] roomSpaceGrid,
                bool[,,] transitionSpaceGrid) {

            bool found = false;

            Parallel.For(
                0,
                interiorSpaceGrid.GetLength(0),
                i => {

                    int r, c;

                    for (r = 0; r < interiorSpaceGrid.GetLength(1); r++) {
                        for (c = 0; c < interiorSpaceGrid.GetLength(2); c++) {
                            if (rampSpaceGrid[i, r, c]) {
                                found = true;
                                roomSpaceGrid[i, r, c] = true;
                                transitionSpaceGrid[i, r, c] = false;
                            }
                        }
                    }
                });

            return found;
        }

        private static void RefineRampSpaceNeighbouringRooms(
                double resolution,
                bool[,,] interiorSpaceGrid,
                bool[,,] wallGrid,
                bool[,,] rampSpaceGrid,
                bool[,,] roomSpaceGrid,
                bool[,,] transitionSpaceGrid,
                int[,,][] reconstructionGrid) {

            int i, r, c;
            int rampSpaceNeighbourMinArea = Parameters
                .AUTONOMOUS_RAMP_SPACE_NEIGHBOUR_ROOM_MIN_AREA
                .GetAreaInVoxels(resolution);
            bool[,,] isSegmented = new bool[
                interiorSpaceGrid.GetLength(0),
                interiorSpaceGrid.GetLength(1),
                interiorSpaceGrid.GetLength(2)];

            for (i = 1; i < interiorSpaceGrid.GetLength(0) - 1; i++) {
                for (r = 0; r < interiorSpaceGrid.GetLength(1); r++) {
                    for (c = 0; c < interiorSpaceGrid.GetLength(2); c++) {

                        if (!IsRampFloor(
                                i,
                                r,
                                c,
                                rampSpaceGrid,
                                reconstructionGrid)) {
                            continue;
                        }

                        foreach (List<(int, int, int)> rampSpaceNeighbourFloorSegment
                                in GetRampSpaceNeighbourFloorSegments(
                                    i,
                                    r,
                                    c,
                                    isSegmented,
                                    interiorSpaceGrid,
                                    wallGrid,
                                    rampSpaceGrid,
                                    roomSpaceGrid,
                                    transitionSpaceGrid,
                                    reconstructionGrid)) {

                            if (rampSpaceNeighbourFloorSegment.Count < rampSpaceNeighbourMinArea) {
                                MergeRampSpaceNeighbourRoomToRampSpace(
                                    rampSpaceGrid,
                                    reconstructionGrid,
                                    rampSpaceNeighbourFloorSegment);
                            }
                            else {
                                RefineRampSpaceNeighbourRoom(
                                    isSegmented,
                                    rampSpaceGrid,
                                    reconstructionGrid,
                                    rampSpaceNeighbourFloorSegment);
                            }
                        }
                    }
                }
            }
        }

        private static bool IsRampFloor(
                int i,
                int r,
                int c,
                bool[,,] rampSpaceGrid,
                int[,,][] reconstructionGrid) {

            int i2;
            int[] voxelState;

            if (!rampSpaceGrid[i, r, c]) {
                return false;
            }

            i2 = i + 1;
            if (rampSpaceGrid[i2, r, c]) {
                return false;
            }

            voxelState = reconstructionGrid[i2, r, c];
            if (voxelState == null) {
                return false;
            }

            return voxelState
                .GetVoxelClassValues(0)
                .Contains(VoxelClassValues.FLOOR);
        }

        private static IEnumerable<List<(int, int, int)>> GetRampSpaceNeighbourFloorSegments(
                int i,
                int r,
                int c,
                bool[,,] isSegmented,
                bool[,,] interiorSpaceGrid,
                bool[,,] wallGrid,
                bool[,,] rampSpaceGrid,
                bool[,,] roomSpaceGrid,
                bool[,,] transitionSpaceGrid,
                int[,,][] reconstructionGrid) {

            int di, i2, i3, i4, dr, dr2, r2, r3, dc, dc2, c2, c3;
            int[] voxelState;
            List<(int, int, int)> rampSpaceNeighbourFloorSegment;
            Queue<(int, int, int)> candidates = new Queue<(int, int, int)>();

            i2 = i + 1;

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    if (dr.Abs() == dc.Abs()) {
                        continue;
                    }

                    r2 = r + dr;
                    c2 = c + dc;
                    if (r2 < 0 || c2 < 0
                            || r2 >= interiorSpaceGrid.GetLength(1)
                            || c2 >= interiorSpaceGrid.GetLength(2)
                            || isSegmented[i2, r2, c2]
                            || rampSpaceGrid[i, r2, c2]
                            || !roomSpaceGrid[i, r2, c2]
                            || !IsFloor(
                                    i2,
                                    r2,
                                    c2,
                                    reconstructionGrid)) {
                        continue;
                    }

                    rampSpaceNeighbourFloorSegment = new List<(int, int, int)>();
                    candidates.Enqueue((i2, r2, c2));
                    do {

                        (int, int, int) candidate = candidates.Dequeue();
                        if (isSegmented[
                                candidate.Item1,
                                candidate.Item2,
                                candidate.Item3]) {
                            continue;
                        }

                        rampSpaceNeighbourFloorSegment.Add(candidate);
                        isSegmented[
                            candidate.Item1,
                            candidate.Item2,
                            candidate.Item3] = true;

                        for (di = -1; di <= 1; di++) {
                            for (dr2 = -1; dr2 <= 1; dr2++) {
                                for (dc2 = -1; dc2 <= 1; dc2++) {

                                    if (dr2.Abs() + dc2.Abs() != 1) {
                                        continue;
                                    }

                                    i3 = candidate.Item1 + di;
                                    r3 = candidate.Item2 + dr2;
                                    c3 = candidate.Item3 + dc2;
                                    if (i3 < 0 || r3 < 0 || c3 < 0
                                            || i3 >= reconstructionGrid.GetLength(0)
                                            || r3 >= reconstructionGrid.GetLength(1)
                                            || c3 >= reconstructionGrid.GetLength(2)
                                            || isSegmented[i3, r3, c3]) {
                                        continue;
                                    }

                                    i4 = i3 - 1;
                                    if (i4 < 0
                                            || rampSpaceGrid[i4, r3, c3]
                                            || transitionSpaceGrid[i4, r3, c3]
                                            || wallGrid[i4, r3, c3]
                                            || !IsProperRoomFloor(
                                                i4,
                                                r3,
                                                c3,
                                                roomSpaceGrid,
                                                reconstructionGrid)) {
                                        continue;
                                    }

                                    voxelState = reconstructionGrid[i3, r3, c3];
                                    if (voxelState == null) {
                                        continue;
                                    }

                                    if (voxelState
                                            .GetVoxelClassValues(0)
                                            .Contains(VoxelClassValues.FLOOR)) {
                                        candidates.Enqueue((i3, r3, c3));
                                    }
                                }
                            }
                        }
                    } while (candidates.Count > 0);

                    yield return rampSpaceNeighbourFloorSegment;
                }
            }
        }

        private static bool IsFloor(
                int i,
                int r,
                int c,
                int[,,][] reconstructionGrid) {

            int[] voxelState = reconstructionGrid[i, r, c];
            if (voxelState == null) {
                return false;
            }

            return voxelState
                .GetVoxelClassValues(0)
                .Contains(VoxelClassValues.FLOOR);
        }

        private static bool IsProperRoomFloor(
                int i, 
                int r,
                int c,
                bool[,,] roomSpaceGrid,
                int[,,][] reconstructionGrid) {

            int[] voxelState;
            int[] voxelClassValues;

            while (true) {

                if (i < 0) {
                    break;
                }

                if (roomSpaceGrid[i, r, c]) {
                    return true;
                }

                voxelState = reconstructionGrid[i, r, c];
                if (voxelState == null) {
                    break;
                }

                voxelClassValues = voxelState.GetVoxelClassValues(0);
                if (!voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                        && !voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                    break;
                }

                i--;
            }
            return false;
        }

        private static void MergeRampSpaceNeighbourRoomToRampSpace(
                bool[,,] rampSpaceGrid,
                int[,,][] reconstructionGrid,
                List<(int, int, int)> rampSpaceNeighbourFloorSegment) {

            int i;
            int[] voxelState;
            int[] voxelClassValues;

            foreach ((int, int, int) voxel in rampSpaceNeighbourFloorSegment) {

                i = voxel.Item1;

                while (true) {

                    i--;
                    if (i < 0) {
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
                }
            }
        }

        private static void RefineRampSpaceNeighbourRoom(
                bool[,,] isSegmented,
                bool[,,] rampSpaceGrid,
                int[,,][] reconstructionGrid,
                List<(int, int, int)> rampSpaceNeighbourFloorSegment) {

            int i, dr, r, dc, c;
            List<(int, int, int)> overlapSegment;
            
            foreach ((int, int, int) voxel in rampSpaceNeighbourFloorSegment) {
                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (dr.Abs() == dc.Abs()) {
                            continue;
                        }

                        i = voxel.Item1;
                        r = voxel.Item2 + dr;
                        c = voxel.Item3 + dc;

                        if (r < 0 || c < 0
                                || r >= reconstructionGrid.GetLength(1)
                                || c >= reconstructionGrid.GetLength(2)
                                || isSegmented[i, r, c]
                                || !IsFloor(
                                    i,
                                    r,
                                    c,
                                    reconstructionGrid)
                                || !HasRampSpaceAbove(
                                    i,
                                    r,
                                    c,
                                    rampSpaceGrid,
                                    reconstructionGrid)) {
                            continue;
                        }

                        overlapSegment = SegmentPartOfRampSpaceNeighbourFloorSegmentOverlappingWithRampSpace(
                            i,
                            r,
                            c,
                            isSegmented,
                            rampSpaceGrid,
                            reconstructionGrid);

                        RemoveOverlapSegmentFromRampSpace(
                            rampSpaceGrid,
                            overlapSegment);
                    }
                }
            }
        }

        private static bool HasRampSpaceAbove(
                int i,
                int r,
                int c,
                bool[,,] rampSpaceGrid,
                int[,,][] reconstructionGrid) {

            int[] voxelState;

            while (true) {

                i--;
                if (i < 0) {
                    break;
                }

                voxelState = reconstructionGrid[i, r, c];
                if (voxelState == null) {
                    break;
                }

                if (voxelState
                        .GetVoxelClassValues(0)
                        .Contains(VoxelClassValues.FLOOR)) {
                    continue;
                }

                if (rampSpaceGrid[i, r, c]) {
                    return true;
                }

                break;
            }
            return false;
        }

        private static List<(int, int, int)> SegmentPartOfRampSpaceNeighbourFloorSegmentOverlappingWithRampSpace(
                int i,
                int r,
                int c,
                bool[,,] isSegmented,
                bool[,,] rampSpaceGrid,
                int[,,][] reconstructionGrid) {

            int dr, dc;
            List<(int, int, int)> overlapSegment = new List<(int, int, int)>();
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

                overlapSegment.Add(candidate);
                isSegmented[
                    candidate.Item1,
                    candidate.Item2,
                    candidate.Item3] = true;

                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (dr.Abs() == dc.Abs()) {
                            continue;
                        }

                        i = candidate.Item1;
                        r = candidate.Item2 + dr;
                        c = candidate.Item3 + dc;

                        if (r >= 0 && c >= 0
                                && r < reconstructionGrid.GetLength(1)
                                && c < reconstructionGrid.GetLength(2)
                                && !isSegmented[i, r, c]
                                && IsFloor(
                                    i, 
                                    r, 
                                    c,
                                    reconstructionGrid)
                                && HasRampSpaceAbove(
                                    i,
                                    r,
                                    c,
                                    rampSpaceGrid,
                                    reconstructionGrid)) {
                            candidates.Enqueue((
                                candidate.Item1,
                                r,
                                c));
                        }
                    }
                }
            } while (candidates.Count > 0);

            return overlapSegment;
        }

        private static void RemoveOverlapSegmentFromRampSpace(
                bool[,,] rampSpaceGrid,
                List<(int, int, int)> overlapSegment) {

            bool hasStarted = false;
            int i;

            foreach ((int, int, int) voxel in overlapSegment) {

                i = voxel.Item1;

                while (true) {

                    i--;
                    if (i < 0) {
                        break;
                    }

                    if (rampSpaceGrid[
                            i,
                            voxel.Item2,
                            voxel.Item3]) {

                        hasStarted = true;
                        rampSpaceGrid[
                            i,
                            voxel.Item2,
                            voxel.Item3] = false;
                    }
                    else if (hasStarted) {
                        break;
                    }
                }
            }
        }

        private static void CreateTransitionSpacesBetweenRampSpacesAndNeighbouringRooms(
                bool[,,] interiorSpaceGrid,
                bool[,,] rampSpaceGrid,
                bool[,,] roomSpaceGrid,
                bool[,,] transitionSpaceGrid,
                int[,,][] reconstructionGrid) {

            Parallel.For(
                0,
                interiorSpaceGrid.GetLength(0),
                i => {

                    int di, i2, dr, r, r2, dc, c, c2;
                    int[] voxelState;
                    int[] voxelClassValues;

                    for (r = 0; r < interiorSpaceGrid.GetLength(1); r++) {
                        for (c = 0; c < interiorSpaceGrid.GetLength(2); c++) {

                            if (!rampSpaceGrid[i, r, c]) {
                                continue;
                            }

                            for (di = -1; di <= 1; di++) {
                                for (dr = -1; dr <= 1; dr++) {
                                    for (dc = -1; dc <= 1; dc++) {

                                        if (di != 0 && (dr != 0 || dc != 0)) {
                                            continue;
                                        }

                                        i2 = i + di;
                                        r2 = r + dr;
                                        c2 = c + dc;
                                        if (i2 < 0 || r2 < 0 || c2 < 0
                                                || i2 >= rampSpaceGrid.GetLength(0)
                                                || r2 >= rampSpaceGrid.GetLength(1)
                                                || c2 >= rampSpaceGrid.GetLength(2)
                                                || rampSpaceGrid[i2, r2, c2]) {
                                            continue;
                                        }

                                        voxelState = reconstructionGrid[i2, r2, c2];
                                        if (voxelState == null) {
                                            continue;
                                        }

                                        voxelClassValues = voxelState.GetVoxelClassValues(0);
                                        if (voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                                                || voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                                            roomSpaceGrid[i, r, c] = false;
                                            transitionSpaceGrid[i, r, c] = true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                });
        }

        private static int[,,] SegmentSpacePartitioningGrid(
                bool[,,] roomSpaceGrid,
                bool[,,] transitionSpaceGrid,
                bool[,,] rampSpaceGrid,
                out int transitionSpaceCount,
                out HashSet<int> rampSpaceIds) {

            object @lock = new object();
            int roomCount = 0;
            int _transitionSpaceCount = 0;
            int[,,] spacePartitioningGrid = new int[
                roomSpaceGrid.GetLength(0),
                roomSpaceGrid.GetLength(1),
                roomSpaceGrid.GetLength(2)];
            List<int> transitions = new List<int>();
            HashSet<int> _rampSpaceIds = new HashSet<int>();
            Dictionary<int, int> mergeMapping;

            Parallel.ForEach(
                Partitioner.Create(
                    0, 
                    roomSpaceGrid.GetLength(2)),
                () => new HashSet<int>(),
                (partition, loopState, localRampSpaceIds) => {

                    lock (@lock) {
                        transitions.Add(partition.Item2);
                    }
                    for (int c = partition.Item1; c < partition.Item2; c++) {
                        SegmentSpacePartitioningGrid(
                            c,
                            ref roomCount,
                            ref _transitionSpaceCount,
                            @lock,
                            roomSpaceGrid,
                            transitionSpaceGrid,
                            rampSpaceGrid,
                            spacePartitioningGrid,
                            partition,
                            localRampSpaceIds);
                    }

                    return localRampSpaceIds;
                },
                localRampSpaceIds => {
                    lock (@lock) {
                        _rampSpaceIds.AddRange(localRampSpaceIds);
                    }
                });

            mergeMapping = Util.GetMergeMapping(
                spacePartitioningGrid,
                transitions,
                (segmentId1, segmentId2) => (segmentId1 < 0) == (segmentId2 < 0),
                voxel => GetSpacePartitioningGridSegmentMergingNeighbours(
                    voxel,
                    spacePartitioningGrid));

            MergeOversegmentedSpacePartitioningGridSegments(
                spacePartitioningGrid,
                mergeMapping);

            transitionSpaceCount = _transitionSpaceCount;
            rampSpaceIds = _rampSpaceIds;

            return spacePartitioningGrid;
        }

        private static void SegmentSpacePartitioningGrid(
                int c,
                ref int roomCount,
                ref int transitionSpaceCount,
                object @lock,
                bool[,,] roomSpaceGrid,
                bool[,,] transitionSpaceGrid,
                bool[,,] rampSpaceGrid,
                int[,,] spacePartitioningGrid,
                Tuple<int, int> partition,
                HashSet<int> rampSpaceIds) {

            int i, r;
            int segmentIndex;

            for (i = 0; i < roomSpaceGrid.GetLength(0); i++) {
                for (r = 0; r < roomSpaceGrid.GetLength(1); r++) {
                    if (spacePartitioningGrid[i, r, c] != 0) {
                        continue;
                    }
                    if (roomSpaceGrid[i, r, c]) {
                        lock (@lock) {
                            roomCount++;
                            segmentIndex = roomCount;
                        }
                    }
                    else if (transitionSpaceGrid[i, r, c]) {
                        lock (@lock) {
                            transitionSpaceCount++;
                            segmentIndex = -transitionSpaceCount;
                        }
                    }
                    else {
                        continue;
                    }
                    GrowSpacePartitioningGridSegment(
                        i,
                        r,
                        c,
                        segmentIndex,
                        rampSpaceGrid,
                        roomSpaceGrid,
                        transitionSpaceGrid,
                        spacePartitioningGrid,
                        rampSpaceIds,
                        partition);
                }
            }
        }

        private static void GrowSpacePartitioningGridSegment(
                int i,
                int r,
                int c,
                int segmentIndex,
                bool[,,] rampSpaceGrid,
                bool[,,] roomSpaceGrid,
                bool[,,] transitionSpaceGrid,
                int[,,] spacePartitioningGrid,
                HashSet<int> rampSpaceIds,
                Tuple<int, int> partition) {

            int di, dr, dc;
            Queue<(int, int, int)> candidates = new Queue<(int, int, int)>();

            candidates.Enqueue((i, r, c));
            do {
                (int, int, int) candidate = candidates.Dequeue();
                if (spacePartitioningGrid[
                        candidate.Item1,
                        candidate.Item2,
                        candidate.Item3] != 0) {
                    continue;
                }
                spacePartitioningGrid[
                    candidate.Item1,
                    candidate.Item2,
                    candidate.Item3] = segmentIndex;
                if (segmentIndex > 0
                        && rampSpaceGrid[
                            candidate.Item1,
                            candidate.Item2,
                            candidate.Item3]) {
                    rampSpaceIds.Add(segmentIndex);
                }
                for (di = -1; di <= 1; di++) {
                    for (dr = -1; dr <= 1; dr++) {
                        for (dc = -1; dc <= 1; dc++) {
                            if ((segmentIndex > 0
                                        && di.Abs() + dr.Abs() + dc.Abs() != 1)
                                    || (segmentIndex < 0
                                        && di != 0
                                        && dr.Abs() + dc.Abs() != 0)) {
                                continue;
                            }
                            i = candidate.Item1 + di;
                            r = candidate.Item2 + dr;
                            c = candidate.Item3 + dc;
                            if (i < 0 || r < 0 || c < partition.Item1
                                    || i >= roomSpaceGrid.GetLength(0)
                                    || r >= roomSpaceGrid.GetLength(1)
                                    || c >= partition.Item2
                                    || spacePartitioningGrid[i, r, c] != 0
                                    || (segmentIndex > 0 && !roomSpaceGrid[i, r, c])
                                    || (segmentIndex < 0 && !transitionSpaceGrid[i, r, c])) {
                                continue;
                            }
                            candidates.Enqueue((i, r, c));
                        }
                    }
                }
            } while (candidates.Count > 0);
        }

        private static List<(int, int, int)> GetSpacePartitioningGridSegmentMergingNeighbours(
                (int, int, int) voxel,
                int[,,] spacePartitioningGrid) {

            int dr;
            int c = voxel.Item3 - 1;
            int segmentId;
            List<(int, int, int)> neighbours = new List<(int, int, int)>();

            segmentId = spacePartitioningGrid[
                voxel.Item1, 
                voxel.Item2, 
                voxel.Item3];
            if (segmentId > 0) {
                neighbours.Add((
                    voxel.Item1,
                    voxel.Item2,
                    c));
            }
            if (segmentId < 0) {
                for (dr = -1; dr <= 1; dr++) {
                    neighbours.Add((
                        voxel.Item1,
                        voxel.Item2 + dr,
                        c));
                }
            }
            return neighbours;
        }

        private static void MergeOversegmentedSpacePartitioningGridSegments(
                int[,,] spacePartitioningGrid,
                Dictionary<int, int> mergeMapping) {

            Parallel.For(
                0,
                spacePartitioningGrid.GetLength(2),
                c => {
                    int i, r;
                    int segmentId;
                    for (i = 0; i < spacePartitioningGrid.GetLength(0); i++) {
                        for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {
                            segmentId = spacePartitioningGrid[i, r, c];
                            if (mergeMapping.ContainsKey(segmentId)) {
                                spacePartitioningGrid[i, r, c] = mergeMapping[segmentId];
                            }
                        }
                    }
                });
        }

        private static bool GrowSpacePartitioningSegmentsOverUnpartitionedInteriorVoxels(
                bool[,,] wallGrid,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                out bool[,,] roomInteriorContactSurfaceGrid) {

            bool hasCreatedNewContactSurfaces = false;
            bool[,,] _roomInteriorContactSurfacegrid = new bool[
                spacePartitioningGrid.GetLength(0),
                spacePartitioningGrid.GetLength(1),
                spacePartitioningGrid.GetLength(2)];

            Parallel.For(
                0,
                reconstructionGrid.GetLength(0),
                i => {

                    while (
                        GrowSpacePartitioningSegmentsOverUnpartitionedInteriorSpaceByOneLayer(
                            i,
                            wallGrid,
                            spacePartitioningGrid,
                            reconstructionGrid)) { };

                    if (InitializeRoomInteriorContactSurfaceGridSection(
                            i,
                            _roomInteriorContactSurfacegrid,
                            normalGrid,
                            spacePartitioningGrid,
                            reconstructionGrid)) {

                        hasCreatedNewContactSurfaces = true;
                    }
                });

            roomInteriorContactSurfaceGrid = _roomInteriorContactSurfacegrid;

            return hasCreatedNewContactSurfaces;
        }

        private static bool GrowSpacePartitioningSegmentsOverUnpartitionedInteriorSpaceByOneLayer(
                int i,
                bool[,,] wallGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid) {

            bool anyChanges = false;
            int segmentId;
            int dr, r, r2, dc, c, c2;
            bool[,] changed = new bool[
                reconstructionGrid.GetLength(1),
                reconstructionGrid.GetLength(2)];

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                    if (spacePartitioningGrid[i, r, c] != 0
                            || wallGrid[i, r, c]
                            || !IsInterior(
                                i,
                                r,
                                c,
                                reconstructionGrid)) {
                        continue;
                    }

                    for (dr = -1; dr <= 1; dr++) {
                        for (dc = -1; dc <= 1; dc++) {

                            if (dr.Abs() == dc.Abs()) {
                                continue;
                            }

                            r2 = r + dr;
                            c2 = c + dc;
                            if (r2 < 0 || c2 < 0
                                    || r2 >= reconstructionGrid.GetLength(1)
                                    || c2 >= reconstructionGrid.GetLength(2)
                                    || changed[r2, c2]) {
                                continue;
                            }

                            segmentId = spacePartitioningGrid[i, r2, c2];
                            if (segmentId > 0) {
                                anyChanges = true;
                                changed[r, c] = true;
                                spacePartitioningGrid[i, r, c] = segmentId;
                            }
                        }
                    }
                }
            }

            return anyChanges;
        }

        private static bool IsInterior(
                int i,
                int r,
                int c,
                int[,,][] reconstructionGrid) {

            int[] voxelState;
            int[] voxelClassValues;

            voxelState = reconstructionGrid[i, r, c];
            if (voxelState == null) {
                return false;
            }

            voxelClassValues = voxelState.GetVoxelClassValues(0);
            return voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                || voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT);
        }

        private static bool InitializeRoomInteriorContactSurfaceGridSection(
                int i,
                bool[,,] roomInteriorContactSurfaceGrid,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid) {

            bool hasCreatedNewContactSurfaces = false;
            int dr, r, r2, dc, c, c2;
            int segmentId, segmentId2;

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                    segmentId = spacePartitioningGrid[i, r, c];
                    if (segmentId <= 0) {
                        continue;
                    }

                    for (dr = -1; dr <= 1; dr++) {
                        for (dc = -1; dc <= 1; dc++) {

                            if (dr.Abs() == dc.Abs()) {
                                continue;
                            }

                            r2 = r + dr;
                            c2 = c + dc;
                            if (r2 < 0 || c2 < 0
                                    || r2 >= reconstructionGrid.GetLength(1)
                                    || c2 >= reconstructionGrid.GetLength(2)) {
                                continue;
                            }

                            segmentId2 = spacePartitioningGrid[i, r2, c2];
                            if (segmentId2 > 0
                                    && segmentId2 != segmentId) {

                                hasCreatedNewContactSurfaces = true;

                                spacePartitioningGrid[i, r, c] = 0;
                                spacePartitioningGrid[i, r2, c2] = 0;

                                roomInteriorContactSurfaceGrid[i, r, c] = true;
                                roomInteriorContactSurfaceGrid[i, r2, c2] = true;

                                reconstructionGrid[i, r, c] = VoxelState.CreateVoxelState(
                                    0,
                                    normalGrid[i, r, c] == NormalGridValues.EMPTY ?
                                        VoxelClassValues.WALL_OPENING :
                                        VoxelClassValues.WALL);

                                reconstructionGrid[i, r2, c2] = VoxelState.CreateVoxelState(
                                    0,
                                    normalGrid[i, r2, c2] == NormalGridValues.EMPTY ?
                                        VoxelClassValues.WALL_OPENING :
                                        VoxelClassValues.WALL);
                            }
                        }
                    }
                }
            }

            return hasCreatedNewContactSurfaces;
        }

        private static void GetSpacePartitioningSegmentAreasAndAdjacency(
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                out Dictionary<int, int> spacePartitioningSegmentAreas,
                out Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            object @lock = new object();
            Dictionary<int, int> _spacePartitioningSegmentAreas = new Dictionary<int, int>();
            Dictionary<int, HashSet<int>> _spacePartitioningSegmentAdjacency = new Dictionary<int, HashSet<int>>();

            Parallel.ForEach(
                Partitioner.Create(
                    0, 
                    spacePartitioningGrid.GetLength(0)),
                () => (
                    new Dictionary<int, int>(),
                    new Dictionary<int, HashSet<int>>()
                ),
                (heightPartition, loopState, localData) => {

                    UpdateSpacePartitioningSegmentAreasAndAdjacency(
                        spacePartitioningGrid,
                        reconstructionGrid,
                        heightPartition,
                        localData.Item1,
                        localData.Item2);
                    
                    return localData;
                },
                localData => {
                    lock (@lock) {
                        _spacePartitioningSegmentAreas.BucketAdd(localData.Item1);
                        _spacePartitioningSegmentAdjacency.BucketAdd(localData.Item2);
                    }
                });

            spacePartitioningSegmentAreas = _spacePartitioningSegmentAreas;
            spacePartitioningSegmentAdjacency = _spacePartitioningSegmentAdjacency;
        }

        private static void UpdateSpacePartitioningSegmentAreasAndAdjacency(
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                Tuple<int, int> heightPartition,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            int i, r, c;
            int segmentId;

            for (i = heightPartition.Item1; i < heightPartition.Item2; i++) {
                for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {
                    for (c = 0; c < spacePartitioningGrid.GetLength(2); c++) {
                        
                        segmentId = spacePartitioningGrid[i, r, c];
                        if (segmentId == 0) {
                            continue;
                        }

                        UpdateSpacePartitioningSegmentAreas(
                            i,
                            r,
                            c,
                            segmentId,
                            reconstructionGrid,
                            spacePartitioningSegmentAreas);

                        UpdateSpacePartitioningSegmentAdjacency(
                            i,
                            r,
                            c,
                            segmentId,
                            spacePartitioningGrid,
                            spacePartitioningSegmentAdjacency);
                             
                    }
                }
            }
        }

        private static void UpdateSpacePartitioningSegmentAreas(
                int i,
                int r,
                int c,
                int segmentId,
                int[,,][] reconstructionGrid,
                Dictionary<int, int> spacePartitioningSegmentAreas) {

            int[] voxelState;

            if (!spacePartitioningSegmentAreas.ContainsKey(segmentId)) {
                spacePartitioningSegmentAreas.Add(
                    segmentId,
                    0);
            }

            if (i < reconstructionGrid.GetLength(0) - 1) {

                voxelState = reconstructionGrid[i + 1, r, c];
                if (voxelState == null) {
                    return;
                }

                if (voxelState
                        .GetVoxelClassValues(0)
                        .Contains(VoxelClassValues.FLOOR)) {
                    spacePartitioningSegmentAreas.BucketIncrement(segmentId);
                }
            }
        }

        private static void UpdateSpacePartitioningSegmentAdjacency(
                int i,
                int r,
                int c,
                int segmentId,
                int[,,] spacePartitioningGrid,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            int dr, r2, dc, c2;
            int segmentId2;

            if (!spacePartitioningSegmentAdjacency.ContainsKey(segmentId)) {
                spacePartitioningSegmentAdjacency.Add(
                    segmentId,
                    new HashSet<int>());
            }

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {
                    if (dr.Abs() == dc.Abs()) {
                        continue;
                    }
                    r2 = r + dr;
                    c2 = c + dc;
                    if (r2 < 0 || c2 < 0
                            || r2 >= spacePartitioningGrid.GetLength(1)
                            || c2 >= spacePartitioningGrid.GetLength(2)) {
                        continue;
                    }
                    segmentId2 = spacePartitioningGrid[i, r2, c2];
                    if (segmentId2 != 0
                            && segmentId2 != segmentId) {
                        spacePartitioningSegmentAdjacency.BucketAdd(
                            segmentId,
                            segmentId2);
                        spacePartitioningSegmentAdjacency.BucketAdd(
                            segmentId2,
                            segmentId);
                    }
                }
            }
        }

        private static void MergeSpacePartitioningSegments(
                int transitionSpaceCount,
                double resolution,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency,
                out HashSet<int> newTransitionSpaceIds) {

            Dictionary<int, HashSet<int>> mergeIdBuckets = GetSpacePartitioningSegmentMergeIdBuckets(
                resolution,
                spacePartitioningSegmentAreas,
                spacePartitioningSegmentAdjacency);

            Dictionary<int, int> mergeIdMapping = GetSpacePartitioningSegmentMergeIdMapping(
                mergeIdBuckets);

            MergeSpacePartitioningSegments(
                spacePartitioningGrid,
                null,
                mergeIdMapping,
                spacePartitioningSegmentAreas,
                spacePartitioningSegmentAdjacency);

            ResolveRoomInteriorContacts(
                transitionSpaceCount,
                resolution,
                normalGrid,
                spacePartitioningGrid,
                reconstructionGrid,
                spacePartitioningSegmentAreas,
                spacePartitioningSegmentAdjacency,
                out newTransitionSpaceIds);
        }

        private static Dictionary<int, HashSet<int>> GetSpacePartitioningSegmentMergeIdBuckets(
                double resolution,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            int roomMinArea = Parameters
                .ROOM_MIN_AREA
                .GetAreaInVoxels(resolution);
            Dictionary<int, HashSet<int>> mergeIdBuckets = new Dictionary<int, HashSet<int>>();

            foreach (int id in spacePartitioningSegmentAreas
                    .Keys
                    .Where(id => id < 0
                        || (id > 0
                            && spacePartitioningSegmentAreas[id] < roomMinArea))) {
                foreach (int adjacentId in spacePartitioningSegmentAdjacency[id]) {
                    if (adjacentId < 0) {
                        mergeIdBuckets.BucketAdd(id, adjacentId);
                    }
                }
            }

            return mergeIdBuckets;
        }

        private static Dictionary<int, int> GetSpacePartitioningSegmentMergeIdMapping(
                Dictionary<int, HashSet<int>> mergeIdBuckets) {

            HashSet<int> removeIds;
            Dictionary<int, int> mergeIdMapping;

            mergeIdBuckets = RestructureSpacePartitioningSegmentMergeIdBuckets(
                mergeIdBuckets);

            removeIds = GetSpacePartitioningSegmentIdsToRemoveFromMergeIdBuckets(
                mergeIdBuckets);

            foreach (int id in removeIds) {
                mergeIdBuckets.Remove(id);
            }

            mergeIdMapping = InitializeSpacePartitioningSegmentMergeIdMapping(
                mergeIdBuckets);

            mergeIdMapping = RestructureSpacePartitioningSegmentMergeIdMapping(
                mergeIdMapping);

            return mergeIdMapping;
        }

        private static Dictionary<int, HashSet<int>> RestructureSpacePartitioningSegmentMergeIdBuckets(
                Dictionary<int, HashSet<int>> mergeIdBuckets) {

            bool first;
            int firstDestinationId = 0;
            Dictionary<int, HashSet<int>> restructuredMergeIdBuckets = new Dictionary<int, HashSet<int>>();

            foreach (int id in mergeIdBuckets.Keys) {
                first = true;
                foreach (int destinationId in mergeIdBuckets[id]) {
                    if (first) {
                        first = false;
                        firstDestinationId = destinationId;
                        restructuredMergeIdBuckets.BucketAdd(id, destinationId);
                    }
                    else {
                        restructuredMergeIdBuckets.BucketAdd(
                            destinationId,
                            firstDestinationId);
                    }
                }
            }

            mergeIdBuckets = restructuredMergeIdBuckets;
            restructuredMergeIdBuckets = new Dictionary<int, HashSet<int>>();

            foreach (int id in mergeIdBuckets.Keys) {
                if (mergeIdBuckets[id].Count > 1) {
                    first = true;
                    foreach (int destinationId in mergeIdBuckets[id].Order()) {
                        if (first) {
                            first = false;
                            firstDestinationId = destinationId;
                            restructuredMergeIdBuckets.BucketAdd(
                                id,
                                firstDestinationId);
                        }
                        else {
                            restructuredMergeIdBuckets.BucketAdd(
                                destinationId,
                                firstDestinationId);
                        }
                    }
                }
                else {
                    restructuredMergeIdBuckets.BucketAdd(
                        id,
                        mergeIdBuckets[id].First());
                }
            }

            return restructuredMergeIdBuckets;
        }

        private static HashSet<int> GetSpacePartitioningSegmentIdsToRemoveFromMergeIdBuckets(
                Dictionary<int, HashSet<int>> mergeIdBuckets) {

            int firstDestinationId;
            HashSet<int> removeIds = new HashSet<int>();

            foreach (int id in mergeIdBuckets.Keys) {
                firstDestinationId = mergeIdBuckets[id].First();
                if (mergeIdBuckets.ContainsKey(firstDestinationId)
                        && mergeIdBuckets[firstDestinationId].First() == id) {
                    if (firstDestinationId < id) {
                        removeIds.Add(firstDestinationId);
                    }
                    else {
                        removeIds.Add(id);
                    }
                }
            }

            return removeIds;
        }

        private static Dictionary<int, int> InitializeSpacePartitioningSegmentMergeIdMapping(
                Dictionary<int, HashSet<int>> mergeIdBuckets) {

            Dictionary<int, int> mergeIdMapping = new Dictionary<int, int>();

            foreach (int id in mergeIdBuckets.Keys) {
                mergeIdMapping.Add(
                    id,
                    mergeIdBuckets[id].First());
            }

            return mergeIdMapping;
        }

        private static Dictionary<int, int> RestructureSpacePartitioningSegmentMergeIdMapping(
                Dictionary<int, int> mergeIdMapping) {

            Dictionary<int, int> restructuredMergeIdMapping = new Dictionary<int, int>();

            foreach (int id in mergeIdMapping.Keys) {
                if (mergeIdMapping.ContainsKey(mergeIdMapping[id])) {
                    restructuredMergeIdMapping.Add(
                        id,
                        mergeIdMapping[mergeIdMapping[id]]);
                }
                else {
                    restructuredMergeIdMapping.Add(
                        id,
                        mergeIdMapping[id]);
                }
            }

            return restructuredMergeIdMapping;
        }

        private static void MergeSpacePartitioningSegments(
                int[,,] spacePartitioningGrid,
                HashSet<int> rampTransitionSpaceIds,
                Dictionary<int, int> mergeIdMapping,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            Parallel.For(
                0,
                spacePartitioningGrid.GetLength(0),
                i => MergeSpacePartitioningSegments(
                    i,
                    spacePartitioningGrid,
                    mergeIdMapping));

            UpdateSpacePartitioningSegmentAreasAndAdjacencyAfterMerging(
                rampTransitionSpaceIds,
                mergeIdMapping,
                spacePartitioningSegmentAreas,
                spacePartitioningSegmentAdjacency);
        }

        private static void MergeSpacePartitioningSegments(
                int i,
                int[,,] spacePartitioningGrid,
                Dictionary<int, int> mergeIdMapping) {

            int r, c;
            int segmentId;

            for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {
                for (c = 0; c < spacePartitioningGrid.GetLength(2); c++) {
                    segmentId = spacePartitioningGrid[i, r, c];
                    if (mergeIdMapping.ContainsKey(segmentId)) {
                        spacePartitioningGrid[i, r, c] = mergeIdMapping[segmentId];
                    }
                }
            }
        }

        private static void UpdateSpacePartitioningSegmentAreasAndAdjacencyAfterMerging(
                HashSet<int> rampTransitionSpaceIds,
                Dictionary<int, int> mergeIdMapping,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            int segmentId;

            foreach (int id in mergeIdMapping.Keys) {
                segmentId = mergeIdMapping[id];
                foreach (int adjacentId in spacePartitioningSegmentAdjacency[id]) {
                    if (!spacePartitioningSegmentAreas.ContainsKey(id)) {
                        spacePartitioningSegmentAreas.Add(id, 0);
                    }
                    if (!spacePartitioningSegmentAreas.ContainsKey(segmentId)) {
                        spacePartitioningSegmentAreas.Add(segmentId, 0);
                    }
                    spacePartitioningSegmentAreas[segmentId] += spacePartitioningSegmentAreas[id];
                    if (!mergeIdMapping.ContainsKey(adjacentId)
                            && adjacentId != segmentId) {
                        spacePartitioningSegmentAdjacency.BucketAdd(
                            segmentId, adjacentId);
                    }
                }
                if (rampTransitionSpaceIds != null
                        && rampTransitionSpaceIds.Contains(id)) {
                    rampTransitionSpaceIds.Add(segmentId);
                }
            }

            RemoveSpacePartitioningSegmentIds(
                mergeIdMapping.Keys,
                rampTransitionSpaceIds,
                spacePartitioningSegmentAreas,
                spacePartitioningSegmentAdjacency);
        }

        private static void RemoveSpacePartitioningSegmentIds(
                IEnumerable<int> ids,
                HashSet<int> rampTransitionSpaceIds,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            foreach (HashSet<int> adjacentIds in spacePartitioningSegmentAdjacency.Values) {
                adjacentIds.RemoveWhere(id => ids.Contains(id));
            }
            foreach (int id in ids) {
                rampTransitionSpaceIds?.Remove(id);
                spacePartitioningSegmentAdjacency.Remove(id);
                spacePartitioningSegmentAreas.Remove(id);
            }
        }

        private static void ResolveRoomInteriorContacts(
                int transitionSpaceCount,
                double resolution,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency,
                out HashSet<int> newTransitionSpaceIds) {

            int i, r, c;
            int openVoxelCount;
            int contactSurfaceMinOpeningArea = Parameters
                .ROOM_CONTACT_SURFACE_MIN_OPENING_AREA
                .GetAreaInVoxels(resolution);
            bool[,,] isSegmented = new bool[
                spacePartitioningGrid.GetLength(0),
                spacePartitioningGrid.GetLength(1),
                spacePartitioningGrid.GetLength(2)];
            bool[,,] roomInteriorContactGrid;
            List<(int, int, int)> roomInteriorContactSurfaceSegment;

            newTransitionSpaceIds = new HashSet<int>();

            roomInteriorContactGrid = InitializeRoomInteriorContactGrid(
                spacePartitioningGrid);

            for (i = 0; i < spacePartitioningGrid.GetLength(0); i++) {
                for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {
                    for (c = 0; c < spacePartitioningGrid.GetLength(2); c++) {

                        if (!roomInteriorContactGrid[i, r, c]
                                || isSegmented[i, r, c]) {
                            continue;
                        }

                        roomInteriorContactSurfaceSegment = GrowRoomInteriorContactSurfaceSegment(
                            i,
                            r,
                            c,
                            isSegmented,
                            roomInteriorContactGrid,
                            normalGrid,
                            spacePartitioningGrid,
                            out openVoxelCount);

                        if (roomInteriorContactSurfaceSegment.Count / 2.0 >= contactSurfaceMinOpeningArea
                                && (double)openVoxelCount / roomInteriorContactSurfaceSegment.Count
                                    >= Parameters.ROOM_CONTACT_SURFACE_MIN_OPENING_RATIO) {

                            ConvertRoomInteriorContactSurfaceSegmentToTransitionSpace(
                                ref transitionSpaceCount,
                                spacePartitioningGrid,
                                newTransitionSpaceIds,
                                roomInteriorContactSurfaceSegment,
                                spacePartitioningSegmentAreas,
                                spacePartitioningSegmentAdjacency);
                        }
                        else {
                            RemoveRoomInteriorContactSurfaceSegment(
                                normalGrid,
                                spacePartitioningGrid,
                                reconstructionGrid,
                                roomInteriorContactSurfaceSegment);
                        }
                    }
                }
            }
        }

        private static bool[,,] InitializeRoomInteriorContactGrid(
                int[,,] spacePartitioningGrid) {

            bool[,,] roomInteriorContactGrid = new bool[
                spacePartitioningGrid.GetLength(0),
                spacePartitioningGrid.GetLength(1),
                spacePartitioningGrid.GetLength(2)];

            Parallel.For(
                0,
                spacePartitioningGrid.GetLength(0),
                 i => {

                     int dr, r, r2, dc, c, c2;
                     int segmentId, segmentId2;

                     for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {
                         for (c = 0; c < spacePartitioningGrid.GetLength(2); c++) {

                             segmentId = spacePartitioningGrid[i, r, c];
                             if (segmentId <= 0) {
                                 continue;
                             }

                             for (dr = -1; dr <= 1; dr++) {
                                 for (dc = -1; dc <= 1; dc++) {

                                     if (dr.Abs() == dc.Abs()) {
                                         continue;
                                     }

                                     r2 = r + dr;
                                     c2 = c + dc;
                                     if (r2 < 0 || c2 < 0
                                             || r2 >= spacePartitioningGrid.GetLength(1)
                                             || c2 >= spacePartitioningGrid.GetLength(2)) {
                                         continue;
                                     }

                                     segmentId2 = spacePartitioningGrid[i, r2, c2];
                                     if (segmentId2 > 0
                                             && segmentId2 != segmentId) {
                                         roomInteriorContactGrid[i, r, c] = true;
                                         roomInteriorContactGrid[i, r2, c2] = true;
                                     }
                                 }
                             }
                         }
                     }
                 });

            return roomInteriorContactGrid;
        }

        private static List<(int, int, int)> GrowRoomInteriorContactSurfaceSegment(
                int i,
                int r,
                int c,
                bool[,,] isSegmented,
                bool[,,] roomInteriorContactGrid,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                out int openVoxelCount) {

            int di, i2, dr, r2, dc, c2;
            Queue<(int, int, int)> candidates = new Queue<(int, int, int)>();
            List<(int, int, int)> roomInteriorContactSurfaceSegment = new List<(int, int, int)>();

            openVoxelCount = 0;
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

                roomInteriorContactSurfaceSegment.Add(candidate);

                if (normalGrid[
                        candidate.Item1,
                        candidate.Item2,
                        candidate.Item3] == NormalGridValues.EMPTY) {
                    openVoxelCount++;
                }

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
                                    && i2 < spacePartitioningGrid.GetLength(0)
                                    && r2 < spacePartitioningGrid.GetLength(1)
                                    && c2 < spacePartitioningGrid.GetLength(2)
                                    && roomInteriorContactGrid[i2, r2, c2]
                                    && !isSegmented[i2, r2, c2]) {
                                candidates.Enqueue((i2, r2, c2));
                            }
                        }
                    }
                }
            } while (candidates.Count > 0);

            return roomInteriorContactSurfaceSegment;
        }

        private static void ConvertRoomInteriorContactSurfaceSegmentToTransitionSpace(
                ref int transitionSpaceCount,
                int[,,] spacePartitioningGrid,
                HashSet<int> newTransitionSpaceIds,
                List<(int, int, int)> roomInteriorContactSurfaceSegment,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            int dr, r, dc, c;
            int segmentId, segmentId2;

            transitionSpaceCount++;
            segmentId = -transitionSpaceCount;
            newTransitionSpaceIds.Add(segmentId);
            spacePartitioningSegmentAreas.Add(segmentId, 0);
            spacePartitioningSegmentAdjacency.Add(
                segmentId,
                new HashSet<int>());

            foreach ((int, int, int) voxel in roomInteriorContactSurfaceSegment) {

                spacePartitioningGrid[
                    voxel.Item1,
                    voxel.Item2,
                    voxel.Item3] = segmentId;

                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (dr.Abs() == dc.Abs()) {
                            continue;
                        }

                        r = voxel.Item2 + dr;
                        c = voxel.Item3 + dc;
                        if (r < 0 || c < 0
                                || r >= spacePartitioningGrid.GetLength(1)
                                || c >= spacePartitioningGrid.GetLength(2)) {
                            continue;
                        }

                        segmentId2 = spacePartitioningGrid[
                            voxel.Item1, 
                            r, 
                            c];

                        if (segmentId2 != 0
                                && segmentId2 != segmentId) {
                            spacePartitioningSegmentAdjacency[segmentId].Add(segmentId2);
                        }
                    }
                }
            }
        }

        private static void RemoveRoomInteriorContactSurfaceSegment(
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                List<(int, int, int)> roomInteriorContactSurfaceSegment) {

            foreach ((int, int, int) voxel in roomInteriorContactSurfaceSegment) {

                spacePartitioningGrid[
                    voxel.Item1,
                    voxel.Item2,
                    voxel.Item3] = 0;

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
                            VoxelClassValues.WALL_OPENING :
                            VoxelClassValues.WALL);
            }
        }

        private static void RemoveDeadEndTransitionSpaces(
                int[,,] spacePartitioningGrid,
                HashSet<int> rampTransitionSpaceIds,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            HashSet<int> deadEndTransitionSpaceIds;
            Dictionary<int, HashSet<int>> transitionSpaceAdjacency;

            transitionSpaceAdjacency = GetTransitionSpaceAdjacency(
                spacePartitioningGrid);

            deadEndTransitionSpaceIds = transitionSpaceAdjacency
                .Keys
                .Where(id => transitionSpaceAdjacency[id].Count <= 1)
                .ToHashSet();

            RemoveDeadEndTransitionSpaces(
                spacePartitioningGrid,
                deadEndTransitionSpaceIds,
                transitionSpaceAdjacency);

            RemoveSpacePartitioningSegmentIds(
                deadEndTransitionSpaceIds,
                rampTransitionSpaceIds,
                spacePartitioningSegmentAreas,
                spacePartitioningSegmentAdjacency);
        }

        private static Dictionary<int, HashSet<int>> GetTransitionSpaceAdjacency(
                int[,,] spacePartitioningGrid) {

            object @lock = new object();
            Dictionary<int, HashSet<int>> transitionSpaceAdjacency = new Dictionary<int, HashSet<int>>();

            Parallel.ForEach(
                Partitioner.Create(
                    0, 
                    spacePartitioningGrid.GetLength(0)),
                () => new Dictionary<int, HashSet<int>>(),
                (partition, loopState, localTransitionSpaceAdjacency) => {

                    int i, dr, r, r2, c, c2, dc;
                    int segmentId, segmentId2;

                    for (i = partition.Item1; i < partition.Item2; i++) {
                        for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {
                            for (c = 0; c < spacePartitioningGrid.GetLength(2); c++) {

                                segmentId = spacePartitioningGrid[i, r, c];
                                if (segmentId >= 0) {
                                    continue;
                                }

                                for (dr = -1; dr <= 1; dr++) {
                                    for (dc = -1; dc <= 1; dc++) {

                                        if (dr.Abs() == dc.Abs()) {
                                            continue;
                                        }

                                        r2 = r + dr;
                                        c2 = c + dc;
                                        if (r2 < 0 || c2 < 0
                                                || r2 >= spacePartitioningGrid.GetLength(1)
                                                || c2 >= spacePartitioningGrid.GetLength(2)) {
                                            continue;
                                        }

                                        segmentId2 = spacePartitioningGrid[i, r2, c2];
                                        if (segmentId2 > 0) {
                                            localTransitionSpaceAdjacency.BucketAdd(
                                                segmentId,
                                                segmentId2);
                                        }
                                    }
                                }
                            }
                        }
                    }

                    return localTransitionSpaceAdjacency;
                },
                localTransitionSpaceAdjacency => {
                    lock (@lock) {
                        transitionSpaceAdjacency.BucketAdd(localTransitionSpaceAdjacency);
                    }
                });

            return transitionSpaceAdjacency;
        }

        private static void RemoveDeadEndTransitionSpaces(
                int[,,] spacePartitioningGrid,
                HashSet<int> deadEndTransitionSpaceIds,
                Dictionary<int, HashSet<int>> transitionSpaceAdjacency) {

            Parallel.For(
                0,
                spacePartitioningGrid.GetLength(0),
                i => {

                    int r, c, segmentId;

                    for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {
                        for (c = 0; c < spacePartitioningGrid.GetLength(2); c++) {

                            segmentId = spacePartitioningGrid[i, r, c];

                            if (deadEndTransitionSpaceIds.Contains(segmentId)) {

                                spacePartitioningGrid[i, r, c] =
                                    transitionSpaceAdjacency[segmentId].Count == 1 ?
                                        transitionSpaceAdjacency[segmentId].First() :
                                        0;
                            }
                        }
                    }
                });
        }

        private static void GrowRampSpacesOverAdjacentTransitionSpacesFromWithin(
                int[,,] spacePartitioningGrid,
                HashSet<int> rampSpaceIds,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            bool isAdjacentToRampSpace, isAdjacentToOtherRoom;
            int i, r, c;
            int rampSpaceId;

            HashSet<int> rampSpaceNeighbouringTransitionSpaceIds;

            rampSpaceNeighbouringTransitionSpaceIds = GetRampSpaceNeighbouringTransitionSpaceIds(
                rampSpaceIds,
                spacePartitioningSegmentAdjacency);

            for (i = 0; i < spacePartitioningGrid.GetLength(0); i++) {
                for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {
                    for (c = 0; c < spacePartitioningGrid.GetLength(2); c++) {

                        if (!rampSpaceNeighbouringTransitionSpaceIds
                                .Contains(spacePartitioningGrid[i, r, c])) {
                            continue;
                        }

                        CheckVoxelAdjacency(
                            i,
                            r,
                            c,
                            spacePartitioningGrid,
                            rampSpaceIds,
                            out isAdjacentToRampSpace,
                            out isAdjacentToOtherRoom,
                            out rampSpaceId);

                        if (!isAdjacentToRampSpace || isAdjacentToOtherRoom) {
                            continue;
                        }

                        GrowRampSpaceOverAdjacentTransitionSpace(
                            i,
                            r,
                            c,
                            rampSpaceId,
                            spacePartitioningGrid,
                            rampSpaceIds);
                    }
                }
            }
        }

        private static HashSet<int> GetRampSpaceNeighbouringTransitionSpaceIds(
                HashSet<int> rampSpaceIds,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            return spacePartitioningSegmentAdjacency
                .Keys
                .Where(id => id < 0
                    && spacePartitioningSegmentAdjacency[id]
                        .Where(adjacentId => rampSpaceIds.Contains(adjacentId))
                        .Count() == 1)
                .ToHashSet();
        }

        private static void CheckVoxelAdjacency(
                int i,
                int r,
                int c,
                int[,,] spacePartitioningGrid,
                HashSet<int> rampSpaceIds,
                out bool isAdjacentToRampSpace,
                out bool isAdjacentToOtherRoom,
                out int rampSpaceId) {

            int di, i2, dr, r2, dc, c2;
            int segmentId;

            isAdjacentToRampSpace = isAdjacentToOtherRoom = false;
            rampSpaceId = 0;

            for (di = -1; di <= 1; di++) {
                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (di != 0 && (dr != 0 || dc != 0)) {
                            continue;
                        }

                        i2 = i + di;
                        r2 = r + dr;
                        c2 = c + dc;
                        if (i2 < 0 || r2 < 0 || c2 < 0
                                || i2 >= spacePartitioningGrid.GetLength(0)
                                || r2 >= spacePartitioningGrid.GetLength(1)
                                || c2 >= spacePartitioningGrid.GetLength(2)) {
                            continue;
                        }

                        segmentId = spacePartitioningGrid[i2, r2, c2];

                        if (rampSpaceIds.Contains(segmentId)) {
                            rampSpaceId = segmentId;
                            isAdjacentToRampSpace = true;
                        }
                        else if (segmentId > 0) {
                            isAdjacentToOtherRoom = true;
                        }
                    }
                }
            }
        }

        private static bool IsAdjacentToOtherRoom(
                int i,
                int r,
                int c,
                int[,,] spacePartitioningGrid,
                HashSet<int> rampSpaceIds) {

            int di, i2, dr, r2, dc, c2;
            int segmentId;

            for (di = -1; di <= 1; di++) {
                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (di != 0 && (dr != 0 || dc != 0)) {
                            continue;
                        }

                        i2 = i + di;
                        r2 = r + dr;
                        c2 = c + dc;
                        if (i2 < 0 || r2 < 0 || c2 < 0
                                || i2 >= spacePartitioningGrid.GetLength(0)
                                || r2 >= spacePartitioningGrid.GetLength(1)
                                || c2 >= spacePartitioningGrid.GetLength(2)) {
                            continue;
                        }

                        segmentId = spacePartitioningGrid[i2, r2, c2];
                        if (segmentId > 0
                                && !rampSpaceIds.Contains(segmentId)) {
                            return true;
                        }
                    }
                }
            }
            return false;
        }

        private static void GrowRampSpaceOverAdjacentTransitionSpace(
                int i,
                int r,
                int c,
                int rampSpaceId,
                int[,,] spacePartitioningGrid,
                HashSet<int> rampSpaceIds) {

            int di, i2, dr, r2, dc, c2;
            (int, int, int) candidate;
            Queue<(int, int, int)> candidates = new Queue<(int, int, int)>();

            candidates.Enqueue((i, r, c));
            do {

                candidate = candidates.Dequeue();
                if (spacePartitioningGrid[
                        candidate.Item1,
                        candidate.Item2,
                        candidate.Item3] >= 0) {
                    continue;
                }

                spacePartitioningGrid[
                    candidate.Item1,
                    candidate.Item2,
                    candidate.Item3] = rampSpaceId;

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
                                    && i2 < spacePartitioningGrid.GetLength(0)
                                    && r2 < spacePartitioningGrid.GetLength(1)
                                    && c2 < spacePartitioningGrid.GetLength(2)
                                    && spacePartitioningGrid[i2, r2, c2] < 0
                                    && !IsAdjacentToOtherRoom(
                                        i2,
                                        r2,
                                        c2,
                                        spacePartitioningGrid,
                                        rampSpaceIds)) {
                                candidates.Enqueue((i2, r2, c2));
                            }
                        }
                    }
                }
            } while (candidates.Count > 0);
        }

        private static bool FindMissingTransitionSpaces(
                ref int transitionSpaceCount,
                double resolution,
                bool[,,] roomInteriorContactSurfaceGrid,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            int contactSurfaceMinOpeningArea = Parameters
                .ROOM_CONTACT_SURFACE_MIN_OPENING_AREA
                .GetAreaInVoxels(resolution);
            bool createdNewTransitionSpaces = false;
            bool[,,] wallOpeningGrid;
            HashSet<int> newTransitionSpaceIds = new HashSet<int>();
            HashSet<int> adjacentRoomIds;
            Dictionary<int, HashSet<(int, int, int)>> wallOpeningSegmentSurfaces;

            wallOpeningGrid = InitializeWallOpeningGrid(
                resolution,
                spacePartitioningGrid,
                reconstructionGrid);

            RefineWallOpeningGrid(
                wallOpeningGrid,
                spacePartitioningGrid,
                reconstructionGrid);

            foreach (List<(int, int, int)> wallOpeningSegment in GetWallOpeningSegments(
                    wallOpeningGrid,
                    spacePartitioningGrid)) {

                GetWallOpeningSegmentAdjacencyAndSurfaces(
                    spacePartitioningGrid,
                    wallOpeningSegment,
                    out adjacentRoomIds,
                    out wallOpeningSegmentSurfaces);

                if (adjacentRoomIds.Count == 1) {
                    AssignWallOpeningSegmentToRoom(
                        adjacentRoomIds.First(),
                        roomInteriorContactSurfaceGrid,
                        normalGrid,
                        spacePartitioningGrid,
                        reconstructionGrid,
                        wallOpeningSegment,
                        spacePartitioningSegmentAreas);
                }
                else if (adjacentRoomIds
                        .Where(id => wallOpeningSegmentSurfaces[id].Count >= contactSurfaceMinOpeningArea)
                        .Count() >= 2) {

                    createdNewTransitionSpaces = true;

                    ConvertWallOpeningSegmentToTransitionSpace(
                        -(++transitionSpaceCount),
                        normalGrid,
                        spacePartitioningGrid,
                        reconstructionGrid,
                        adjacentRoomIds,
                        newTransitionSpaceIds,
                        wallOpeningSegment,
                        spacePartitioningSegmentAreas,
                        spacePartitioningSegmentAdjacency);
                }
            }

            if (newTransitionSpaceIds.Count > 0) {
                RefineNewTransitionSpaces(
                    normalGrid,
                    spacePartitioningGrid,
                    reconstructionGrid,
                    newTransitionSpaceIds);
            }

            return createdNewTransitionSpaces;
        }

        private static bool[,,] InitializeWallOpeningGrid(
                double resolution,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid) {

            int minOpeningHeight = Parameters
                .MIN_WALL_OPENING_HEIGHT
                .GetDistanceInVoxels(resolution);
            int maxOpeningJumpSize = (int)(Parameters.MAX_WALL_OPENING_JUMP_SIZE / resolution).Round();
            bool[,,] isProcessed = new bool[
                spacePartitioningGrid.GetLength(0),
                spacePartitioningGrid.GetLength(1),
                spacePartitioningGrid.GetLength(2)];
            bool[,,] wallOpeningGrid = new bool[
                spacePartitioningGrid.GetLength(0),
                spacePartitioningGrid.GetLength(1),
                spacePartitioningGrid.GetLength(2)];

            Parallel.For(
                0,
                spacePartitioningGrid.GetLength(2),
                c => {

                    bool isOpening;
                    int i, i2, r;
                    int wallOpeningHeightStopIndex;

                    for (i = 0; i < spacePartitioningGrid.GetLength(0); i++) {
                        for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {

                            if (isProcessed[i, r, c]
                                    || !IsWallOpening(
                                        i,
                                        r,
                                        c,
                                        reconstructionGrid)) {
                                continue;
                            }

                            wallOpeningHeightStopIndex = GetWallOpeningHeightStopIndex(
                                i,
                                r,
                                c,
                                maxOpeningJumpSize,
                                reconstructionGrid);

                            isOpening = wallOpeningHeightStopIndex - i >= minOpeningHeight;

                            for (i2 = i; i2 < wallOpeningHeightStopIndex; i2++) {
                                isProcessed[i2, r, c] = true;
                                if (isOpening) {
                                    wallOpeningGrid[i2, r, c] = true;
                                }
                            }
                        }
                    }
                });

            return wallOpeningGrid;
        }

        private static bool IsWallOpening(
                int i,
                int r,
                int c,
                int[,,][] reconstructionGrid) {

            int[] voxelState = reconstructionGrid[i, r, c];

            if (voxelState == null) {
                return false;
            }

            return voxelState
                .GetVoxelClassValues(0)
                .Contains(VoxelClassValues.WALL_OPENING);
        }

        private static int GetWallOpeningHeightStopIndex(
                int i,
                int r,
                int c,
                int maxOpeningJumpSize,
                int[,,][] reconstructionGrid) {

            bool isClosed = false;
            int i2 = i + 1;
            int closedStartHeight = 0;
            int[] voxelState;
            int[] voxelClassValues;

            while (true) {

                if (i2 >= reconstructionGrid.GetLength(0)) {
                    break;
                }

                voxelState = reconstructionGrid[i2, r, c];
                if (voxelState == null) {
                    break;
                }

                voxelClassValues = voxelState.GetVoxelClassValues(0);

                if (voxelClassValues.Contains(VoxelClassValues.WALL_OPENING)) {

                    i2++;
                    isClosed = false;
                    continue;
                }
                else if (voxelClassValues.Contains(VoxelClassValues.WALL)) {

                    if (!isClosed) {
                        closedStartHeight = i2;
                    }

                    isClosed = true;

                    if (i2 - closedStartHeight > maxOpeningJumpSize) {
                        break;
                    }

                    i2++;
                    continue;
                }

                break;
            }

            return i2;
        }

        private static void RefineWallOpeningGrid(
                bool[,,] wallOpeningGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid) {

            bool[,,] isProcessed = new bool[
                spacePartitioningGrid.GetLength(0),
                spacePartitioningGrid.GetLength(1),
                spacePartitioningGrid.GetLength(2)];

            Parallel.For(
                0,
                spacePartitioningGrid.GetLength(0),
                i => {

                    int r, c;

                    for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {
                        for (c = 0; c < spacePartitioningGrid.GetLength(2); c++) {

                            if (!wallOpeningGrid[i, r, c]
                                    && IsWallSurface(
                                        i,
                                        r,
                                        c,
                                        reconstructionGrid)
                                    && GetWallOpeningCountInNeighbourhood(
                                        i,
                                        r,
                                        c,
                                        isProcessed,
                                        wallOpeningGrid,
                                        spacePartitioningGrid) >= 2) {

                                isProcessed[i, r, c] = true;
                                wallOpeningGrid[i, r, c] = true;
                            }
                        }
                    }
                });
        }

        private static bool IsWallSurface(
                int i,
                int r,
                int c,
                int[,,][] reconstructionGrid) {

            int[] voxelState;
            int[] voxelClassValues;

            voxelState = reconstructionGrid[i, r, c];
            if (voxelState == null) {
                return false;
            }

            voxelClassValues = voxelState.GetVoxelClassValues(0);

            return voxelClassValues.Contains(VoxelClassValues.WALL)
                || voxelClassValues.Contains(VoxelClassValues.WALL_OPENING);
        }

        private static int GetWallOpeningCountInNeighbourhood(
                int i,
                int r,
                int c,
                bool[,,] isProcessed,
                bool[,,] wallOpeningGrid,
                int[,,] spacePartitioningGrid) {

            int dr, r2, dc, c2;
            int counter = 0;

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    r2 = r + dr;
                    c2 = c + dc;
                    if (r2 >= 0 && c2 >= 0
                            && r2 < spacePartitioningGrid.GetLength(1)
                            && c2 < spacePartitioningGrid.GetLength(2)
                            && wallOpeningGrid[i, r2, c2]
                            && !isProcessed[i, r2, c2]) {

                        counter++;
                    }
                }
            }

            return counter;
        }

        private static IEnumerable<List<(int, int, int)>> GetWallOpeningSegments(
                bool[,,] wallOpeningGrid,
                int[,,] spacePartitioningGrid) {

            int di, i, i2, dr, r, r2, dc, c, c2;
            bool[,,] isSegmented = new bool[
                spacePartitioningGrid.GetLength(0),
                spacePartitioningGrid.GetLength(1),
                spacePartitioningGrid.GetLength(2)];
            List<(int, int, int)> wallOpeningSegment;
            Queue<(int, int, int)> candidates = new Queue<(int, int, int)>();

            for (i = 0; i < spacePartitioningGrid.GetLength(0); i++) {
                for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {
                    for (c = 0; c < spacePartitioningGrid.GetLength(2); c++) {

                        if (!wallOpeningGrid[i, r, c]
                                || isSegmented[i, r, c]) {
                            continue;
                        }

                        wallOpeningSegment = new List<(int, int, int)>();
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
                            wallOpeningSegment.Add(candidate);

                            for (di = -1; di <= 1; di++) {
                                for (dr = -1; dr <= 1; dr++) {
                                    for (dc = -1; dc <= 1; dc++) {

                                        if (di.Abs() == 1 && dr.Abs() + dc.Abs() == 2) {
                                            continue;
                                        }

                                        i2 = candidate.Item1 + di;
                                        r2 = candidate.Item2 + dr;
                                        c2 = candidate.Item3 + dc;
                                        if (i2 >= 0 && r2 >= 0 && c2 >= 0
                                                && i2 < spacePartitioningGrid.GetLength(0)
                                                && r2 < spacePartitioningGrid.GetLength(1)
                                                && c2 < spacePartitioningGrid.GetLength(2)
                                                && wallOpeningGrid[i2, r2, c2]
                                                && !isSegmented[i2, r2, c2]) {
                                            candidates.Enqueue((i2, r2, c2));
                                        }
                                    }
                                }
                            }
                        } while (candidates.Count > 0);

                        yield return wallOpeningSegment;
                    }
                }
            }
        }

        private static void GetWallOpeningSegmentAdjacencyAndSurfaces(
                int[,,] spacePartitioningGrid,
                List<(int, int, int)> wallOpeningSegment,
                out HashSet<int> adjacentRoomIds,
                out Dictionary<int, HashSet<(int, int, int)>> wallOpeningSegmentSurfaces) {

            int dr, r2, dc, c2;
            int roomId;

            adjacentRoomIds = new HashSet<int>();
            wallOpeningSegmentSurfaces = new Dictionary<int, HashSet<(int, int, int)>>();

            foreach ((int, int, int) voxel in wallOpeningSegment) {
                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (dr.Abs() == dc.Abs()) {
                            continue;
                        }

                        r2 = voxel.Item2 + dr;
                        c2 = voxel.Item3 + dc;
                        if (r2 < 0 || c2 < 0
                                || r2 >= spacePartitioningGrid.GetLength(1)
                                || c2 >= spacePartitioningGrid.GetLength(2)) {
                            continue;
                        }

                        roomId = spacePartitioningGrid[voxel.Item1, r2, c2];
                        if (roomId > 0) {
                            adjacentRoomIds.Add(roomId);
                            wallOpeningSegmentSurfaces.BucketAdd(roomId, voxel);
                        }
                    }
                }
            }
        }

        private static void AssignWallOpeningSegmentToRoom(
                int roomId,
                bool[,,] roomInteriorContactSurfaceGrid,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                List<(int, int, int)> wallOpeningSegment,
                Dictionary<int, int> spacePartitioningSegmentAreas) {

            int i, r, c;

            foreach ((int, int, int) voxel in wallOpeningSegment) {

                i = voxel.Item1;
                r = voxel.Item2;
                c = voxel.Item3;

                if (roomInteriorContactSurfaceGrid[i, r, c]) {
                    continue;
                }

                spacePartitioningGrid[i, r, c] = roomId;

                reconstructionGrid[i, r, c] = VoxelState.CreateVoxelState(
                    0,
                    normalGrid[i, r, c] == NormalGridValues.EMPTY ?
                        VoxelClassValues.EMPTY_INTERIOR :
                        VoxelClassValues.INTERIOR_OBJECT);

                UpdateRoomArea(
                    i + 1,
                    r,
                    c,
                    roomId,
                    reconstructionGrid,
                    spacePartitioningSegmentAreas);
            }
        }

        private static void UpdateRoomArea(
                int i,
                int r,
                int c,
                int segmentId,
                int[,,][] reconstructionGrid,
                Dictionary<int, int> spacePartitioningSegmentAreas) {

            int[] voxelState;

            if (i < reconstructionGrid.GetLength(0)) {

                voxelState = reconstructionGrid[i, r, c];
                if (voxelState != null) {
                    if (voxelState
                            .GetVoxelClassValues(0)
                            .Contains(VoxelClassValues.FLOOR)) {

                        spacePartitioningSegmentAreas[segmentId]++;
                    }
                }
                else {
                    spacePartitioningSegmentAreas[segmentId]++;
                    reconstructionGrid[i, r, c] = VoxelState.CreateVoxelState(
                        0,
                        VoxelClassValues.FLOOR);
                }
            }
        }

        private static void ConvertWallOpeningSegmentToTransitionSpace(
                int transitionSpaceId,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                HashSet<int> adjacentRoomIds,
                HashSet<int> newTransitionSpaceIds,
                List<(int, int, int)> wallOpeningSegment,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            int i, r, c;

            newTransitionSpaceIds.Add(transitionSpaceId);
            spacePartitioningSegmentAreas.Add(transitionSpaceId, 0);
            spacePartitioningSegmentAdjacency.Add(
                transitionSpaceId,
                adjacentRoomIds.ToHashSet());

            foreach (int id in adjacentRoomIds) {
                spacePartitioningSegmentAdjacency.BucketAdd(
                    id,
                    transitionSpaceId);
            }

            foreach ((int, int, int) voxel in wallOpeningSegment) {

                i = voxel.Item1;
                r = voxel.Item2;
                c = voxel.Item3;

                spacePartitioningGrid[i, r, c] = transitionSpaceId;

                reconstructionGrid[i, r, c] = VoxelState.CreateVoxelState(
                    0,
                    normalGrid[i, r, c] == NormalGridValues.EMPTY ?
                        VoxelClassValues.EMPTY_INTERIOR :
                        VoxelClassValues.INTERIOR_OBJECT);

                UpdateRoomArea(
                    i + 1,
                    r,
                    c,
                    transitionSpaceId,
                    reconstructionGrid,
                    spacePartitioningSegmentAreas);
            }
        }

        private static void RefineNewTransitionSpaces(
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                HashSet<int> newTransitionSpaceIds) {

            Dictionary<int, int> transitionSpaceRefinementMapping;
            Dictionary<int, Dictionary<int, int>> surfaceAreas;

            surfaceAreas = GetSurfaceAreasOfTransitionSpaceContactSurfacesToAdjacentRooms(
                spacePartitioningGrid,
                newTransitionSpaceIds);

            transitionSpaceRefinementMapping = GetTransitionSpaceRefinementMapping(
                surfaceAreas);

            ApplyTransitionSpaceRefinementMapping(
                normalGrid,
                spacePartitioningGrid,
                reconstructionGrid,
                transitionSpaceRefinementMapping);
        }

        private static Dictionary<int, Dictionary<int, int>> GetSurfaceAreasOfTransitionSpaceContactSurfacesToAdjacentRooms(
                int[,,] spacePartitioningGrid,
                HashSet<int> newTransitionSpaceIds) {

            object @lock = new object();
            Dictionary<int, Dictionary<int, int>> surfaceAreas = new Dictionary<int, Dictionary<int, int>>();

            Parallel.ForEach(
                Partitioner.Create(
                    0, 
                    spacePartitioningGrid.GetLength(0)),
                () => new Dictionary<int, Dictionary<int, int>>(),
                (partition, loopState, localSurfaceAreas) => {

                    UpdateTransitionSpaceSurfaceAreasToAdjacentRooms(
                        spacePartitioningGrid,
                        partition,
                        newTransitionSpaceIds,
                        localSurfaceAreas);
                    
                    return localSurfaceAreas;
                },
                localSurfaceAreas => {
                    lock (@lock) {

                        foreach (int transitionSpaceId in localSurfaceAreas.Keys) {
                            if (!surfaceAreas.ContainsKey(transitionSpaceId)) {
                                surfaceAreas.Add(
                                    transitionSpaceId,
                                    new Dictionary<int, int>());
                            }
                            surfaceAreas[transitionSpaceId].BucketAdd(
                                localSurfaceAreas[transitionSpaceId]);
                        }
                    }
                });

            return surfaceAreas;
        }

        private static void UpdateTransitionSpaceSurfaceAreasToAdjacentRooms(
                int[,,] spacePartitioningGrid,
                Tuple<int, int> heightPartition,
                HashSet<int> newTransitionSpaceIds,
                Dictionary<int, Dictionary<int, int>> surfaceAreas) {

            int i, r, c;
            int segmentId;
            HashSet<int> roomIds;

            for (i = heightPartition.Item1; i < heightPartition.Item2; i++) {
                for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {
                    for (c = 0; c < spacePartitioningGrid.GetLength(2); c++) {

                        segmentId = spacePartitioningGrid[i, r, c];
                        if (!newTransitionSpaceIds.Contains(segmentId)) {
                            continue;
                        }

                        roomIds = GetAdjacentRoomIds(
                            i,
                            r,
                            c,
                            spacePartitioningGrid);
                        if (roomIds.Count == 0) {
                            continue;
                        }

                        if (!surfaceAreas.ContainsKey(segmentId)) {
                            surfaceAreas.Add(
                                segmentId,
                                new Dictionary<int, int>());
                        }

                        foreach (int roomId in roomIds) {
                            surfaceAreas[segmentId]
                                .BucketIncrement(roomId);
                        }
                    }
                }
            }
        }

        private static HashSet<int> GetAdjacentRoomIds(
                int i,
                int r,
                int c,
                int[,,] spacePartitioningGrid) {

            int dr, r2, dc, c2;
            int segmentId;
            HashSet<int> roomIds = new HashSet<int>();

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    if (dr.Abs() == dc.Abs()) {
                        continue;
                    }

                    r2 = r + dr;
                    c2 = c + dc;
                    if (r2 < 0 || c2 < 0
                            || r2 >= spacePartitioningGrid.GetLength(1)
                            || c2 >= spacePartitioningGrid.GetLength(2)) {
                        continue;
                    }

                    segmentId = spacePartitioningGrid[i, r2, c2];
                    if (segmentId > 0) {
                        roomIds.Add(segmentId);
                    }
                }
            }

            return roomIds;
        }

        private static Dictionary<int, int> GetTransitionSpaceRefinementMapping(
                Dictionary<int, Dictionary<int, int>> surfaceAreas) {

            Dictionary<int, int> transitionSpaceRefinementMapping = new Dictionary<int, int>();

            foreach (int transitionSpaceId in surfaceAreas.Keys) {

                if (surfaceAreas[transitionSpaceId].Count > 2) {
                    continue;
                }

                transitionSpaceRefinementMapping.Add(
                    transitionSpaceId,
                    surfaceAreas[transitionSpaceId]
                        .Keys
                        .WhereMax(roomId => surfaceAreas[transitionSpaceId][roomId])
                        .First());
            }

            return transitionSpaceRefinementMapping;
        }

        private static void ApplyTransitionSpaceRefinementMapping(
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                Dictionary<int, int> transitionSpaceRefinementMapping) {

            Parallel.For(
                0,
                spacePartitioningGrid.GetLength(0),
                i => {

                    int r, c;
                    int segmentId;

                    for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {
                        for (c = 0; c < spacePartitioningGrid.GetLength(2); c++) {

                            segmentId = spacePartitioningGrid[i, r, c];
                            if (!transitionSpaceRefinementMapping.ContainsKey(segmentId)) {
                                continue;
                            }

                            if (!NeighboursOtherRoom(
                                    i,
                                    r,
                                    c,
                                    segmentId,
                                    transitionSpaceRefinementMapping[segmentId],
                                    spacePartitioningGrid)) {

                                spacePartitioningGrid[i, r, c] = transitionSpaceRefinementMapping[segmentId];

                                reconstructionGrid[i, r, c] = VoxelState.CreateVoxelState(
                                    0,
                                    normalGrid[i, r, c] == NormalGridValues.EMPTY ?
                                        VoxelClassValues.EMPTY_INTERIOR :
                                        VoxelClassValues.INTERIOR_OBJECT);
                            }
                        }
                    }
                });
        }

        private static bool NeighboursOtherRoom(
                int i,
                int r,
                int c,
                int transitionSpaceId,
                int destinationRoomId,
                int[,,] spacePartitioningGrid) {

            int dr, r2, dc, c2;
            int segmentId;

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    if (dr.Abs() == dc.Abs()) {
                        continue;
                    }

                    r2 = r + dr;
                    c2 = c + dc;
                    if (r2 < 0 || c2 < 0
                            || r2 >= spacePartitioningGrid.GetLength(1)
                            || c2 >= spacePartitioningGrid.GetLength(2)) {
                        continue;
                    }

                    segmentId = spacePartitioningGrid[i, r2, c2];
                    if (segmentId != destinationRoomId) {
                        return true;
                    }
                }
            }
            return false;
        }

        private static void RemoveDeadEndTransitionSpacesAndSmallDeadEndRooms(
                double resolution,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            int roomMinArea = Parameters
                .ROOM_MIN_AREA
                .GetAreaInVoxels(resolution);

            HashSet<int> removeSegmentIds = spacePartitioningSegmentAdjacency
                .Keys
                .Where(id => (id < 0
                        && spacePartitioningSegmentAdjacency[id].Count == 0)
                    || (id > 0
                        && spacePartitioningSegmentAreas[id] < roomMinArea
                        && spacePartitioningSegmentAdjacency[id].Count == 0))
                .ToHashSet();

            RemoveSpacePartitioningSegments(
                normalGrid,
                spacePartitioningGrid,
                reconstructionGrid,
                removeSegmentIds,
                spacePartitioningSegmentAreas,
                spacePartitioningSegmentAdjacency);
        }

        private static void RemoveSpacePartitioningSegments(
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                HashSet<int> removeSegmentIds,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            Parallel.For(
                0,
                spacePartitioningGrid.GetLength(0),
                i => {

                    int r, c;

                    for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {
                        for (c = 0; c < spacePartitioningGrid.GetLength(2); c++) {

                            if (removeSegmentIds.Contains(
                                    spacePartitioningGrid[i, r, c])) {

                                spacePartitioningGrid[i, r, c] = 0;

                                reconstructionGrid[i, r, c] = VoxelState.CreateVoxelState(
                                    0,
                                    normalGrid[i, r, c] == NormalGridValues.EMPTY ?
                                        VoxelClassValues.EMPTY_INTERIOR :
                                        VoxelClassValues.INTERIOR_OBJECT);
                            }
                        }
                    }
                });

            RemoveSpacePartitioningSegmentIds(
                removeSegmentIds,
                null,
                spacePartitioningSegmentAreas,
                spacePartitioningSegmentAdjacency);
        }

        private static bool RefineMultiRoomTransitionSpaces(
                ref int transitionSpaceCount,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            int[,,][] roomGrowGrid;
            HashSet<int> multiRoomTransitionSpaceIds;
            Dictionary<(int, int), int> newTransitionSpaceIds;
            Dictionary<(int, int), int> newTransitionSpaceAreas;
            Dictionary<(int, int), HashSet<(int, int, int)>> newTransitionSpaceVoxels;

            multiRoomTransitionSpaceIds = GetMultiRoomTransitionSpaceIds(
                spacePartitioningSegmentAdjacency);
            
            if (multiRoomTransitionSpaceIds.Count == 0) {
                return false;
            }

            roomGrowGrid = InitializeRoomGrowGrid(
                spacePartitioningGrid,
                multiRoomTransitionSpaceIds);

            while (GrowRoomGrowGridByOneLayer(
                spacePartitioningGrid,
                roomGrowGrid,
                multiRoomTransitionSpaceIds)) {}

            ApplyRoomGrowGrid(
                normalGrid,
                spacePartitioningGrid,
                reconstructionGrid,
                roomGrowGrid,
                multiRoomTransitionSpaceIds,
                out newTransitionSpaceAreas,
                out newTransitionSpaceVoxels);

            CreateNewTransitionSpaces(
                ref transitionSpaceCount,
                spacePartitioningGrid,
                newTransitionSpaceVoxels,
                out newTransitionSpaceIds);

            UpdateSpacePartitioningSegmentAreasAndAdjacency(
                multiRoomTransitionSpaceIds,
                spacePartitioningSegmentAreas,
                newTransitionSpaceIds,
                newTransitionSpaceAreas,
                spacePartitioningSegmentAdjacency);

            return true;
        }

        private static HashSet<int> GetMultiRoomTransitionSpaceIds(
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            HashSet<int> multiRoomTransitionSpaceIds = new HashSet<int>();

            foreach (int id in spacePartitioningSegmentAdjacency.Keys) {
                if (id < 0
                        && spacePartitioningSegmentAdjacency[id].Count > 2) {
                    multiRoomTransitionSpaceIds.Add(id);
                }
            }

            return multiRoomTransitionSpaceIds;
        }

        private static int[,,][] InitializeRoomGrowGrid(
                int[,,] spacePartitioningGrid,
                HashSet<int> multiRoomTransitionSpaceIds) {

            int[,,][] roomGrowGrid = new int[
                spacePartitioningGrid.GetLength(0),
                spacePartitioningGrid.GetLength(1),
                spacePartitioningGrid.GetLength(2)][];

            Parallel.For(
                0,
                spacePartitioningGrid.GetLength(0),
                i => {

                    int dr, r, r2, dc, c, c2;
                    int segmentId;

                    for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {
                        for (c = 0; c < spacePartitioningGrid.GetLength(2); c++) {

                            segmentId = spacePartitioningGrid[i, r, c];
                            if (segmentId >= 0
                                    || !multiRoomTransitionSpaceIds.Contains(segmentId)) {
                                continue;
                            }

                            for (dr = -1; dr <= 1; dr++) {
                                for (dc = -1; dc <= 1; dc++) {

                                    if (dr.Abs() == dc.Abs()) {
                                        continue;
                                    }

                                    r2 = r + dr;
                                    c2 = c + dc;
                                    if (r2 < 0 || c2 < 0
                                            || r2 >= spacePartitioningGrid.GetLength(1)
                                            || c2 >= spacePartitioningGrid.GetLength(2)) {
                                        continue;
                                    }

                                    segmentId = spacePartitioningGrid[i, r2, c2];
                                    if (segmentId > 0) {
                                        roomGrowGrid[i, r, c] = roomGrowGrid[i, r, c]
                                            .AddRoomIdToRoomGrowVoxelState(segmentId);
                                    }
                                }
                            }
                        }
                    }
                });

            return roomGrowGrid;
        }

        private static int[] AddRoomIdToRoomGrowVoxelState(
                this int[] roomGrowVoxelState,
                int roomId) {

            int j;
            int[] result;

            if (roomGrowVoxelState == null) {
                return new int[] {
                    roomId
                };
            }

            if (roomGrowVoxelState.Contains(roomId)) {
                return roomGrowVoxelState;
            }

            result = new int[roomGrowVoxelState.Length + 1];
            for (j = 0; j < roomGrowVoxelState.Length; j++) {
                result[j] = roomGrowVoxelState[j];
            }
            result[roomGrowVoxelState.Length] = roomId;

            return result;
        }

        private static bool GrowRoomGrowGridByOneLayer(
                int[,,] spacePartitioningGrid,
                int[,,][] roomGrowGrid,
                HashSet<int> multiRoomTransitionSpaceIds) {

            bool anyChanges = false;
            bool[,,] isNew = new bool[
                spacePartitioningGrid.GetLength(0),
                spacePartitioningGrid.GetLength(1),
                spacePartitioningGrid.GetLength(2)];

            Parallel.For(
                0,
                spacePartitioningGrid.GetLength(0),
                i => {

                    int dr, r, r2, dc, c, c2;
                    int segmentId;
                    int[] roomGrowVoxelState, roomGrowVoxelState2;

                    for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {
                        for (c = 0; c < spacePartitioningGrid.GetLength(2); c++) {

                            roomGrowVoxelState = roomGrowGrid[i, r, c];
                            if (roomGrowVoxelState != null
                                    && roomGrowVoxelState.Length > 1) {
                                continue;
                            }

                            segmentId = spacePartitioningGrid[i, r, c];
                            if (segmentId >= 0
                                    || !multiRoomTransitionSpaceIds.Contains(segmentId)) {
                                continue;
                            }

                            for (dr = -1; dr <= 1; dr++) {
                                for (dc = -1; dc <= 1; dc++) {

                                    if (dr.Abs() == dc.Abs()) {
                                        continue;
                                    }

                                    r2 = r + dr;
                                    c2 = c + dc;
                                    if (r2 < 0 || c2 < 0
                                            || r2 >= spacePartitioningGrid.GetLength(1)
                                            || c2 >= spacePartitioningGrid.GetLength(2)
                                            || isNew[i, r2, c2]
                                            || spacePartitioningGrid[i, r2, c2] != segmentId) {
                                        continue;
                                    }

                                    roomGrowVoxelState2 = roomGrowGrid[i, r2, c2];
                                    if (roomGrowVoxelState2 != null
                                            && roomGrowVoxelState2.Length == 1
                                            && (roomGrowVoxelState == null
                                                || roomGrowVoxelState[0] != roomGrowVoxelState2[0])) {

                                        anyChanges = true;
                                        isNew[i, r, c] = true;
                                        roomGrowGrid[i, r, c] = roomGrowGrid[i, r, c]
                                            .AddRoomIdToRoomGrowVoxelState(roomGrowVoxelState2[0]);

                                        if (roomGrowVoxelState != null
                                                && roomGrowVoxelState.Length == 1) {

                                            isNew[i, r2, c2] = true;
                                            roomGrowGrid[i, r2, c2] = roomGrowGrid[i, r2, c2]
                                                .AddRoomIdToRoomGrowVoxelState(roomGrowVoxelState[0]);
                                        }
                                    }
                                }
                            }
                        }
                    }
                });

            return anyChanges;
        }

        private static void ApplyRoomGrowGrid(
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                int[,,][] roomGrowGrid,
                HashSet<int> multiRoomTransitionSpaceIds,
                out Dictionary<(int, int), int> newTransitionSpaceAreas,
                out Dictionary<(int, int), HashSet<(int, int, int)>> newTransitionSpaceVoxels) {

            object @lock = new object();
            Dictionary<(int, int), int> _newTransitionSpaceAreas 
                = new Dictionary<(int, int), int>();
            Dictionary<(int, int), HashSet<(int, int, int)>> _newTransitionSpaceVoxels
                = new Dictionary<(int, int), HashSet<(int, int, int)>>();

            Parallel.ForEach(
                Partitioner.Create(
                    0, 
                    spacePartitioningGrid.GetLength(2)),
                () => (
                    new Dictionary<(int, int), int>(),
                    new Dictionary<(int, int), HashSet<(int, int, int)>>()
                ),
                (partition, loopState, localData) => {

                    int i, i2, r, c;
                    int segmentId;
                    (int, int) roomContactId;
                    int[] voxelState;
                    int[] roomGrowVoxelState;

                    for (c = partition.Item1; c < partition.Item2; c++) {
                        for (i = 0; i < spacePartitioningGrid.GetLength(0); i++) {
                            for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {

                                segmentId = spacePartitioningGrid[i, r, c];
                                if (segmentId >= 0
                                        || !multiRoomTransitionSpaceIds.Contains(segmentId)) {
                                    continue;
                                }

                                roomGrowVoxelState = roomGrowGrid[i, r, c];
                                if (roomGrowVoxelState == null) {
                                    spacePartitioningGrid[i, r, c] = 0;
                                    reconstructionGrid[i, r, c] = null;
                                }
                                else if (roomGrowVoxelState.Length == 1) {

                                    spacePartitioningGrid[i, r, c] = roomGrowVoxelState[0];
                                    i2 = i + 1;

                                    if (i2 < spacePartitioningGrid.GetLength(0)) {

                                        voxelState = reconstructionGrid[i2, r, c];
                                        if (voxelState != null 
                                                && voxelState
                                                    .GetVoxelClassValues(0)
                                                    .Contains(VoxelClassValues.FLOOR)) {

                                            localData.Item1.BucketIncrement((
                                                roomGrowVoxelState[0], 
                                                0));
                                        }
                                    }
                                }
                                else if (roomGrowVoxelState.Length == 2) {

                                    roomContactId = Util.GetUnambiguousOrder(
                                        roomGrowVoxelState[0],
                                        roomGrowVoxelState[1]);
                                    localData.Item2.BucketAdd(
                                        roomContactId,
                                        (i, r, c));
                                    i2 = i + 1;

                                    if (i2 < spacePartitioningGrid.GetLength(0)) {

                                        voxelState = reconstructionGrid[i2, r, c];
                                        if (voxelState != null
                                                && voxelState
                                                    .GetVoxelClassValues(0)
                                                    .Contains(VoxelClassValues.FLOOR)) {

                                            localData.Item1.BucketIncrement(roomContactId);
                                        }
                                    }
                                }
                                else {
                                    spacePartitioningGrid[i, r, c] = 0;
                                    reconstructionGrid[i, r, c] = VoxelState.CreateVoxelState(
                                        0,
                                        normalGrid[i, r, c] == NormalGridValues.EMPTY ?
                                            VoxelClassValues.WALL_OPENING :
                                            VoxelClassValues.WALL);
                                }
                            }
                        }
                    }
                    return localData;
                },
                localData => {
                    lock (@lock) {
                        _newTransitionSpaceAreas.BucketAdd(localData.Item1);
                        _newTransitionSpaceVoxels.BucketAdd(localData.Item2);
                    }
                });

            newTransitionSpaceAreas = _newTransitionSpaceAreas;
            newTransitionSpaceVoxels = _newTransitionSpaceVoxels;
        }

        private static void CreateNewTransitionSpaces(
                ref int transitionSpaceCount,
                int[,,] spacePartitioningGrid,
                Dictionary<(int, int), HashSet<(int, int, int)>> newTransitionSpaceVoxels,
                out Dictionary<(int, int), int> newTransitionSpaceIds) {

            int newTransitionSpaceId;

            newTransitionSpaceIds = new Dictionary<(int, int), int>();

            foreach ((int, int) roomTransitionId in newTransitionSpaceVoxels.Keys) {

                newTransitionSpaceId = -(++transitionSpaceCount);
                newTransitionSpaceIds.Add(
                    roomTransitionId,
                    newTransitionSpaceId);

                foreach ((int, int, int) voxel in newTransitionSpaceVoxels[roomTransitionId]) {

                    spacePartitioningGrid[
                        voxel.Item1,
                        voxel.Item2,
                        voxel.Item3] = newTransitionSpaceId;
                }
            }
        }

        private static void UpdateSpacePartitioningSegmentAreasAndAdjacency(
                HashSet<int> multiRoomTransitionSpaceIds,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<(int, int), int> newTransitionSpaceIds,
                Dictionary<(int, int), int> newTransitionSpaceAreas,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            int newTransitionSpaceId;

            foreach ((int, int) roomTransitionId in newTransitionSpaceAreas.Keys) {
                if (roomTransitionId.Item2 == 0) {
                    spacePartitioningSegmentAreas.BucketAdd(
                        roomTransitionId.Item1,
                        newTransitionSpaceAreas[roomTransitionId]);
                }
                else {
                    spacePartitioningSegmentAreas.BucketAdd(
                        newTransitionSpaceIds[roomTransitionId],
                        newTransitionSpaceAreas[roomTransitionId]);
                }
            }

            foreach (int id in multiRoomTransitionSpaceIds) {
                spacePartitioningSegmentAreas.Remove(id);
                spacePartitioningSegmentAdjacency.Remove(id);
            }

            foreach (int id in spacePartitioningSegmentAdjacency.Keys) {
                foreach (int id2 in multiRoomTransitionSpaceIds) {
                    if (spacePartitioningSegmentAdjacency[id].Contains(id2)) {
                        spacePartitioningSegmentAdjacency[id].Remove(id2);
                    }
                }
            }

            foreach ((int, int) roomIds in newTransitionSpaceIds.Keys) {
                newTransitionSpaceId = newTransitionSpaceIds[roomIds];
                spacePartitioningSegmentAdjacency.BucketAdd(roomIds.Item1, newTransitionSpaceId);
                spacePartitioningSegmentAdjacency.BucketAdd(roomIds.Item2, newTransitionSpaceId);
                spacePartitioningSegmentAdjacency.BucketAdd(newTransitionSpaceId, roomIds.Item1);
                spacePartitioningSegmentAdjacency.BucketAdd(newTransitionSpaceId, roomIds.Item2);
            }
        }

        private static HashSet<int> GetRampTransitionSpaceIds(
                HashSet<int> rampSpaceIds,
                HashSet<int> newTransitionSpaceIds,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            return spacePartitioningSegmentAdjacency
                .Keys
                .Where(id => id < 0
                    && !newTransitionSpaceIds.Contains(id)
                    && spacePartitioningSegmentAdjacency[id].Any(adjacentId => rampSpaceIds.Contains(adjacentId)))
                .ToHashSet();
        }

        private static void RefineTransitionSpaces(
                bool isFirstCall,
                double resolution,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                HashSet<int> rampTransitionSpaceIds,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency,
                out HashSet<int> unmergeableRampTransitionSpaceIds,
                out Dictionary<int, int> rampTransitionSpaceAreas) {

            List<int> transitionSpaceIds;
            List<int> transitionSpaceIdsToMerge = new List<int>();
            Dictionary<int, List<int>> transitionSpaceRefinementAdjacentRoomHeights = new Dictionary<int, List<int>>();
            Dictionary<int, List<List<(int, int, int)>>> transitionSpaceColumns;

            unmergeableRampTransitionSpaceIds = new HashSet<int>();
            rampTransitionSpaceAreas = new Dictionary<int, int>();

            transitionSpaceColumns = InitializeTransitionSpaceColumns(
                isFirstCall,
                spacePartitioningGrid,
                rampTransitionSpaceIds);

            transitionSpaceIds = transitionSpaceColumns.Keys.ToList();

            if (isFirstCall) {

                RemoveNegligibleTransitionSpaces(
                    resolution,
                    normalGrid,
                    spacePartitioningGrid,
                    reconstructionGrid,
                    ref transitionSpaceIds,
                    rampTransitionSpaceIds,
                    unmergeableRampTransitionSpaceIds,
                    spacePartitioningSegmentAreas,
                    spacePartitioningSegmentAdjacency,
                    transitionSpaceColumns);

                UpdateRampTransitionSpaceAreas(
                    rampTransitionSpaceIds,
                    rampTransitionSpaceAreas,
                    transitionSpaceColumns);
            }

            if (transitionSpaceIds.Count > 0) {

                AnalyzeTransitionSpaceMergeability(
                    isFirstCall,
                    resolution,
                    normalGrid,
                    spacePartitioningGrid,
                    transitionSpaceIds,
                    transitionSpaceIdsToMerge,
                    rampTransitionSpaceIds,
                    unmergeableRampTransitionSpaceIds,
                    transitionSpaceRefinementAdjacentRoomHeights,
                    spacePartitioningSegmentAdjacency,
                    transitionSpaceColumns);
            }

            if (transitionSpaceIdsToMerge.Count > 0) {

                MergeTransitionSpacesWithAdjacentRooms(
                    spacePartitioningGrid,
                    transitionSpaceIdsToMerge,
                    rampTransitionSpaceIds,
                    spacePartitioningSegmentAreas,
                    spacePartitioningSegmentAdjacency);
            }

            if (isFirstCall) {

                RefineTransitionSpaceBoundariesToRooms(
                    resolution,
                    normalGrid,
                    spacePartitioningGrid,
                    reconstructionGrid,
                    spacePartitioningSegmentAreas,
                    transitionSpaceRefinementAdjacentRoomHeights,
                    transitionSpaceColumns);
            }

            RemoveDeadEndTransitionSpaces(
                spacePartitioningGrid,
                rampTransitionSpaceIds,
                spacePartitioningSegmentAreas,
                spacePartitioningSegmentAdjacency);
        }

        private static Dictionary<int, List<List<(int, int, int)>>> InitializeTransitionSpaceColumns(
                bool isFirstCall,
                int[,,] spacePartitioningGrid,
                HashSet<int> rampTransitionSpaceIds) {

            object @lock = new object();
            bool[,,] isSegmented = new bool[
                spacePartitioningGrid.GetLength(0),
                spacePartitioningGrid.GetLength(1),
                spacePartitioningGrid.GetLength(2)];
            Dictionary<int, List<List<(int, int, int)>>> transitionSpaceColumns 
                = new Dictionary<int, List<List<(int, int, int)>>>();

            Parallel.ForEach(
                Partitioner.Create(
                    0, 
                    spacePartitioningGrid.GetLength(2)),
                () => new Dictionary<int, List<List<(int, int, int)>>>(),
                (partition, loopState, localColumns) => {

                    int i, r, c;
                    int segmentId;

                    for (c = partition.Item1; c < partition.Item2; c++) {
                        for (i = 0; i < spacePartitioningGrid.GetLength(0); i++) {
                            for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {

                                if (isSegmented[i, r, c]) {
                                    continue;
                                }

                                segmentId = spacePartitioningGrid[i, r, c];
                                if (segmentId >= 0
                                        || (!isFirstCall
                                            && rampTransitionSpaceIds.Contains(segmentId))) {
                                    continue;
                                }

                                List<(int, int, int)> transitionSpaceColumn = GrowTransitionSpaceColumn(
                                    i,
                                    r,
                                    c,
                                    segmentId,
                                    isSegmented,
                                    spacePartitioningGrid);

                                localColumns.BucketAdd(
                                    segmentId,
                                    transitionSpaceColumn);
                            }
                        }
                    }
                    return localColumns;
                },
                localColumns => {
                    lock (@lock) {
                        transitionSpaceColumns.BucketAdd(localColumns);
                    }
                });

            return transitionSpaceColumns;
        }

        private static List<(int, int, int)> GrowTransitionSpaceColumn(
                int i,
                int r,
                int c,
                int transitionSpaceId,
                bool[,,] isSegmented,
                int[,,] spacePartitioningGrid) {

            List<(int, int, int)> transitionSpaceColumn = new List<(int, int, int)>();

            while (true) {

                if (i == spacePartitioningGrid.GetLength(0)
                        || spacePartitioningGrid[i, r, c] != transitionSpaceId) {
                    break;
                }

                isSegmented[i, r, c] = true;
                transitionSpaceColumn.Add((i, r, c));
                i++;
            }

            return transitionSpaceColumn;
        }

        private static void RemoveNegligibleTransitionSpaces(
                double resolution,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                ref List<int> transitionSpaceIds,
                HashSet<int> rampTransitionSpaceIds,
                HashSet<int> unmergeableRampTransitionSpaceIds,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency,
                Dictionary<int, List<List<(int, int, int)>>> transitionSpaceColumns) {

            HashSet<int> transitionSpaceIdsToRemove = new HashSet<int>();

            if (transitionSpaceIds.Count > 0) {

                RemoveTransitionSpaceColumnsWithNegligibleHeight(
                    resolution,
                    normalGrid,
                    spacePartitioningGrid,
                    reconstructionGrid,
                    transitionSpaceIds,
                    rampTransitionSpaceIds,
                    transitionSpaceIdsToRemove,
                    ref transitionSpaceColumns);

                transitionSpaceIds = transitionSpaceColumns.Keys.ToList();
            }

            if (transitionSpaceIds.Count > 0) {

                RemoveTransitionSpacesWithNegligibleSurfaceWidth(
                    resolution,
                    normalGrid,
                    spacePartitioningGrid,
                    reconstructionGrid,
                    transitionSpaceIds,
                    rampTransitionSpaceIds,
                    transitionSpaceIdsToRemove,
                    unmergeableRampTransitionSpaceIds,
                    ref transitionSpaceColumns);
            }

            RemoveSpacePartitioningSegmentIds(
                transitionSpaceIdsToRemove,
                rampTransitionSpaceIds,
                spacePartitioningSegmentAreas,
                spacePartitioningSegmentAdjacency);
        }

        private static void RemoveTransitionSpaceColumnsWithNegligibleHeight(
                double resolution,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                List<int> transitionSpaceIds,
                HashSet<int> rampTransitionSpaceIds,
                HashSet<int> transitionSpaceIdsToRemove,
                ref Dictionary<int, List<List<(int, int, int)>>> transitionSpaceColumns) {

            object @lock = new object();
            int minOpeningHeight = Parameters
                .MIN_WALL_OPENING_HEIGHT
                .GetDistanceInVoxels(resolution);
            Dictionary<int, List<List<(int, int, int)>>> _transitionSpaceColumns = transitionSpaceColumns;
            Dictionary<int, List<List<(int, int, int)>>> result
                = new Dictionary<int, List<List<(int, int, int)>>>();

            Parallel.ForEach(
                    Partitioner.Create(
                        0, 
                        transitionSpaceIds.Count),
                    () => new Dictionary<int, List<List<(int, int, int)>>>(),
                    (partition, loopState, localResult) => {

                        for (int j = partition.Item1; j < partition.Item2; j++) {
                            if (!rampTransitionSpaceIds.Contains(transitionSpaceIds[j])) {

                                RemoveTransitionSpaceColumnsWithNegligibleHeight(
                                    transitionSpaceIds[j],
                                    minOpeningHeight,
                                    normalGrid,
                                    spacePartitioningGrid,
                                    reconstructionGrid,
                                    _transitionSpaceColumns,
                                    localResult);
                            }
                        }

                        return localResult;
                    },
                    localResults => {
                        lock (@lock) {
                            result.BucketAdd(localResults);
                        }
                    });

            foreach (int rampTransitionSpaceId in rampTransitionSpaceIds) {
                if (transitionSpaceColumns.ContainsKey(rampTransitionSpaceId)) {
                    result.Add(
                        rampTransitionSpaceId,
                        transitionSpaceColumns[rampTransitionSpaceId]);
                }
            }

            foreach (int segmentId in transitionSpaceColumns.Keys) {
                if (!result.ContainsKey(segmentId)) {
                    transitionSpaceIdsToRemove.Add(segmentId);
                }
            }

            transitionSpaceColumns = result;
        }

        private static void RemoveTransitionSpaceColumnsWithNegligibleHeight(
                int transitionSpaceId,
                int minOpeningHeight,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                Dictionary<int, List<List<(int, int, int)>>> transitionSpaceColumns,
                Dictionary<int, List<List<(int, int, int)>>> result) {

            foreach (List<(int, int, int)> transitionSpaceColumn in transitionSpaceColumns[transitionSpaceId]) {

                if (transitionSpaceColumn.Count < minOpeningHeight) {

                    RemoveTransitionSpaceVoxels(
                        normalGrid,
                        spacePartitioningGrid,
                        reconstructionGrid,
                        transitionSpaceColumn);
                }
                else {
                    result.BucketAdd(
                        transitionSpaceId, 
                        transitionSpaceColumn);
                }
            }
        }

        private static void RemoveTransitionSpaceVoxels(
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                IEnumerable<(int, int, int)> voxels) {

            foreach ((int, int, int) voxel in voxels) {

                spacePartitioningGrid[
                    voxel.Item1,
                    voxel.Item2,
                    voxel.Item3] = 0;

                reconstructionGrid[
                    voxel.Item1,
                    voxel.Item2,
                    voxel.Item3] = VoxelState.CreateVoxelState(
                        0,
                        normalGrid[
                            voxel.Item1,
                            voxel.Item2,
                            voxel.Item3] == NormalGridValues.EMPTY ?
                                VoxelClassValues.WALL_OPENING :
                                VoxelClassValues.WALL);
            }
        }

        private static void RemoveTransitionSpacesWithNegligibleSurfaceWidth(
                double resolution,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                List<int> transitionSpaceIds,
                HashSet<int> rampTransitionSpaceIds,
                HashSet<int> transitionSpaceIdsToRemove,
                HashSet<int> unmergeableRampTransitionSpaceIds,
                ref Dictionary<int, List<List<(int, int, int)>>> transitionSpaceColumns) {

            object @lock = new object();
            int minSurfaceWidth = Parameters
                .TRANSITION_SPACE_REFINEMENT_MIN_SURFACE_WIDTH
                .GetDistanceInVoxels(resolution);
            Dictionary<int, List<List<(int, int, int)>>> _transitionSpaceColumns = transitionSpaceColumns;
            Dictionary<int, List<List<(int, int, int)>>> result
                = new Dictionary<int, List<List<(int, int, int)>>>();

            Parallel.ForEach(
                Partitioner.Create(
                    0, 
                    transitionSpaceIds.Count),
                () => (
                    new Dictionary<int, List<List<(int, int, int)>>>(),
                    new HashSet<int>(),
                    new HashSet<int>()
                ),
                (partition, loopState, localResults) => {

                    for (int j = partition.Item1; j < partition.Item2; j++) {
                        RemoveTransitionSpacesWithNegligibleSurfaceWidth(
                            transitionSpaceIds[j],
                            minSurfaceWidth,
                            normalGrid,
                            spacePartitioningGrid,
                            reconstructionGrid,
                            rampTransitionSpaceIds,
                            localResults.Item2,
                            localResults.Item3,
                            _transitionSpaceColumns,
                            localResults.Item1);
                    }

                    return localResults;
                },
                localResults => {
                    lock (@lock) {
                        result.BucketAdd(localResults.Item1);
                        transitionSpaceIdsToRemove.AddRange(localResults.Item2);
                        unmergeableRampTransitionSpaceIds.AddRange(localResults.Item3);
                    }
                });
        }

        private static void RemoveTransitionSpacesWithNegligibleSurfaceWidth(
                int transitionSpaceId,
                int minSurfaceWidth,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                HashSet<int> rampTransitionSpaceIds,
                HashSet<int> transitionSpaceIdsToRemove,
                HashSet<int> unmergeableRampTransitionSpaceIds,
                Dictionary<int, List<List<(int, int, int)>>> transitionSpaceColumns,
                Dictionary<int, List<List<(int, int, int)>>> result) {

            Dictionary<int, HashSet<(int, int, int)>> transitionSpaceSurfaces = GetTransitionSpaceSurfaces(
                spacePartitioningGrid,
                transitionSpaceColumns[transitionSpaceId]);
            
            if (GetTransitionSpaceSurfaceCountWithProperWidth(
                    minSurfaceWidth,
                    transitionSpaceSurfaces) >= 2) {

                result.Add(
                    transitionSpaceId,
                    transitionSpaceColumns[transitionSpaceId]);
            }
            else {

                if (rampTransitionSpaceIds.Contains(transitionSpaceId)) {

                    result.Add(
                        transitionSpaceId,
                        transitionSpaceColumns[transitionSpaceId]);

                    unmergeableRampTransitionSpaceIds.Add(transitionSpaceId);

                    return;
                }

                transitionSpaceIdsToRemove.Add(transitionSpaceId);

                RemoveTransitionSpaceVoxels(
                    normalGrid,
                    spacePartitioningGrid,
                    reconstructionGrid,
                    transitionSpaceColumns[transitionSpaceId]
                        .SelectMany(column => column));
            }
        }

        private static Dictionary<int, HashSet<(int, int, int)>> GetTransitionSpaceSurfaces(
                int[,,] spacePartitioningGrid,
                List<List<(int, int, int)>> transitionSpaceColumns) {

            int dr, r, dc, c;
            int segmentId;
            Dictionary<int, HashSet<(int, int, int)>>  transitionSpaceSurfaces 
                = new Dictionary<int, HashSet<(int, int, int)>>();

            foreach ((int, int, int) voxel in transitionSpaceColumns.SelectMany(column => column)) {

                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        r = voxel.Item2 + dr;
                        c = voxel.Item3 + dc;
                        if (r < 0 || c < 0
                                || r >= spacePartitioningGrid.GetLength(1)
                                || c >= spacePartitioningGrid.GetLength(2)) {
                            continue;
                        }

                        segmentId = spacePartitioningGrid[voxel.Item1, r, c];
                        if (segmentId > 0) {
                            transitionSpaceSurfaces.BucketAdd(
                                segmentId,
                                voxel);
                        }
                    }
                }
            }

            return transitionSpaceSurfaces;
        }

        private static int GetTransitionSpaceSurfaceCountWithProperWidth(
                int minSurfaceWidth,
                Dictionary<int, HashSet<(int, int, int)>> transitionSpaceSurfaces) {

            return transitionSpaceSurfaces
                .Values
                .Where(surface => surface
                    .Select(voxel => (voxel.Item2, voxel.Item3))
                    .Distinct()
                    .Count() >= minSurfaceWidth)
                .Count();
        }

        private static void UpdateRampTransitionSpaceAreas(
                HashSet<int> rampTransitionSpaceIds,
                Dictionary<int, int> rampTransitionSpaceAreas,
                Dictionary<int, List<List<(int, int, int)>>> transitionSpaceColumns) {

            foreach (int rampTransitionSpaceId in rampTransitionSpaceIds) {
                rampTransitionSpaceAreas.Add(
                    rampTransitionSpaceId,
                    transitionSpaceColumns.ContainsKey(rampTransitionSpaceId) ? 
                        transitionSpaceColumns[rampTransitionSpaceId]
                            .Select(column => column.Count)
                            .Sum() :
                        0);
            }
        }

        private static List<int> AnalyzeTransitionSpaceMergeability(
                bool isFirstCall,
                double resolution,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                List<int> transitionSpaceIds,
                List<int> transitionSpaceIdsToMerge,
                HashSet<int> rampTransitionSpaceIds,
                HashSet<int> unmergeableRampTransitionSpaceIds,
                Dictionary<int, List<int>> transitionSpaceRefinementAdjacentRoomHeights,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency,
                Dictionary<int, List<List<(int, int, int)>>> transitionSpaceColumns) {

            int searchDistance = Parameters
                .TRANSITION_SPACE_REFINEMENT_SEARCH_DISTANCE
                .GetDistanceInVoxels(resolution);
            int searchDistanceDiagonal = Parameters
                .TRANSITION_SPACE_REFINEMENT_SEARCH_DISTANCE
                .GetDistanceInVoxels(
                    resolution.GetVoxelSizeDiagonal());
            int transitionSpaceMinHeight = Parameters
                .TRANSITION_SPACE_MIN_HEIGHT
                .GetDistanceInVoxels(resolution);
            int verticalResolution = Parameters
                .TRANSITION_SPACE_REFINEMENT_VERTICAL_RESOLUTION
                .GetDistanceInVoxels(resolution);
            int sureMergeWidth = Parameters
                .TRANSITION_SPACE_REFINEMENT_SURE_MERGE_MIN_WIDTH
                .GetDistanceInVoxels(resolution);
            object @lock = new object();

            Parallel.ForEach(
                    Partitioner.Create(
                        0,
                        transitionSpaceIds.Count),
                    () => (
                        new List<int>(),
                        new Dictionary<int, List<int>>(),
                        new HashSet<int>()
                    ),
                    (partition, loopState, localResults) => {

                        for (int j = partition.Item1; j < partition.Item2; j++) {
                            AnalyzeTransitionSpaceMergeability(
                                isFirstCall,
                                transitionSpaceIds[j],
                                searchDistance,
                                searchDistanceDiagonal,
                                transitionSpaceMinHeight,
                                verticalResolution,
                                sureMergeWidth,
                                normalGrid,
                                spacePartitioningGrid,
                                localResults.Item1,
                                rampTransitionSpaceIds,
                                localResults.Item3,
                                localResults.Item2,
                                spacePartitioningSegmentAdjacency,
                                transitionSpaceColumns);
                        }

                        return localResults;
                    },
                    localResults => {
                        lock (@lock) {
                            transitionSpaceIdsToMerge.AddRange(localResults.Item1);
                            transitionSpaceRefinementAdjacentRoomHeights.BucketAdd(localResults.Item2);
                            if (isFirstCall) {
                                unmergeableRampTransitionSpaceIds.AddRange(localResults.Item3);
                            }
                        }
                    });

            return transitionSpaceIdsToMerge;
        }

        private static void AnalyzeTransitionSpaceMergeability(
                bool isFirstCall,
                int transitionSpaceId,
                int searchDistance,
                int searchDistanceDiagonal,
                int transitionSpaceMinHeight,
                int verticalResolution,
                int sureMergeWidth,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                List<int> transitionSpaceIdsToMerge,
                HashSet<int> rampTransitionSpaceIds,
                HashSet<int> unmergeableRampTransitionSpaceIds,
                Dictionary<int, List<int>> transitionSpaceRefinementAdjacentRoomHeights,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency,
                Dictionary<int, List<List<(int, int, int)>>> transitionSpaceColumns) {

            List<int> adjacentRoomHeights;

            adjacentRoomHeights = GetAdjacentRoomHeights(
                    searchDistance,
                    searchDistanceDiagonal,
                    spacePartitioningGrid,
                    transitionSpaceColumns[transitionSpaceId]);

            if (DoesTransitionSpaceHaveProperHeight(
                    transitionSpaceMinHeight,
                    verticalResolution,
                    normalGrid,
                    adjacentRoomHeights,
                    transitionSpaceColumns[transitionSpaceId])) {

                if (isFirstCall
                        && rampTransitionSpaceIds.Contains(transitionSpaceId)) {
                    unmergeableRampTransitionSpaceIds.Add(transitionSpaceId);
                }
                else {
                    transitionSpaceRefinementAdjacentRoomHeights.Add(
                        transitionSpaceId,
                        adjacentRoomHeights);
                }
            }
            else if (!rampTransitionSpaceIds.Contains(transitionSpaceId)
                    && CanMergeTransitionSpaceWithAdjacentRooms(
                        transitionSpaceId,
                        sureMergeWidth,
                        spacePartitioningGrid,
                        transitionSpaceColumns[transitionSpaceId],
                        spacePartitioningSegmentAdjacency)) {

                transitionSpaceIdsToMerge.Add(transitionSpaceId);
            }
        }

        private static List<int> GetAdjacentRoomHeights(
                int searchDistance,
                int searchDistanceDiagonal,
                int[,,] spacePartitioningGrid,
                List<List<(int, int, int)>> transitionSpaceColumns) {

            (int, int, int) transitionSpaceColumnCenterVoxel;
            Dictionary<int, int> neighbourCounts;
            Dictionary<int, HashSet<(int, int)>> directions;
            Dictionary<int, Dictionary<(int, int), int>> directionCounts;
            Dictionary<int, List<int>> adjacentRoomHeights = new Dictionary<int, List<int>>();

            foreach (List<(int, int, int)> transitionSpaceColumn in transitionSpaceColumns) {

                transitionSpaceColumnCenterVoxel = GetTransitionSpaceColumnCenterVoxel(transitionSpaceColumn);

                AnalyzeTransitionSpaceColumnAdjacency(
                    spacePartitioningGrid,
                    transitionSpaceColumn,
                    out neighbourCounts,
                    out directions,
                    out directionCounts);

                UpdateAdjacentRoomHeights(
                    searchDistance,
                    searchDistanceDiagonal,
                    transitionSpaceColumnCenterVoxel,
                    spacePartitioningGrid,
                    neighbourCounts,
                    directions,
                    directionCounts,
                    adjacentRoomHeights);
            }

            return adjacentRoomHeights
                .Values
                .Select(heights => heights.Median())
                .ToList();
        }

        private static (int, int, int) GetTransitionSpaceColumnCenterVoxel(
                List<(int, int, int)> transitionSpaceColumn) {

            return (
                transitionSpaceColumn
                    .Select(voxel => voxel.Item1)
                    .Median(),
                transitionSpaceColumn[0].Item2,
                transitionSpaceColumn[0].Item3);
        }

        private static void AnalyzeTransitionSpaceColumnAdjacency(
                int[,,] spacePartitioningGrid,
                List<(int, int, int)> transitionSpaceColumn,
                out Dictionary<int, int> neighbourCounts,
                out Dictionary<int, HashSet<(int, int)>> directions,
                out Dictionary<int, Dictionary<(int, int), int>> directionCounts) {

            int dr, r, dc, c;
            int segmentId;

            neighbourCounts = new Dictionary<int, int>();
            directions = new Dictionary<int, HashSet<(int, int)>>();
            directionCounts = new Dictionary<int, Dictionary<(int, int), int>>();

            foreach ((int, int, int) voxel in transitionSpaceColumn) {
                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (dr.Abs() == dc.Abs()) {
                            continue;
                        }

                        r = voxel.Item2 + dr;
                        c = voxel.Item3 + dc;
                        if (r < 0 || c < 0
                                || r >= spacePartitioningGrid.GetLength(1)
                                || c >= spacePartitioningGrid.GetLength(2)) {
                            continue;
                        }

                        segmentId = spacePartitioningGrid[voxel.Item1, r, c];
                        if (segmentId <= 0) {
                            continue;
                        }

                        neighbourCounts.BucketIncrement(segmentId);
                        directions.BucketAdd(
                            segmentId,
                            (dr, dc));
                        if (!directionCounts.ContainsKey(segmentId)) {
                            directionCounts.Add(
                                segmentId,
                                new Dictionary<(int, int), int>());
                        }
                        directionCounts[segmentId].BucketIncrement((dr, dc));
                    }
                }
            }
        }

        private static void UpdateAdjacentRoomHeights(
                int searchDistance,
                int searchDistanceDiagonal,
                (int, int, int) transitionSpaceColumnCenterVoxel,
                int[,,] spacePartitioningGrid,
                Dictionary<int, int> neighbourCounts,
                Dictionary<int, HashSet<(int, int)>> directions,
                Dictionary<int, Dictionary<(int, int), int>> directionCounts,
                Dictionary<int, List<int>> adjacentRoomHeights) {

            int i, r, c, d;
            (int, int) mainDirection;

            foreach (int adjacentRoomId in directions.Keys) {

                mainDirection = GetMainDirection(
                    adjacentRoomId,
                    neighbourCounts,
                    directions,
                    directionCounts);

                if (mainDirection == (0, 0)) {
                    continue;
                }

                d = mainDirection.IsDirectionDiagonal() ?
                    searchDistanceDiagonal :
                    searchDistance;
                r = transitionSpaceColumnCenterVoxel.Item2 + mainDirection.Item1 * d;
                c = transitionSpaceColumnCenterVoxel.Item3 + mainDirection.Item2 * d;

                if (r < 0 || c < 0
                        || r >= spacePartitioningGrid.GetLength(1)
                        || c >= spacePartitioningGrid.GetLength(2)
                        || spacePartitioningGrid[transitionSpaceColumnCenterVoxel.Item1, r, c] <= 0) {
                    continue;
                }

                i = transitionSpaceColumnCenterVoxel.Item1;
                while (true) {
                    if (i < 0
                            || spacePartitioningGrid[i, r, c] <= 0) {
                        break;
                    }
                    i--;
                }

                adjacentRoomHeights.BucketAdd(adjacentRoomId, i + 1);
            }
        }

        private static (int, int) GetMainDirection(
                int adjacentRoomId,
                Dictionary<int, int> neighbourCounts,
                Dictionary<int, HashSet<(int, int)>> directions,
                Dictionary<int, Dictionary<(int, int), int>> directionCounts) {

            (int, int) mainDirection = (0, 0);

            foreach ((int, int) direction in directions[adjacentRoomId]
                    .Where(direction => (double)directionCounts[adjacentRoomId][direction] / neighbourCounts[adjacentRoomId]
                        >= Parameters.TRANSITION_SPACE_REFINEMENT_MIN_DIRECTION_RATIO)) {
                mainDirection.Item1 += direction.Item1;
                mainDirection.Item2 += direction.Item2;
            }

            return mainDirection;
        }

        private static bool DoesTransitionSpaceHaveProperHeight(
                int transitionSpaceMinHeight,
                int verticalResolution,
                byte[,,] normalGrid,
                List<int> adjacentRoomHeights,
                List<List<(int, int, int)>> transitionSpaceColumns) {

            List<int> largeTransitionSpaceHeightClusterIndices;
            List<int> transitionSpaceHeightClusterCenters;
            List<List<int>> transitionSpaceHeightClusters;

            GetTransitionSpaceHeightClusters(
                verticalResolution,
                normalGrid,
                transitionSpaceColumns,
                out transitionSpaceHeightClusterCenters,
                out transitionSpaceHeightClusters);

            largeTransitionSpaceHeightClusterIndices = GetLargeTransitionSpaceHeightClusterIndices(
                transitionSpaceMinHeight,
                verticalResolution,
                adjacentRoomHeights,
                transitionSpaceHeightClusterCenters,
                transitionSpaceHeightClusters,
                transitionSpaceColumns);

            return largeTransitionSpaceHeightClusterIndices.Count > 0;
        }

        private static void GetTransitionSpaceHeightClusters(
                int verticalResolution,
                byte[,,] normalGrid,
                List<List<(int, int, int)>> transitionSpaceColumns,
                out List<int> transitionSpaceHeightClusterCenters,
                out List<List<int>> transitionSpaceHeightClusters) {

            int i, minDistanceIndex;
            (int, int, int) voxel;

            transitionSpaceHeightClusterCenters = new List<int>();
            transitionSpaceHeightClusters = new List<List<int>>();

            foreach (List<(int, int, int)> column in transitionSpaceColumns) {

                voxel = column.First();
                i = voxel.Item1;

                if (i <= 0
                        || normalGrid[
                            i - 1,
                            voxel.Item2,
                            voxel.Item3] == NormalGridValues.EMPTY) {
                    continue;
                }

                if (transitionSpaceHeightClusterCenters.Count == 0) {
                    transitionSpaceHeightClusters.Add(new List<int> {
                        i
                    });
                    transitionSpaceHeightClusterCenters.Add(i);
                    continue;
                }

                minDistanceIndex = GetTransitionSpaceHeightClusterMinDistanceIndex(
                    i,
                    transitionSpaceHeightClusterCenters);

                if ((transitionSpaceHeightClusterCenters[minDistanceIndex] - i).Abs() >= verticalResolution) {
                    transitionSpaceHeightClusters.Add(new List<int> {
                        i
                    });
                    transitionSpaceHeightClusterCenters.Add(i);
                }
                else {
                    transitionSpaceHeightClusters[minDistanceIndex].Add(i);
                    transitionSpaceHeightClusterCenters[minDistanceIndex] = transitionSpaceHeightClusters[minDistanceIndex].Median();
                }
            }
        }

        private static int GetTransitionSpaceHeightClusterMinDistanceIndex(
                int i,
                List<int> transitionSpaceHeightClusterCenters) {

            return Enumerable
                .Range(0, transitionSpaceHeightClusterCenters.Count)
                .WhereMin(j => (transitionSpaceHeightClusterCenters[j] - i).Abs())
                .First();
        }

        private static List<int> GetLargeTransitionSpaceHeightClusterIndices(
                int transitionSpaceMinHeight,
                int verticalResolution,
                List<int> adjacentRoomHeights,
                List<int> transitionSpaceHeightClusterCenters,
                List<List<int>> transitionSpaceHeightClusters,
                List<List<(int, int, int)>> transitionSpaceColumns) {

            int floorLevel = transitionSpaceColumns
                .SelectMany(column => column)
                .Max(voxel => voxel.Item1);

            return Enumerable
                .Range(0, transitionSpaceHeightClusters.Count)
                .Where(j => (double)transitionSpaceHeightClusters[j].Count / transitionSpaceColumns.Count
                        >= Parameters.TRANSITION_SPACE_REFINEMENT_HEIGHT_CLUSTER_RATIO_THRESHOLD
                    && floorLevel - transitionSpaceHeightClusterCenters[j] >= transitionSpaceMinHeight
                    && adjacentRoomHeights.All(adjacentRoomHeight =>
                        transitionSpaceHeightClusterCenters[j] - adjacentRoomHeight >= verticalResolution))
                .ToList();
        }

        private static bool CanMergeTransitionSpaceWithAdjacentRooms(
                int transitionSpaceId,
                int sureMergeWidth,
                int[,,] spacePartitioningGrid,
                List<List<(int, int, int)>> transitionSpaceColumns,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            int searchDistance;
            (int, int) minTransitionSpacePosition;
            (int, int) maxTransitionSpacePosition;
            (int, int, int) voxel;
            (int, int, int) centerVoxel;
            List<(int, int)> directionsToAdjacentRooms;
            Dictionary<int, HashSet<(int, int)>> contactSurfacePositionsToAdjacentRooms;

            centerVoxel = GetTransitionSpaceCenterVoxel(transitionSpaceColumns);

            GetTransitionSpaceConnectionsToAdjacentRooms(
                transitionSpaceId,
                spacePartitioningGrid,
                transitionSpaceColumns,
                spacePartitioningSegmentAdjacency,
                out directionsToAdjacentRooms,
                out contactSurfacePositionsToAdjacentRooms);

            if (contactSurfacePositionsToAdjacentRooms
                    .Keys
                    .All(adjacentRoomId => contactSurfacePositionsToAdjacentRooms[adjacentRoomId].Count >= sureMergeWidth)) {
                return true;
            }

            if (directionsToAdjacentRooms.Count < 2) {
                return false;
            }

            GetHorizontalTransitionSpaceExtent(
                transitionSpaceColumns,
                out minTransitionSpacePosition,
                out maxTransitionSpacePosition);

            foreach ((int, int) direction in directionsToAdjacentRooms) {

                searchDistance = GetSearchDistanceForDeterminingTransitionSpaceNeighbourRoomWidth(
                    transitionSpaceId,
                    direction,
                    centerVoxel,
                    spacePartitioningGrid);

                voxel = (
                    centerVoxel.Item1,
                    centerVoxel.Item2 + searchDistance * direction.Item1,
                    centerVoxel.Item3 + searchDistance * direction.Item2);

                if ((double)GetSegmentWidthWidth(
                            true,
                            transitionSpaceId,
                            direction,
                            minTransitionSpacePosition,
                            maxTransitionSpacePosition,
                            centerVoxel,
                            spacePartitioningGrid)
                        / GetSegmentWidthWidth(
                                false,
                                transitionSpaceId,
                                direction,
                                minTransitionSpacePosition,
                                maxTransitionSpacePosition,
                                voxel,
                                spacePartitioningGrid)
                            < Parameters.TRANSITION_SPACE_REFINEMENT_MERGE_WIDTH_RATIO) {
                    return false;
                }
            }
            return true;
        }

        private static (int, int, int) GetTransitionSpaceCenterVoxel(
                List<List<(int, int, int)>> transitionSpaceColumns) {

            List<(int, int, int)> transitionSpaceVoxels = transitionSpaceColumns
                .SelectMany(column => column)
                .ToList();

            return (
                transitionSpaceVoxels.Select(voxel => voxel.Item1).Median(),
                transitionSpaceVoxels.Select(voxel => voxel.Item2).Median(),
                transitionSpaceVoxels.Select(voxel => voxel.Item3).Median());
        }

        private static void GetTransitionSpaceConnectionsToAdjacentRooms(
                int transitionSpaceId,
                int[,,] spacePartitioningGrid,
                List<List<(int, int, int)>> transitionSpaceColumns,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency,
                out List<(int, int)> directionsToAdjacentRooms,
                out Dictionary<int, HashSet<(int, int)>> contactSurfacePositionsToAdjacentRooms) {

            Dictionary<int, List<(int, int)>> directions;
            Dictionary<int, Dictionary<(int, int), int>> directionCounts
                = new Dictionary<int, Dictionary<(int, int), int>>();

            contactSurfacePositionsToAdjacentRooms = new Dictionary<int, HashSet<(int, int)>>();

            foreach (List<(int, int, int)> column in transitionSpaceColumns) {
                foreach ((int, int, int) voxel in column) {

                    directions = GetTransitionSpaceDirectionsToAdjacentRooms(
                        transitionSpaceId,
                        voxel,
                        spacePartitioningGrid,
                        spacePartitioningSegmentAdjacency,
                        contactSurfacePositionsToAdjacentRooms);

                    UpdateTransitionSpaceDirectionToOtherRoomCounts(
                        directions,
                        directionCounts);
                }
            }

            directionsToAdjacentRooms = directionCounts
                .Keys
                .Select(adjacentRoomId => directionCounts[adjacentRoomId]
                    .Keys
                    .WhereMax(direction => directionCounts[adjacentRoomId][direction])
                    .First())
                .Distinct()
                .ToList();
        }

        private static Dictionary<int, List<(int, int)>> GetTransitionSpaceDirectionsToAdjacentRooms(
                int transitionSpaceId,
                (int, int, int) voxel,
                int[,,] spacePartitioningGrid,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency,
                Dictionary<int, HashSet<(int, int)>> contactSurfacePositionsToAdjacentRooms) {

            int dr, r, dc, c;
            int segmentId;
            Dictionary<int, List<(int, int)>> directions = new Dictionary<int, List<(int, int)>>();

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    if (dr.Abs() == dc.Abs()) {
                        continue;
                    }

                    r = voxel.Item2 + dr;
                    c = voxel.Item3 + dc;
                    if (r < 0 || c < 0
                            || r >= spacePartitioningGrid.GetLength(1)
                            || c >= spacePartitioningGrid.GetLength(2)) {
                        continue;
                    }

                    segmentId = spacePartitioningGrid[voxel.Item1, r, c];

                    if (segmentId > 0
                            && spacePartitioningSegmentAdjacency.ContainsKey(transitionSpaceId)
                            && spacePartitioningSegmentAdjacency[transitionSpaceId].Contains(segmentId)) {

                        contactSurfacePositionsToAdjacentRooms.BucketAdd(
                            segmentId,
                            (voxel.Item2, voxel.Item3));
                        directions.BucketAdd(
                            segmentId,
                            (dr, dc));
                    }
                }
            }

            return directions;
        }

        private static void UpdateTransitionSpaceDirectionToOtherRoomCounts(
                Dictionary<int, List<(int, int)>> directionsToAdjacentRooms,
                Dictionary<int, Dictionary<(int, int), int>> directionCounts) {

            (int, int) mainDirection;

            foreach (int adjacentRoomId in directionsToAdjacentRooms.Keys) {

                mainDirection = (0, 0);

                foreach ((int, int) direction in directionsToAdjacentRooms[adjacentRoomId]) {
                    mainDirection.Item1 += direction.Item1;
                    mainDirection.Item2 += direction.Item2;
                }

                if (mainDirection != (0, 0)) {

                    if (!directionCounts.ContainsKey(adjacentRoomId)) {
                        directionCounts.Add(
                            adjacentRoomId,
                            new Dictionary<(int, int), int>());
                    }

                    directionCounts[adjacentRoomId].BucketIncrement(mainDirection);
                }
            }
        }

        private static void GetHorizontalTransitionSpaceExtent(
                List<List<(int, int, int)>> transitionSpaceColumns,
                out (int, int) minTransitionSpacePosition,
                out (int, int) maxTransitionSpacePosition) {

            minTransitionSpacePosition = (int.MaxValue, int.MaxValue);
            maxTransitionSpacePosition = (int.MinValue, int.MinValue);

            foreach ((int, int) position in transitionSpaceColumns
                    .Select(column => column.First())
                    .Select(voxel => (voxel.Item2, voxel.Item3))) {

                if (position.Item1 < minTransitionSpacePosition.Item1) {
                    minTransitionSpacePosition.Item1 = position.Item1;
                }
                if (position.Item2 < minTransitionSpacePosition.Item2) {
                    minTransitionSpacePosition.Item2 = position.Item2;
                }
                if (position.Item1 > maxTransitionSpacePosition.Item1) {
                    maxTransitionSpacePosition.Item1 = position.Item1;
                }
                if (position.Item2 > maxTransitionSpacePosition.Item2) {
                    maxTransitionSpacePosition.Item2 = position.Item2;
                }
            }
        }

        private static int GetSearchDistanceForDeterminingTransitionSpaceNeighbourRoomWidth(
                    int transitionSpaceId,
                    (int, int) directionToAdjacentRoom,
                    (int, int, int) transitionSpaceCenterVoxel,
                    int[,,] spacePartitioningGrid) {

            int r, c, d = 1;
            int segmentId;

            while (true) {

                r = transitionSpaceCenterVoxel.Item2 + d * directionToAdjacentRoom.Item1;
                c = transitionSpaceCenterVoxel.Item3 + d * directionToAdjacentRoom.Item2;

                if (r < 0 || c < 0
                        || r >= spacePartitioningGrid.GetLength(1)
                        || c >= spacePartitioningGrid.GetLength(2)) {
                    break;
                }

                segmentId = spacePartitioningGrid[
                    transitionSpaceCenterVoxel.Item1,
                    r,
                    c];

                if (segmentId == 0
                        || (segmentId < 0
                            && segmentId != transitionSpaceId)) {
                    break;
                }

                d++;
            }

            return d / 2;
        }

        private static int GetSegmentWidthWidth(
                bool isTransitionSpace,
                int segmentId,
                (int, int) direction,
                (int, int) transitionSpaceMinPosition,
                (int, int) transitionSpaceMaxPosition,
                (int, int, int) voxel,
                int[,,] spacePartitioningGrid) {

            bool found;
            int j, r, r2, c, c2, d, d2, segmentId2;
            int width = 1;

            foreach ((int, int) perpendicularDirection in direction.GetPerpendicularDirections()) {
                
                d = 1;

                while (true) {

                    r = voxel.Item2 + d * perpendicularDirection.Item1;
                    c = voxel.Item3 + d * perpendicularDirection.Item2;
                    if (r < 0 || c < 0
                            || r >= spacePartitioningGrid.GetLength(1)
                            || c >= spacePartitioningGrid.GetLength(2)) {
                        break;
                    }

                    segmentId2 = spacePartitioningGrid[voxel.Item1, r, c];
                    if (isTransitionSpace
                            && (r < transitionSpaceMinPosition.Item1
                                    || c < transitionSpaceMinPosition.Item2
                                    || r > transitionSpaceMaxPosition.Item1
                                    || c > transitionSpaceMaxPosition.Item2)) {
                        break;
                    }

                    if (segmentId2 <= 0 && segmentId2 != segmentId) {

                        if (!isTransitionSpace) {
                            break;
                        }

                        found = false;

                        for (j = -1; j <= 1; j += 2) {

                            d2 = 1;

                            while (true) {

                                r2 = r + j * d2 * direction.Item1;
                                c2 = c + j * d2 * direction.Item2;
                                if (r2 < 0 || c2 < 0
                                        || r2 >= spacePartitioningGrid.GetLength(1)
                                        || c2 >= spacePartitioningGrid.GetLength(2)) {
                                    break;
                                }

                                segmentId2 = spacePartitioningGrid[voxel.Item1, r2, c2];
                                if (segmentId2 == segmentId) {
                                    found = true;
                                    break;
                                }

                                if (segmentId2 < 0 && segmentId2 != segmentId) {
                                    break;
                                }

                                d2++;
                            }
                        }

                        if (!found) {
                            break;
                        }
                    }

                    d++;
                    width++;
                }
            }

            return width;
        }

        private static void MergeTransitionSpacesWithAdjacentRooms(
                int[,,] spacePartitioningGrid,
                List<int> transitionSpaceIdsToMerge,
                HashSet<int> rampTransitionSpaceIds,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            HashSet<(int, int)> mergeIds = new HashSet<(int, int)>();

            foreach (int transitionSpaceId in transitionSpaceIdsToMerge) {

                if (!spacePartitioningSegmentAdjacency.ContainsKey(transitionSpaceId)) {
                    continue;
                }

                foreach (int adjacentRoomId in spacePartitioningSegmentAdjacency[transitionSpaceId]) {
                    mergeIds.Add((transitionSpaceId, adjacentRoomId));
                }
            }

            MergeSpacePartitioningSegments(
                spacePartitioningGrid,
                rampTransitionSpaceIds,
                Util.GetMergeMapping(
                    mergeIds,
                    cluster => cluster
                        .Where(id => id > 0)
                        .First()),
                spacePartitioningSegmentAreas,
                spacePartitioningSegmentAdjacency);
        }

        private static void RefineTransitionSpaceBoundariesToRooms(
                double resolution,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<int, List<int>> transitionSpaceRefinementAdjacentRoomHeights,
                Dictionary<int, List<List<(int, int, int)>>> transitionSpaceColumns) {

            int verticalResolution = Parameters
                .TRANSITION_SPACE_REFINEMENT_VERTICAL_RESOLUTION
                .GetDistanceInVoxels(resolution);
            bool[,,] transitionSpaceReclaimableForRoomSpace;
            List<int> transitionSpaceIds;
            Dictionary<int, List<(int, int, int)>> transitionSpaceGrowVoxels;

            transitionSpaceIds = transitionSpaceRefinementAdjacentRoomHeights.Keys.ToList();

            transitionSpaceReclaimableForRoomSpace = DetectTransitionSpaceReclaimableForRoomSpace(
                verticalResolution,
                spacePartitioningGrid,
                transitionSpaceIds,
                transitionSpaceRefinementAdjacentRoomHeights,
                transitionSpaceColumns);

            transitionSpaceGrowVoxels = DetectTransitionSpaceGrowVoxels(
                verticalResolution,
                spacePartitioningGrid,
                transitionSpaceRefinementAdjacentRoomHeights);

            GrowTransitionSpaces(
                resolution,
                spacePartitioningGrid,
                reconstructionGrid,
                spacePartitioningSegmentAreas,
                transitionSpaceGrowVoxels);

            ReclaimTransitionSpaceForRoomSpace(
                transitionSpaceReclaimableForRoomSpace,
                normalGrid,
                spacePartitioningGrid,
                reconstructionGrid);
        }

        private static bool[,,] DetectTransitionSpaceReclaimableForRoomSpace(
                int verticalResolution,
                int[,,] spacePartitioningGrid,
                List<int> transitionSpaceIds,
                Dictionary<int, List<int>> transitionSpaceRefinementAdjacentRoomHeights,
                Dictionary<int, List<List<(int, int, int)>>> transitionSpaceColumns) {

            bool[,,] transitionSpaceReclaimableForRoomSpace = new bool[
                spacePartitioningGrid.GetLength(0),
                spacePartitioningGrid.GetLength(1),
                spacePartitioningGrid.GetLength(2)];

            Parallel.For(
                0,
                transitionSpaceIds.Count,
                j => {
                    foreach (List<(int, int, int)> column in transitionSpaceColumns[transitionSpaceIds[j]]) {

                        if (transitionSpaceRefinementAdjacentRoomHeights[transitionSpaceIds[j]]
                                .Any(height => (height - column.First().Item1).Abs() < verticalResolution)) {

                            foreach ((int, int, int) voxel in column) {

                                spacePartitioningGrid[
                                    voxel.Item1,
                                    voxel.Item2,
                                    voxel.Item3] = 0;
                                transitionSpaceReclaimableForRoomSpace[
                                    voxel.Item1,
                                    voxel.Item2,
                                    voxel.Item3] = true;
                            }
                        }
                    }
                });

            return transitionSpaceReclaimableForRoomSpace;
        }

        private static Dictionary<int, List<(int, int, int)>> DetectTransitionSpaceGrowVoxels(
                int verticalResolution,
                int[,,] spacePartitioningGrid,
                Dictionary<int, List<int>> transitionSpaceRefinementAdjacentRoomHeights) {
            
            int i, i2, i3;
            int dr, dr2, r, r2, r3;
            int dc, dc2, c, c2, c3;
            int segmentId;
            (int, int, int) candidate;
            bool[,,] isProcessed = new bool[
                spacePartitioningGrid.GetLength(0),
                spacePartitioningGrid.GetLength(1),
                spacePartitioningGrid.GetLength(2)];
            Queue<(int, int, int)> candidates = new Queue<(int, int, int)>();
            Dictionary<int, List<(int, int, int)>> transitionSpaceGrowVoxels = new Dictionary<int, List<(int, int, int)>>();

            for (i = 0; i < spacePartitioningGrid.GetLength(0); i++) {
                for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {
                    for (c = 0; c < spacePartitioningGrid.GetLength(2); c++) {

                        segmentId = spacePartitioningGrid[i, r, c];
                        if (!transitionSpaceRefinementAdjacentRoomHeights.ContainsKey(segmentId)) {
                            continue;
                        }

                        for (dr = -1; dr <= 1; dr++) {
                            for (dc = -1; dc <= 1; dc++) {

                                if (dr.Abs() == dc.Abs()) {
                                    continue;
                                }

                                r2 = r + dr;
                                c2 = c + dc;
                                if (r2 < 0 || c2 < 0
                                        || r2 >= spacePartitioningGrid.GetLength(1)
                                        || c2 >= spacePartitioningGrid.GetLength(2)
                                        || spacePartitioningGrid[i, r2, c2] <= 0) {
                                    continue;
                                }

                                i2 = i - 1;
                                while (true) {
                                    if (i2 < 0
                                            || spacePartitioningGrid[i2, r2, c2] <= 0) {
                                        break;
                                    }
                                    i2--;
                                }

                                i2++;
                                if ((i - i2).Abs() >= verticalResolution) {
                                    continue;
                                }

                                for (i3 = i2; i3 <= i; i3++) {
                                    candidates.Enqueue((i3, r2, c2));
                                }

                                do {

                                    candidate = candidates.Dequeue();
                                    if (isProcessed[
                                                candidate.Item1,
                                                candidate.Item2,
                                                candidate.Item3]
                                            || spacePartitioningGrid[
                                                candidate.Item1,
                                                candidate.Item2,
                                                candidate.Item3] <= 0) {
                                        continue;
                                    }
                                    isProcessed[
                                        candidate.Item1,
                                        candidate.Item2,
                                        candidate.Item3] = true;
                                    transitionSpaceGrowVoxels.BucketAdd(
                                        segmentId,
                                        candidate);

                                    for (dr2 = -1; dr2 <= 1; dr2++) {
                                        for (dc2 = -1; dc2 <= 1; dc2++) {

                                            if (dr2.Abs() == dc2.Abs()) {
                                                continue;
                                            }

                                            r3 = candidate.Item2 + dr2;
                                            c3 = candidate.Item3 + dc2;
                                            if (r3 < 0 || c3 < 0
                                                    || r3 >= spacePartitioningGrid.GetLength(1)
                                                    || c3 >= spacePartitioningGrid.GetLength(2)
                                                    || isProcessed[candidate.Item1, r3, c3]
                                                    || spacePartitioningGrid[candidate.Item1, r3, c3] <= 0) {
                                                continue;
                                            }

                                            i2 = candidate.Item1 - 1;

                                            while (true) {
                                                if (i2 < 0
                                                        || isProcessed[i2, r3, c3]
                                                        || spacePartitioningGrid[i2, r3, c3] <= 0) {
                                                    break;
                                                }
                                                i2--;
                                            }

                                            i2++;
                                            if ((i - i2).Abs() >= verticalResolution) {
                                                continue;
                                            }

                                            for (i3 = i; i3 <= i2; i3++) {
                                                candidates.Enqueue((i3, r2, c2));
                                            }
                                        }
                                    }
                                } while (candidates.Count > 0);
                            }
                        }
                    }
                }
            }

            return transitionSpaceGrowVoxels;
        }

        private static void GrowTransitionSpaces(
                double resolution,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<int, List<(int, int, int)>> transitionSpaceGrowVoxels) {

            int areaThreshold = Parameters
                .FINAL_ROOM_REFINEMENT_MIN_ROOM_AREA
                .GetAreaInVoxels(resolution);

            transitionSpaceGrowVoxels
                .Keys
                .AsParallel()
                .Where(segmentId => {

                    int i;
                    int additionalArea = 0;
                    int[] voxelState;

                    foreach ((int, int, int) voxel in transitionSpaceGrowVoxels[segmentId]) {

                        i = voxel.Item1 + 1;
                        if (i >= reconstructionGrid.GetLength(0)) {
                            continue;
                        }

                        voxelState = reconstructionGrid[i, voxel.Item2, voxel.Item3];
                        if (voxelState == null) {
                            continue;
                        }

                        if (voxelState
                                .GetVoxelClassValues(0)
                                .Contains(VoxelClassValues.FLOOR)) {
                            additionalArea++;
                        }
                    }
                    return (spacePartitioningSegmentAreas.ContainsKey(segmentId) ? 
                        spacePartitioningSegmentAreas[segmentId] :
                        0) + additionalArea < areaThreshold;
                })
                .ForEach(segmentId => {
                    foreach ((int, int, int) voxel in transitionSpaceGrowVoxels[segmentId]) {
                        spacePartitioningGrid[
                            voxel.Item1,
                            voxel.Item2,
                            voxel.Item3] = segmentId;
                    }
                });
        }

        private static void ReclaimTransitionSpaceForRoomSpace(
                bool[,,] transitionSpaceReclaimableForRoomSpace,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid) {

            bool[,,] isSegmented;
            HashSet<int> adjacentRoomIds;
            List<List<(int, int, int)>> transitionSpaceSegmentsReclaimableForRoomSpace;

            transitionSpaceSegmentsReclaimableForRoomSpace = SegmentTransitionSpaceReclaimableForRoomSpace(
                transitionSpaceReclaimableForRoomSpace,
                spacePartitioningGrid,
                out isSegmented);

            foreach (List<(int, int, int)> segment in transitionSpaceSegmentsReclaimableForRoomSpace) {

                adjacentRoomIds = GetAdjacentRoomIds(
                    spacePartitioningGrid,
                    segment);

                if (adjacentRoomIds.Count == 0) {
                    continue;
                }

                if (adjacentRoomIds.Count == 1) {

                    ReclaimForRoom(
                        false,
                        adjacentRoomIds.First(),
                        normalGrid,
                        spacePartitioningGrid,
                        reconstructionGrid,
                        segment);
                }
                else {
                    ReclaimForRoom(
                        true,
                        GetRoomIdOfDominantNeighbour(
                            isSegmented,
                            spacePartitioningGrid,
                            segment),
                        normalGrid,
                        spacePartitioningGrid,
                        reconstructionGrid,
                        segment);
                }
            }
        }

        private static List<List<(int, int, int)>> SegmentTransitionSpaceReclaimableForRoomSpace(
                bool[,,] transitionSpaceReclaimableForRoomSpace,
                int[,,] spacePartitioningGrid,
                out bool[,,] isSegmented) {

            int di, i, i2, dr, r, r2, dc, c, c2;
            isSegmented = new bool[
                spacePartitioningGrid.GetLength(0),
                spacePartitioningGrid.GetLength(1),
                spacePartitioningGrid.GetLength(2)];
            Queue<(int, int, int)> candidates = new Queue<(int, int, int)>();
            List<List<(int, int, int)>> transitionSpaceSegmentsReclaimableForRoomSpace = new List<List<(int, int, int)>>();

            for (i = 0; i < spacePartitioningGrid.GetLength(0); i++) {
                for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {
                    for (c = 0; c < spacePartitioningGrid.GetLength(2); c++) {

                        if (!transitionSpaceReclaimableForRoomSpace[i, r, c]
                                || isSegmented[i, r, c]) {
                            continue;
                        }

                        List<(int, int, int)> segment = new List<(int, int, int)>();
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
                                                && i2 < spacePartitioningGrid.GetLength(0)
                                                && r2 < spacePartitioningGrid.GetLength(1)
                                                && c2 < spacePartitioningGrid.GetLength(2)
                                                && transitionSpaceReclaimableForRoomSpace[i2, r2, c2]
                                                && !isSegmented[i2, r2, c2]) {
                                            candidates.Enqueue((i2, r2, c2));
                                        }
                                    }
                                }
                            }
                        } while (candidates.Count > 0);

                        transitionSpaceSegmentsReclaimableForRoomSpace.Add(segment);
                    }
                }
            }

            return transitionSpaceSegmentsReclaimableForRoomSpace;
        }

        private static HashSet<int> GetAdjacentRoomIds(
                int[,,] spacePartitioningGrid,
                List<(int, int, int)> transitionSpaceSegment) {

            int dr, r, dc, c;
            int segmentId;
            HashSet<int> adjacentRoomIds = new HashSet<int>();

            foreach ((int, int, int) voxel in transitionSpaceSegment) {

                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (dr.Abs() == dc.Abs()) {
                            continue;
                        }

                        r = voxel.Item2 + dr;
                        c = voxel.Item3 + dc;

                        if (r >= 0 && c >= 0
                                && r < spacePartitioningGrid.GetLength(1)
                                && c < spacePartitioningGrid.GetLength(2)) {

                            segmentId = spacePartitioningGrid[voxel.Item1, r, c];

                            if (segmentId > 0) {
                                adjacentRoomIds.Add(segmentId);
                            }
                        }
                    }
                }
            }

            return adjacentRoomIds;
        }

        private static int GetRoomIdOfDominantNeighbour(
                bool[,,] isSegmented,
                int[,,] spacePartitioningGrid,
                List<(int, int, int)> transitionSpaceSegment) {

            int dr, r, dc, c;
            int segmentId;
            Dictionary<int, int> roomSegmentCounts = new Dictionary<int, int>();

            foreach ((int, int, int) voxel in transitionSpaceSegment) {
                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (dr.Abs() == dc.Abs()) {
                            continue;
                        }

                        r = voxel.Item2 + dr;
                        c = voxel.Item3 + dc;
                        if (r < 0 || c < 0
                                || r >= spacePartitioningGrid.GetLength(1)
                                || c >= spacePartitioningGrid.GetLength(2)
                                || isSegmented[voxel.Item1, r, c]) {
                            continue;
                        }

                        segmentId = spacePartitioningGrid[voxel.Item1, r, c];
                        if (segmentId > 0) {
                            roomSegmentCounts.BucketIncrement(segmentId);
                        }
                    }
                }
            }

            return  roomSegmentCounts
                .Keys
                .WhereMax(id => roomSegmentCounts[id])
                .First();
        }

        private static void ReclaimForRoom(
                bool createWallToOtherRooms,
                int roomId,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                List<(int, int, int)> transitionSpaceSegment) {

            int dr, r, dc, c;
            int segmentId;

            foreach ((int, int, int) voxel in transitionSpaceSegment) {

                spacePartitioningGrid[
                    voxel.Item1,
                    voxel.Item2,
                    voxel.Item3] = roomId;
                if (!createWallToOtherRooms) {
                    continue;
                }

                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        r = voxel.Item2 + dr;
                        c = voxel.Item3 + dc;
                        if (r < 0 || c < 0
                                || r >= spacePartitioningGrid.GetLength(1)
                                || c >= spacePartitioningGrid.GetLength(2)) {
                            continue;
                        }

                        segmentId = spacePartitioningGrid[voxel.Item1, r, c];

                        if (segmentId > 0
                                && segmentId != roomId) {

                            spacePartitioningGrid[
                                voxel.Item1,
                                r,
                                c] = 0;

                            reconstructionGrid[
                                    voxel.Item1,
                                    r,
                                    c]
                                = VoxelState.CreateVoxelState(
                                    0,
                                    normalGrid[
                                        voxel.Item1,
                                        r,
                                        c] == NormalGridValues.EMPTY ?
                                            VoxelClassValues.WALL_OPENING :
                                            VoxelClassValues.WALL);
                        }
                    }
                }
            }
        }

        private static void MergeSmallRampSpaceNeighboursToRampSpace(
                double resolution,
                int[,,] spacePartitioningGrid,
                HashSet<int> rampSpaceIds,
                HashSet<int> rampTransitionSpaceIds,
                HashSet<int> unmergeableRampTransitionSpaceIds,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<int, int> rampTransitionSpaceAreas,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            int sourceId, destinationId;
            int rampNeighbourMinArea = Parameters
                .INDEPENDENT_RAMP_NEIGHBOUR_MIN_AREA
                .GetAreaInVoxels(resolution);
            Dictionary<int, int> mergeMapping = new Dictionary<int, int>();

            foreach (int id in rampTransitionSpaceIds) {

                if (!unmergeableRampTransitionSpaceIds.Contains(id)
                        && spacePartitioningSegmentAdjacency[id].Count == 2
                        && rampTransitionSpaceAreas[id] >= 1.0.GetAreaInVoxels(resolution)
                        && spacePartitioningSegmentAdjacency[id].Any(adjacentId => rampSpaceIds.Contains(adjacentId))
                        && spacePartitioningSegmentAdjacency[id].Any(adjacentId => !rampSpaceIds.Contains(adjacentId)
                            && spacePartitioningSegmentAreas[adjacentId] < rampNeighbourMinArea
                            && spacePartitioningSegmentAdjacency[adjacentId].Count == 1)) {

                    destinationId = spacePartitioningSegmentAdjacency[id]
                        .Where(adjacentId => rampSpaceIds.Contains(adjacentId))
                        .First();

                    mergeMapping.Add(
                        id,
                        destinationId);

                    sourceId = spacePartitioningSegmentAdjacency[id]
                        .Where(adjacentId => !rampSpaceIds.Contains(adjacentId))
                        .First();

                    if (!mergeMapping.ContainsKey(sourceId)) {
                        mergeMapping.Add(sourceId, destinationId);
                    }
                }
            }
            if (mergeMapping.Count > 0) {

                MergeSpacePartitioningSegments(
                    spacePartitioningGrid,
                    rampTransitionSpaceIds,
                    mergeMapping,
                    spacePartitioningSegmentAreas,
                    spacePartitioningSegmentAdjacency);
            }
        }

        private static void RefineRoomsVertically(
                double resolution,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                HashSet<int> rampSpaceIds) {

            object @lock = new object();
            int heightThreshold = Parameters
                .ROOM_SPACE_REFINEMENT_FLOOR_SEARCH_HEIGHT_THRESHOLD
                .GetDistanceInVoxels(resolution);
            HashSet<int> roomIdsToResegment = new HashSet<int>();

            Parallel.ForEach(
                Partitioner.Create(
                    0, 
                    spacePartitioningGrid.GetLength(2)),
                () => new HashSet<int>(),
                (partition, loopState, localRoomIdsToResegment) => {

                    for (int c = partition.Item1; c < partition.Item2; c++) {

                        ExtendRoomSpaceVerticallyDownwardsUntilFloor(
                            c,
                            heightThreshold,
                            normalGrid,
                            spacePartitioningGrid,
                            reconstructionGrid);

                        ResolveRoomSpreadOvers(
                            c,
                            spacePartitioningGrid,
                            reconstructionGrid,
                            localRoomIdsToResegment);

                        RemoveRoomColumnsWithoutLowerBoundingSurface(
                            c,
                            spacePartitioningGrid,
                            reconstructionGrid,
                            rampSpaceIds,
                            localRoomIdsToResegment);
                    }

                    return localRoomIdsToResegment;
                },
                localRoomIdsToResegment => {
                    lock (@lock) {
                        roomIdsToResegment.AddRange(localRoomIdsToResegment);
                    }
                });

            if (roomIdsToResegment.Count > 0) {
                ResegmentRooms(
                    spacePartitioningGrid,
                    reconstructionGrid,
                    roomIdsToResegment);
            }
        }

        private static void ExtendRoomSpaceVerticallyDownwardsUntilFloor(
                int c,
                int heightThreshold,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid) {

            int i, i2, r;
            int segmentId;
            int startHeight, stopHeight;

            for (i = 0; i < spacePartitioningGrid.GetLength(0); i++) {
                for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {

                    segmentId = spacePartitioningGrid[i, r, c];
                    if (segmentId <= 0) {
                        continue;
                    }

                    startHeight = i + 1;

                    if (HasFloorUnderneath(
                            startHeight,
                            r,
                            c,
                            heightThreshold,
                            spacePartitioningGrid,
                            reconstructionGrid,
                            out stopHeight)) {

                        for (i2 = startHeight; i2 < stopHeight; i2++) {

                            spacePartitioningGrid[i2, r, c] = segmentId;
                            reconstructionGrid[i2, r, c] = VoxelState.CreateVoxelState(
                                0,
                                normalGrid[i2, r, c] == NormalGridValues.EMPTY ?
                                    VoxelClassValues.EMPTY_INTERIOR :
                                    VoxelClassValues.INTERIOR_OBJECT);
                        }
                    }
                }
            }
        }

        private static bool HasFloorUnderneath(
                int i,
                int r,
                int c,
                int heightThreshold,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                out int stopHeight) {

            bool foundFloor = false;
            int[] voxelState;
            int[] voxelClassValues;

            while (true) {

                if (i >= spacePartitioningGrid.GetLength(0)
                        || spacePartitioningGrid[i, r, c] != 0) {
                    break;
                }

                if ((i - i) >= heightThreshold) {
                    break;
                }

                voxelState = reconstructionGrid[i, r, c];
                if (voxelState != null) {

                    voxelClassValues = voxelState.GetVoxelClassValues(0);
                    if (!voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                            && !voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)) {

                        if (voxelClassValues.Contains(VoxelClassValues.FLOOR)) {
                            foundFloor = true;
                        }

                        break;
                    }
                }
                i++;
            }

            stopHeight = i;

            return foundFloor;
        }

        private static void ResolveRoomSpreadOvers(
                int c,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                HashSet<int> roomIdsToResegment) {

            int i, i2, r;
            int segmentId;
            int startHeight, stopHeight;
            bool[,] isProcessed = new bool[
                spacePartitioningGrid.GetLength(0),
                spacePartitioningGrid.GetLength(1)];
            HashSet<int> roomIds;
            Dictionary<int, int> roomCounters;

            for (i = 0; i < spacePartitioningGrid.GetLength(0); i++) {
                for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {
                    
                    if (isProcessed[i, r]) {
                        continue;
                    }

                    segmentId = spacePartitioningGrid[i, r, c];
                    if (segmentId <= 0) {
                        continue;
                    }

                    startHeight = i + 1;

                    SegmentRoomColumnByRoomId(
                        startHeight,
                        r,
                        c,
                        segmentId,
                        spacePartitioningGrid,
                        reconstructionGrid,
                        out stopHeight,
                        out roomIds,
                        out roomCounters);

                    if (roomIds.Count == 1) {
                        for (i2 = startHeight; i2 < stopHeight; i2++) {
                            isProcessed[i2, r] = true;
                        }
                    }
                    else {
                        AssignRoomColumnToDominantRoom(
                            i,
                            r,
                            c,
                            stopHeight,
                            isProcessed,
                            spacePartitioningGrid,
                            roomIdsToResegment,
                            roomIds,
                            roomCounters);
                    }
                }
            }
        }

        private static void SegmentRoomColumnByRoomId(
                int i,
                int r,
                int c,
                int roomId,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                out int stopHeight,
                out HashSet<int> roomIds,
                out Dictionary<int, int> roomCounters) {

            int[] voxelState;
            int[] voxelClassValues;

            roomIds = new HashSet<int> {
                roomId
            };
            roomCounters = new Dictionary<int, int> {
                { roomId, 1 }
            };

            while (true) {

                if (i >= spacePartitioningGrid.GetLength(0)) {
                    break;
                }

                voxelState = reconstructionGrid[i, r, c];
                if (voxelState == null) {
                    break;
                }

                voxelClassValues = voxelState.GetVoxelClassValues(0);
                if (voxelClassValues.Contains(VoxelClassValues.CEILING)
                        || voxelClassValues.Contains(VoxelClassValues.FLOOR)) {
                    break;
                }

                roomId = spacePartitioningGrid[i, r, c];
                if (roomId > 0) {
                    roomIds.Add(roomId);
                    roomCounters.BucketIncrement(roomId);
                }

                i++;
            }

            stopHeight = i;
        }

        private static void AssignRoomColumnToDominantRoom(
                int i,
                int r,
                int c,
                int stopHeight,
                bool[,] isProcessed,
                int[,,] spacePartitioningGrid,
                HashSet<int> roomIdsToResegment,
                HashSet<int> roomIds,
                Dictionary<int, int> roomCounters) {

            int i2;
            int destinationRoomId = roomIds
                .WhereMax(roomId => roomCounters[roomId])
                .First();

            foreach (int roomId in roomIds) {
                if (roomId != destinationRoomId) {
                    roomIdsToResegment.Add(roomId);
                }
            }

            for (i2 = i; i2 < stopHeight; i2++) {
                isProcessed[i2, r] = true;
                if (spacePartitioningGrid[i2, r, c] > 0) {
                    spacePartitioningGrid[i2, r, c] = destinationRoomId;
                }
            }
        }

        private static void RemoveRoomColumnsWithoutLowerBoundingSurface(
                int c,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                HashSet<int> rampSpaceIds,
                HashSet<int> roomIdsToResegment) {

            bool didChange;
            int i, i2, r;
            int segmentId;

            for (i = 0; i < spacePartitioningGrid.GetLength(0); i++) {
                for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {

                    segmentId = spacePartitioningGrid[i, r, c];
                    if (segmentId <= 0
                            || rampSpaceIds.Contains(segmentId)) {
                        continue;
                    }

                    i2 = i + 1;
                    if (i2 < spacePartitioningGrid.GetLength(0)
                            && (spacePartitioningGrid[i2, r, c] != 0
                                || reconstructionGrid[i2, r, c] != null)) {
                        continue;
                    }

                    i2 = i;
                    didChange = false;
                    while (true) {
                        if (i2 < 0
                                || spacePartitioningGrid[i2, r, c] != segmentId) {
                            break;
                        }
                        didChange = true;
                        spacePartitioningGrid[i2, r, c] = 0;
                        reconstructionGrid[i2, r, c] = null;
                        i2--;
                    }

                    if (didChange) {
                        roomIdsToResegment.Add(segmentId);
                    }
                }
            }
        }

        private static void ResegmentRooms(
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                HashSet<int> roomIdsToResegment) {

            int di, i, i2, dr, r, r2, dc, c, c2;
            int segmentId;
            bool[,,] isSegmented = new bool[
                spacePartitioningGrid.GetLength(0),
                spacePartitioningGrid.GetLength(1),
                spacePartitioningGrid.GetLength(2)];
            Queue<(int, int, int)> candidates = new Queue<(int, int, int)>();
            Dictionary<int, List<List<(int, int, int)>>> roomSegments = new Dictionary<int, List<List<(int, int, int)>>>();

            for (i = 0; i < spacePartitioningGrid.GetLength(0); i++) {
                for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {
                    for (c = 0; c < spacePartitioningGrid.GetLength(2); c++) {

                        if (isSegmented[i, r, c]) {
                            continue;
                        }

                        segmentId = spacePartitioningGrid[i, r, c];
                        if (segmentId <= 0
                                || !roomIdsToResegment.Contains(segmentId)) {
                            continue;
                        }

                        List<(int, int, int)> segment = new List<(int, int, int)>();
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
                                                && i2 < spacePartitioningGrid.GetLength(0)
                                                && r2 < spacePartitioningGrid.GetLength(1)
                                                && c2 < spacePartitioningGrid.GetLength(2)
                                                && !isSegmented[i2, r2, c2]
                                                && spacePartitioningGrid[i2, r2, c2] == segmentId) {
                                            candidates.Enqueue((i2, r2, c2));
                                        }
                                    }
                                }
                            }
                        } while (candidates.Count > 0);
                        roomSegments.BucketAdd(
                            segmentId,
                            segment);
                    }
                }
            }

            RemoveMinorRoomSegmentsPerRoomId(
                spacePartitioningGrid,
                reconstructionGrid,
                roomSegments);
        }

        private static void RemoveMinorRoomSegmentsPerRoomId(
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                Dictionary<int, List<List<(int, int, int)>>> roomSegments) {

            List<int> roomIds = roomSegments.Keys.ToList();

            Parallel.For(
                0,
                roomIds.Count,
                j => {

                    List<List<(int, int, int)>> segments = roomSegments[roomIds[j]];

                    HashSet<int> maxSegmentIndices = Enumerable
                        .Range(0, segments.Count)
                        .WhereMax(index => segments[index].Count)
                        .ToHashSet();

                    foreach ((int, int, int) voxel in Enumerable
                            .Range(0, segments.Count)
                            .Where(index => !maxSegmentIndices.Contains(index))
                            .SelectMany(index => segments[index])) {

                        spacePartitioningGrid[
                            voxel.Item1,
                            voxel.Item2,
                            voxel.Item3] = 0;

                        reconstructionGrid[
                            voxel.Item1,
                            voxel.Item2,
                            voxel.Item3] = null;
                    }
                });
        }

        private static bool RefineRooms(
                double resolution,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                HashSet<int> rampTransitionSpaceIds,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            bool hasChangedTransitionSpaces = false;
            double minRoomArea = Parameters
                .FINAL_ROOM_REFINEMENT_MIN_ROOM_AREA
                .GetAreaInVoxels(resolution);
            HashSet<int> roomIdsToRemove = new HashSet<int>();
            HashSet<(int, int)> mergeIds = new HashSet<(int, int)>();

            foreach (int roomId in spacePartitioningSegmentAreas.Keys
                    .Where(id => id > 0
                        && spacePartitioningSegmentAreas[id] < minRoomArea)) {

                if (spacePartitioningSegmentAdjacency[roomId].Count == 0) {
                    roomIdsToRemove.Add(roomId);
                }
                else {
                    foreach (int transitionSpaceId in spacePartitioningSegmentAdjacency[roomId]
                            .Where(id => id < 0)) {

                        mergeIds.Add((roomId, transitionSpaceId));
                    }
                }
            }

            if (mergeIds.Count > 0) {

                hasChangedTransitionSpaces = true;

                MergeSpacePartitioningSegments(
                    spacePartitioningGrid,
                    rampTransitionSpaceIds,
                    Util.GetMergeMapping(
                        mergeIds,
                        cluster => cluster
                            .Where(id => id < 0)
                            .First()),
                    spacePartitioningSegmentAreas,
                    spacePartitioningSegmentAdjacency);

                RemoveDeadEndTransitionSpaces(
                    spacePartitioningGrid,
                    rampTransitionSpaceIds,
                    spacePartitioningSegmentAreas,
                    spacePartitioningSegmentAdjacency);
            }

            if (roomIdsToRemove.Count > 0) {

                RemoveSpacePartitioningSegments(
                    normalGrid,
                    spacePartitioningGrid,
                    reconstructionGrid,
                    roomIdsToRemove,
                    spacePartitioningSegmentAreas,
                    spacePartitioningSegmentAdjacency);
            }

            return hasChangedTransitionSpaces;
        }

        private static void MergeAdjacentTransitionSpaces(
                ref int transitionSpaceCount,
                int[,,] spacePartitioningGrid,
                HashSet<int> rampTransitionSpaceIds,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            HashSet<(int, int)> adjacentTransitionSpaceIds;
            Dictionary<int, int> mergeMapping;

            adjacentTransitionSpaceIds = GetAdjacentTransitionSpaceIds(spacePartitioningGrid);

            mergeMapping = GetMergeMappingFromAdjacentTransitionSpaces(
                ref transitionSpaceCount,
                adjacentTransitionSpaceIds,
                spacePartitioningSegmentAreas,
                spacePartitioningSegmentAdjacency);

            MergeSpacePartitioningSegments(
                spacePartitioningGrid,
                rampTransitionSpaceIds,
                mergeMapping,
                spacePartitioningSegmentAreas,
                spacePartitioningSegmentAdjacency);
        }

        private static HashSet<(int, int)> GetAdjacentTransitionSpaceIds(
                int[,,] spacePartitioningGrid) {

            object @lock = new object();
            HashSet<(int, int)> adjacentTransitionSpaceIds = new HashSet<(int, int)>();

            Parallel.ForEach(
                Partitioner.Create(
                    0, 
                    spacePartitioningGrid.GetLength(0)),
                () => new HashSet<(int, int)>(),
                (partition, loopState, localAdjacentTransitionSpaceIds) => {

                    for (int i = partition.Item1; i < partition.Item2; i++) {
                        UpdateAdjacentTransitionSpaceIds(
                            i,
                            spacePartitioningGrid,
                            localAdjacentTransitionSpaceIds);
                    }

                    return localAdjacentTransitionSpaceIds;
                },
                localAdjacentTransitionSpaceIds => {

                    lock (@lock) {
                        adjacentTransitionSpaceIds.AddRange(localAdjacentTransitionSpaceIds);
                    }
                });

            return adjacentTransitionSpaceIds;
        }

        private static void UpdateAdjacentTransitionSpaceIds(
                int i,
                int[,,] spacePartitioningGrid,
                HashSet<(int, int)> adjacentTransitionSpaceIds) {

            int dr, r, r2, dc, c, c2;
            int segmentId, segmentId2;

            for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {
                for (c = 0; c < spacePartitioningGrid.GetLength(2); c++) {

                    segmentId = spacePartitioningGrid[i, r, c];
                    if (segmentId >= 0) {
                        continue;
                    }

                    for (dr = -1; dr <= 1; dr++) {
                        for (dc = -1; dc <= 1; dc++) {

                            if (dr.Abs() == dc.Abs()) {
                                continue;
                            }

                            r2 = r + dr;
                            c2 = c + dc;
                            if (r2 < 0 || c2 < 0
                                    || r2 >= spacePartitioningGrid.GetLength(1)
                                    || c2 >= spacePartitioningGrid.GetLength(2)) {
                                continue;
                            }

                            segmentId2 = spacePartitioningGrid[i, r2, c2];
                            if (segmentId2 < 0
                                    && segmentId2 != segmentId) {

                                adjacentTransitionSpaceIds.Add(
                                    Util.GetUnambiguousOrder(
                                        segmentId,
                                        segmentId2));
                            }
                        }
                    }
                }
            }
        }

        private static Dictionary<int, int> GetMergeMappingFromAdjacentTransitionSpaces(
                ref int transitionSpaceCount,
                HashSet<(int, int)> adjacentTransitionSpaceIds,
                Dictionary<int, int> spacePartitioningSegmentAreas,
                Dictionary<int, HashSet<int>> spacePartitioningSegmentAdjacency) {

            int transitionSpaceId;
            IdSet adjacentRoomIds;
            Dictionary<int, int> mergeMapping = new Dictionary<int, int>();
            Dictionary<IdSet, HashSet<int>> mergeBuckets = new Dictionary<IdSet, HashSet<int>>();

            foreach ((int, int) transitionSpaceIdPair in adjacentTransitionSpaceIds) {

                if (!spacePartitioningSegmentAdjacency.ContainsKey(transitionSpaceIdPair.Item1)) {
                    continue;
                }

                adjacentRoomIds = new IdSet(
                    spacePartitioningSegmentAdjacency[transitionSpaceIdPair.Item1]);

                if (spacePartitioningSegmentAdjacency.ContainsKey(transitionSpaceIdPair.Item2)
                        && new IdSet(spacePartitioningSegmentAdjacency[transitionSpaceIdPair.Item2])
                            .Equals(adjacentRoomIds)) {

                    mergeBuckets.BucketAdd(
                        adjacentRoomIds,
                        transitionSpaceIdPair.Item1);

                    mergeBuckets.BucketAdd(
                        adjacentRoomIds,
                        transitionSpaceIdPair.Item2);
                }
            }

            foreach (IdSet roomIds in mergeBuckets.Keys) {

                transitionSpaceId = -(++transitionSpaceCount);
                spacePartitioningSegmentAreas.Add(transitionSpaceId, 0);
                spacePartitioningSegmentAdjacency.Add(
                    transitionSpaceId,
                    new HashSet<int>());

                foreach (int id in mergeBuckets[roomIds]) {

                    if (!mergeMapping.ContainsKey(id)) {
                        mergeMapping.Add(id, transitionSpaceId);
                    }
                }
            }

            return mergeMapping;
        }

        private static void AssignHorizontalTransitionSpaceSectionSegmentsToSingleAdjacentRoom(
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid) {

            Parallel.For(
                0,
                spacePartitioningGrid.GetLength(0),
                i => {

                    int r, c;
                    int segmentId;
                    bool[,] isSegmented = new bool[
                        spacePartitioningGrid.GetLength(1),
                        spacePartitioningGrid.GetLength(2)];
                    HashSet<int> roomIds;
                    List<(int, int)> transitionSpaceSegment2D;

                    for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {
                        for (c = 0; c < spacePartitioningGrid.GetLength(2); c++) {

                            if (isSegmented[r, c]) {
                                continue;
                            }

                            segmentId = spacePartitioningGrid[i, r, c];
                            if (segmentId >= 0) {
                                continue;
                            }

                            transitionSpaceSegment2D = SegmentTransitionSpacePositions(
                                i,
                                r,
                                c,
                                segmentId,
                                isSegmented,
                                spacePartitioningGrid);

                            roomIds = GetRoomIdsAdjacentToTransitionSpaceSegment2D(
                                i,
                                segmentId,
                                spacePartitioningGrid,
                                transitionSpaceSegment2D);
                            
                            if (roomIds.Count == 1) {
                                AssignTransitionSpaceSegment2DToRoom(
                                    i,
                                    roomIds.First(),
                                    normalGrid,
                                    spacePartitioningGrid,
                                    reconstructionGrid,
                                    transitionSpaceSegment2D);
                            }
                        }
                    }
                });
        }

        private static List<(int, int)> SegmentTransitionSpacePositions(
                int i,
                int r,
                int c,
                int transitionSpaceId,
                bool[,] isSegmented,
                int[,,] spacePartitioningGrid) {

            int dr, r2, dc, c2;
            List<(int, int)> segment = new List<(int, int)>();
            Queue<(int, int)> candidates = new Queue<(int, int)>();

            candidates.Enqueue((r, c));

            do {

                (int, int) candidate = candidates.Dequeue();
                if (isSegmented[
                        candidate.Item1,
                        candidate.Item2]) {
                    continue;
                }
                isSegmented[
                    candidate.Item1,
                    candidate.Item2] = true;
                segment.Add(candidate);

                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (dr.Abs() == dc.Abs()) {
                            continue;
                        }

                        r2 = candidate.Item1 + dr;
                        c2 = candidate.Item2 + dc;
                        if (r2 >= 0 && c2 >= 0
                                && r2 < spacePartitioningGrid.GetLength(1)
                                && c2 < spacePartitioningGrid.GetLength(2)
                                && !isSegmented[r2, c2]
                                && spacePartitioningGrid[i, r2, c2] == transitionSpaceId) {
                            
                            candidates.Enqueue((r2, c2));
                        }
                    }
                }
            } while (candidates.Count > 0);

            return segment;
        }

        private static HashSet<int> GetRoomIdsAdjacentToTransitionSpaceSegment2D(
                int i,
                int transitionSpaceId,
                int[,,] spacePartitioningGrid,
                List<(int, int)> transitionSpaceSegment2D) {

            int dr, r2, dc, c2;
            HashSet<int> roomIds = new HashSet<int>();

            foreach ((int, int) position in transitionSpaceSegment2D) {

                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (dr.Abs() == dc.Abs()) {
                            continue;
                        }

                        r2 = position.Item1 + dr;
                        c2 = position.Item2 + dc;
                        if (r2 >= 0 && c2 >= 0
                                && r2 < spacePartitioningGrid.GetLength(1)
                                && c2 < spacePartitioningGrid.GetLength(2)) {

                            transitionSpaceId = spacePartitioningGrid[i, r2, c2];

                            if (transitionSpaceId > 0) {
                                roomIds.Add(transitionSpaceId);
                            }
                        }
                    }
                }
            }

            return roomIds;
        }

        private static void AssignTransitionSpaceSegment2DToRoom(
                int i,
                int roomId,
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid,
                List<(int, int)> transitionSpaceSegment2D) {

            foreach ((int, int) position in transitionSpaceSegment2D) {

                spacePartitioningGrid[
                    i,
                    position.Item1,
                    position.Item2] = roomId;

                reconstructionGrid[
                    i,
                    position.Item1,
                    position.Item2] = VoxelState.CreateVoxelState(
                        0,
                        normalGrid[
                                i,
                                position.Item1,
                                position.Item2] == NormalGridValues.EMPTY ?
                            VoxelClassValues.EMPTY_INTERIOR :
                            VoxelClassValues.INTERIOR_OBJECT);
            }
        }

        private static void AssignVerticallyAdjacentWallOpeningVoxelsToTransitionSpaces(
                byte[,,] normalGrid,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid) {

            Parallel.For(
                0,
                spacePartitioningGrid.GetLength(2),
                c => {

                    int i, i2, r, d;
                    int segmentId;
                    int[] voxelState;

                    for (i = 0; i < spacePartitioningGrid.GetLength(0); i++) {
                        for (r = 0; r < spacePartitioningGrid.GetLength(1); r++) {

                            segmentId = spacePartitioningGrid[i, r, c];
                            if (segmentId >= 0) {
                                continue;
                            }

                            for (d = -1; d <= 1; d += 2) {

                                i2 = i + d;

                                while (true) {

                                    if (i2 < 0
                                            || i2 >= reconstructionGrid.GetLength(0)
                                            || spacePartitioningGrid[i2, r, c] != 0) {
                                        break;
                                    }

                                    voxelState = reconstructionGrid[i2, r, c];
                                    if (voxelState == null
                                            || !voxelState
                                                .GetVoxelClassValues(0)
                                                .Contains(VoxelClassValues.WALL_OPENING)) {
                                        break;
                                    }

                                    spacePartitioningGrid[i2, r, c] = segmentId;
                                    reconstructionGrid[i2, r, c] = VoxelState.CreateVoxelState(
                                        0,
                                        normalGrid[i2, r, c] == NormalGridValues.EMPTY ?
                                            VoxelClassValues.EMPTY_INTERIOR :
                                            VoxelClassValues.INTERIOR_OBJECT);

                                    i2 += d;
                                }
                            }
                        }
                    }
                });
        }

        private static void ApplySpacePartitioningSegmentIdsToInteriorSpace(
                int c,
                int[,,] spacePartitioningGrid,
                int[,,][] reconstructionGrid) {

            int i, r;
            int segmentId;
            int[] voxelState;

            for (i = 0; i < reconstructionGrid.GetLength(0); i++) {
                for (r = 0; r < reconstructionGrid.GetLength(1); r++) {

                    segmentId = spacePartitioningGrid[i, r, c];
                    if (segmentId == 0) {
                        continue;
                    }

                    voxelState = reconstructionGrid[i, r, c];
                    if (voxelState == null) {
                        continue;
                    }

                    reconstructionGrid[i, r, c] = VoxelState.CreateVoxelState(
                        segmentId,
                        voxelState.GetVoxelClassValues(0));
                }
            }
        }

        private static void TransferSpacePartitioningFromInteriorToCeilingsAndFloors(
                int c,
                int[,,][] reconstructionGrid) {

            int i, r;
            int[] voxelState;
            int[] voxelClassValues;

            for (i = 0; i < reconstructionGrid.GetLength(0); i++) {
                for (r = 0; r < reconstructionGrid.GetLength(1); r++) {

                    voxelState = reconstructionGrid[i, r, c];
                    if (voxelState == null) {
                        continue;
                    }

                    foreach (int roomId in voxelState.GetRoomIds()) {

                        if (roomId == 0) {
                            continue;
                        }

                        voxelClassValues = voxelState.GetVoxelClassValues(roomId);
                        if (!voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                                && !voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                            continue;
                        }

                        TransferSpacePartitioningToCeilings(
                            i,
                            r,
                            c,
                            roomId,
                            reconstructionGrid);

                        TransferSpacePartitioningToFloors(
                            i,
                            r,
                            c,
                            roomId,
                            reconstructionGrid);
                    }
                }
            }
        }

        private static void TransferSpacePartitioningToCeilings(
                int i,
                int r,
                int c,
                int roomId,
                int[,,][] reconstructionGrid) {

            int[] roomIds;
            int[] voxelState;
            int[] voxelClassValues;

            i--;

            while (true) {

                if (i < 0) {
                    break;
                }

                voxelState = reconstructionGrid[i, r, c];
                if (voxelState == null) {
                    break;
                }

                roomIds = voxelState.GetRoomIds();
                if (roomIds.Contains(roomId)) {
                    break;
                }

                if (roomIds.Length > 1) {
                    throw new ApplicationException();
                }

                voxelClassValues = voxelState.GetVoxelClassValues(roomIds[0]);
                if (!voxelClassValues.Contains(VoxelClassValues.CEILING)) {
                    break;
                }

                reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyRemoveRoom(0);
                reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyChangeVoxelClassValues(
                    roomId,
                    voxelClassValues);

                i--;
            }
        }

        private static void TransferSpacePartitioningToFloors(
                int i,
                int r,
                int c,
                int roomId,
                int[,,][] reconstructionGrid) {

            int[] roomIds;
            int[] voxelState;
            int[] voxelClassValues;

            i++;

            while (true) {

                if (i >= reconstructionGrid.GetLength(0)) {
                    break;
                }

                voxelState = reconstructionGrid[i, r, c];
                if (voxelState == null) {
                    break;
                }

                roomIds = voxelState.GetRoomIds();
                if (roomIds.Contains(roomId)) {
                    break;
                }

                if (roomIds.Length > 1) {
                    throw new ApplicationException();
                }

                voxelClassValues = voxelState.GetVoxelClassValues(roomIds[0]);
                if (!voxelClassValues.Contains(VoxelClassValues.FLOOR)) {
                    break;
                }

                reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyRemoveRoom(0);
                reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyChangeVoxelClassValues(
                    roomId,
                    voxelClassValues);

                i++;
            }
        }

        private static void TransferSpacePartitioningToWalls(
                int i,
                int[,,][] reconstructionGrid) {

            bool anyChanges;
            int r, c;
            int[] voxelState;
            int[] voxelClassValues;
            int[] newVoxelClassValues;
            bool[,] isNew;
            List<int> adjecantRoomIds;

            do {

                anyChanges = false;
                isNew = new bool[
                    reconstructionGrid.GetLength(1),
                    reconstructionGrid.GetLength(2)];

                for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                    for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                        voxelState = reconstructionGrid[i, r, c];
                        if (voxelState == null
                                || voxelState
                                    .GetRoomIds()
                                    .Any(id => id != 0)) {
                            continue;
                        }

                        voxelClassValues = voxelState.GetVoxelClassValues(0);
                        if ((voxelClassValues.Contains(VoxelClassValues.FLOOR)
                                    || voxelClassValues.Contains(VoxelClassValues.CEILING))
                                && !voxelClassValues.Contains(VoxelClassValues.WALL)) {
                            continue;
                        }

                        adjecantRoomIds = GetAdjacentRoomIds(
                            i,
                            r,
                            c,
                            isNew,
                            reconstructionGrid);
                        if (adjecantRoomIds.Count == 0) {
                            continue;
                        }

                        newVoxelClassValues = voxelClassValues.Copy();

                        anyChanges = true;
                        isNew[r, c] = true;

                        if (voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)) {
                            voxelClassValues = new int[] {
                                VoxelClassValues.WALL_OPENING
                            };
                        }
                        if (voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                            voxelClassValues = new int[] {
                                VoxelClassValues.WALL
                            };
                        }

                        reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyRemoveRoom(0);
                        foreach (int id in adjecantRoomIds) {
                            reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyChangeVoxelClassValues(
                                id,
                                voxelClassValues);
                        }
                    }
                }
            } while (anyChanges);
        }

        private static List<int> GetAdjacentRoomIds(
                int i,
                int r,
                int c,
                bool[,] isNew,
                int[,,][] reconstructionGrid) {

            int dr, r2, dc, c2;
            int[] voxelState;
            int[] roomIds;
            List<int> adjacentRoomIds = new List<int>();

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {
                    if (dr.Abs() == dc.Abs()) {
                        continue;
                    }

                    r2 = r + dr;
                    c2 = c + dc;
                    if (r2 < 0 || c2 < 0
                            || r2 >= reconstructionGrid.GetLength(1)
                            || c2 >= reconstructionGrid.GetLength(2)
                            || isNew[r2, c2]) {
                        continue;
                    }

                    voxelState = reconstructionGrid[i, r2, c2];
                    if (voxelState == null) {
                        continue;
                    }

                    roomIds = voxelState.GetRoomIds();
                    if (roomIds.Length == 1
                            && roomIds[0] != 0) {
                        adjacentRoomIds.Add(roomIds[0]);
                    }
                }
            }

            return adjacentRoomIds;
        }

        private static void RemoveRoomlessVoxelsExceptForCeilings(
                int i,
                int[,,][] reconstructionGrid) {

            int r, c;
            int[] voxelState;
            int[] voxelClassValues;

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                    voxelState = reconstructionGrid[i, r, c];
                    if (voxelState == null
                            || voxelState.HasRoomId(0)) {
                        continue;
                    }

                    voxelClassValues = voxelState.GetVoxelClassValues(0);
                    if (!voxelClassValues.Contains(VoxelClassValues.CEILING)
                            || voxelClassValues.Contains(VoxelClassValues.WALL)) {

                        reconstructionGrid[i, r, c] = voxelState.CopyRemoveRoom(0);
                    }
                }
            }
        }

        private static void ResolveRoomlessCeilingVoxels(
                int c,
                int[,,][] reconstructionGrid) {

            int r, i, i2;
            int[] voxelState;
            int[] voxelState2;

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (i = reconstructionGrid.GetLength(0) - 1; i >= 0; i--) {

                    voxelState = reconstructionGrid[i, r, c];
                    if (voxelState == null) {
                        continue;
                    }

                    if (voxelState.HasRoomId(0)) {
                        reconstructionGrid[i, r, c] = voxelState.CopyRemoveRoom(0);
                    }

                    voxelState = reconstructionGrid[i, r, c];
                    if (voxelState == null) {
                        continue;
                    }

                    i2 = i - 1;
                    if (i2 < 0) {
                        continue;
                    }

                    voxelState2 = reconstructionGrid[i2, r, c];
                    if (voxelState2 == null
                            || !voxelState2.HasRoomId(0)) {
                        continue;
                    }

                    reconstructionGrid[i2, r, c] = VoxelState.CreateVoxelState(
                        voxelState.GetRoomIds().First(),
                        VoxelClassValues.CEILING);
                }
            }
        }

        private static void RemoveFalseWallLabelsFromFloorsAndCeilings(
                int c,
                int[,,][] reconstructionGrid) {

            int i, r;
            int[] directions;
            int[] voxelState;
            int[] voxelClassValues;

            for (i = 0; i < reconstructionGrid.GetLength(0); i++) {
                for (r = 0; r < reconstructionGrid.GetLength(1); r++) {

                    voxelState = reconstructionGrid[i, r, c];
                    if (voxelState == null) {
                        continue;
                    }

                    foreach (int roomId in voxelState.GetRoomIds()) {

                        voxelClassValues = voxelState.GetVoxelClassValues(roomId);
                        if (!voxelClassValues.Contains(VoxelClassValues.WALL)
                                && !(voxelClassValues.Contains(VoxelClassValues.CEILING)
                                    || voxelClassValues.Contains(VoxelClassValues.FLOOR))) {
                            continue;
                        }

                        directions = GetDirections(voxelClassValues);

                        foreach (int direction in directions) {
                            RemoveFalseWallLabelsFromFloorsAndCeilings(
                                i,
                                r,
                                c,
                                roomId,
                                direction,
                                reconstructionGrid);
                        }
                    }
                }
            }
        }

        private static int[] GetDirections(
                int[] voxelClassValues) {

            if (voxelClassValues.Contains(VoxelClassValues.CEILING)
                    && voxelClassValues.Contains(VoxelClassValues.FLOOR)) {
                return new int[] { 1, -1 };
            }

            if (voxelClassValues.Contains(VoxelClassValues.CEILING)) {
                return new int[] { 1 };
            }

            return new int[] { -1 };
        }

        private static void RemoveFalseWallLabelsFromFloorsAndCeilings(
                int i,
                int r,
                int c,
                int roomId,
                int direction,
                int[,,][] reconstructionGrid) {

            int i2;
            int voxelClassValue;
            int[] voxelState;
            int[] voxelClassValues;

            i2 = i + direction;
            if (i2 < 0 
                    || i2 >= reconstructionGrid.GetLength(0)) {
                return;
            }

            voxelState = reconstructionGrid[i2, r, c];
            if (voxelState == null
                    || !voxelState.HasRoomId(roomId)) {
                return;
            }

            voxelClassValues = voxelState.GetVoxelClassValues(roomId);
            if (!voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                    && !voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                return;
            }

            reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyRemoveVoxelClassValue(
                roomId,
                VoxelClassValues.WALL);

            voxelClassValue = direction < 0 ?
                VoxelClassValues.FLOOR :
                VoxelClassValues.CEILING;

            do {
                i -= direction;
                if (i < 0
                        || i >= reconstructionGrid.GetLength(0)) {
                    return;
                }

                voxelState = reconstructionGrid[i, r, c];
                if (voxelState == null
                        || !voxelState.HasRoomId(roomId)) {
                    return;
                }

                voxelClassValues = voxelState.GetVoxelClassValues(roomId);
                if (voxelClassValues.Contains(voxelClassValue)
                        && voxelClassValues.Contains(VoxelClassValues.WALL)) {

                    reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyRemoveVoxelClassValue(
                        roomId,
                        VoxelClassValues.WALL);
                }
                else {
                    return;
                }

            } while (true);
        }
    }
}
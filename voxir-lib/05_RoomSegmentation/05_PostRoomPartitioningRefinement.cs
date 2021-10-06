using HuePat.VoxIR.Util.Grid;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace HuePat.VoxIR.RoomSegmentation {
    public static class PostRoomPartitioningRefinement {
        public static void RefineReconstructionGridAfterSpacePartitioning(
                double resolution,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            int minWallOpeningWidth = Parameters
                .POST_SPACE_PARTITIONING_REFINEMENT_WALL_OPENING_MIN_WIDTH
                .GetDistanceInVoxels(resolution);
            bool[,,] outsideSpaceGrid = new bool[
                reconstructionGrid.GetLength(0),
                reconstructionGrid.GetLength(1),
                reconstructionGrid.GetLength(2)];

            Parallel.For(
                0,
                reconstructionGrid.GetLength(0),
                i => {

                    // Remove wall opening voxels from the outside, if they do not neighbour interior space
                    ShrinkWallOpeningsFromOutside(
                        i,
                        reconstructionGrid);

                    // Remove wall opening voxels from the inside, if they do not neighbour the outside
                    ShrinkWallOpeningsFromInside(
                        i,
                        reconstructionGrid);

                    InitializeOutsideSpaceGrid(
                        i,
                        outsideSpaceGrid,
                        reconstructionGrid);

                    CloseHolesInInteriorSpace(
                        i,
                        outsideSpaceGrid,
                        reconstructionGrid);

                    // restore missing walls where the interior neighbours the outside
                    RestoreMissingWalls(
                        i,
                        minWallOpeningWidth,
                        normalGrid,
                        reconstructionGrid);

                    // second clean step from inside
                    ShrinkWallOpeningsFromInside(
                        i,
                        reconstructionGrid);
                });

            RemoveFalseInteriorWallFragments(
                resolution,
                normalGrid,
                reconstructionGrid);
        }

        private static void ShrinkWallOpeningsFromOutside(
                int i,
                int[,,][] reconstructionGrid) {

            int r, c;
            int[] voxelState;
            HashSet<(int, int)> segment2D;

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                    voxelState = reconstructionGrid[i, r, c];
                    if (voxelState == null) {
                        continue;
                    }

                    foreach (int roomId in voxelState.GetRoomIds()) {

                        if (!voxelState
                                    .GetVoxelClassValues(roomId)
                                    .Contains(VoxelClassValues.WALL_OPENING)
                                || !IsAdjacentToOutside(
                                    false,
                                    i,
                                    r,
                                    c,
                                    reconstructionGrid)) {
                            continue;
                        }

                        segment2D = HorizontallySegmentWallOpeningWithoutContactToRoomInterior(
                            i,
                            r,
                            c,
                            roomId,
                            reconstructionGrid);

                        RemoveSegment2DFromRoom(
                            i,
                            roomId,
                            reconstructionGrid,
                            segment2D);
                    }
                }
            }
        }

        private static bool IsAdjacentToOutside(
                bool considerDiagonalAdjacency,
                int i,
                int r,
                int c,
                int[,,][] reconstructionGrid) {

            int dr, r2, dc, c2;

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    if (!considerDiagonalAdjacency
                            && dr.Abs() == dc.Abs()) {
                        continue;
                    }

                    r2 = r + dr;
                    c2 = c + dc;
                    if (r2 < 0 || c2 < 0
                            || r2 >= reconstructionGrid.GetLength(1)
                            || c2 >= reconstructionGrid.GetLength(2)) {
                        continue;
                    }

                    if (reconstructionGrid[i, r2, c2] == null) {
                        return true;
                    }
                }
            }

            return false;
        }

        private static HashSet<(int, int)> HorizontallySegmentWallOpeningWithoutContactToRoomInterior(
                int i,
                int r,
                int c,
                int roomId,
                int[,,][] reconstructionGrid) {

            int dr, r2, dc, c2;
            (int, int) candidate;
            int[] voxelState;
            Queue<(int, int)> candidates = new Queue<(int, int)>();
            HashSet<(int, int)> segment2D = new HashSet<(int, int)>();

            candidates.Enqueue((r, c));

            do {
                candidate = candidates.Dequeue();

                if (segment2D.Contains(candidate)
                        || IsAdjacentToRoomInterior(
                            i,
                            candidate.Item1,
                            candidate.Item2,
                            reconstructionGrid)) {
                    continue;
                }

                segment2D.Add((
                    candidate.Item1,
                    candidate.Item2));

                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        r2 = candidate.Item1 + dr;
                        c2 = candidate.Item2 + dc;
                        if (r2 < 0 || c2 < 0
                                || r2 >= reconstructionGrid.GetLength(1)
                                || c2 >= reconstructionGrid.GetLength(2)) {
                            continue;
                        }

                        voxelState = reconstructionGrid[i, r2, c2];
                        if (voxelState != null
                                && voxelState.HasRoomId(roomId)
                                && voxelState
                                    .GetVoxelClassValues(roomId)
                                    .Contains(VoxelClassValues.WALL_OPENING)
                                && !segment2D.Contains((r2, c2))) {
                            candidates.Enqueue((r2, c2));
                        }
                    }
                }
            } while (candidates.Count > 0);

            return segment2D;
        }

        private static bool IsAdjacentToRoomInterior(
                int i,
                int r,
                int c,
                int[,,][] reconstructionGrid) {

            int dr, r2, dc, c2;
            int[] voxelState;

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

                    voxelState = reconstructionGrid[i, r2, c2];
                    if (voxelState != null
                            && voxelState
                                .GetVoxelClassValues()
                                .Any(voxelClassValue =>
                                    voxelClassValue == VoxelClassValues.EMPTY_INTERIOR
                                        || voxelClassValue == VoxelClassValues.INTERIOR_OBJECT)) {
                        return true;
                    }
                }
            }
            return false;
        }

        private static void RemoveSegment2DFromRoom(
                int i,
                int roomId,
                int[,,][] reconstructionGrid,
                HashSet<(int, int)> segment2D) {

            foreach ((int, int) position in segment2D) {
                reconstructionGrid[
                        i,
                        position.Item1,
                        position.Item2]
                    = reconstructionGrid[
                        i,
                        position.Item1,
                        position.Item2].CopyRemoveRoom(roomId);
            }
        }

        private static void ShrinkWallOpeningsFromInside(
                int i,
                int[,,][] reconstructionGrid) {

            int r, c;
            int[] voxelState;

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                    voxelState = reconstructionGrid[i, r, c];
                    if (voxelState == null
                            || IsAdjacentToOutside(
                                false,
                                i,
                                r,
                                c,
                                reconstructionGrid)) {
                        continue;
                    }

                    foreach (int roomId in voxelState.GetRoomIds()) {

                        if (!voxelState
                                .GetVoxelClassValues(roomId)
                                .Contains(VoxelClassValues.WALL_OPENING)) {
                            continue;
                        }

                        reconstructionGrid[i, r, c] = voxelState.CopyChangeVoxelClassValues(
                            roomId,
                            IsAdjacentToRoomInterior(
                                    i,
                                    r,
                                    c,
                                    roomId,
                                    reconstructionGrid) ?
                                VoxelClassValues.EMPTY_INTERIOR :
                                VoxelClassValues.WALL);
                    }
                }
            }
        }

        private static bool IsAdjacentToRoomInterior(
                int i,
                int r,
                int c,
                int roomId,
                int[,,][] reconstructionGrid) {

            int dr, r2, dc, c2;
            int[] voxelState;
            int[] voxelClassValues;

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

                    voxelState = reconstructionGrid[i, r2, c2];
                    if (voxelState == null
                            || !voxelState.HasRoomId(roomId)) {
                        continue;
                    }

                    voxelClassValues = voxelState.GetVoxelClassValues(roomId);
                    if (voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)
                            || voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                        return true;
                    }
                }
            }
            return false;
        }

        private static void InitializeOutsideSpaceGrid(
                int i,
                bool[,,] outsideSpaceGrid,
                int[,,][] reconstructionGrid) {

            int dr, r, r2, dc, c, c2;
            (int, int) candidate;
            Queue<(int, int)> candidates = new Queue<(int, int)>();

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                    if ((i != 0 && r != 0 && c != 0
                                && i != reconstructionGrid.GetLength(0) - 1
                                && r != reconstructionGrid.GetLength(1) - 1
                                && c != reconstructionGrid.GetLength(2) - 1)
                            || outsideSpaceGrid[i, r, c]
                            || reconstructionGrid[i, r, c] != null) {
                        continue;
                    }

                    candidates.Enqueue((r, c));
                    do {

                        candidate = candidates.Dequeue();
                        if (outsideSpaceGrid[
                                i,
                                candidate.Item1,
                                candidate.Item2]) {
                            continue;
                        }

                        outsideSpaceGrid[
                            i,
                            candidate.Item1,
                            candidate.Item2] = true;

                        for (dr = -1; dr <= 1; dr++) {
                            for (dc = -1; dc <= 1; dc++) {

                                if (dr.Abs() == dc.Abs()) {
                                    continue;
                                }

                                r2 = candidate.Item1 + dr;
                                c2 = candidate.Item2 + dc;
                                if (r2 >= 0 && c2 >= 0
                                        && r2 < reconstructionGrid.GetLength(1)
                                        && c2 < reconstructionGrid.GetLength(2)
                                        && !outsideSpaceGrid[i, r2, c2]
                                        && reconstructionGrid[i, r2, c2] == null) {
                                    candidates.Enqueue((r2, c2));
                                }
                            }
                        }
                    } while (candidates.Count > 0);
                }
            }
        }

        private static void CloseHolesInInteriorSpace(
                int i,
                bool[,,] outsideSpaceGrid,
                int[,,][] reconstructionGrid) {

            int r, c;
            HashSet<int> adjacentRoomIds;
            HashSet<(int, int)> segment2D;

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                    if (outsideSpaceGrid[i, r, c]
                            || reconstructionGrid[i, r, c] != null) {
                        continue;
                    }

                    segment2D = HorizontallySegmentEmptySpace(
                        i,
                        r,
                        c,
                        outsideSpaceGrid,
                        reconstructionGrid);

                    adjacentRoomIds = GetAdjacentRoomIds(
                        i,
                        reconstructionGrid,
                        segment2D);

                    if (adjacentRoomIds.Count > 1) {
                        foreach ((int, int) position in segment2D) {
                            outsideSpaceGrid[
                                i,
                                position.Item1,
                                position.Item2] = true;
                        }
                    }
                    else {
                        foreach ((int, int) position in segment2D) {
                            reconstructionGrid[
                                i,
                                position.Item1,
                                position.Item2] = VoxelState.CreateVoxelState(
                                    adjacentRoomIds.First(),
                                    VoxelClassValues.EMPTY_INTERIOR);
                        }
                    }
                }
            }
        }

        private static HashSet<(int, int)> HorizontallySegmentEmptySpace(
                int i,
                int r,
                int c,
                bool[,,] outsideSpaceGrid,
                int[,,][] reconstructionGrid) {

            int dr, r2, dc, c2;
            (int, int) candidate;
            Queue<(int, int)> candidates = new Queue<(int, int)>();
            HashSet<(int, int)> segment2D = new HashSet<(int, int)>();

            candidates.Enqueue((r, c));
            do {

                candidate = candidates.Dequeue();
                if (segment2D.Contains(candidate)) {
                    continue;
                }

                segment2D.Add(candidate);

                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (dr.Abs() == dc.Abs()) {
                            continue;
                        }

                        r2 = candidate.Item1 + dr;
                        c2 = candidate.Item2 + dc;
                        if (r2 >= 0 && c2 >= 0
                                && r2 < reconstructionGrid.GetLength(1)
                                && c2 < reconstructionGrid.GetLength(2)
                                && !outsideSpaceGrid[i, r2, c2]
                                && reconstructionGrid[i, r2, c2] == null
                                && !segment2D.Contains((r2, c2))) {
                            candidates.Enqueue((r2, c2));
                        }
                    }
                }
            } while (candidates.Count > 0);

            return segment2D;
        }

        private static HashSet<int> GetAdjacentRoomIds(
                int i,
                int[,,][] reconstructionGrid,
                HashSet<(int, int)> segment2D) {

            int dr, r, dc, c;
            int[] voxelState;
            HashSet<int> adjacentRoomIds = new HashSet<int>();

            foreach ((int, int) position in segment2D) {
                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {
                        if (dr.Abs() == dc.Abs()) {
                            continue;
                        }

                        r = position.Item1 + dr;
                        c = position.Item2 + dc;
                        if (r < 0 || c < 0
                                || r >= reconstructionGrid.GetLength(1)
                                || c >= reconstructionGrid.GetLength(2)) {
                            continue;
                        }

                        voxelState = reconstructionGrid[i, r, c];
                        if (voxelState == null) {
                            continue;
                        }

                        adjacentRoomIds.AddRange(
                            voxelState.GetRoomIds());

                        if (adjacentRoomIds.Count > 1) {
                            break;
                        }
                    }
                    if (adjacentRoomIds.Count > 1) {
                        break;
                    }
                }
                if (adjacentRoomIds.Count > 1) {
                    break;
                }
            }

            return adjacentRoomIds;
        }

        private static void RestoreMissingWalls(
                int i,
                int minWallOpeningWidth,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            int r, c;
            int closedCount;
            int newVoxelClassValue;
            int[] voxelState;
            int[] voxelClassValues;
            HashSet<(int, int)> segment2D;

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                    voxelState = reconstructionGrid[i, r, c];
                    if (voxelState == null
                            || !IsAdjacentToOutside(
                                true,
                                i,
                                r,
                                c,
                                reconstructionGrid)) {
                        continue;
                    }

                    foreach (int roomId in voxelState.GetRoomIds()) {

                        voxelClassValues = voxelState.GetVoxelClassValues(roomId);
                        if (!voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                                && !voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                            continue;
                        }

                        segment2D = HorizontallySegmentInteriorSpaceDirectlyAdjacentToOutside(
                            i,
                            r,
                            c,
                            roomId,
                            normalGrid,
                            reconstructionGrid,
                            out closedCount);

                        newVoxelClassValue =
                            segment2D.Count < minWallOpeningWidth
                                    || (double)closedCount / segment2D.Count
                                        > Parameters.POST_SPACE_PARTITIONING_REFINEMENT_WALL_OPENING_MAX_CLOSED_RATIO ?
                                VoxelClassValues.WALL :
                                VoxelClassValues.WALL_OPENING;

                        foreach ((int, int) position in segment2D) {

                            reconstructionGrid[
                                i,
                                position.Item1,
                                position.Item2] = reconstructionGrid[
                                        i,
                                        position.Item1,
                                        position.Item2]
                                    .CopyChangeVoxelClassValues(
                                        roomId,
                                        newVoxelClassValue);
                        }
                    }
                }
            }
        }

        private static HashSet<(int, int)> HorizontallySegmentInteriorSpaceDirectlyAdjacentToOutside(
                int i,
                int r,
                int c,
                int roomId,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid,
                out int closedCount) {

            int dr, r2, dc, c2;
            (int, int) candidate;
            int[] voxelState;
            int[] voxelClassValues;
            Queue<(int, int)> candidates = new Queue<(int, int)>();
            HashSet<(int, int)> segment2D = new HashSet<(int, int)>();

            closedCount = 0;

            candidates.Enqueue((r, c));
            do {
                candidate = candidates.Dequeue();

                if (segment2D.Contains(candidate)) {
                    continue;
                }

                segment2D.Add(candidate);
                if (normalGrid[
                        i,
                        candidate.Item1,
                        candidate.Item2] != NormalGridValues.EMPTY) {
                    closedCount++;
                }

                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {
                        if (dr.Abs() == dc.Abs()) {
                            continue;
                        }

                        r2 = candidate.Item1 + dr;
                        c2 = candidate.Item2 + dc;
                        if (r2 < 0 || c2 < 0
                                || r2 >= reconstructionGrid.GetLength(1)
                                || c2 >= reconstructionGrid.GetLength(2)
                                || segment2D.Contains((r2, c2))) {
                            continue;
                        }

                        voxelState = reconstructionGrid[i, r2, c2];
                        if (voxelState == null
                                || !voxelState.HasRoomId(roomId)) {
                            continue;
                        }

                        voxelClassValues = voxelState.GetVoxelClassValues(roomId);
                        if (!voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                                && !voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                            continue;
                        }

                        if (IsAdjacentToOutside(
                                true,
                                i,
                                r2,
                                c2,
                                reconstructionGrid)) {

                            candidates.Enqueue((r2, c2));
                        }
                    }
                }
            } while (candidates.Count > 0);

            return segment2D;
        }

        private static void RemoveFalseInteriorWallFragments(
                double resolution,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            int minWallHeight = Parameters
                .POST_SPACE_PARTITIONING_REFINEMENT_MIN_WALL_HEIGHT
                .GetDistanceInVoxels(resolution);
            int maxWallThickness = Parameters
                .MAX_WALL_THICKNESS
                .GetDistanceInVoxels(resolution);
            int maxWallThicknessDiagonal = Parameters
                .MAX_WALL_THICKNESS
                .GetDistanceInVoxels(
                resolution.GetVoxelSizeDiagonal());
            bool[,,] removeCandidateGrid = new bool[
                reconstructionGrid.GetLength(0),
                reconstructionGrid.GetLength(1),
                reconstructionGrid.GetLength(2)];
            bool[,,] isProcessed = new bool[
                reconstructionGrid.GetLength(0),
                reconstructionGrid.GetLength(1),
                reconstructionGrid.GetLength(2)];

            Parallel.For(
                0,
                reconstructionGrid.GetLength(2),
                c => {
                    DetectRemoveCandidates(
                        c,
                        minWallHeight,
                        isProcessed,
                        removeCandidateGrid,
                        reconstructionGrid);
                });

            Parallel.For(
                0,
                reconstructionGrid.GetLength(0),
                i => {
                    RemoveFalseInteriorWallFragments(
                        i,
                        maxWallThickness,
                        maxWallThicknessDiagonal,
                        removeCandidateGrid,
                        normalGrid,
                        reconstructionGrid);
                });
        }

        private static void DetectRemoveCandidates(
                int c,
                int minWallHeight,
                bool[,,] isProcessed,
                bool[,,] removeCandidateGrid,
                int[,,][] reconstructionGrid) {

            int i, i2, r;
            int roomId;
            int stopHeight;
            int[] voxelState;
            int[] voxelClassValues;

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (i = 0; i < reconstructionGrid.GetLength(0); i++) {

                    if (removeCandidateGrid[i, r, c]
                            || isProcessed[i, r, c]) {
                        continue;
                    }

                    voxelState = reconstructionGrid[i, r, c];
                    if (voxelState == null
                            || voxelState.GetRoomCount() != 1) {
                        continue;
                    }

                    roomId = voxelState
                        .GetRoomIds()
                        .First();

                    voxelClassValues = voxelState.GetVoxelClassValues(roomId);
                    if (!voxelClassValues.Contains(VoxelClassValues.WALL)
                            && !voxelClassValues.Contains(VoxelClassValues.WALL_OPENING)) {
                        continue;
                    }

                    if (IsRemoveCandidate(
                            i,
                            r,
                            c,
                            roomId,
                            minWallHeight,
                            reconstructionGrid,
                            out stopHeight)) {

                        stopHeight--;
                        for (i2 = i; i2 <= stopHeight; i2++) {
                            removeCandidateGrid[i2, r, c] = true;
                        }
                    }
                    else {
                        for (i2 = i; i2 <= stopHeight; i2++) {
                            isProcessed[i2, r, c] = true;
                        }
                    }
                }
            }
        }

        private static bool IsRemoveCandidate(
                int i,
                int r,
                int c,
                int roomId,
                int minWallHeight,
                int[,,][] reconstructionGrid,
                out int stopHeight) {

            bool isRemoveCandidate = true;
            int i2 = i + 1;
            int[] voxelState;
            int[] voxelClassValues;

            while (true) {

                if (i2 >= reconstructionGrid.GetLength(0)) {
                    break;
                }

                if (i2 - i >= minWallHeight) {
                    isRemoveCandidate = false;
                    break;
                }

                voxelState = reconstructionGrid[i2, r, c];
                if (voxelState == null
                        || !voxelState.HasRoomId(roomId)
                        || voxelState.HasOtherRoomIds(roomId)) {
                    break;
                }

                voxelClassValues = voxelState.GetVoxelClassValues(roomId);
                if (!voxelClassValues.Contains(VoxelClassValues.WALL)
                        && !voxelClassValues.Contains(VoxelClassValues.WALL_OPENING)) {
                    break;
                }

                i2++;
            }

            stopHeight = i2;

            return isRemoveCandidate;
        }

        private static void RemoveFalseInteriorWallFragments(
                int i,
                int maxWallThickness,
                int maxWallThicknessDiagonal,
                bool[,,] removeCandidateGrid,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            int r, c;
            int roomId;
            int newVoxelClassValue;
            int[] voxelState;
            int[] voxelClassValues;

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                    if (!removeCandidateGrid[i, r, c]) {
                        continue;
                    }

                    roomId = reconstructionGrid[i, r, c]
                        .GetRoomIds()
                        .First();

                    if (CanRemoveFallsInteriorWallVoxel(
                            i,
                            r,
                            c,
                            roomId,
                            maxWallThickness,
                            maxWallThicknessDiagonal,
                            reconstructionGrid)) {

                        voxelState = reconstructionGrid[i, r, c];
                        voxelClassValues = voxelState.GetVoxelClassValues(roomId);

                        if (voxelClassValues.Contains(VoxelClassValues.FLOOR)
                                && voxelClassValues.Contains(VoxelClassValues.WALL)) {
                            newVoxelClassValue = VoxelClassValues.FLOOR;
                        }
                        else if (voxelClassValues.Contains(VoxelClassValues.CEILING)
                                && voxelClassValues.Contains(VoxelClassValues.WALL)) {
                            newVoxelClassValue = VoxelClassValues.CEILING;
                        }
                        else {
                            newVoxelClassValue = normalGrid[i, r, c] == NormalGridValues.EMPTY ?
                                VoxelClassValues.EMPTY_INTERIOR :
                                VoxelClassValues.INTERIOR_OBJECT;
                        }

                        reconstructionGrid[i, r, c] = voxelState.CopyChangeVoxelClassValues(
                            roomId,
                            newVoxelClassValue);
                    }
                }
            }
        }

        private static bool CanRemoveFallsInteriorWallVoxel(
                int i,
                int r,
                int c,
                int roomId,
                int maxWallThickness,
                int maxWallThicknessDiagonal,
                int[,,][] reconstructionGrid) {

            int dr, r2, dc, c2, d, maxD;
            int[] voxelState;
            int[] voxelClassValues;

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {
                    if (dr == 0 && dc == 0) {
                        continue;
                    }

                    maxD = (dr, dc).IsDirectionDiagonal() ?
                        maxWallThicknessDiagonal :
                        maxWallThickness;

                    for (d = 1; d <= maxD; d++) {

                        r2 = r + d * dr;
                        c2 = c + d * dc;
                        if (r2 < 0 || c2 < 0
                                || r2 >= reconstructionGrid.GetLength(1)
                                || c2 >= reconstructionGrid.GetLength(2)) {
                            return false;
                        }

                        voxelState = reconstructionGrid[i, r2, c2];
                        if (voxelState == null
                                || !voxelState.HasRoomId(roomId)
                                || voxelState.HasOtherRoomIds(roomId)) {
                            return false;
                        }

                        voxelClassValues = voxelState.GetVoxelClassValues(roomId);
                        if (voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                                || voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)
                                || ((voxelClassValues.Contains(VoxelClassValues.CEILING)
                                        || voxelClassValues.Contains(VoxelClassValues.FLOOR))
                                    && !voxelClassValues.Contains(VoxelClassValues.WALL))) {
                            break;
                        }
                    }
                }
            }
            return true;
        }
    }
}
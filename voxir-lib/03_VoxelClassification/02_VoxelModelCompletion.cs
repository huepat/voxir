using HuePat.VoxIR.Util.Grid;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace HuePat.VoxIR.VoxelClassification {
    public static class VoxelModelCompletion {
        public static void AssignRoomlessVoxelsToCeilingsAndFloors(
                double resolution,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            int searchDistance = Parameters
                .FLOOR_CEILING_COMPLETION_SEARCH_DISTANCE
                .GetDistanceInVoxels(resolution);

            bool[,,] isRoomless = GetRoomlessVoxels(
                normalGrid,
                reconstructionGrid);

            AssignRoomlessVoxelsToCeilingsAndFloors(
                searchDistance,
                isRoomless,
                normalGrid,
                reconstructionGrid);
        }

        public static void DetectMissingIndoorSpaces(
                int roomCount,
                double resolution,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid,
                HashSet<int> roomIdsWithoutFloor,
                out HashSet<int> roomletIds) {

            bool hasCeiling;
            int id;
            int floorCandidateCount;
            int minFloorArea = Parameters
                .ROOMLET_MIN_AREA
                .GetAreaInVoxels(resolution);
            int minCeilingGuessHeight = Parameters
                .ROOMLET_MIN_CEILING_GUESS_HEIGHT
                .GetDistanceInVoxels(resolution);
            int minCeilingGuessFloorArea = Parameters
                .ROOMLET_MIN_CEILING_GUESS_FLOOR_AREA
                .GetAreaInVoxels(resolution);
            int floorSegmentMinArea = Parameters
                .ROOMLET_MIN_FLOOR_SEGMENT_AREA
                .GetAreaInVoxels(resolution);
            int[,] floorHeightGrid;
            int[,] ceilingHeightGrid;
            bool[,,] isRoomless;
            bool[,,] isFloor = new bool[
                reconstructionGrid.GetLength(0),
                reconstructionGrid.GetLength(1),
                reconstructionGrid.GetLength(2)];
            bool[,,] isSegmentedForCeiling = new bool[
                reconstructionGrid.GetLength(0),
                reconstructionGrid.GetLength(1),
                reconstructionGrid.GetLength(2)];
            int[,,] roomletAssignment = new int[
                reconstructionGrid.GetLength(0),
                reconstructionGrid.GetLength(1),
                reconstructionGrid.GetLength(2)];
            GridBBox3D floorCandidateBBox;
            List<(int, int, int)> ceilingCandidateVoxels;
            List<RoomletFloorCandidate> floorCandidates;
            List<List<(int, int, int)>> floorCandidateSegments;
            List<List<(int, int, int)>> ceilingCandidateSegments;

            roomletIds = new HashSet<int>();

            // find roomlet floor candidate segments

            isRoomless = GetRoomlessVoxels(
                normalGrid,
                reconstructionGrid);

            floorCandidates = SegmentRoomletFloorCandidates(
                resolution,
                isRoomless,
                isFloor,
                normalGrid,
                roomletAssignment,
                reconstructionGrid);

            // find corresponding ceiling candidate segments and write final roomlets to voxelClassification
            id = roomCount;
            foreach (RoomletFloorCandidate floorCandidate in floorCandidates
                    .OrderByDescending(voxels => voxels.Count)) {

                ceilingCandidateVoxels = DetectCeilingCandidateVoxels(
                    resolution,
                    normalGrid,
                    roomletAssignment,
                    reconstructionGrid,
                    floorCandidate,
                    out floorCandidateCount);


                if (floorCandidateCount < minFloorArea) {
                    break;
                }

                ceilingCandidateSegments = GetRoomletCeilingCandidateSegments(
                    floorCandidate.Id,
                    floorCandidateCount,
                    resolution,
                    isFloor,
                    isSegmentedForCeiling,
                    roomletAssignment,
                    ceilingCandidateVoxels);

                hasCeiling = ceilingCandidateSegments.Count > 0;

                if (!hasCeiling
                        && floorCandidateCount < minCeilingGuessFloorArea) {
                    continue;
                }

                // Create Grids
                floorCandidateBBox = new GridBBox3D(floorCandidate);
                InitializeHeightGrids(
                    floorCandidateBBox,
                    floorCandidate,
                    ceilingCandidateSegments,
                    out floorHeightGrid,
                    out ceilingHeightGrid);

                // close floor holes
                CloseFloorHoles(
                    hasCeiling,
                    minCeilingGuessHeight,
                    resolution,
                    floorHeightGrid,
                    normalGrid,
                    floorCandidateBBox);

                // set ceiling height
                SetCeilingHeight(
                    hasCeiling,
                    minCeilingGuessHeight,
                    floorHeightGrid,
                    ceilingHeightGrid,
                    normalGrid,
                    reconstructionGrid,
                    floorCandidateBBox,
                    ceilingCandidateVoxels);

                // remove voxel columns from roomlet that intersect with a room
                ResolveOverlapsWithRooms(
                    floorHeightGrid,
                    ceilingHeightGrid,
                    reconstructionGrid,
                    floorCandidateBBox);

                floorCandidateSegments = ResegmentFloorCandidate(
                    floorHeightGrid,
                    floorCandidateBBox);

                foreach (List<(int, int, int)> floorCandidateSegment in floorCandidateSegments) {

                    if (floorCandidateSegment.Count < minFloorArea) {
                        continue;
                    }

                    if (floorCandidateSegment.Count < floorSegmentMinArea) {
                        roomletIds.Add(id);
                    }

                    // add voxels to voxelClassification and do classification sweep
                    AddToReconstructionGrid(
                        id,
                        floorHeightGrid,
                        ceilingHeightGrid,
                        normalGrid,
                        reconstructionGrid,
                        floorCandidateBBox,
                        roomIdsWithoutFloor,
                        floorCandidateSegment);

                    // complete ceiling/floor
                    CompleteCeilingsAndFloors(
                        id,
                        resolution,
                        floorHeightGrid,
                        ceilingHeightGrid,
                        normalGrid,
                        reconstructionGrid,
                        floorCandidateBBox,
                        floorCandidateSegment);

                    id++;
                }
            }
        }

        private static bool[,,] GetRoomlessVoxels(
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            bool[,,] isRoomless = new bool[
                reconstructionGrid.GetLength(0),
                reconstructionGrid.GetLength(1),
                reconstructionGrid.GetLength(2)];

            Parallel.For(
                0,
                reconstructionGrid.GetLength(2),
                c => {

                    int i, r;

                    for (i = 0; i < reconstructionGrid.GetLength(0); i++) {
                        for (r = 0; r < reconstructionGrid.GetLength(1); r++) {

                            if (reconstructionGrid[i, r, c] == null
                                    && normalGrid[i, r, c] != NormalGridValues.EMPTY) {

                                isRoomless[i, r, c] = true;
                            }
                        }
                    }
                });

            return isRoomless;
        }

        private static void AssignRoomlessVoxelsToCeilingsAndFloors(
                int searchDistance,
                bool[,,] isRoomless,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            Parallel.For(
                0,
                reconstructionGrid.GetLength(2),
                c => {

                    int i, r;
                    int directionIndex, assignmentDirectionIndex;
                    bool[] canAssign = new bool[2];
                    int[] directions;
                    int[] iMax = new int[2];
                    List<int>[] roomIdsPerDirection = new List<int>[2];
                    List<int[]>[] voxelClassValuesPerDirection = new List<int[]>[2];

                    for (i = 0; i < reconstructionGrid.GetLength(0); i++) {
                        for (r = 0; r < reconstructionGrid.GetLength(1); r++) {

                            if (!isRoomless[i, r, c]) {
                                continue;
                            }

                            directions = GetVerticalGridNormalDirections(normalGrid[i, r, c]);

                            for (directionIndex = 0; directionIndex < directions.Length; directionIndex++) {

                                UpdateVerticalDirection(
                                    i,
                                    r,
                                    c,
                                    directionIndex,
                                    searchDistance,
                                    canAssign,
                                    directions,
                                    iMax,
                                    isRoomless,
                                    reconstructionGrid,
                                    roomIdsPerDirection,
                                    voxelClassValuesPerDirection);
                            }

                            if (!Enumerable
                                    .Range(0, directions.Length)
                                    .Any(index => canAssign[index])) {
                                continue;
                            }

                            assignmentDirectionIndex = GetAssignmentDirectionIndex(
                                i,
                                canAssign,
                                directions,
                                iMax);

                            AssignRoomlessVoxelsToCeilingsAndFloors(
                                i,
                                r,
                                c,
                                assignmentDirectionIndex,
                                directions,
                                iMax,
                                reconstructionGrid,
                                roomIdsPerDirection,
                                voxelClassValuesPerDirection);
                        }
                    }
                });
        }

        private static int[] GetVerticalGridNormalDirections(
                byte normalGridValue) {

            if (normalGridValue == NormalGridValues.NORMAL_HORIZONTAL) {
                return new int[] { -1, 1 };
            }
            else if (normalGridValue == NormalGridValues.NORMAL_DOWN) {
                return new int[] { 1 };
            }
            else {
                return new int[] { -1 };
            }
        }

        private static void UpdateVerticalDirection(
                int i,
                int r,
                int c,
                int directionIndex,
                int searchDistance,
                bool[] canAssign,
                int[] directions,
                int[] iMax,
                bool[,,] isRoomless,
                int[,,][] reconstructionGrid,
                List<int>[] roomIdsPerDirection,
                List<int[]>[] voxelClassValuesPerDirection) {

            int i2 = i;
            int[] voxelState;
            int[] voxelClassValues;

            canAssign[directionIndex] = false;
            do {

                i2 += directions[directionIndex];
                if (i2 < 0
                        || i2 >= reconstructionGrid.GetLength(0)
                        || (i2 - i).Abs() >= searchDistance) {
                    break;
                }

                if (isRoomless[i2, r, c]) {
                    continue;
                }

                voxelState = reconstructionGrid[i2, r, c];
                if (voxelState == null) {
                    continue;
                }

                foreach (int roomId in voxelState.GetRoomIds()) {

                    voxelClassValues = voxelState.GetVoxelClassValues(roomId);

                    if ((directions[directionIndex] == 1
                                && voxelClassValues.Contains(VoxelClassValues.CEILING))
                            || (directions[directionIndex] == -1
                                && voxelClassValues.Contains(VoxelClassValues.FLOOR))) {

                        if (!canAssign[directionIndex]) {
                            roomIdsPerDirection[directionIndex] = new List<int>();
                            voxelClassValuesPerDirection[directionIndex] = new List<int[]>();
                        }

                        canAssign[directionIndex] = true;
                        roomIdsPerDirection[directionIndex].Add(roomId);
                        voxelClassValuesPerDirection[directionIndex].Add(voxelClassValues);
                    }
                }
                break;
            } while (true);

            iMax[directionIndex] = i2 - directions[directionIndex];
        }

        private static int GetAssignmentDirectionIndex(
                int i,
                bool[] canAssign,
                int[] directions,
                int[] iMax) {

            int directionIndex;
            int assignmentDirectionIndex = -1;
            int distance;
            int minDistance = int.MaxValue;

            for (directionIndex = 0; directionIndex < directions.Length; directionIndex++) {

                distance = (iMax[directionIndex] - i).Abs();
                if (canAssign[directionIndex] && distance < minDistance) {
                    minDistance = distance;
                    assignmentDirectionIndex = directionIndex;
                }
            }

            return assignmentDirectionIndex;
        }

        private static void AssignRoomlessVoxelsToCeilingsAndFloors(
                int i,
                int r,
                int c,
                int assignmentDirectionIndex,
                int[] directions,
                int[] iMax,
                int[,,][] reconstructionGrid,
                List<int>[] roomIdsPerDirection,
                List<int[]>[] voxelClassValuesPerDirection) {

            int directionIndex;

            i -= directions[assignmentDirectionIndex];

            do {

                i += directions[assignmentDirectionIndex];

                for (directionIndex = 0; directionIndex < roomIdsPerDirection[assignmentDirectionIndex].Count; directionIndex++) {

                    reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyChangeVoxelClassValues(
                        roomIdsPerDirection[assignmentDirectionIndex][directionIndex],
                        voxelClassValuesPerDirection[assignmentDirectionIndex][directionIndex]);
                }

                if (i == iMax[assignmentDirectionIndex]) {
                    break;
                }
            } while (true);
        }

        private static List<RoomletFloorCandidate> SegmentRoomletFloorCandidates(
                double resolution,
                bool[,,] isRoomless,
                bool[,,] isFloor,
                byte[,,] normalGrid,
                int[,,] roomletAssignment,
                int[,,][] reconstructionGrid) {

            int di, i, i2, dr, r, r2, dc, c, c2, id;
            int maxHeightDifference = Parameters
                .FLOOR_MAX_HEIGHT_DIFFERENCE
                .GetDistanceInVoxels(resolution);
            bool[,,] isSegmented = new bool[
                reconstructionGrid.GetLength(0),
                reconstructionGrid.GetLength(1),
                reconstructionGrid.GetLength(2)];
            List<(int, int, int)> segment;
            Queue<(int, int, int)> candidates = new Queue<(int, int, int)>();
            List<RoomletFloorCandidate> roomletFloorCandidates = new List<RoomletFloorCandidate>();

            for (i = 0; i < reconstructionGrid.GetLength(0); i++) {
                for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                    for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                        if (!isRoomless[i, r, c]
                                || isSegmented[i, r, c]
                                || normalGrid[i, r, c] != NormalGridValues.NORMAL_UP) {
                            continue;
                        }

                        segment = new List<(int, int, int)>();
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

                                    if (dr == 0 && dc == 0) {
                                        continue;
                                    }

                                    r2 = candidate.Item2 + dr;
                                    c2 = candidate.Item3 + dc;
                                    if (r2 < 0 || c2 < 0
                                            || r2 >= reconstructionGrid.GetLength(1)
                                            || c2 >= reconstructionGrid.GetLength(2)) {
                                        continue;
                                    }

                                    if (isSegmented[candidate.Item1, r2, c2]) {
                                        continue;
                                    }

                                    if (isRoomless[candidate.Item1, r2, c2]
                                            && normalGrid[candidate.Item1, r2, c2] == NormalGridValues.NORMAL_UP) {
                                        candidates.Enqueue((candidate.Item1, r2, c2));
                                        continue;
                                    }

                                    for (di = 1; di <= maxHeightDifference; di++) {

                                        i2 = candidate.Item1 + di;
                                        if (i2 >= reconstructionGrid.GetLength(0)
                                                || isSegmented[i2, r2, c2]) {
                                            break;
                                        }

                                        if (isRoomless[i2, r2, c2]
                                                && normalGrid[i2, r2, c2] == NormalGridValues.NORMAL_UP) {
                                            candidates.Enqueue((i2, r2, c2));
                                            break;
                                        }

                                        i2 = candidate.Item1 - di;
                                        if (i2 < 0
                                                || isSegmented[i2, r2, c2]) {
                                            break;
                                        }

                                        if (isRoomless[i2, r2, c2]
                                                && normalGrid[i2, r2, c2] == NormalGridValues.NORMAL_UP) {
                                            candidates.Enqueue((i2, r2, c2));
                                            break;
                                        }
                                    }
                                }
                            }
                        } while (candidates.Count > 0);

                        id = roomletFloorCandidates.Count;

                        foreach ((int, int, int) voxel in segment) {
                            isFloor[
                                voxel.Item1,
                                voxel.Item2,
                                voxel.Item3] = true;
                            roomletAssignment[
                                voxel.Item1,
                                voxel.Item2,
                                voxel.Item3] = id;
                        }

                        roomletFloorCandidates.Add(
                            new RoomletFloorCandidate(id, segment));
                    }
                }
            }
            return roomletFloorCandidates;
        }

        private static List<(int, int, int)> DetectCeilingCandidateVoxels(
                double resolution,
                byte[,,] normalGrid,
                int[,,] roomletAssignment,
                int[,,][] reconstructionGrid,
                RoomletFloorCandidate roomletFloorCandidate,
                out int floorCandidateCount) {

            bool incrementFloorCandidateCount;
            int i, r, c;
            int minHeight = (int)(Parameters.ROOMLET_MIN_HEIGHT / resolution);
            int[] voxelState;
            List<(int, int, int)> ceilingCandidateVoxels = new List<(int, int, int)>();

            floorCandidateCount = 0;

            foreach ((int, int, int) voxel in roomletFloorCandidate) {

                r = voxel.Item2;
                c = voxel.Item3;
                incrementFloorCandidateCount = false;

                for (i = voxel.Item1 - 1; i >= 0; i--) {

                    if (roomletAssignment[i, r, c] == roomletFloorCandidate.Id) {
                        break;
                    }

                    incrementFloorCandidateCount = true;
                    voxelState = reconstructionGrid[i, r, c];

                    if (voxelState != null) {

                        foreach (int roomId in voxelState.GetRoomIds()) {

                            if (voxelState
                                    .GetVoxelClassValues(roomId)
                                    .Contains(VoxelClassValues.FLOOR)) {

                                roomletAssignment[i, r, c] = roomletFloorCandidate.Id;
                                ceilingCandidateVoxels.Add((i, r, c));
                                break;
                            }
                        }
                        break;
                    }

                    if (normalGrid[i, r, c] != NormalGridValues.EMPTY
                            && voxel.Item1 - i >= minHeight) {
                        roomletAssignment[i, r, c] = roomletFloorCandidate.Id;
                        ceilingCandidateVoxels.Add((i, r, c));
                        break;
                    }
                }

                if (incrementFloorCandidateCount) {
                    floorCandidateCount++;
                }
            }

            return ceilingCandidateVoxels;
        }

        private static List<List<(int, int, int)>> GetRoomletCeilingCandidateSegments(
                int roomletFloorCandidateId,
                int floorCandidateCount,
                double resolution,
                bool[,,] isFloor,
                bool[,,] isSegmented,
                int[,,] roomletAssignment,
                List<(int, int, int)> ceilingCandidateVoxels) {

            int minCeilingArea = Parameters
                .ROOMLET_MIN_AREA
                .GetAreaInVoxels(resolution);
            List<List<(int, int, int)>> ceilingSegments = new List<List<(int, int, int)>>();

            if ((double)ceilingCandidateVoxels.Count / floorCandidateCount
                    < Parameters.ROOMLET_MIN_CEILING_COMPLETENESS) {
                return ceilingSegments;
            }

            // only keep roomlet candidates with sufficiently large connected ceiling
            ceilingSegments = SegmentRoomletCeilingCandidateVoxels(
                roomletFloorCandidateId,
                isFloor,
                isSegmented,
                roomletAssignment,
                ceilingCandidateVoxels);

            ceilingSegments = ceilingSegments
                .Where(ceilingSegment => ceilingSegment.Count >= minCeilingArea)
                .ToList();

            return ceilingSegments;
        }

        private static List<List<(int, int, int)>> SegmentRoomletCeilingCandidateVoxels(
                int roomletFloorCandidateId,
                bool[,,] isFloor,
                bool[,,] isSegmented,
                int[,,] roomletAssignment,
                List<(int, int, int)> ceilingCandidateVoxels) {

            int di, i, dr, r, dc, c;
            Queue<(int, int, int)> candidates = new Queue<(int, int, int)>();
            List<List<(int, int, int)>> ceilingSegments = new List<List<(int, int, int)>>();

            foreach ((int, int, int) voxel in ceilingCandidateVoxels) {
                
                if (isSegmented[
                            voxel.Item1,
                            voxel.Item2,
                            voxel.Item3]
                        || roomletAssignment[
                            voxel.Item1,
                            voxel.Item2,
                            voxel.Item3] != roomletFloorCandidateId
                        || isFloor[
                            voxel.Item1,
                            voxel.Item2,
                            voxel.Item3]) {
                    continue;
                }

                candidates.Enqueue(voxel);
                List<(int, int, int)> segment = new List<(int, int, int)>();
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

                                i = candidate.Item1 + di;
                                r = candidate.Item2 + dr;
                                c = candidate.Item3 + dc;
                                if (i >= 0 && r >= 0 && c >= 0
                                        && i < isSegmented.GetLength(0)
                                        && r < isSegmented.GetLength(1)
                                        && c < isSegmented.GetLength(2)
                                        && !isSegmented[i, r, c]
                                        && !isFloor[i, r, c]
                                        && roomletAssignment[i, r, c] == roomletFloorCandidateId) {
                                    candidates.Enqueue((i, r, c));
                                }
                            }
                        }
                    }
                } while (candidates.Count > 0);
                ceilingSegments.Add(segment);
            }

            return ceilingSegments;
        }

        private static void InitializeHeightGrids(
                GridBBox3D floorCandidateBBox,
                RoomletFloorCandidate floorCandidate,
                List<List<(int, int, int)>> ceilingCandidateSegments,
                out int[,] floorHeightGrid,
                out int[,] ceilingHeightGrid) {

            int r, c;

            floorHeightGrid = new int[
                floorCandidateBBox.Size.Item2,
                floorCandidateBBox.Size.Item3];
            ceilingHeightGrid = new int[
                floorCandidateBBox.Size.Item2,
                floorCandidateBBox.Size.Item3];

            for (r = 0; r < floorCandidateBBox.Size.Item2; r++) {
                for (c = 0; c < floorCandidateBBox.Size.Item3; c++) {
                    floorHeightGrid[r, c] = -1;
                    ceilingHeightGrid[r, c] = -1;
                }
            }

            foreach ((int, int, int) voxel in floorCandidate) {
                r = voxel.Item2 - floorCandidateBBox.Min.Item2;
                c = voxel.Item3 - floorCandidateBBox.Min.Item3;
                if (floorHeightGrid[r, c] < 0
                        || floorHeightGrid[r, c] > voxel.Item1) {
                    floorHeightGrid[r, c] = voxel.Item1;
                }
            }

            foreach (List<(int, int, int)> ceilingSegment in ceilingCandidateSegments) {
                foreach ((int, int, int) voxel in ceilingSegment) {
                    ceilingHeightGrid[
                        voxel.Item2 - floorCandidateBBox.Min.Item2,
                        voxel.Item3 - floorCandidateBBox.Min.Item3] = voxel.Item1;
                }
            }
        }

        private static void CloseFloorHoles(
                bool hasCeiling,
                int minCeilingGuessHeight,
                double resolution,
                int[,] floorHeightGrid,
                byte[,,] normalGrid,
                GridBBox3D floorCandidateBBox) {

            bool[,] holeGrid;
            List<List<(int, int)>> holes;

            if (hasCeiling) {
                DetectAndInterpolateFloorHoles(
                    resolution,
                    floorHeightGrid,
                    floorCandidateBBox);
            }
            else {
                holeGrid = DetectHoles(floorHeightGrid);
                holes = SegmentHoles(
                    holeGrid,
                    floorHeightGrid);
                RemoveHolesWithClosedBorder(
                    minCeilingGuessHeight,
                    holeGrid,
                    floorHeightGrid,
                    normalGrid,
                    floorCandidateBBox,
                    holes);
                InterpolateHoleHeight(
                    holeGrid,
                    floorHeightGrid);
            }
        }

        private static bool[,] DetectHoles(
                int[,] floorHeightGrid) {

            bool isHole;
            int dr, r, r2, dc, c, c2, d, dMax;
            bool[,] holeGrid = new bool[
                floorHeightGrid.GetLength(0),
                floorHeightGrid.GetLength(1)];

            for (r = 0; r < floorHeightGrid.GetLength(0); r++) {
                for (c = 0; c < floorHeightGrid.GetLength(1); c++) {

                    if (floorHeightGrid[r, c] < 0) {
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
                                    || r2 >= floorHeightGrid.GetLength(0)
                                    || c2 >= floorHeightGrid.GetLength(1)
                                    || floorHeightGrid[r2, c2] >= 0) {
                                continue;
                            }

                            d = 2;
                            isHole = false;
                            while (true) {

                                r2 = r + d * dr;
                                c2 = c + d * dc;
                                if (r2 < 0 || c2 < 0
                                        || r2 >= floorHeightGrid.GetLength(0)
                                        || c2 >= floorHeightGrid.GetLength(1)) {
                                    break;
                                }

                                if (floorHeightGrid[r2, c2] >= 0) {
                                    isHole = true;
                                    break;
                                }
                                d++;
                            }

                            if (!isHole) {
                                continue;
                            }

                            dMax = d - 1;
                            for (d = 1; d <= dMax; d++) {
                                holeGrid[
                                    r + d * dr,
                                    c + d * dc] = true;
                            }
                        }
                    }
                }
            }

            return holeGrid;
        }

        private static List<List<(int, int)>> SegmentHoles(
                bool[,] holeGrid,
                int[,] floorHeightGrid) {

            int dr, r, r2, dc, c, c2;
            bool[,] isHoleSegmented = new bool[
                floorHeightGrid.GetLength(0),
                floorHeightGrid.GetLength(1)];
            Queue<(int, int)> candidates = new Queue<(int, int)>();
            List<List<(int, int)>> holes = new List<List<(int, int)>>();

            for (r = 0; r < floorHeightGrid.GetLength(0); r++) {
                for (c = 0; c < floorHeightGrid.GetLength(1); c++) {

                    if (!holeGrid[r, c]
                            || isHoleSegmented[r, c]) {
                        continue;
                    }

                    List<(int, int)> hole = new List<(int, int)>();
                    candidates.Enqueue((r, c));
                    do {

                        (int, int) candidate = candidates.Dequeue();
                        if (isHoleSegmented[
                                candidate.Item1,
                                candidate.Item2]) {
                            continue;
                        }
                        isHoleSegmented[
                            candidate.Item1,
                            candidate.Item2] = true;
                        hole.Add(candidate);

                        for (dr = -1; dr <= 1; dr++) {
                            for (dc = -1; dc <= 1; dc++) {

                                if (dr.Abs() == dc.Abs()) {
                                    continue;
                                }

                                r2 = candidate.Item1 + dr;
                                c2 = candidate.Item2 + dc;
                                if (r2 >= 0 && c2 >= 0
                                        && r2 < floorHeightGrid.GetLength(0)
                                        && c2 < floorHeightGrid.GetLength(1)
                                        && holeGrid[r2, c2]
                                        && !isHoleSegmented[r2, c2]) {
                                    candidates.Enqueue((r2, c2));
                                }
                            }
                        }
                    } while (candidates.Count > 0);
                    holes.Add(hole);
                }
            }

            return holes;
        }

        private static void RemoveHolesWithClosedBorder(
                int minCeilingGuessHeight,
                bool[,] holeGrid,
                int[,] floorHeightGrid,
                byte[,,] normalGrid,
                GridBBox3D floorCandidateBBox,
                List<List<(int, int)>> holes) {

            int i, i2, dr, r, r2, dc, c, c2;
            byte normalValue;
            int borderCount, closedBorderCount;
            bool[,] isHoleSegmented;

            foreach (List<(int, int)> hole in holes) {

                isHoleSegmented = new bool[
                    floorHeightGrid.GetLength(0),
                    floorHeightGrid.GetLength(1)];
                borderCount = closedBorderCount = 0;

                foreach ((int, int) position in hole) {
                    for (dr = -1; dr <= 1; dr++) {
                        for (dc = -1; dc <= 1; dc++) {

                            r = position.Item1 + dr;
                            c = position.Item2 + dc;
                            if (r < 0 || c < 0
                                    || r >= floorHeightGrid.GetLength(0)
                                    || c >= floorHeightGrid.GetLength(1)
                                    || isHoleSegmented[r, c]) {
                                continue;
                            }

                            i = floorHeightGrid[r, c];
                            if (i < 0) {
                                continue;
                            }

                            borderCount++;
                            isHoleSegmented[r, c] = true;
                            r2 = r + floorCandidateBBox.Min.Item2;
                            c2 = c + floorCandidateBBox.Min.Item3;
                            for (i2 = i - 1; i2 >= i - minCeilingGuessHeight; i2--) {
                                if (i2 < 0) {
                                    break;
                                }
                                normalValue = normalGrid[i2, r2, c2];
                                if (normalValue == NormalGridValues.NORMAL_DOWN) {
                                    break;
                                }
                                if (normalValue == NormalGridValues.NORMAL_HORIZONTAL) {
                                    closedBorderCount++;
                                    break;
                                }
                            }
                        }
                    }
                }

                if ((double)closedBorderCount / borderCount
                        >= Parameters.HOLE_BORDER_WALL_MAX_COMPLETENESS) {
                    foreach ((int, int) position in hole) {
                        holeGrid[
                            position.Item1,
                            position.Item2] = false;
                    }
                }
            }
        }

        private static void InterpolateHoleHeight(
                bool[,] holeGrid,
                int[,] floorHeightGrid) {

            bool found;
            int dr, r, r2, dc, c, c2, d, dMax;
            double dh;

            for (r = 0; r < floorHeightGrid.GetLength(0); r++) {
                for (c = 0; c < floorHeightGrid.GetLength(1); c++) {

                    if (floorHeightGrid[r, c] < 0) {
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
                                    || r2 >= floorHeightGrid.GetLength(0)
                                    || c2 >= floorHeightGrid.GetLength(1)
                                    || !holeGrid[r2, c2]) {
                                continue;
                            }

                            d = 2;
                            found = false;
                            while (true) {

                                r2 = r + d * dr;
                                c2 = c + d * dc;
                                if (r2 < 0 || c2 < 0
                                    || r2 >= floorHeightGrid.GetLength(0)
                                    || c2 >= floorHeightGrid.GetLength(1)) {
                                    break;
                                }

                                if (floorHeightGrid[r2, c2] >= 0) {
                                    found = true;
                                    break;
                                }

                                d++;
                            }

                            if (found) {

                                dMax = d - 1;
                                dh = (double)(floorHeightGrid[r2, c2] - floorHeightGrid[r, c]) / dMax;

                                for (d = 1; d <= dMax; d++) {

                                    floorHeightGrid[
                                        r + d * dr,
                                        c + d * dc] = floorHeightGrid[r, c] + (int)(d * dh);
                                }
                            }
                        }
                    }
                }
            }
        }

        private static void DetectAndInterpolateFloorHoles(
                double resolution,
                int[,] floorHeightGrid,
                GridBBox3D floorCandidateBBox) {

            bool isHoleCandidate, isHole;
            int j, dr, r, r2, dc, c, c2, d, dMax;
            int maxHoleClosingDistance = Parameters
                .ROOMLET_FLOOR_HOLE_CLOSING_MAX_DISTANCE
                .GetDistanceInVoxels(resolution);
            double dh;
            List<(int, int)> pixels;

            r2 = c2 = 0;
            dMax = maxHoleClosingDistance + 1;

            for (r = 0; r < floorCandidateBBox.Size.Item2; r++) {
                for (c = 0; c < floorCandidateBBox.Size.Item3; c++) {

                    if (floorHeightGrid[r, c] < 0) {
                        continue;
                    }

                    for (dr = -1; dr <= 1; dr++) {
                        for (dc = -1; dc <= 1; dc++) {

                            if (dr.Abs() == dc.Abs()) {
                                continue;
                            }

                            isHoleCandidate = isHole = false;
                            pixels = new List<(int, int)>();

                            for (d = 1; d <= dMax; d++) {

                                r2 = r + d * dr;
                                c2 = c + d * dc;
                                if (r2 < 0 || c2 < 0
                                        || r2 >= floorCandidateBBox.Size.Item2
                                        || c2 >= floorCandidateBBox.Size.Item3) {
                                    break;
                                }

                                if (floorHeightGrid[r2, c2] >= 0) {
                                    if (isHoleCandidate) {
                                        isHole = true;
                                    }
                                    break;
                                }

                                isHoleCandidate = true;
                                pixels.Add((r2, c2));
                            }

                            if (!isHole) {
                                continue;
                            }

                            dh = (double)(floorHeightGrid[r2, c2] - floorHeightGrid[r, c]) / pixels.Count;
                            for (j = 0; j < pixels.Count; j++) {
                                floorHeightGrid[
                                    pixels[j].Item1,
                                    pixels[j].Item2] = floorHeightGrid[r, c] + (int)(j * dh);
                            }
                        }
                    }
                }
            }
        }

        private static void SetCeilingHeight(
                bool hasCeiling,
                int minCeilingGuessHeight,
                int[,] floorHeightGrid,
                int[,] ceilingHeightGrid,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid,
                GridBBox3D floorCandidateBBox,
                List<(int, int, int)> ceilingCandidateVoxels) {

            int ceilingHeight;
            Dictionary<int, int> heightCounters;
            HashSet<(int, int)> heightCounterPixels;

            if (hasCeiling) {
                SetCeilingHeight(
                    floorHeightGrid,
                    ceilingHeightGrid,
                    floorCandidateBBox);
            }
            else {

                heightCounters = new Dictionary<int, int>();
                heightCounterPixels = new HashSet<(int, int)>();

                InitializeHeightCounter(
                    minCeilingGuessHeight,
                    floorHeightGrid,
                    floorCandidateBBox,
                    heightCounters,
                    heightCounterPixels,
                    ceilingCandidateVoxels);

                if (heightCounters.Count == 0) {

                    SearchOtherCeilingCandidatesAndUpdateHeightCounter(
                        minCeilingGuessHeight,
                        floorHeightGrid,
                        ceilingHeightGrid,
                        normalGrid,
                        reconstructionGrid,
                        floorCandidateBBox,
                        heightCounters,
                        heightCounterPixels);
                }

                ceilingHeight = DetermineCeilingHeight(
                    minCeilingGuessHeight,
                    heightCounters);

                SetCeilingHeight(
                    ceilingHeight,
                    floorHeightGrid,
                    ceilingHeightGrid,
                    normalGrid,
                    reconstructionGrid,
                    floorCandidateBBox);
            }
        }

        private static void SetCeilingHeight(
                int[,] floorHeightGrid,
                int[,] ceilingHeightGrid,
                GridBBox3D floorCandidateBBox) {

            bool found;
            int i, dr, r, dc, c;
            List<(int, int)> pixels = new List<(int, int)>();
            List<(int, int)> newPixels;

            for (r = 0; r < floorCandidateBBox.Size.Item2; r++) {
                for (c = 0; c < floorCandidateBBox.Size.Item3; c++) {
                    if (floorHeightGrid[r, c] >= 0) {
                        pixels.Add((r, c));
                    }
                }
            }

            while (pixels.Count > 0) {

                newPixels = new List<(int, int)>();
                foreach ((int, int) pixel in pixels) {

                    if (ceilingHeightGrid[
                            pixel.Item1,
                            pixel.Item2] >= 0) {
                        continue;
                    }

                    found = false;
                    i = int.MaxValue;

                    for (dr = -1; dr <= 1; dr++) {
                        for (dc = -1; dc <= 1; dc++) {

                            if (dr == 0 && dc == 0) {
                                continue;
                            }

                            r = pixel.Item1 + dr;
                            c = pixel.Item2 + dc;
                            if (r >= 0 && r < floorCandidateBBox.Size.Item2
                                    && c >= 0 && c < floorCandidateBBox.Size.Item3
                                    && ceilingHeightGrid[r, c] >= 0
                                    && ceilingHeightGrid[r, c] < i) {
                                i = ceilingHeightGrid[r, c];
                                found = true;
                            }
                        }
                    }

                    if (found) {
                        ceilingHeightGrid[
                            pixel.Item1,
                            pixel.Item2] = i;
                    }
                    else {
                        newPixels.Add(pixel);
                    }
                }

                pixels = newPixels;
            }
        }

        private static void InitializeHeightCounter(
                int minCeilingGuessHeight,
                int[,] floorHeightGrid,
                GridBBox3D floorCandidateBBox,
                Dictionary<int, int> heightCounters,
                HashSet<(int, int)> heightCounterPixels,
                List<(int, int, int)> ceilingCandidateVoxels) {

            int i;

            foreach ((int, int, int) voxel in ceilingCandidateVoxels) {

                i = floorHeightGrid[
                    voxel.Item2 - floorCandidateBBox.Min.Item2,
                    voxel.Item3 - floorCandidateBBox.Min.Item3];

                if (i >= 0 && (i - voxel.Item1) >= minCeilingGuessHeight) {

                    heightCounterPixels.Add((
                        voxel.Item2,
                        voxel.Item3));

                    heightCounters.BucketIncrement(i - voxel.Item1);
                }
            }
        }

        private static void SearchOtherCeilingCandidatesAndUpdateHeightCounter(
                int minCeilingGuessHeight,
                int[,] floorHeightGrid,
                int[,] ceilingHeightGrid,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid,
                GridBBox3D floorCandidateBBox,
                Dictionary<int, int> heightCounters,
                HashSet<(int, int)> heightCounterPixels) {

            int i, i2, dr, r, r2, r3, dc, c, c2, c3;
            int[] voxelState;

            for (r = 0; r < ceilingHeightGrid.GetLength(0); r++) {
                for (c = 0; c < ceilingHeightGrid.GetLength(1); c++) {

                    i = floorHeightGrid[r, c];
                    if (i < 0) {
                        continue;
                    }

                    r2 = r + floorCandidateBBox.Min.Item2;
                    c2 = c + floorCandidateBBox.Min.Item3;

                    for (i2 = i - minCeilingGuessHeight; i2 >= 0; i2--) {

                        if (reconstructionGrid[i2, r2, c2] != null) {
                            if (!heightCounterPixels.Contains((r2, c2))) {
                                heightCounterPixels.Add((r2, c2));
                                heightCounters.BucketIncrement(i2);
                            }
                            break;
                        }

                        if (normalGrid[i2, r2, c2] == NormalGridValues.NORMAL_DOWN) {
                            if (!heightCounterPixels.Contains((r2, c2))) {
                                heightCounterPixels.Add((r2, c2));
                                heightCounters.BucketIncrement(i2);
                            }
                            break;
                        }

                        for (dr = -1; dr <= 1; dr++) {
                            for (dc = -1; dc <= 1; dc++) {

                                if (dr.Abs() == dc.Abs()) {
                                    continue;
                                }

                                r3 = r2 + dr;
                                c3 = c2 + dc;
                                if (r3 < 0 || c3 < 0
                                        || r3 >= reconstructionGrid.GetLength(1)
                                        || c3 >= reconstructionGrid.GetLength(2)) {
                                    continue;   
                                }

                                voxelState = reconstructionGrid[i2, r3, c3];

                                if (voxelState != null
                                        && voxelState
                                            .GetRoomIds()
                                            .Any(roomId => voxelState
                                                .GetVoxelClassValues(roomId)
                                                .Contains(VoxelClassValues.CEILING))
                                        && !heightCounterPixels.Contains((r3, c3))) {

                                    heightCounterPixels.Add((r3, c3));
                                    heightCounters.BucketIncrement(i - i2);
                                }
                            }
                        }
                    }
                }
            }
        }

        private static int DetermineCeilingHeight(
                int minCeilingGuessHeight,
                Dictionary<int, int> heightCounters) {

            if (heightCounters.Count == 0) {
                return 2 * minCeilingGuessHeight;
            }

            return heightCounters
                .Keys
                .WhereMax(height => heightCounters[height])
                .First();
        }

        private static void SetCeilingHeight(
                int ceilingHeight,
                int[,] floorHeightGrid,
                int[,] ceilingHeightGrid,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid,
                GridBBox3D floorCandidateBBox) {

            bool found;
            int i, i2, r, r2, c, c2;

            for (r = 0; r < ceilingHeightGrid.GetLength(0); r++) {
                for (c = 0; c < ceilingHeightGrid.GetLength(1); c++) {

                    i = floorHeightGrid[r, c];
                    if (i < 0) {
                        continue;
                    }

                    r2 = r + floorCandidateBBox.Min.Item2;
                    c2 = c + floorCandidateBBox.Min.Item3;
                    found = false;
                    for (i2 = i - 1; i2 < i - ceilingHeight; i2--) {
                        if (i2 == 0
                                || reconstructionGrid[i2, r2, c2] != null
                                || normalGrid[i2, r2, c2] == NormalGridValues.NORMAL_DOWN) {
                            found = true;
                            ceilingHeightGrid[r, c] = i2;
                            break;
                        }
                    }

                    if (!found) {
                        i2 = i - ceilingHeight;
                        ceilingHeightGrid[r, c] = i2 >= 0 ? i2 : 0;
                    }
                }
            }
        }

        private static void ResolveOverlapsWithRooms(
                int[,] floorHeightGrid,
                int[,] ceilingHeightGrid,
                int[,,][] reconstructionGrid,
                GridBBox3D floorCandidateBBox) {

            int r, r2, c, c2;
            int floorHeight, ceilingHeight;
            List<int> overlap;

            for (r = 0; r < floorCandidateBBox.Size.Item2; r++) {
                for (c = 0; c < floorCandidateBBox.Size.Item3; c++) {

                    if (floorHeightGrid[r, c] < 0) {
                        continue;
                    }

                    floorHeight = floorHeightGrid[r, c];
                    ceilingHeight = ceilingHeightGrid[r, c];
                    r2 = r + floorCandidateBBox.Min.Item2;
                    c2 = c + floorCandidateBBox.Min.Item3;

                    overlap = GetOverlap(
                        r2,
                        c2,
                        floorHeight,
                        ceilingHeight,
                        reconstructionGrid);

                    if (overlap.Count > 0) {

                        ResolveOverlap(
                            r, c,
                            r2, c2,
                            floorHeight,
                            ceilingHeight,
                            floorHeightGrid,
                            ceilingHeightGrid,
                            reconstructionGrid,
                            overlap);
                    }
                }
            }
        }

        private static List<int> GetOverlap(
                int r, 
                int c,
                int floorHeight,
                int ceilingHeight,
                int[,,][] reconstructionGrid) {

            int i;
            int[] voxelState;
            int[] voxelClassValues;
            List<int> overlap = new List<int>();

            for (i = ceilingHeight; i <= floorHeight; i++) {

                voxelState = reconstructionGrid[i, r, c];
                if (voxelState == null) {
                    continue;
                }

                if (i != ceilingHeight && i != floorHeight) {
                    overlap.Add(i);
                    continue;
                }

                foreach (int roomId in voxelState.GetRoomIds()) {

                    voxelClassValues = voxelState.GetVoxelClassValues(roomId);

                    if (i == ceilingHeight
                            && voxelClassValues.Contains(VoxelClassValues.FLOOR)) {
                        continue;
                    }

                    if (i == floorHeight
                            && voxelClassValues.Contains(VoxelClassValues.CEILING)) {
                        continue;
                    }

                    overlap.Add(i);
                    break;
                }
            }

            return overlap;
        }

        private static void ResolveOverlap(
                int r, 
                int c,
                int r2,
                int c2,
                int floorHeight,
                int ceilingHeight,
                int[,] floorHeightGrid,
                int[,] ceilingHeightGrid,
                int[,,][] reconstructionGrid,
                List<int> overlap) {

            int maxFloorOverlap, maxCeilingOverlap;

            if (overlap.Count >= (floorHeight - ceilingHeight) / 2) {
                floorHeightGrid[r, c] = -1;
                ceilingHeightGrid[r, c] = -1;
                return;
            }

            GetMaxOverlap(
                r2,
                c2,
                reconstructionGrid,
                overlap,
                out maxFloorOverlap,
                out maxCeilingOverlap);

            if ((maxCeilingOverlap < 0 && maxFloorOverlap < 0)
                    || (maxCeilingOverlap >= 0
                        && maxFloorOverlap >= 0
                        && (maxFloorOverlap > maxCeilingOverlap
                            || maxCeilingOverlap - maxFloorOverlap < (floorHeight - ceilingHeight) / 2))) {
                floorHeightGrid[r, c] = -1;
                ceilingHeightGrid[r, c] = -1;
                return;
            }

            if (maxCeilingOverlap >= 0) {
                floorHeightGrid[r, c] = maxCeilingOverlap;
            }
            if (maxFloorOverlap >= 0) {
                ceilingHeightGrid[r, c] = maxFloorOverlap;
            }
        }

        private static void GetMaxOverlap(
                int r, 
                int c,
                int[,,][] reconstructionGrid,
                List<int> overlap,
                out int maxFloorOverlap,
                out int maxCeilingOverlap) {

            int[] voxelState;

            maxFloorOverlap = -1;
            maxCeilingOverlap = -1;

            foreach (int i in overlap) {

                voxelState = reconstructionGrid[i, r, c];

                if (voxelState
                            .GetRoomIds()
                            .Any(roomId => voxelState
                                .GetVoxelClassValues(roomId)
                                .Contains(VoxelClassValues.CEILING))
                        && (maxCeilingOverlap < 0 || i < maxCeilingOverlap)) {
                    maxCeilingOverlap = i;
                }

                if (voxelState
                            .GetRoomIds()
                            .Any(roomId => voxelState
                                .GetVoxelClassValues(roomId)
                                .Contains(VoxelClassValues.FLOOR))
                        && (maxFloorOverlap < 0 || i > maxFloorOverlap)) {
                    maxFloorOverlap = i;
                }
            }
        }

        private static List<List<(int, int, int)>> ResegmentFloorCandidate(
                int[,] floorHeightGrid,
                GridBBox3D floorCandidateBBox) {

            int dr, r, r2, dc, c, c2;
            bool[,] isSegmented = new bool[
                floorHeightGrid.GetLength(0),
                floorHeightGrid.GetLength(1)];
            Queue<(int, int)> candidates = new Queue<(int, int)>();
            List<List<(int, int, int)>> floorCandidateSegments = new List<List<(int, int, int)>>();

            for (r = 0; r < floorHeightGrid.GetLength(0); r++) {
                for (c = 0; c < floorHeightGrid.GetLength(1); c++) {

                    if (isSegmented[r, c] 
                            || floorHeightGrid[r, c] < 0) {
                        continue;
                    }

                    candidates.Enqueue((r, c));
                    List<(int, int, int)> segment = new List<(int, int, int)>();
                    while (candidates.Count > 0) {

                        (int, int) candidate = candidates.Dequeue();
                        if (isSegmented[
                                candidate.Item1,
                                candidate.Item2]) {
                            continue;
                        }
                        isSegmented[
                            candidate.Item1,
                            candidate.Item2] = true;
                        segment.Add(
                            (
                                floorHeightGrid[
                                    candidate.Item1,
                                    candidate.Item2],
                                candidate.Item1 + floorCandidateBBox.Min.Item2,
                                candidate.Item2 + floorCandidateBBox.Min.Item3
                            ));

                        for (dr = -1; dr <= 1; dr++) {
                            for (dc = -1; dc <= 1; dc++) {

                                if (dr == 0 && dc == 0) {
                                    continue;
                                }

                                r2 = candidate.Item1 + dr;
                                c2 = candidate.Item2 + dc;
                                if (r2 >= 0 && c2 >= 0
                                        && r2 < floorHeightGrid.GetLength(0)
                                        && c2 < floorHeightGrid.GetLength(1)
                                        && !isSegmented[r2, c2]
                                        && floorHeightGrid[r2, c2] >= 0) {
                                    candidates.Enqueue((r2, c2));
                                }
                            }
                        }
                    }

                    floorCandidateSegments.Add(segment);
                }
            }

            return floorCandidateSegments;
        }

        private static void AddToReconstructionGrid(
                int id,
                int[,] floorHeightGrid,
                int[,] ceilingHeightGrid,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid,
                GridBBox3D floorCandidateBBox,
                HashSet<int> roomIdsWithoutFloor,
                List<(int, int, int)> floorCandidateSegment) {

            bool isBorder;
            int dr, r, r2, dc, c, c2;

            foreach ((int, int, int) voxel in floorCandidateSegment) {

                r = voxel.Item2 - floorCandidateBBox.Min.Item2;
                c = voxel.Item3 - floorCandidateBBox.Min.Item3;

                reconstructionGrid[
                    floorHeightGrid[r, c],
                    voxel.Item2,
                    voxel.Item3] = VoxelState.CreateVoxelState(
                        id,
                        VoxelClassValues.FLOOR);

                isBorder = false;

                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (dr.Abs() == dc.Abs()) {
                            continue;
                        }

                        r2 = r + dr;
                        c2 = c + dc;
                        if (r2 < 0 || c2 < 0
                                || r2 >= floorCandidateBBox.Size.Item2
                                || c2 >= floorCandidateBBox.Size.Item3
                                || ceilingHeightGrid[r2, c2] < 0) {
                            isBorder = true;
                            break;
                        }
                    }

                    if (isBorder) {
                        break;
                    }
                }

                reconstructionGrid[
                        ceilingHeightGrid[r, c],
                        voxel.Item2,
                        voxel.Item3]
                    = reconstructionGrid[
                            ceilingHeightGrid[r, c],
                            voxel.Item2,
                            voxel.Item3]
                        .CopyAddRoom(
                            id,
                            isBorder ? 
                                new int[] { 
                                    VoxelClassValues.CEILING,
                                    VoxelClassValues.WALL
                                } :
                                new int[] {
                                    VoxelClassValues.CEILING
                                });

                Util.DoVerticalVoxelClassificationSweep(
                    id,
                    voxel.Item2,
                    voxel.Item3,
                    ceilingHeightGrid[r, c],
                    floorHeightGrid[r, c],
                    normalGrid,
                    reconstructionGrid,
                    roomIdsWithoutFloor);
            }
        }

        private static void CompleteCeilingsAndFloors(
                int id,
                double resolution,
                int[,] floorHeightGrid,
                int[,] ceilingHeightGrid,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid,
                GridBBox3D floorCandidateBBox,
                List<(int, int, int)> floorCandidateSegment) {

            int i, r, c, d, dMax;
            int searchDistance = Parameters
                .FLOOR_CEILING_COMPLETION_SEARCH_DISTANCE
                .GetDistanceInVoxels(resolution);

            dMax = searchDistance + 1;

            foreach ((int, int, int) voxel in floorCandidateSegment) {

                r = voxel.Item2 - floorCandidateBBox.Min.Item2;
                c = voxel.Item3 - floorCandidateBBox.Min.Item3;

                foreach (int direction in new int[] { -1, 1 }) {
                    for (d = 1; d <= dMax; d++) {

                        if (direction == 1) {
                            i = floorHeightGrid[r, c];
                        }
                        else {
                            i = ceilingHeightGrid[r, c];
                        }

                        i += direction * d;

                        if (i < 0
                                || i >= reconstructionGrid.GetLength(0)
                                || reconstructionGrid[
                                    i,
                                    voxel.Item2,
                                    voxel.Item3] != null
                                || normalGrid[
                                    i,
                                    voxel.Item2,
                                    voxel.Item3] == NormalGridValues.EMPTY) {
                            d -= 1;
                            break;
                        }
                    }

                    if (d > searchDistance) {
                        continue;
                    }

                    for (; d >= 1; d--) {

                        if (direction == 1) {
                            i = floorHeightGrid[r, c];
                        }
                        else {
                            i = ceilingHeightGrid[r, c];
                        }

                        i += direction * d;
                        if (i < 0 || i >= reconstructionGrid.GetLength(0)) {
                            continue;
                        }

                        reconstructionGrid[
                            i,
                            voxel.Item2,
                            voxel.Item3] = VoxelState.CreateVoxelState(
                                id,
                                direction == 1 ?
                                    VoxelClassValues.FLOOR :
                                    VoxelClassValues.CEILING);
                    }
                }
            }
        }
    }
}
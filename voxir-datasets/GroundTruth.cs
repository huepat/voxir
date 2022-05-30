using HuePat.VoxIR.Datasets.Util.Octree;
using HuePat.VoxIR.Datasets.Util.RayTracing;
using HuePat.VoxIR.Util.Geometry;
using HuePat.VoxIR.Util.Grid;
using OpenTK.Mathematics;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace HuePat.VoxIR.Datasets {
    public static class GroundTruth {
        private static readonly Vector3d UP_AXIS = new Vector3d(0.0, -1.0, 0.0);

        public class Parameters {
            public const int OCTREE_CLUMPING_DEATH_THRESHOLD = 2;
            public const int OCTREE_MAX_OCCUPATION = 100;
            public const double RAYTRACING_DISTANCE_THRESHOLD = 0.5;
            public const double GROUNDTRUTH_GRID_REFINEMENT_MIN_INTERIOR_SPACE_SEGMENT_AREA = 0.5;
        }

        public static int[,,][] CreateGroundTruthGrid(
                double resolution,
                AABox gridExtent,
                Dictionary<int, List<(Mesh, GroundTruthInfo)>> groundTruthMeshes) {

            (int, int, int) gridSize;
             int[,,][] groundTruthGrid;

            if (gridExtent == null) {

                gridExtent = AABox.FromContainedGeometries(
                    groundTruthMeshes
                        .UnwrapValues()
                        .Select(groundTruthMesh => groundTruthMesh.Item1)
                        .ToList(),
                    true);
            }

            gridSize = Voxelizer.DetermineGridSize(
                resolution,
                ref gridExtent);

            groundTruthGrid = new int[
                gridSize.Item1,
                gridSize.Item2,
                gridSize.Item3][];

            foreach ((Mesh, GroundTruthInfo) groundTruthMesh in groundTruthMeshes.UnwrapValues()) {

                Voxelizer.VoxelizeMesh(
                    resolution,
                    groundTruthGrid,
                    gridExtent.Min,
                    groundTruthMesh.Item1,
                    (triangle, voxel) => VoxelState.CreateVoxelState(
                        groundTruthMesh.Item2.RoomId,
                        groundTruthMesh.Item2.ClassValue),
                    (groundTruthVoxelState, groundTruthVoxelState2) => groundTruthVoxelState.Update(groundTruthVoxelState2));
            }

            PostProcessWallOpenings(groundTruthGrid);

            AddEmptyInterior(groundTruthGrid);

            FillRoomlessInteriorSpaceVoxels(groundTruthGrid);

            CompleteCeilingsAndFloors(groundTruthGrid);

            RemoveNegligibleInteriorSpaceSegments(
                resolution,
                groundTruthGrid);

            CreateRampTransitionSpaces(
                groundTruthGrid,
                groundTruthMeshes);

            return groundTruthGrid;
        }

        public static void TransferGroundTruthInfoToTestMesh(
                Mesh testMesh,
                Dictionary<int, List<(Mesh, GroundTruthInfo)>> groundTruthMeshes,
                out bool[] isRampSpace,
                out int[] classValues,
                out int[] roomIds) {

            Octree groundTruthOctree = new Octree(groundTruthMeshes
                .Values
                .SelectMany(value => value)
                .ToList());

            bool[] _isRampSpace = new bool[testMesh.Count];
            int[] _classValues = new int[testMesh.Count];
            int[] _roomIds = new int[testMesh.Count];

            Parallel.For(
                0,
                testMesh.Count,
                j => {

                    List<Intersection> intersections = Intersect(
                        testMesh[j].Geometry,
                        groundTruthOctree);

                    GetGroundTruthInfo(
                        testMesh[j].Geometry,
                        intersections,
                        out _isRampSpace[j],
                        out _classValues[j],
                        out _roomIds[j]);
                });

            isRampSpace = _isRampSpace;
            classValues = _classValues;
            roomIds = _roomIds;
        }

        private  static int[] Update(
                this int[] voxelState,
                int[] voxelState2) {

            int[] result;

            if (voxelState == null) {
                return voxelState2;
            }
            if (voxelState2 == null) {
                return voxelState;
            }

            result = voxelState.Copy();

            foreach (int roomId in voxelState2.GetRoomIds()) {

                result = result.CopyAddVoxelClassValues(
                    roomId,
                    voxelState2.GetVoxelClassValues(roomId));
            }

            return result;
        }

        private static void PostProcessWallOpenings(
                int[,,][] groundTruthGrid) {

            Parallel.For(
                0,
                groundTruthGrid.GetLength(0),
                i => {

                    int r, c;
                    int[] voxelState;
                    int[] voxelClassValues;

                    for (r = 0; r < groundTruthGrid.GetLength(1); r++) {
                        for (c = 0; c < groundTruthGrid.GetLength(2); c++) {

                            voxelState = groundTruthGrid[i, r, c];
                            if (voxelState == null) {
                                continue;
                            }

                            foreach (int roomId in voxelState.GetRoomIds()) {

                                voxelClassValues = voxelState.GetVoxelClassValues(roomId);
                                if (voxelClassValues.Length > 1
                                        && voxelClassValues.Contains(VoxelClassValues.WALL_OPENING)) {

                                    groundTruthGrid[i, r, c] = voxelState.CopyRemoveVoxelClassValue(
                                        roomId,
                                        VoxelClassValues.WALL_OPENING);
                                }
                            }
                        }
                    }
                });
        }

        private static void AddEmptyInterior(
                int[,,][] groundTruthGrid) {

            Parallel.For(
                0,
                groundTruthGrid.GetLength(1),
                r => {

                    int i, c;
                    int startHeight, stopHeight;
                    int[] voxelState;

                    for (i = 0; i < groundTruthGrid.GetLength(0); i++) {
                        for (c = 0; c < groundTruthGrid.GetLength(2); c++) {

                            voxelState = groundTruthGrid[i, r, c];
                            if (voxelState == null) {
                                continue;
                            }

                            foreach (int roomId in voxelState.GetRoomIds()) {

                                if (!voxelState
                                        .GetVoxelClassValues(roomId)
                                        .Contains(VoxelClassValues.CEILING)) {
                                    continue;
                                }

                                startHeight = i + 1;
                                if (HasFloorBeneath(
                                        startHeight,
                                        r,
                                        c,
                                        roomId,
                                        groundTruthGrid,
                                        out stopHeight)) {

                                    AddEmptyInterior(
                                        startHeight,
                                        stopHeight,
                                        r,
                                        c,
                                        roomId,
                                        groundTruthGrid);
                                }
                            }
                        }
                    }
                });
        }

        private static bool HasFloorBeneath(
                int i,
                int r,
                int c,
                int roomId,
                int[,,][] groundTruthGrid,
                out int floorHeight) {

            floorHeight = -1;
            int[] voxelState;
            int[] voxelClassValues;

            do {

                voxelState = groundTruthGrid[i++, r, c];
                if (voxelState == null
                        || !voxelState.HasRoomId(roomId)) {
                    continue;
                }

                voxelClassValues = voxelState.GetVoxelClassValues(roomId);
                if (voxelClassValues.Contains(VoxelClassValues.FLOOR)) {
                    floorHeight = i;
                    return true;
                }
            } while (i < groundTruthGrid.GetLength(0));

            return false;
        }

        private static void AddEmptyInterior(
                int startHeight,
                int stopHeight,
                int r,
                int c,
                int roomId,
                int[,,][] groundTruthGrid) {

            int i;
            int[] voxelState;

            for (i = startHeight; i < stopHeight; i++) {

                voxelState = groundTruthGrid[i, r, c];
                if (voxelState == null
                        || !voxelState.HasRoomId(roomId)) {

                    groundTruthGrid[i, r, c] = voxelState.CopyAddRoom(
                        roomId,
                        VoxelClassValues.EMPTY_INTERIOR);
                }
            }
        }

        private static void FillRoomlessInteriorSpaceVoxels(
                int[,,][] groundTruthGrid) {
            
            Parallel.For(
                0,
                groundTruthGrid.GetLength(0),
                i => {

                    int r, c;
                    int roomId;
                    bool[,] outsideGrid;
                    bool[,] isSegmented;
                    List<(int, int)> segment2D;

                    isSegmented = new bool[
                    groundTruthGrid.GetLength(1),
                    groundTruthGrid.GetLength(2)];
                    outsideGrid = InitializeOutsideGrid(
                        i,
                        groundTruthGrid);

                    for (r = 0; r < groundTruthGrid.GetLength(1); r++) {
                        for (c = 0; c < groundTruthGrid.GetLength(2); c++) {

                            if (groundTruthGrid[i, r, c] != null
                                    || outsideGrid[r, c]
                                    || isSegmented[r, c]) {
                                continue;
                            }

                            segment2D = HorizontallySegmentRoomlessSpace(
                                i,
                                r,
                                c,
                                isSegmented,
                                outsideGrid,
                                groundTruthGrid);

                            if (ConnectsToMultipleRooms(
                                    i,
                                    groundTruthGrid,
                                    segment2D,
                                    out roomId)) {
                                continue;
                            }

                            foreach ((int, int) position in segment2D) {
                                groundTruthGrid[
                                    i,
                                    position.Item1,
                                    position.Item2] = VoxelState.CreateVoxelState(
                                        roomId,
                                        VoxelClassValues.EMPTY_INTERIOR);
                            }
                        }
                    }
                });
        }

        private static bool[,] InitializeOutsideGrid(
                int i,
                int[,,][] groundTruthGrid) {

            int dr, r, r2, dc, c, c2;
            bool[,] outsideGrid = new bool[
                groundTruthGrid.GetLength(1),
                groundTruthGrid.GetLength(2)];
            Queue<(int, int)> candidates = new Queue<(int, int)>();

            for (r = 0; r < groundTruthGrid.GetLength(1); r++) {
                for (c = 0; c < groundTruthGrid.GetLength(2); c++) {

                    if (outsideGrid[r, c]
                            || groundTruthGrid[i, r, c] != null
                            || (i > 0 && r > 0 && c > 0
                                && i < groundTruthGrid.GetLength(0) - 1
                                && r < groundTruthGrid.GetLength(1) - 1
                                && c < groundTruthGrid.GetLength(2) - 1)) {
                        continue;
                    }

                    candidates.Enqueue((r, c));
                    do {

                        (int, int) candidate = candidates.Dequeue();
                        if (outsideGrid[
                                candidate.Item1,
                                candidate.Item2]) {
                            continue;
                        }
                        outsideGrid[
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
                                        && r2 < groundTruthGrid.GetLength(1)
                                        && c2 < groundTruthGrid.GetLength(2)
                                        && !outsideGrid[r2, c2]
                                        && groundTruthGrid[i, r2, c2] == null) {

                                    candidates.Enqueue((r2, c2));
                                }
                            }
                        }
                    } while (candidates.Count > 0);
                }
            }

            return outsideGrid;
        }

        private static List<(int, int)> HorizontallySegmentRoomlessSpace(
                int i,
                int r, 
                int c,
                bool[,] isSegmented,
                bool[,] outsideGrid,
                int[,,][] groundTruthGrid) {

            int dr, r2, dc, c2;
            List<(int, int)> segment2D = new List<(int, int)>();
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
                segment2D.Add((
                    candidate.Item1,
                    candidate.Item2));

                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (dr.Abs() == dc.Abs()) {
                            continue;
                        }

                        r2 = candidate.Item1 + dr;
                        c2 = candidate.Item2 + dc;
                        if (r2 >= 0 && c2 >= 0
                                && r2 < groundTruthGrid.GetLength(1)
                                && c2 < groundTruthGrid.GetLength(2)
                                && !outsideGrid[r2, c2]
                                && !isSegmented[r2, c2]
                                && groundTruthGrid[i, r2, c2] == null) {

                            candidates.Enqueue((r2, c2));
                        }
                    }
                }
            } while (candidates.Count > 0);

            return segment2D;
        }

        private static bool ConnectsToMultipleRooms(
                int i,
                int[,,][] groundTruthGrid,
                List<(int, int)> segment2D,
                out int uniqueRoomId) {

            bool hasRoomId = false;
            int dr, r, dc, c;
            int[] roomIds;
            int[] voxelClassValues;

            uniqueRoomId = 0;
            
            foreach ((int, int) position in segment2D) {
                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (dr.Abs() == dc.Abs()) {
                            continue;
                        }

                        r = position.Item1 + dr;
                        c = position.Item2 + dc;
                        if (r < 0 || c < 0
                                || r >= groundTruthGrid.GetLength(1)
                                || c >= groundTruthGrid.GetLength(2)
                                || groundTruthGrid[i, r, c] == null) {
                            continue;
                        }

                        roomIds = groundTruthGrid[i, r, c].GetRoomIds();
                        if (roomIds.Length > 1) {
                            return true;
                        }

                        if (hasRoomId) {
                            if (roomIds.First() != uniqueRoomId) {
                                return true;
                            }
                        }
                        else {
                            hasRoomId = true;
                            uniqueRoomId = roomIds.First();
                        }

                        voxelClassValues = groundTruthGrid[i, r, c].GetVoxelClassValues(uniqueRoomId);
                        if (voxelClassValues.Contains(VoxelClassValues.CEILING)
                                || voxelClassValues.Contains(VoxelClassValues.FLOOR)) {
                            return true;
                        }
                    }
                }
            }
            return false;
        }

        private static void CompleteCeilingsAndFloors(
                int[,,][] groundTruthGrid) {

            Parallel.For(
                0,
                groundTruthGrid.GetLength(1),
                r => {

                    int di, i, i2, c;

                    for (i = 0; i < groundTruthGrid.GetLength(0); i++) {
                        for (c = 0; c < groundTruthGrid.GetLength(2); c++) {

                            if (groundTruthGrid[i, r, c] == null) {
                                continue;
                            }

                            foreach (int roomId in groundTruthGrid[i, r, c].GetRoomIds()) {

                                if (groundTruthGrid[i, r, c]
                                        .GetVoxelClassValues(roomId)
                                        .Contains(VoxelClassValues.EMPTY_INTERIOR)) {

                                    for (di = -1; di <= 1; di += 2) {

                                        i2 = i + di;
                                        if (i2 < 0
                                                || i2 >= groundTruthGrid.GetLength(0)) {
                                            continue;
                                        }

                                        if (groundTruthGrid[i2, r, c] == null
                                                || !groundTruthGrid[i2, r, c].HasRoomId(roomId)) {

                                            groundTruthGrid[i2, r, c] = groundTruthGrid[i2, r, c].CopyAddRoom(
                                                roomId,
                                                di == 1 ?
                                                    VoxelClassValues.FLOOR :
                                                    VoxelClassValues.CEILING);
                                        }
                                    }
                                }
                            }
                        }
                    }
                });
        }

        private static void RemoveNegligibleInteriorSpaceSegments(
                double resolution,
                int[,,][] groundTruthGrid) {

            int i, r, c;
            int minVoxelCount = Parameters
                .GROUNDTRUTH_GRID_REFINEMENT_MIN_INTERIOR_SPACE_SEGMENT_AREA
                .GetAreaInVoxels(resolution);
            int[] voxelClassValues;
            bool[,,] isSegmented = new bool[
                groundTruthGrid.GetLength(0),
                groundTruthGrid.GetLength(1),
                groundTruthGrid.GetLength(2)];
            List<(int, int, int)> segment;
            Queue<(int, int, int)> candidates = new Queue<(int, int, int)>();

            for (i = 0; i < groundTruthGrid.GetLength(0); i++) {
                for (r = 0; r < groundTruthGrid.GetLength(1); r++) {
                    for (c = 0; c < groundTruthGrid.GetLength(2); c++) {

                        if (isSegmented[i, r, c]
                                || groundTruthGrid[i, r, c] == null) {
                            continue;
                        }

                        foreach (int roomId in groundTruthGrid[i, r, c].GetRoomIds()) {

                            if (roomId <= 0) {
                                continue;
                            }

                            voxelClassValues = groundTruthGrid[i, r, c].GetVoxelClassValues(roomId);
                            if (voxelClassValues.Length != 1
                                    || (!voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                                        && !voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT))) {
                                continue;
                            }

                            segment = SegmentInteriorSpace(
                                i,
                                r,
                                c,
                                roomId,
                                isSegmented,
                                groundTruthGrid);

                            if (segment
                                    .Select(voxel => (voxel.Item2, voxel.Item3))
                                    .Distinct()
                                    .Count() < minVoxelCount) {

                                RemoveInteriorSpaceSegment(
                                    roomId,
                                    groundTruthGrid,
                                    segment);
                            }
                            break;
                        }
                    }
                }
            }
        }

        private static List<(int, int, int)> SegmentInteriorSpace(
                int i,
                int r,
                int c,
                int roomId,
                bool[,,] isSegmented,
                int[,,][] groundTruthGrid) {

            int di, i2, dr, r2, dc, c2;
            int[] voxelClassValues;
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
                segment.Add((
                    candidate.Item1,
                    candidate.Item2,
                    candidate.Item3));
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
                                    && i2 < groundTruthGrid.GetLength(0)
                                    && r2 < groundTruthGrid.GetLength(1)
                                    && c2 < groundTruthGrid.GetLength(2)
                                    && !isSegmented[i2, r2, c2]
                                    && groundTruthGrid[i2, r2, c2] != null
                                    && groundTruthGrid[i2, r2, c2].HasRoomId(roomId)) {
                                voxelClassValues = groundTruthGrid[i2, r2, c2].GetVoxelClassValues(roomId);
                                if (voxelClassValues.Length == 1
                                    && (voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                                        || voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT))) {
                                    candidates.Enqueue((i2, r2, c2));
                                }
                            }
                        }
                    }
                }
            } while (candidates.Count > 0);

            return segment;
        }

        private static void RemoveInteriorSpaceSegment(
                int roomId,
                int[,,][] groundTruthGrid,
                List<(int, int, int)> segment) {

            foreach ((int, int, int) voxel in segment) {
                groundTruthGrid[
                        voxel.Item1,
                        voxel.Item2,
                        voxel.Item3]
                    = groundTruthGrid[
                            voxel.Item1,
                            voxel.Item2,
                            voxel.Item3]
                        .CopyRemoveVoxelClassValue(
                            roomId,
                            groundTruthGrid[
                                        voxel.Item1,
                                        voxel.Item2,
                                        voxel.Item3]
                                    .GetVoxelClassValues(roomId)
                                    .Contains(VoxelClassValues.INTERIOR_OBJECT) ?
                                VoxelClassValues.INTERIOR_OBJECT :
                                VoxelClassValues.EMPTY_INTERIOR);
            }
        }

        private static void CreateRampTransitionSpaces(
                int[,,][] groundTruthGrid,
                Dictionary<int, List<(Mesh, GroundTruthInfo)>> groundTruthMeshes) {

            int di, i, i2, dr, r, r2, dc, c, c2;
            int[] voxelClassValues;
            int[] neighbourRoomIds;
            Dictionary<(int, int), List<(int, int, int)>> RampTransitionSpaceVoxels 
                = new Dictionary<(int, int), List<(int, int, int)>>();

            for (i = 0; i < groundTruthGrid.GetLength(0); i++) {
                for (r = 0; r < groundTruthGrid.GetLength(1); r++) {
                    for (c = 0; c < groundTruthGrid.GetLength(2); c++) {
                        foreach (int roomId in groundTruthGrid[i, r, c].GetRoomIds()) {

                            if (!groundTruthMeshes[roomId]
                                    .Any(groundTruthMesh => groundTruthMesh.Item2.IsRampSpace)) {
                                continue;
                            }

                            voxelClassValues = groundTruthGrid[i, r, c].GetVoxelClassValues(roomId);
                            if (!voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                                    && !voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)
                                    && !voxelClassValues.Contains(VoxelClassValues.FLOOR)
                                    && !voxelClassValues.Contains(VoxelClassValues.CEILING)) {
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
                                                || i2 >= groundTruthGrid.GetLength(0)
                                                || r2 >= groundTruthGrid.GetLength(1)
                                                || c2 >= groundTruthGrid.GetLength(2)) {
                                            continue;
                                        }

                                        neighbourRoomIds = groundTruthGrid[i2, r2, c2].GetRoomIds();
                                        if (!neighbourRoomIds.Any(neighbourRoomId => 
                                                neighbourRoomId != roomId
                                                    && !groundTruthGrid[i, r, c].HasRoomId(neighbourRoomId))) {
                                            continue;
                                        }

                                        foreach (int otherRoomId in neighbourRoomIds
                                                .Where(neighbourRoomId => neighbourRoomId != roomId)) {

                                            voxelClassValues = groundTruthGrid[i2, r2, c2].GetVoxelClassValues(otherRoomId);
                                            if ((voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                                                        || voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)
                                                        || voxelClassValues.Contains(VoxelClassValues.CEILING)
                                                        || voxelClassValues.Contains(VoxelClassValues.FLOOR))
                                                    && !voxelClassValues.Contains(VoxelClassValues.WALL)) {
                                                RampTransitionSpaceVoxels.BucketAdd(
                                                    (roomId, otherRoomId),
                                                    (i, r, c));
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            CreateRampTransitionSpaces(
                groundTruthGrid,
                RampTransitionSpaceVoxels);
        }

        private static void CreateRampTransitionSpaces(
                int[,,][] groundTruthGrid,
                Dictionary<(int, int), List<(int, int, int)>> newRampTransitionSpaceVoxels) {

            int transitionSpaceId = GetGroundTruthRoomIds(groundTruthGrid).Min();

            foreach ((int, int) transitionId in newRampTransitionSpaceVoxels.Keys) {
                transitionSpaceId--;
                foreach ((int, int, int) voxel in newRampTransitionSpaceVoxels[transitionId]) {
                    groundTruthGrid[
                            voxel.Item1,
                            voxel.Item2,
                            voxel.Item3]
                        = groundTruthGrid[
                                voxel.Item1,
                                voxel.Item2,
                                voxel.Item3]
                            .CopyAddRoom(
                                transitionSpaceId,
                                groundTruthGrid[
                                        voxel.Item1,
                                        voxel.Item2,
                                        voxel.Item3]
                                    .GetVoxelClassValues(transitionId.Item1));
                }
            }
        }

        private static HashSet<int> GetGroundTruthRoomIds(
                int[,,][] groundTruthGrid) {

            int i, r, c;
            HashSet<int> roomIds = new HashSet<int>();

            for (i = 0; i < groundTruthGrid.GetLength(0); i++) {
                for (r = 0; r < groundTruthGrid.GetLength(1); r++) {
                    for (c = 0; c < groundTruthGrid.GetLength(2); c++) {
                        roomIds.AddRange(
                            groundTruthGrid[i, r, c].GetRoomIds());
                    }
                }
            }

            return roomIds;
        }

        private static List<Intersection> Intersect(
                Triangle testTriangle,
                Octree groundTruthOctree) {

            List<Intersection> intersections = new List<Intersection>();

            intersections.AddRange(
                IntersectAlongNormal(
                    testTriangle.Centroid,
                    testTriangle.Normal,
                    groundTruthOctree));

            if (intersections.Count == 0) {

                intersections.AddRange(
                    IntersectAlongNormal(
                        testTriangle.Corner1,
                        testTriangle.Normal,
                        groundTruthOctree));

                intersections.AddRange(
                    IntersectAlongNormal(
                        testTriangle.Corner2,
                        testTriangle.Normal,
                        groundTruthOctree));

                intersections.AddRange(
                    IntersectAlongNormal(
                        testTriangle.Corner3,
                        testTriangle.Normal,
                        groundTruthOctree));
            }

            if (intersections.Count == 0) {

                intersections.AddRange(
                    GetNearest(
                        testTriangle.Centroid,
                        testTriangle,
                        groundTruthOctree));
            }

            if (intersections.Count == 0) {

                intersections.AddRange(
                    GetNearest(
                        testTriangle.Corner1,
                        testTriangle,
                        groundTruthOctree));

                intersections.AddRange(
                    GetNearest(
                        testTriangle.Corner2,
                        testTriangle,
                        groundTruthOctree));

                intersections.AddRange(
                    GetNearest(
                        testTriangle.Corner3,
                        testTriangle,
                        groundTruthOctree));
            }

            return intersections;
        }

        private static List<Intersection> IntersectAlongNormal(
                Vector3d anchorPoint,
                Vector3d direction,
                Octree groundTruthOctree) {

            List<Intersection> intersections = new List<Intersection>();

            intersections.AddRange(
                groundTruthOctree.Intersect(
                    Parameters.RAYTRACING_DISTANCE_THRESHOLD,
                    new Ray(
                        anchorPoint,
                        direction)));
            intersections.AddRange(
                groundTruthOctree.Intersect(
                    Parameters.RAYTRACING_DISTANCE_THRESHOLD,
                    new Ray(
                        anchorPoint,
                        -direction)));

            return intersections;
        }

        private static List<Intersection> GetNearest(
                Vector3d anchorPoint,
                Triangle testTriangle,
                Octree groundTruthOctree) {

            (Triangle, GroundTruthInfo) candidateTriangle;
            List<Intersection> intersections = new List<Intersection>();

            candidateTriangle = groundTruthOctree.GetNearest(
                Parameters.RAYTRACING_DISTANCE_THRESHOLD,
                anchorPoint);

            if (candidateTriangle != (null, null)) {
                intersections.Add(
                    new Intersection(
                        testTriangle.Centroid.DistanceTo(
                            candidateTriangle.Item1.Centroid),
                        candidateTriangle));
            }

            return intersections;
        }

        private static void GetGroundTruthInfo(
                Triangle testTriangle,
                List<Intersection> intersections,
                out bool isRampSpace,
                out int classValue,
                out int roomId) {

            (Triangle, GroundTruthInfo) candidate;
            List<(Triangle, GroundTruthInfo)> candidates;
            List<(Triangle, GroundTruthInfo)> groundTruthTriangles;

            isRampSpace = false;
            classValue = -1;
            roomId = 0;

            groundTruthTriangles = intersections
                .WhereMin(intersection => intersection.Distance)
                .Select(intersection => intersection.Triangle)
                .ToList();

            if (groundTruthTriangles.Count > 0) {

                if (groundTruthTriangles.Count == 1) {

                    classValue = groundTruthTriangles[0].Item2.ClassValue;
                    roomId = groundTruthTriangles[0].Item2.RoomId;
                }
                else {

                    candidates = groundTruthTriangles
                        .Where(groundTruthTriangle => groundTruthTriangle.Item2.ClassValue != VoxelClassValues.INTERIOR_OBJECT)
                        .ToList();

                    if (candidates.Count == 1) {

                        classValue = candidates[0].Item2.ClassValue;
                        roomId = candidates[0].Item2.RoomId;
                    }
                    else {
                        if (candidates.Any(groundTruthTriangle => groundTruthTriangle.Item2.ClassValue == VoxelClassValues.CEILING)
                                && Vector3d.CalculateAngle(
                                    testTriangle.Normal,
                                    -UP_AXIS).Abs() < HuePat.VoxIR.Parameters.DIRECTION_DOWN_ANGLE_THRESHOLD) {

                            candidate = candidates
                                .Where(groundTruthTriangle => groundTruthTriangle.Item2.ClassValue == VoxelClassValues.CEILING)
                                .WhereMax(groundTruthTriangle => groundTruthTriangle.Item1.Area)
                                .First();
                            isRampSpace = candidate.Item2.IsRampSpace;
                            classValue = VoxelClassValues.CEILING;
                            roomId = candidate.Item2.RoomId;
                        }
                        else if (candidates.Any(groundTruthTriangle => groundTruthTriangle.Item2.ClassValue == VoxelClassValues.FLOOR)
                                && Vector3d.CalculateAngle(
                                    testTriangle.Normal,
                                    UP_AXIS).Abs() < HuePat.VoxIR.Parameters.DIRECTION_UP_ANGLE_THRESHOLD) {

                            candidate = candidates
                                .Where(groundTruthTriangle => groundTruthTriangle.Item2.ClassValue == VoxelClassValues.FLOOR)
                                .WhereMax(groundTruthTriangle => groundTruthTriangle.Item1.Area)
                                .First();
                            isRampSpace = candidate.Item2.IsRampSpace;
                            classValue = VoxelClassValues.FLOOR;
                            roomId = candidate.Item2.RoomId;
                        }
                        else {

                            candidate = candidates
                                .WhereMax(groundTruthTriangle => groundTruthTriangle.Item1.Area)
                                .First();
                            isRampSpace = candidate.Item2.IsRampSpace;
                            classValue = VoxelClassValues.WALL;
                            roomId = candidate.Item2.RoomId;
                        }
                    }
                }
            }
        }
    }
}
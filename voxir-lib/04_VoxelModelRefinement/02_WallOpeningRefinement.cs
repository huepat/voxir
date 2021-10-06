using HuePat.VoxIR.Util.Grid;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace HuePat.VoxIR.VoxelModelRefinement {
    public static class WallOpeningRefinement {
        public static void InitializeWallNormals(
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid,
                HashSet<int> roomletIds) {

            bool[,,] isMarked = new bool[
                reconstructionGrid.GetLength(0),
                reconstructionGrid.GetLength(1),
                reconstructionGrid.GetLength(2)];

            Parallel.For(
                0,
                reconstructionGrid.GetLength(0),
                i => {

                    // 1st pass, initialize first layer of normals
                    InitializeInnermostLayerOfNormals(
                        i,
                        reconstructionGrid,
                        roomletIds);

                    // 2nd pass, smooth normal distribution in inner layer preferring diagonal normals
                    SmoothNormals(
                        i,
                        reconstructionGrid,
                        normal => normal.IsDirectionDiagonal());

                    // 3rd pass, smooth normal distribution in inner layer preferring orthogonal normals
                    SmoothNormals(
                        i,
                        reconstructionGrid,
                        normal => !normal.IsDirectionDiagonal());
                });

            Parallel.ForEach(
                Partitioner.Create(
                    0,
                    reconstructionGrid.GetLength(0)),
                (partition, loopState) => {
                    for (int i = partition.Item1; i < partition.Item2; i++) {

                        // 4th pass, more normal smoothing, this time in 3x3x3 neighbourhood
                        SmootNormalsThreedimensionally(
                            i,
                            normalGrid,
                            reconstructionGrid,
                            partition);

                        // iteratively apply 4th pass on wall voxels without normal until all have one
                        do { } while (PropagateNormalsThroughWallLayers(
                            i,
                            isMarked,
                            reconstructionGrid,
                            partition));
                    }
                });
        }

        public static void CloseHolesInWalls(
                double resolution,
                int[,,][] reconstructionGrid) {

            int maxWallThickness = Parameters
                .MAX_WALL_THICKNESS
                .GetDistanceInVoxels(resolution);
            int maxWallThicknessDiagonal = Parameters
                .MAX_WALL_THICKNESS
                .GetDistanceInVoxels(
                    resolution.GetVoxelSizeDiagonal());
            object @lock = new object();
            Dictionary<int, object> locks = new Dictionary<int, object>();

            Parallel.ForEach(
                Partitioner.Create(
                    0,
                    reconstructionGrid.GetLength(0)),
                (partition, loopState) => {

                    bool couldBeFloor, couldBeCeiling;
                    int j, i, r, c, maxD, dStart, dStop;
                    int[] voxelState;
                    int[] voxelClassValues;

                    dStart = dStop = 0;

                    for (i = partition.Item1; i < partition.Item2; i++) {
                        for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                            for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                                voxelState = reconstructionGrid.GetSave(
                                    i,
                                    r,
                                    c,
                                    partition,
                                    @lock,
                                    locks);

                                if (voxelState == null) {
                                    continue;
                                }

                                foreach (int roomId in voxelState.GetRoomIds()) {

                                    voxelClassValues = voxelState.GetVoxelClassValues(roomId);
                                    if (!voxelClassValues.Contains(VoxelClassValues.WALL)
                                            && !voxelClassValues.Contains(VoxelClassValues.WALL_OPENING)) {
                                        continue;
                                    }

                                    couldBeFloor = couldBeCeiling = false;
                                    if (voxelClassValues.Contains(VoxelClassValues.FLOOR)) {
                                        couldBeFloor = true;
                                    }
                                    if (voxelClassValues.Contains(VoxelClassValues.CEILING)) {
                                        couldBeCeiling = true;
                                    }

                                    foreach ((int, int) normal in voxelState.GetNormals(roomId)) {

                                        maxD = normal.IsDirectionDiagonal() ?
                                            maxWallThicknessDiagonal :
                                            maxWallThickness;

                                        for (j = -1; j <= 1; j += 2) {
                                            if (IsHole(
                                                    ref couldBeCeiling,
                                                    ref couldBeFloor,
                                                    i,
                                                    r,
                                                    c,
                                                    roomId,
                                                    j,
                                                    maxD,
                                                    @lock,
                                                    normal,
                                                    reconstructionGrid,
                                                    partition,
                                                    locks,
                                                    out dStart,
                                                    out dStop)) {

                                                CloseHole(
                                                    couldBeCeiling,
                                                    couldBeFloor,
                                                    i,
                                                    r,
                                                    c,
                                                    roomId,
                                                    j,
                                                    dStart,
                                                    dStop,
                                                    @lock,
                                                    normal,
                                                    reconstructionGrid,
                                                    partition,
                                                    locks);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                });
        }

        public static void RefineWallOpenings(
                double resolution,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            double voxelSizeDiagonal = resolution.GetVoxelSizeDiagonal();
            int maxWallThickness = Parameters
                .MAX_WALL_THICKNESS
                .GetDistanceInVoxels(resolution);
            int maxWallThicknessDiagonal = Parameters
                .MAX_WALL_THICKNESS
                .GetDistanceInVoxels(voxelSizeDiagonal);
            int radiusOutside = (int)((Parameters.WALL_REFINEMENT_RADIUS_OUTSIDE - resolution / 2) / resolution).Ceil();
            int radiusOutsideDiagonal = (int)((Parameters.WALL_REFINEMENT_RADIUS_OUTSIDE - voxelSizeDiagonal / 2) / voxelSizeDiagonal).Ceil();
            int wallOpeningOcclusionSearchDistance = (int)((Parameters.WALL_OPENING_OCCLUSION_SEARCH_DISTANCE - resolution / 2) / resolution).Ceil();
            int wallOpeningOcclusionSearchDistanceDiagonal = (int)((Parameters.WALL_OPENING_OCCLUSION_SEARCH_DISTANCE - voxelSizeDiagonal / 2) / voxelSizeDiagonal).Ceil();

            Parallel.For(
                0,
                reconstructionGrid.GetLength(0),
                i => {

                    // 1st pass
                    RefineWallOpeningsPerRoom(
                        i,
                        radiusOutside,
                        radiusOutsideDiagonal,
                        wallOpeningOcclusionSearchDistance,
                        wallOpeningOcclusionSearchDistanceDiagonal,
                        normalGrid,
                        reconstructionGrid);

                    // 2nd pass: close if there is another room (that is not a roomlet) with closed wall behind
                    RefineWallOpeningsBetweenRooms(
                        i,
                        maxWallThickness,
                        maxWallThicknessDiagonal,
                        reconstructionGrid);
                });
        }

        public static void RemoveWallNormals(
                int[,,][] reconstructionGrid) {

            Parallel.For(
                0,
                reconstructionGrid.GetLength(0),
                i => {

                    for (int r = 0; r < reconstructionGrid.GetLength(1); r++) {
                        for (int c = 0; c < reconstructionGrid.GetLength(2); c++) {
                            reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].GetBaseVoxelState();
                        }
                    }
                });
        }

        private static IEnumerable<(int, int)> DiagonalizeNormals(
                this List<(int, int)> normals) {

            int j, k;
            (int, int) normal;
            bool[] isUsed = new bool[normals.Count];

            for (j = 0; j < normals.Count; j++) {
                for (k = j + 1; k < normals.Count; k++) {

                    normal = (
                        normals[j].Item1 + normals[k].Item1,
                        normals[j].Item2 + normals[k].Item2
                    );

                    if (normal.IsDirectionDiagonal()) {
                        isUsed[j] = true;
                        isUsed[k] = true;
                        yield return normal;
                    }
                }
            }

            for (j = 0; j < normals.Count; j++) {
                if (!isUsed[j]) {
                    yield return normals[j];
                }
            }
        }

        private static bool IsNeighbourDirection(
                this (int, int) direction,
                (int, int) otherDirection) {

            return ((direction.Item1 == 0 || otherDirection.Item1 == 0) 
                    || direction.Item1 == otherDirection.Item1)
                && ((direction.Item2 == 0 || otherDirection.Item2 == 0) 
                    || direction.Item2 == otherDirection.Item2);
        }

        private static void InitializeInnermostLayerOfNormals(
                int i,
                int[,,][] reconstructionGrid,
                HashSet<int> roomletIds) {

            int r, c;
            int[] voxelState;
            int[] voxelClassValues;
            List<(int, int)> normals;

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                    voxelState = reconstructionGrid[i, r, c];
                    if (voxelState == null) {
                        continue;
                    }

                    foreach (int roomId in voxelState.GetRoomIds()) {

                        voxelClassValues = voxelState.GetVoxelClassValues(roomId);
                        if (!voxelClassValues.Contains(VoxelClassValues.WALL)
                                && !voxelClassValues.Contains(VoxelClassValues.WALL_OPENING)) {
                            continue;
                        }

                        normals = GetNormalDirections(
                                i,
                                r,
                                c,
                                roomId,
                                reconstructionGrid,
                                roomletIds);

                        if (normals.Count > 0) {

                            reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopySetNormals( 
                                roomId,
                                normals
                                    .DiagonalizeNormals()
                                    .ToArray());
                        }
                    }
                }
            }
        }

        private static List<(int, int)> GetNormalDirections(
                int i, 
                int r, 
                int c,
                int roomId,
                int[,,][] reconstructionGrid,
                HashSet<int> roomletIds) {

            int dr, r2, dc, c2;
            int[] voxelState;
            int[] voxelClassValues;
            List<(int, int)> normals = new List<(int, int)>();

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

                    if (roomletIds.Contains(roomId)) {

                        if (voxelState == null 
                                || !voxelState.HasRoomId(roomId)) {
                            normals.Add((-dr, -dc));
                        }
                    }
                    else {

                        if (voxelState == null || !voxelState.HasRoomId(roomId)) {
                            continue;
                        }

                        voxelClassValues = voxelState.GetVoxelClassValues(roomId);
                        if (voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                                || voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)
                                || (!voxelClassValues.Contains(VoxelClassValues.WALL)
                                    && (voxelClassValues.Contains(VoxelClassValues.CEILING)
                                        || voxelClassValues.Contains(VoxelClassValues.FLOOR)))) {
                            normals.Add((dr, dc));
                        }
                    }
                }
            }

            return normals;
        }

        private static void SmoothNormals(
                int i,
                int[,,][] reconstructionGrid,
                Func<(int, int), bool> normalFilter) {

            int r, c;
            (int, int) maxNeighbourNormal;
            int[] voxelState;
            (int, int)[] normals;
            Dictionary<(int, int), int> neighbourNormalCounters;

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                    voxelState = reconstructionGrid[i, r, c];
                    if (voxelState == null) {
                        continue;
                    }

                    foreach (int roomId in voxelState.GetRoomIds()) {

                        if (!voxelState
                                .GetVoxelClassValues(roomId)
                                .Contains(VoxelClassValues.WALL)) {
                            continue;
                        }

                        normals = voxelState.GetNormals(roomId);

                        if (normals.Length != 1
                                || normalFilter(normals[0])) {
                            continue;
                        }

                        neighbourNormalCounters = CountNeighbourNormals2D(
                            i,
                            r,
                            c,
                            roomId,
                            normals[0],
                            reconstructionGrid,
                            normalFilter);

                        if (neighbourNormalCounters.Count == 0) {
                            continue;
                        }

                        maxNeighbourNormal = neighbourNormalCounters.Keys
                            .WhereMax(normal => neighbourNormalCounters[normal])
                            .First();

                        if (neighbourNormalCounters[maxNeighbourNormal]
                                >= Parameters.NORMAL_SMOOTHING_MIN_NEIGHBOUR_NORMAL_COUNT) {

                            reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopySetNormals(
                                roomId,
                                new (int, int)[] {
                                    (
                                        maxNeighbourNormal.Item1,
                                        maxNeighbourNormal.Item2
                                    )
                                });
                        }
                    }
                }
            }
        }

        private static Dictionary<(int, int), int> CountNeighbourNormals2D(
                int i,
                int r,
                int c,
                int roomId,
                (int, int) referenceNormal,
                int[,,][] reconstructionGrid,
                Func<(int, int), bool> normalFilter) {

            int dr, r2, dc, c2;
            int[] voxelState;
            (int, int)[] neighbourNormals;
            Dictionary<(int, int), int> neighbourNormalCounters = new Dictionary<(int, int), int>();

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    if (dr == 0 && dc == 0) {
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

                    neighbourNormals = voxelState.GetNormals(roomId);

                    if (neighbourNormals.Length == 1
                            && normalFilter(neighbourNormals[0])
                            && neighbourNormals[0].IsNeighbourDirection(referenceNormal)) {

                        neighbourNormalCounters.BucketIncrement((
                            neighbourNormals[0].Item1,
                            neighbourNormals[0].Item2
                        ));
                    }
                }
            }

            return neighbourNormalCounters;
        }

        private static void SmootNormalsThreedimensionally(
                int i,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid,
                Tuple<int, int> partition) {

            bool isOutside, isInside, neighboursOtherRoom;
            int r, c;
            (int, int) maxNeighbourNormal;
            int[] voxelState;
            (int, int)[] normals;
            Dictionary<(int, int), int> neighbourNormalCounters;

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                    voxelState = reconstructionGrid[i, r, c];
                    if (voxelState == null) {
                        continue;
                    }

                    foreach (int roomId in voxelState.GetRoomIds()) {

                        normals = voxelState.GetNormals(roomId);

                        if (normals.Length == 0) {
                            continue;
                        }

                        neighbourNormalCounters = CountNeighbourNormals3D(
                            i,
                            r,
                            c,
                            roomId,
                            reconstructionGrid,
                            partition);

                        if (neighbourNormalCounters.Count > 0
                                && (normals.Length > 1
                                    || !neighbourNormalCounters.ContainsKey((
                                        normals[0].Item1,
                                        normals[0].Item2
                                    )))) {

                            maxNeighbourNormal = neighbourNormalCounters.Keys
                                .WhereMax(neighbourNormal => neighbourNormalCounters[neighbourNormal])
                                .First();

                            reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopySetNormals(
                                roomId,
                                new (int, int)[] {
                                    (
                                        maxNeighbourNormal.Item1,
                                        maxNeighbourNormal.Item2
                                    )
                                });
                        }
                        else if (normals.Length > 1 && neighbourNormalCounters.Count == 0) {

                            CheckVoxelState(
                                i,
                                r,
                                c,
                                roomId,
                                voxelState,
                                reconstructionGrid,
                                out isOutside,
                                out isInside,
                                out neighboursOtherRoom);

                            if ((isInside && isOutside) || (!isInside && !isOutside)) {
                                continue;
                            }

                            if (isInside) {

                                reconstructionGrid[i, r, c] = voxelState
                                    .CopyChangeVoxelClassValues(
                                        roomId,
                                        normalGrid[i, r, c] == NormalGridValues.EMPTY ?
                                            VoxelClassValues.EMPTY_INTERIOR :
                                            VoxelClassValues.INTERIOR_OBJECT)
                                    .CopyRemoveNormals(roomId);
                            }

                            if (isOutside 
                                    && !neighboursOtherRoom) {

                                reconstructionGrid[i, r, c] = voxelState
                                    .CopyRemoveRoom(roomId)
                                    .CopyRemoveNormals(roomId);
                            }
                        }
                    }
                }
            }
        } 

        private static Dictionary<(int, int), int> CountNeighbourNormals3D(
                int i,
                int r, 
                int c,
                int roomId,
                int[,,][] reconstructionGrid,
                Tuple<int, int> partition) {

            int di, i2, dr, r2, dc, c2;
            int[] voxelState;
            (int, int)[] neighbourNormals;
            Dictionary<(int, int), int> neighbourNormalCounters = new Dictionary<(int, int), int>();

            for (di = -1; di <= 1; di++) {
                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (di == 0 && dr == 0 && dc == 0) {
                            continue;
                        }

                        i2 = i + di;
                        r2 = r + dr;
                        c2 = c + dc;
                        if (i2 < partition.Item1 || r2 < 0 || c2 < 0
                                || i2 >= partition.Item2
                                || r2 >= reconstructionGrid.GetLength(1)
                                || c2 >= reconstructionGrid.GetLength(2)) {
                            continue;
                        }

                        voxelState = reconstructionGrid[i2, r2, c2];
                        if (voxelState == null
                                || !voxelState.HasRoomId(roomId)) {
                            continue;
                        }

                        neighbourNormals = voxelState.GetNormals(roomId);

                        if (neighbourNormals.Length == 1) {
                            neighbourNormalCounters.BucketIncrement((
                                neighbourNormals[0].Item1,
                                neighbourNormals[0].Item2
                            ));
                        }
                    }
                }
            }

            return neighbourNormalCounters;
        }

        private static void CheckVoxelState(
                int i,
                int r,
                int c,
                int roomId,
                int[] voxelState,
                int[,,][] reconstructionGrid,
                out bool isOutside,
                out bool isInside,
                out bool neighboursOtherRoom) {

            int dr, r2, dc, c2;
            int[] voxelState2;
            int[] voxelClassValues;

            isOutside = isInside = neighboursOtherRoom = false;

            if (voxelState.HasOtherRoomIds(roomId)) {
                neighboursOtherRoom = true;
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

                    voxelState2 = reconstructionGrid[i, r2, c2];
                    if (voxelState2 == null
                            || !voxelState2.HasRoomId(roomId)) {

                        if (voxelState2 != null
                                && voxelState2.HasOtherRoomIds(roomId)) {
                            neighboursOtherRoom = true;
                        }

                        isOutside = true;
                        continue;
                    }

                    voxelClassValues = voxelState2.GetVoxelClassValues(roomId);
                    if (voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                            || voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                        isInside = true;
                    }
                }
            }
        }
        
        private static bool PropagateNormalsThroughWallLayers(
                int i,
                bool[,,] isMarked,
                int[,,][] reconstructionGrid,
                Tuple<int, int> partition) {

            bool anyChanges = false;
            int r, c;
            (int, int) maxNeighbourNormal;
            int[] voxelState;
            int[] voxelClassValues;
            Dictionary<(int, int), int> neighbourNormalCounters;

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (c = 0; c < reconstructionGrid.GetLength(2); c++) {
                    isMarked[i, r, c] = false;
                }
            }

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                    voxelState = reconstructionGrid[i, r, c];
                    if (voxelState == null) {
                        continue;
                    }

                    foreach (int roomId in voxelState.GetRoomIds()) {

                        voxelClassValues = voxelState.GetVoxelClassValues(roomId);
                        if ((!voxelClassValues.Contains(VoxelClassValues.WALL)
                                    && !voxelClassValues.Contains(VoxelClassValues.WALL_OPENING))
                                || voxelState.GetNormalCount(roomId) != 0
                                || !DoesNeedNormalUpdate(
                                    i,
                                    r,
                                    c,
                                    roomId,
                                    isMarked,
                                    reconstructionGrid)) {
                            continue;
                        }

                        neighbourNormalCounters = CountNeighbourNormals3D(
                            i,
                            r,
                            c,
                            roomId,
                            reconstructionGrid,
                            partition);

                        if (neighbourNormalCounters.Count > 0) {

                            maxNeighbourNormal = neighbourNormalCounters
                                .Keys
                                .WhereMax(neighbourNormal => neighbourNormalCounters[neighbourNormal])
                                .First();

                            anyChanges = true;
                            isMarked[i, r, c] = true;

                            reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopySetNormals(
                                roomId,
                                new (int, int)[] {
                                    (
                                        maxNeighbourNormal.Item1,
                                        maxNeighbourNormal.Item2
                                    )
                                });
                        }
                    }
                }
            }

            return anyChanges;
        }

        private static bool DoesNeedNormalUpdate(
                int i,
                int r,
                int c,
                int roomId,
                bool[,,] isMarked,
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
                    if (!isMarked[i, r2, c2]
                            && voxelState != null
                            && voxelState.HasRoomId(roomId)
                            && voxelState.GetNormalCount(roomId) > 0) {
                        return true;
                    }
                }
            }
            return false;
        }

        private static bool IsHole(
                ref bool couldBeCeiling,
                ref bool couldBeFloor,
                int i, 
                int r,
                int c,
                int roomId,
                int j,
                int maxD,
                object @lock,
                (int, int) normal,
                int[,,][] reconstructionGrid,
                Tuple<int, int> partition,
                Dictionary<int, object> locks,
                out int dStart,
                out int dStop) {

            bool isHoleCandidate = false;
            int r2, c2, d;
            int[] voxelState;
            int[] voxelClassValues;

            dStart = dStop = -1;

            for (d = 1; d <= maxD; d++) {

                r2 = r + j * d * normal.Item1;
                c2 = c + j * d * normal.Item2;
                if (r2 < 0 || c2 < 0
                        || r2 >= reconstructionGrid.GetLength(1)
                        || c2 >= reconstructionGrid.GetLength(2)) {
                    break;
                }

                voxelState = reconstructionGrid.GetSave(
                    i, 
                    r2, 
                    c2,
                    partition,
                    @lock,
                    locks);

                if (voxelState == null) {
                    continue;
                }

                if (voxelState.HasRoomId(roomId)) {

                    voxelClassValues = voxelState.GetVoxelClassValues(roomId);

                    if (voxelClassValues.Contains(VoxelClassValues.WALL)
                            || voxelClassValues.Contains(VoxelClassValues.WALL_OPENING)) {

                        if (isHoleCandidate) {
                            if (voxelClassValues.Contains(VoxelClassValues.FLOOR)) {
                                couldBeFloor = true;
                            }
                            if (voxelClassValues.Contains(VoxelClassValues.CEILING)) {
                                couldBeCeiling = true;
                            }
                            dStop = d;
                            return true;
                        }

                        foreach ((int, int) normal2 in voxelState.GetNormals(roomId)) {

                            if ((normal.Item1 != 0 
                                        && normal.Item1 != normal2.Item1)
                                    || (normal.Item2 != 0 
                                        && normal.Item2 != normal2.Item2)) {
                                return false;
                            }
                        }
                        continue;
                    }
                    return false;
                }
                else if (isHoleCandidate) {
                    return false;
                }

                if (!isHoleCandidate) {
                    isHoleCandidate = true;
                    dStart = d;
                }
            }
            return false;
        }

        private static void CloseHole(
                bool couldBeCeiling,
                bool couldBeFloor,
                int i,
                int r,
                int c,
                int roomId,
                int j,
                int dStart,
                int dStop,
                object @lock,
                (int, int) normal,
                int[,,][] reconstructionGrid,
                Tuple<int, int> partition,
                Dictionary<int, object> locks) {

            int i2, r2, c2, d;
            int[] voxelState;
            int[] voxelClassValues = null;

            for (d = dStart; d <= dStop; d++) {

                r2 = r + j * d * normal.Item1;
                c2 = c + j * d * normal.Item2;

                if ((couldBeCeiling || couldBeFloor)
                        && !(couldBeCeiling && couldBeFloor)) {

                    i2 = couldBeCeiling ? i - 1 : i + 1;

                    if (i2 >= 0 && i2 < reconstructionGrid.GetLength(0)) {

                        voxelState = reconstructionGrid
                            .GetSave(
                                i2, 
                                r2, 
                                c2,
                                partition,
                                @lock,
                                locks);

                        if (voxelState == null
                                || !voxelState.HasRoomId(roomId)) {

                            voxelClassValues = new int[] {
                                VoxelClassValues.WALL,
                                couldBeCeiling ?
                                    VoxelClassValues.CEILING :
                                    VoxelClassValues.FLOOR
                            };
                        }
                    }
                }

                voxelState = reconstructionGrid
                    .GetSave(
                        i, 
                        r2, 
                        c2,
                        partition,
                        @lock,
                        locks);

                if (voxelState.DoAllOtherRoomIdsHaveVoxelClassWall(roomId)) {

                    if (voxelClassValues == null) {
                        voxelClassValues = new int[] {
                            VoxelClassValues.WALL_OPENING
                        };
                    }

                    reconstructionGrid.SetSave(
                        i, 
                        r2, 
                        c2,
                        voxelState
                            .CopyChangeVoxelClassValues(
                                roomId,
                                voxelClassValues)
                            .CopySetNormal(
                                roomId,
                                normal),
                        partition,
                        @lock,
                        locks);
                }
            }
        }

        private static bool DoAllOtherRoomIdsHaveVoxelClassWall(
                this int[] voxelState,
                int roomId) {

            int[] voxelClassValues;

            if (voxelState == null) {
                return true;
            }

            foreach (int otherRoomId in voxelState.GetRoomIds()) {

                if (otherRoomId == roomId) {
                    continue;
                }

                voxelClassValues = voxelState.GetVoxelClassValues(otherRoomId);
                if (!voxelClassValues.Contains(VoxelClassValues.WALL)
                        && !voxelClassValues.Contains(VoxelClassValues.WALL_OPENING)) {
                    return false;
                }
            }

            return true;
        }

        private static void RefineWallOpeningsPerRoom(
                int i,
                int radiusOutside,
                int radiusOutsideDiagonal,
                int wallOpeningOcclusionSearchDistance,
                int wallOpeningOcclusionSearchDistanceDiagonal,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            bool close, isDiagonal;
            int r, c;
            int[] voxelState;

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                    voxelState = reconstructionGrid[i, r, c];
                    if (voxelState == null) {
                        continue;
                    }

                    foreach (int roomId in voxelState.GetRoomIds()) {

                        if (!voxelState
                                .GetVoxelClassValues(roomId)
                                .Contains(VoxelClassValues.WALL_OPENING)) {
                            continue;
                        }

                        close = false;

                        foreach ((int, int) normal in voxelState.GetNormals(roomId)) {

                            isDiagonal = normal.IsDirectionDiagonal();

                            // close if there is already a closed wall voxel along normal direction to the inside
                            close = CheckForClosedWallTowardsInside(
                                isDiagonal,
                                i,
                                r,
                                c,
                                roomId,
                                normal,
                                reconstructionGrid);

                            // close if there is already a closed wall voxel along normal direction to the outside
                            if (!close) {
                                close = CheckForClosedWallTowardsOutside(
                                    isDiagonal,
                                    i,
                                    r,
                                    c,
                                    roomId,
                                    radiusOutside,
                                    radiusOutsideDiagonal,
                                    normal,
                                    normalGrid,
                                    reconstructionGrid);
                            }

                            // close if there is furniture in front of it
                            if (!close) {
                                close = CheckForFurnitureTowardsInside(
                                    isDiagonal,
                                    i,
                                    r,
                                    c,
                                    roomId,
                                    wallOpeningOcclusionSearchDistance,
                                    wallOpeningOcclusionSearchDistanceDiagonal,
                                    normal,
                                    reconstructionGrid);
                            }

                            // close along normal
                            if (close) {
                                CloseWall(
                                    i,
                                    r,
                                    c,
                                    roomId,
                                    normal,
                                    reconstructionGrid);
                            }
                        }
                    }
                }
            }
        }

        private static bool CheckForClosedWallTowardsInside(
                bool isDiagonal,
                int i,
                int r,
                int c,
                int roomId,
                (int, int) normal,
                int[,,][] reconstructionGrid) {

            int r2, c2;
            int distance = 1;
            int[] voxelState;
            int[] voxelState2;
            int[] voxelClassValues;
            int[] voxelClassValues2;

            do {

                r2 = r + distance * normal.Item1;
                c2 = c + distance * normal.Item2;
                if (r2 < 0 || c2 < 0
                        || r2 >= reconstructionGrid.GetLength(1)
                        || c2 >= reconstructionGrid.GetLength(2)) {
                    return false;
                }

                voxelState = reconstructionGrid[i, r2, c2];

                if (voxelState != null
                        && voxelState.HasRoomId(roomId)) {

                    voxelClassValues = voxelState.GetVoxelClassValues(roomId);

                    if (voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)
                            || (voxelClassValues.Contains(VoxelClassValues.WALL)
                                && !voxelClassValues.Contains(VoxelClassValues.CEILING)
                                && !voxelClassValues.Contains(VoxelClassValues.FLOOR))) {
                        return true;
                    }
                    else if (!voxelClassValues.Contains(VoxelClassValues.WALL_OPENING)) {
                        return false;
                    }
                }
                else {
                    return false;
                }

                if (isDiagonal) {

                    voxelState = reconstructionGrid[i, r2 - normal.Item1, c2];
                    voxelState2 = reconstructionGrid[i, r2, c2 - normal.Item2];

                    if (voxelState != null && voxelState2 != null
                            && voxelState.HasRoomId(roomId)
                            && voxelState2.HasRoomId(roomId)) {

                        voxelClassValues = voxelState.GetVoxelClassValues(roomId);
                        voxelClassValues2 = voxelState2.GetVoxelClassValues(roomId);

                        if ((voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)
                                    || (voxelClassValues.Contains(VoxelClassValues.WALL)
                                        && !voxelClassValues.Contains(VoxelClassValues.CEILING)
                                        && !voxelClassValues.Contains(VoxelClassValues.FLOOR)))
                                && (voxelClassValues2.Contains(VoxelClassValues.INTERIOR_OBJECT)
                                    || (voxelClassValues2.Contains(VoxelClassValues.WALL)
                                        && !voxelClassValues2.Contains(VoxelClassValues.CEILING)
                                        && !voxelClassValues2.Contains(VoxelClassValues.FLOOR)))) {
                            return true;
                        }
                    }
                }

                distance++;

            } while (true);
        }

        private static bool CheckForClosedWallTowardsOutside(
                bool isDiagonal,
                int i,
                int r,
                int c,
                int roomId,
                int radiusOutside,
                int radiusOutsideDiagonal,
                (int, int) normal,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            int r2, r3, c2, c3, d;
            int distance = -1;
            int distanceThreshold;
            int[] voxelState;
            int[] voxelState2;
            int[] voxelClassValues;
            int[] voxelClassValues2;

            do {

                r2 = r + distance * normal.Item1;
                c2 = c + distance * normal.Item2;
                if (r2 < 0 || c2 < 0
                        || r2 >= reconstructionGrid.GetLength(1)
                        || c2 >= reconstructionGrid.GetLength(2)) {
                    return false;
                }

                voxelState = reconstructionGrid[i, r2, c2];

                if (voxelState != null
                        && voxelState.HasRoomId(roomId)) {

                    voxelClassValues = voxelState.GetVoxelClassValues(roomId);

                    if (voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)
                            || (voxelClassValues.Contains(VoxelClassValues.WALL)
                                && !voxelClassValues.Contains(VoxelClassValues.CEILING)
                                && !voxelClassValues.Contains(VoxelClassValues.FLOOR))) {
                        return true;
                    }
                    else if (!voxelClassValues.Contains(VoxelClassValues.WALL_OPENING)) {
                        return false;
                    }
                }
                else {

                    if (voxelState == null) {

                        distanceThreshold = isDiagonal ?
                           radiusOutsideDiagonal :
                           radiusOutside;

                        for (d = 1; d <= distanceThreshold; d++) {

                            r3 = r2 + distance * normal.Item1 - d;
                            c3 = c2 + distance * normal.Item2 - d;
                            if (r3 < 0 || c3 < 0
                                    || r3 >= reconstructionGrid.GetLength(1)
                                    || c3 >= reconstructionGrid.GetLength(2)
                                    || reconstructionGrid[i, r3, c3] != null) {
                                break;
                            }

                            if (normalGrid[i, r3, c3] != NormalGridValues.EMPTY) {
                                return true;
                            }
                        }
                    }

                    return false;
                }

                if (isDiagonal) {

                    voxelState = reconstructionGrid[i, r2 + normal.Item1, c2];
                    voxelState2 = reconstructionGrid[i, r2, c2 + normal.Item2];

                    if (voxelState != null && voxelState2 != null
                            && voxelState.HasRoomId(roomId)
                            && voxelState2.HasRoomId(roomId)) {

                        voxelClassValues = voxelState.GetVoxelClassValues(roomId);
                        voxelClassValues2 = voxelState2.GetVoxelClassValues(roomId);

                        if ((voxelClassValues.Contains(VoxelClassValues.WALL)
                                    && !voxelClassValues.Contains(VoxelClassValues.CEILING)
                                    && !voxelClassValues.Contains(VoxelClassValues.FLOOR))
                                && (voxelClassValues2.Contains(VoxelClassValues.WALL)
                                    && !voxelClassValues2.Contains(VoxelClassValues.CEILING)
                                    && !voxelClassValues2.Contains(VoxelClassValues.FLOOR))) {
                            return true;
                        }
                    }
                }

                distance--;

            } while (true);
        }

        private static bool CheckForFurnitureTowardsInside(
                bool isDiagonal,
                int i,
                int r,
                int c,
                int roomId,
                int wallOpeningOcclusionSearchDistance,
                int wallOpeningOcclusionSearchDistanceDiagonal,
                (int, int) normal,
                int[,,][] reconstructionGrid) {

            int r2, c2;
            int distance;
            int distanceThreshold = isDiagonal ?
                wallOpeningOcclusionSearchDistanceDiagonal :
                wallOpeningOcclusionSearchDistance;
            int[] voxelState;
            int[] voxelState2;
            int[] voxelClassValues;
            int[] voxelClassValues2;

            for (distance = 1; distance <= distanceThreshold; distance++) {

                r2 = r + distance * normal.Item1;
                c2 = c + distance * normal.Item2;
                if (r2 < 0 || c2 < 0
                        || r2 >= reconstructionGrid.GetLength(1)
                        || c2 >= reconstructionGrid.GetLength(2)) {
                    return false;
                }

                voxelState = reconstructionGrid[i, r2, c2];
                if (voxelState != null 
                        && voxelState.HasRoomId(roomId)) {

                    if (voxelState
                            .GetVoxelClassValues(roomId)
                            .Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                        return true;
                    }
                }
                else {
                    return false;
                }

                if (isDiagonal) {

                    voxelState = reconstructionGrid[i, r2 - normal.Item1, c2];
                    voxelState2 = reconstructionGrid[i, r2, c2 - normal.Item2];

                    if (voxelState != null && voxelState2 != null
                            && voxelState.HasRoomId(roomId)
                            && voxelState2.HasRoomId(roomId)) {

                        voxelClassValues = voxelState.GetVoxelClassValues(roomId);
                        voxelClassValues2 = voxelState2.GetVoxelClassValues(roomId);

                        if (voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)
                                && voxelClassValues2.Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                            return true;
                        }
                    }
                }
            }
            return false;
        }

        private static void CloseWall(
                int i,
                int r,
                int c,
                int roomId,
                (int, int) normal,
                int[,,][] reconstructionGrid) {

            reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyChangeVoxelClassValues(
                roomId,
                VoxelClassValues.WALL);

            CloseWallInDirection(
                i, 
                r, 
                c,
                roomId,
                1,
                normal,
                reconstructionGrid);

            CloseWallInDirection(
                i,
                r,
                c,
                roomId,
                -1,
                normal,
                reconstructionGrid);
        }

        private static void CloseWallInDirection(
                int i,
                int r,
                int c,
                int roomId,
                int direction,
                (int, int) normal,
                int[,,][] reconstructionGrid) {

            int r2, c2;
            int distance = direction;
            int[] voxelState;
            int[] voxelClassValues;

            do {

                r2 = r + distance * normal.Item1;
                c2 = c + distance * normal.Item2;
                if (r2 < 0 || c2 < 0
                        || r2 >= reconstructionGrid.GetLength(1)
                        || c2 >= reconstructionGrid.GetLength(2)) {
                    break;
                }

                voxelState = reconstructionGrid[i, r2, c2];
                if (voxelState == null 
                        || !voxelState.HasRoomId(roomId)) {
                    break;
                }

                voxelClassValues = voxelState.GetVoxelClassValues(roomId);
                if (!voxelClassValues.Contains(VoxelClassValues.WALL)
                        && !voxelClassValues.Contains(VoxelClassValues.WALL_OPENING)) {
                    break;
                }

                foreach ((int, int) normal2 in voxelState.GetNormals(roomId)) {

                    if ((normal.Item1 != 0 
                                && normal.Item1 != normal2.Item1)
                            || (normal.Item2 != 0 
                                && normal.Item2 != normal2.Item2)) {
                        return;
                    }
                }

                if (voxelClassValues.Contains(VoxelClassValues.WALL_OPENING)) {
                    reconstructionGrid[i, r2, c2] = voxelState.CopyChangeVoxelClassValues(
                        roomId,
                        VoxelClassValues.WALL);
                }

                distance += direction;
            } while (true);
        }

        private static void RefineWallOpeningsBetweenRooms(
                int i,
                int maxWallThickness,
                int maxWallThicknessDiagonal,
                int[,,][] reconstructionGrid) {

            bool isDiagonal;
            int r, c;
            int distanceThreshold;
            int[] voxelState;

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                    voxelState = reconstructionGrid[i, r, c];
                    if (voxelState == null) {
                        continue;
                    }

                    foreach (int roomId in voxelState.GetRoomIds()) {

                        if (!voxelState
                                .GetVoxelClassValues(roomId)
                                .Contains(VoxelClassValues.WALL_OPENING)) {
                            continue;
                        }

                        foreach ((int, int) normal in voxelState.GetNormals(roomId)) {

                            isDiagonal = normal.IsDirectionDiagonal();
                            distanceThreshold = isDiagonal ?
                                maxWallThicknessDiagonal :
                                maxWallThickness;

                            if (CheckForClosedWallOnAdjacentRoom(
                                    isDiagonal,
                                    i,
                                    r,
                                    c,
                                    roomId,
                                    distanceThreshold,
                                    normal,
                                    reconstructionGrid)) {

                                // close along normal
                                CloseWall(
                                    i,
                                    r,
                                    c,
                                    roomId,
                                    normal,
                                    reconstructionGrid);
                            }
                        }
                    }
                }
            }
        }

        private static bool CheckForClosedWallOnAdjacentRoom(
                bool isDiagonal,
                int i,
                int r,
                int c,
                int roomId,
                int distanceThreshold,
                (int, int) normal,
                int[,,][] reconstructionGrid) {

            int r2, c2;
            int distance;
            int[] voxelState;
            int[] voxelState2;
            int[] voxelClassValues;
            int[] voxelClassValues2;

            for (distance = 1; distance <= distanceThreshold; distance++) {

                r2 = r - distance * normal.Item1;
                c2 = c - distance * normal.Item2;
                if (r2 < 0 || c2 < 0
                        || r2 >= reconstructionGrid.GetLength(1)
                        || c2 >= reconstructionGrid.GetLength(2)) {
                    break;
                }

                voxelState = reconstructionGrid[i, r2, c2];

                if (voxelState != null) {

                    if (voxelState.HasRoomId(roomId)) {
                        continue;
                    }

                    foreach (int otherRoomId in voxelState.GetRoomIds()) {

                        voxelClassValues = voxelState.GetVoxelClassValues(otherRoomId);
                        if (voxelClassValues.Contains(VoxelClassValues.WALL_OPENING)) {
                            continue;
                        }
                        else {
                            if (voxelClassValues.Contains(VoxelClassValues.WALL)) {
                                return true;
                            }
                            return false;
                        }
                    }
                }

                if (isDiagonal) {

                    voxelState = reconstructionGrid[i, r2 + normal.Item1, c2];
                    voxelState2 = reconstructionGrid[i, r2, c2 + normal.Item2];
                    if (voxelState == null 
                            || voxelState2 == null) {
                        continue;
                    }

                    foreach (int otherRoomId in voxelState.GetRoomIds()) {

                        if (otherRoomId == roomId
                                || !voxelState2.HasRoomId(otherRoomId)) {
                            continue;
                        }

                        voxelClassValues = voxelState.GetVoxelClassValues(otherRoomId);
                        voxelClassValues2 = voxelState2.GetVoxelClassValues(otherRoomId);

                        if (voxelClassValues.Contains(VoxelClassValues.WALL)
                                && voxelClassValues2.Contains(VoxelClassValues.WALL)) {
                            return true;
                        }
                    }
                }
            }
            return false;
        }
    }
}
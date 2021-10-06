using HuePat.VoxIR.Util.Grid;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace HuePat.VoxIR.VoxelModelRefinement {
    public static class VoxelGeometryRefinement {
        public static void AddMissingWalls(
                double resolution,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            bool isCeiling, stop, canSwitchToWall;
            int i, i2, r, c;
            int minWallHeight = Parameters
                .MIN_WALL_HEIGHT
                .GetDistanceInVoxels(resolution);
            int[] voxelState;
            int[] voxelState2;
            int[] voxelClassValues;

            for (i = 0; i < reconstructionGrid.GetLength(0); i++) {
                for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                    for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                        voxelState = reconstructionGrid[i, r, c];
                        if (voxelState == null) {
                            continue;
                        }

                        foreach (int roomId in voxelState.GetRoomIds()) {

                            voxelClassValues = voxelState.GetVoxelClassValues(roomId);

                            // interior voxel
                            if (!voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                                    && !voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                                continue;
                            }

                            // does interior voxel touch outside
                            if (!DoesInteriorVoxelTouchOutside(
                                    i,
                                    r,
                                    c,
                                    roomId,
                                    reconstructionGrid)) {
                                continue;
                            }

                            // does voxel have ceiling around it
                            voxelState2 = reconstructionGrid[i - 1, r, c];

                            isCeiling = IsCeiling(
                                roomId,
                                voxelState2);

                            if (isCeiling) {
                                reconstructionGrid[i - 1, r, c] = voxelState2.CopyChangeVoxelClassValues(
                                    roomId,
                                    VoxelClassValues.CEILING);
                            }

                            // get height of stack touching outside
                            for (i2 = i + 1; i2 < reconstructionGrid.GetLength(0); i2++) {

                                voxelState2 = reconstructionGrid[i2, r, c];

                                // stop on hitting floor or wall
                                if (DoesWallGapEndHere(
                                        i2,
                                        r,
                                        c,
                                        roomId,
                                        voxelState2,
                                        reconstructionGrid)) {
                                    break;
                                }

                                // stop if no longer touches outside
                                if (DoesNoLongerTouchOutside(
                                        i2,
                                        r,
                                        c,
                                        roomId,
                                        reconstructionGrid)) {
                                    break;
                                }

                                // stop if touches ceiling again
                                stop = DoesTouchCeiling(
                                    i2,
                                    r,
                                    c,
                                    roomId,
                                    reconstructionGrid);

                                // unless the voxel below is also touching outside
                                if (stop) {
                                    stop = !DoesVoxelBelowTouchOutside(
                                        i2,
                                        r,
                                        c,
                                        roomId,
                                        reconstructionGrid);
                                }
                                if (stop) {
                                    break;
                                }
                            }

                            // set to wall (is stack high enough)?
                            canSwitchToWall = CanSwitchToWall(
                                i,
                                i2,
                                r,
                                c,
                                roomId,
                                minWallHeight,
                                reconstructionGrid);

                            // change voxels in stack
                            SwitchVoxels(
                                canSwitchToWall,
                                isCeiling,
                                i,
                                i2,
                                r,
                                c,
                                roomId,
                                normalGrid,
                                reconstructionGrid);
                        }
                    }
                }
            }
        }

        public static void RefineCeilingAndFloorGeometry(
                double resolution,
                int[,,][] reconstructionGrid) {

            int distance = Parameters
                .HORIZONTAL_SURFACE_REFINEMENT_DISTANCE
                .GetDistanceInVoxels(resolution);
            bool[,,] markGrid = new bool[
                reconstructionGrid.GetLength(0),
                reconstructionGrid.GetLength(1),
                reconstructionGrid.GetLength(2)];

            Parallel.For(
                0,
                reconstructionGrid.GetLength(2),
                c => {

                    int i, r;
                    int[] voxelState;
                    int[] voxelClassValues;

                    for (i = 0; i < reconstructionGrid.GetLength(0); i++) {
                        for (r = 0; r < reconstructionGrid.GetLength(1); r++) {

                            if (markGrid[i, r, c]) {
                                continue;
                            }

                            voxelState = reconstructionGrid[i, r, c];
                            if (voxelState == null) {
                                continue;
                            }

                            foreach (int roomId in voxelState.GetRoomIds()) {

                                voxelClassValues = voxelState.GetVoxelClassValues(roomId);

                                if (voxelClassValues.Contains(VoxelClassValues.CEILING)
                                        && !voxelClassValues.Contains(VoxelClassValues.WALL)) {

                                    RefineSurface(
                                        true,
                                        i,
                                        r,
                                        c,
                                        roomId,
                                        distance,
                                        markGrid,
                                        reconstructionGrid);
                                }
                                else if (voxelClassValues.Contains(VoxelClassValues.FLOOR)
                                        && !voxelClassValues.Contains(VoxelClassValues.WALL)) {

                                    RefineSurface(
                                        false,
                                        i,
                                        r,
                                        c,
                                        roomId,
                                        distance,
                                        markGrid,
                                        reconstructionGrid);
                                }
                            }
                        }
                    }
                });
        }

        public static void RefineWallGeometry(
                double resolution,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            double voxelSizeDiagonal = resolution.GetVoxelSizeDiagonal();
            int radiusInside = (int)((Parameters.WALL_REFINEMENT_RADIUS_INSIDE - resolution / 2) / resolution).Ceil();
            int radiusInsideDiagonal = (int)((Parameters.WALL_REFINEMENT_RADIUS_INSIDE - voxelSizeDiagonal / 2) / voxelSizeDiagonal).Ceil();
            int radiusOutside = (int)((Parameters.WALL_REFINEMENT_RADIUS_OUTSIDE - resolution / 2) / resolution).Ceil();
            int radiusOutsideDiagonal = (int)((Parameters.WALL_REFINEMENT_RADIUS_OUTSIDE - voxelSizeDiagonal / 2) / voxelSizeDiagonal).Ceil();
            bool[,,] isMarkedInside = new bool[
                reconstructionGrid.GetLength(0),
                reconstructionGrid.GetLength(1),
                reconstructionGrid.GetLength(2)];
            bool[,,] isMarkedOutside = new bool[
                reconstructionGrid.GetLength(0),
                reconstructionGrid.GetLength(1),
                reconstructionGrid.GetLength(2)];

            Parallel.For(
                0,
                reconstructionGrid.GetLength(0),
                i => {

                    int r, c;
                    int[] voxelState;

                    for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                        for (c = 0; c < reconstructionGrid.GetLength(2); c++) {
                            voxelState = reconstructionGrid[i, r, c];

                            // voxels outside of rooms
                            if (voxelState == null) {

                                if (normalGrid[i, r, c] == NormalGridValues.EMPTY) {
                                    continue;
                                }

                                RefineWallGeometryFromOutside(
                                    i,
                                    r,
                                    c,
                                    radiusOutside,
                                    radiusOutsideDiagonal,
                                    isMarkedOutside,
                                    normalGrid,
                                    reconstructionGrid);
                            }

                            // interior objects
                            else {
                                foreach (int roomId in voxelState.GetRoomIds()) {

                                    if (!voxelState
                                            .GetVoxelClassValues(roomId)
                                            .Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                                        continue;
                                    }

                                    RefineWallGeometryFromInside(
                                        i,
                                        r,
                                        c,
                                        roomId,
                                        radiusInside,
                                        radiusInsideDiagonal,
                                        isMarkedInside,
                                        reconstructionGrid);
                                }
                            }
                        }
                    }
                });
        }

        private static bool DoesInteriorVoxelTouchOutside(
                int i,
                int r,
                int c,
                int roomId,
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
                    if (voxelState == null
                            || !voxelState.HasRoomId(roomId)) {
                        return true;
                    }
                }
            }
            return false;
        }

        private static bool IsCeiling(
                int roomId,
                int[] voxelState) {

            if (voxelState == null) {
                return false;   
            }

            if (voxelState.HasRoomId(roomId)) {
                return voxelState
                    .GetVoxelClassValues(roomId)
                    .Contains(VoxelClassValues.CEILING);
            }
            else if (voxelState.GetRoomIds().Any(otherRoomId => voxelState
                    .GetVoxelClassValues(otherRoomId)
                    .Contains(VoxelClassValues.FLOOR))) {
                return true;
            }

            return false;
        }

        private static bool DoesWallGapEndHere(
                int i,
                int r,
                int c,
                int roomId,
                int[] voxelState,
                int[,,][] reconstructionGrid) {

            int[] voxelClassValues;

            if (voxelState == null
                    || !voxelState.HasRoomId(roomId)) {
                return true;
            }

            voxelClassValues = reconstructionGrid[i, r, c].GetVoxelClassValues(roomId);

            return voxelClassValues.Contains(VoxelClassValues.FLOOR)
                || voxelClassValues.Contains(VoxelClassValues.WALL_OPENING)
                || (voxelClassValues.Contains(VoxelClassValues.WALL)
                    && !voxelClassValues.Contains(VoxelClassValues.CEILING));
        }

        private static bool DoesNoLongerTouchOutside(
                int i,
                int r,
                int c,
                int roomId,
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
                    if (voxelState == null
                            || !voxelState.HasRoomId(roomId)) {
                        return false;
                    }
                }
            }

            return true;
        }

        private static bool DoesTouchCeiling(
                int i,
                int r,
                int c,
                int roomId,
                int[,,][] reconstructionGrid) {

            int dr, r2, dc, c2;
            int[] voxelState;

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

                    if (voxelState
                            .GetVoxelClassValues(roomId)
                            .Contains(VoxelClassValues.CEILING)) {
                        return true;
                    }
                }
            }

            return false;
        }

        private static bool DoesVoxelBelowTouchOutside(
                int i,
                int r,
                int c,
                int roomId,
                int[,,][] reconstructionGrid) {

            int dr, r2, dc, c2;
            int[] voxelState;

            i += 1;
            if (i == reconstructionGrid.GetLength(0)) {
                return false;
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

                    voxelState = reconstructionGrid[i, r2, c2];
                    if (voxelState == null
                            || !voxelState.HasRoomId(roomId)) {
                        return true;
                    }
                }
            }
            return false;
        }

        private static bool CanSwitchToWall(
                int iStart,
                int iStop,
                int r,
                int c,
                int roomId,
                int minWallHeight,
                int[,,][] reconstructionGrid) {

            int[] voxelState;
            int[] voxelClassValues;

            if (iStop - iStart + 1 >= minWallHeight) {
                return true;
            }

            voxelState = reconstructionGrid[iStart - 1, r, c];
            if (voxelState != null
                    || !voxelState.HasRoomId(roomId)) {
                return false;   
            }

            voxelClassValues = voxelState.GetVoxelClassValues(roomId);

            return voxelClassValues.Contains(VoxelClassValues.WALL)
                || voxelClassValues.Contains(VoxelClassValues.WALL_OPENING);
        }

        private static void SwitchVoxels(
                bool canSwitchToWall,
                bool isCeiling,
                int iStart,
                int iStop,
                int r,
                int c,
                int roomId,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            int i;
            int[] voxelState;

            for (i = iStart; i <= iStop; i++) {

                voxelState = reconstructionGrid[i, r, c];
                if (voxelState == null
                        || !voxelState.HasRoomId(roomId)) {
                    continue;
                }

                if (canSwitchToWall) {

                    SwitchToWall(
                        isCeiling,
                        iStart,
                        iStop,
                        i,
                        r,
                        c,
                        roomId,
                        normalGrid,
                        reconstructionGrid);
                }
                else {

                    SwitchVoxel(
                        isCeiling,
                        i,
                        r,
                        c,
                        roomId,
                        reconstructionGrid);
                }
            }
        }

        private static void SwitchToWall(
                bool isCeiling,
                int iStart,
                int iStop,
                int i,
                int r,
                int c,
                int roomId,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            int[] voxelState;
            int[] voxelClassValues;

            if (i == iStart) {

                voxelState = reconstructionGrid[iStart - 1, r, c];

                if (isCeiling) {
                    reconstructionGrid[iStart - 1, r, c] = voxelState.CopyChangeVoxelClassValues(
                        roomId,
                        new int[] { 
                            VoxelClassValues.WALL,
                            VoxelClassValues.CEILING
                        });
                }
                else {
                    reconstructionGrid[iStart - 1, r, c] = voxelState.CopyChangeVoxelClassValues(
                        roomId,
                        new int[] {
                            VoxelClassValues.WALL,
                            VoxelClassValues.FLOOR
                        });
                }
            }

            voxelState = reconstructionGrid[i, r, c];
            voxelClassValues = voxelState.GetVoxelClassValues(roomId);

            if (i == iStop
                    && voxelClassValues.Contains(VoxelClassValues.FLOOR)
                    && !voxelClassValues.Contains(VoxelClassValues.WALL)) {

                reconstructionGrid[i, r, c] = voxelState.CopyChangeVoxelClassValues(
                    roomId,
                    new int[] {
                        VoxelClassValues.WALL,
                        VoxelClassValues.FLOOR
                    });
            }
            else if (i == iStop 
                    && isCeiling) {

                reconstructionGrid[i, r, c] = voxelState.CopyChangeVoxelClassValues(
                    roomId,
                    new int[] {
                        VoxelClassValues.CEILING,
                        VoxelClassValues.FLOOR
                    });
            }
            else if (normalGrid[i, r, c] == NormalGridValues.EMPTY) {

                reconstructionGrid[i, r, c] = voxelState.CopyChangeVoxelClassValues(
                    roomId,
                    VoxelClassValues.WALL_OPENING);
            }
            else {
                reconstructionGrid[i, r, c] = voxelState.CopyChangeVoxelClassValues(
                    roomId,
                    VoxelClassValues.WALL);
            }
        }

        private static void SwitchVoxel(
                bool isCeiling,
                int i,
                int r,
                int c,
                int roomId,
                int[,,][] reconstructionGrid) {

            bool canSwitchToWall = false;
            int[] voxelState;
            int[] voxelClassValues;

            voxelState = reconstructionGrid[i, r, c];

            foreach (int otherRoomId in voxelState.GetRoomIds()) {

                if (otherRoomId == roomId) {
                    continue;
                }

                voxelClassValues = voxelState.GetVoxelClassValues(otherRoomId);

                if (voxelClassValues.Contains(VoxelClassValues.WALL)
                        || voxelClassValues.Contains(VoxelClassValues.WALL_OPENING)) {

                    canSwitchToWall = true;
                    break;
                }
            }

            reconstructionGrid[i, r, c] = voxelState.CopyChangeVoxelClassValues(
                roomId,
                isCeiling ?
                    canSwitchToWall ?
                        new int[] {
                            VoxelClassValues.CEILING,
                            VoxelClassValues.WALL
                        } :
                        new int[] { 
                            VoxelClassValues.CEILING 
                        } :
                    canSwitchToWall ?
                        new int[] {
                            VoxelClassValues.FLOOR,
                            VoxelClassValues.WALL
                        } :
                        new int[] { 
                            VoxelClassValues.FLOOR 
                        });
        }

        private static void RefineSurface(
                bool isCeiling,
                int i, 
                int r,
                int c,
                int roomId,
                int distance,
                bool[,,] markGrid,
                int[,,][] reconstructionGrid) {

            int i2, d, maxD;
            int direction;
            int newVoxelClassValue;
            int[] voxelState;

            direction = isCeiling ? 1 : -1;
            newVoxelClassValue = isCeiling ?
                VoxelClassValues.CEILING :
                VoxelClassValues.FLOOR;

            for (d = 1; d <= distance; d++) {

                i2 = i + direction * d;
                if (i2 < 0 || i2 >= reconstructionGrid.GetLength(0)) {
                    break;
                }

                voxelState = reconstructionGrid[i2, r, c];
                if (voxelState == null
                        || !voxelState.HasRoomId(roomId)) {
                    break;
                }

                if (!voxelState
                        .GetVoxelClassValues(roomId)
                        .Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                    break;
                }
            }

            maxD = d - 1;

            if (maxD == distance) {

                i2 = i + direction * maxD + direction;
                voxelState = reconstructionGrid[i2, r, c];

                if (voxelState != null
                        && voxelState.HasRoomId(roomId)) {

                    if (voxelState
                            .GetVoxelClassValues(roomId)
                            .Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                        maxD = 0;
                    }
                }
                else {
                    maxD = 0;
                }
            }

            for (d = 1; d <= maxD; d++) {

                i2 = i + direction * d;
                markGrid[i2, r, c] = true;
                reconstructionGrid[i2, r, c] = reconstructionGrid[i2, r, c].CopyChangeVoxelClassValues(
                    roomId,
                    newVoxelClassValue);
            }
        }

        private static void RefineWallGeometryFromOutside(
                int i,
                int r,
                int c,
                int radius,
                int radiusDiagonal,
                bool[,,] isMarked,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            List<WallOccurence> wallOccurences = FindWallOccurences(
                i,
                r,
                c,
                radius,
                radiusDiagonal,
                isMarked,
                reconstructionGrid);

            AddToWall(
                i,
                r,
                c,
                isMarked,
                normalGrid,
                reconstructionGrid,
                wallOccurences);
        }

        private static List<WallOccurence> FindWallOccurences(
                int i,
                int r,
                int c,
                int radius,
                int radiusDiagonal,
                bool[,,] isMarked,
                int[,,][] reconstructionGrid) {

            bool stop, foundWall;
            int dr, r2, dc, c2, d;
            int distanceThreshold;
            int[] voxelState;
            int[] voxelClassValues;
            List<WallOccurence> wallOccurences = new List<WallOccurence>();
            List<WallOccurence> wallOccurenceCandidates;

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    if (dr == 0 && dc == 0) {
                        continue;
                    }

                    stop = foundWall = false;
                    distanceThreshold = dr.Abs() + dc.Abs() == 2 ?
                        radiusDiagonal :
                        radius;

                    for (d = 1; d <= distanceThreshold; d++) {

                        r2 = r + d * dr;
                        c2 = c + d * dc;
                        if (r2 < 0 || r2 >= reconstructionGrid.GetLength(1)
                            || c2 < 0 || c2 >= reconstructionGrid.GetLength(2)) {
                            break;
                        }

                        voxelState = reconstructionGrid[i, r2, c2];
                        if (voxelState == null) {
                            continue;
                        }

                        wallOccurenceCandidates = new List<WallOccurence>();

                        foreach (int roomId in voxelState.GetRoomIds()) {

                            voxelClassValues = voxelState.GetVoxelClassValues(roomId);
                            if (voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                                    || voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)
                                    || (!voxelClassValues.Contains(VoxelClassValues.WALL)
                                        && (voxelClassValues.Contains(VoxelClassValues.CEILING)
                                            || voxelClassValues.Contains(VoxelClassValues.FLOOR)))) {

                                stop = true;
                                break;
                            }

                            if (!isMarked[i, r2, c2] && (
                                    voxelClassValues.Contains(VoxelClassValues.WALL)
                                        || voxelClassValues.Contains(VoxelClassValues.WALL_OPENING))) {

                                foundWall = true;
                                wallOccurenceCandidates.Add(
                                    new WallOccurence(
                                        roomId,
                                        d,
                                        (dr, dc),
                                        voxelClassValues));
                                break;
                            }
                        }

                        if (foundWall || stop) {
                            if (foundWall && !stop) {
                                wallOccurences.AddRange(
                                    wallOccurenceCandidates);
                            }
                            break;
                        }
                    }
                }
            }

            return wallOccurences;
        }

        private static void AddToWall(
                int i,
                int r,
                int c,
                bool[,,] isMarked,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid,
                List<WallOccurence> wallOccurences) {

            int r2, c2, d;
            int[] voxelClassValues;

            foreach (WallOccurence wallOccurence in wallOccurences
                    .WhereMin(wallOccurence => wallOccurence.Distance)) {

                isMarked[i, r, c] = true;
                voxelClassValues = GetVoxelClassValues(wallOccurence);
                reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyChangeVoxelClassValues(
                    wallOccurence.RoomId,
                    voxelClassValues);

                for (d = wallOccurence.Distance - 1; d >= 1; d--) {

                    r2 = r + d * wallOccurence.Direction.Item1;
                    c2 = c + d * wallOccurence.Direction.Item2;

                    isMarked[i, r2, c2] = true;
                    reconstructionGrid[i, r2, c2] = reconstructionGrid[i, r2, c2].CopyChangeVoxelClassValues(
                        wallOccurence.RoomId,
                        voxelClassValues.Length == 1 ?
                            new int[] {
                                normalGrid[i, r2, c2] == NormalGridValues.EMPTY ?
                                    VoxelClassValues.WALL_OPENING :
                                    VoxelClassValues.WALL
                            } :
                            voxelClassValues);
                }
            }
        }

        private static int[] GetVoxelClassValues(
                WallOccurence wallOccurence) {

            if (wallOccurence.VoxelClassValues.Contains(VoxelClassValues.CEILING)
                        && wallOccurence.VoxelClassValues.Contains(VoxelClassValues.FLOOR)) {

                return new int[] {
                    VoxelClassValues.WALL,
                    VoxelClassValues.CEILING,
                    VoxelClassValues.FLOOR
                };
            }

            if (wallOccurence.VoxelClassValues.Contains(VoxelClassValues.CEILING)) {
                return new int[] {
                    VoxelClassValues.WALL,
                    VoxelClassValues.CEILING
                };
            }
            
            if (wallOccurence.VoxelClassValues.Contains(VoxelClassValues.FLOOR)) {
                return new int[] {
                    VoxelClassValues.WALL,
                    VoxelClassValues.FLOOR
                };
            }

            return new int[] {
                VoxelClassValues.WALL
            };
        }

        private static void RefineWallGeometryFromInside(
                int i,
                int r,
                int c,
                int roomId,
                int radius,
                int radiusDiagonal,
                bool[,,] isMarked,
                int[,,][] reconstructionGrid) {

            bool foundWall = false;
            bool stop = false;
            int dr, r2, dc, c2, d, d2;
            int distanceThreshold;
            int[] voxelState = null;
            int[] voxelClassValues;

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    if (dr == 0 && dc == 0) {
                        continue;
                    }

                    distanceThreshold = dr.Abs() + dc.Abs() == 2 ?
                        radiusDiagonal :
                        radius;

                    for (d = 1; d < distanceThreshold; d++) {

                        r2 = r + d * dr;
                        c2 = c + d * dc;
                        if (r2 < 0 || r2 >= reconstructionGrid.GetLength(1)
                            || c2 < 0 || c2 >= reconstructionGrid.GetLength(2)) {
                            break;
                        }

                        voxelState = reconstructionGrid[i, r2, c2];
                        if (voxelState == null
                                || !voxelState.HasRoomId(roomId)) {
                            break;
                        }

                        voxelClassValues = voxelState.GetVoxelClassValues(roomId);
                        if (voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)
                                || isMarked[i, r2, c2]) {
                            continue;
                        }

                        if (voxelClassValues.Contains(VoxelClassValues.WALL)
                                || voxelClassValues.Contains(VoxelClassValues.WALL_OPENING)) {

                            stop = true;
                            for (d2 = 1; d2 <= (distanceThreshold - d); d2++) {

                                r2 = r - d2 * dr;
                                c2 = c - d2 * dc;
                                if (r2 < 0 || c2 < 0 
                                        || r2 >= reconstructionGrid.GetLength(1)
                                        || c2 >= reconstructionGrid.GetLength(2)) {
                                    stop = false;
                                    foundWall = true;
                                    break;
                                }

                                voxelState = reconstructionGrid[i, r2, c2];
                                if (voxelState == null
                                        || !voxelState.HasRoomId(roomId)
                                        || !voxelState
                                            .GetVoxelClassValues(roomId)
                                            .Contains(VoxelClassValues.INTERIOR_OBJECT)) {
                                    stop = false;
                                    foundWall = true;
                                    break;
                                }
                            }
                            if (stop) {
                                foundWall = false;
                            }
                        }

                        break;
                    }

                    if (stop) {
                        break;
                    }
                }

                if (stop) {
                    break;
                }
            }

            if (foundWall) {
                isMarked[i, r, c] = true;
                reconstructionGrid[i, r, c] = voxelState.CopyChangeVoxelClassValues(
                    roomId,
                    VoxelClassValues.WALL);
            }
        }
    }
}
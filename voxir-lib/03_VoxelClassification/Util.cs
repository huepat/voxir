using HuePat.VoxIR.Util.Grid;
using System.Collections.Generic;
using System.Linq;

namespace HuePat.VoxIR.VoxelClassification {
    static class Util {
        public static void DoVerticalVoxelClassificationSweep(
                int roomId,
                int r, 
                int c,
                int minI,
                int maxI,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid,
                HashSet<int> roomIdsWithoutFloor) {

            bool stopSweep;
            int[] voxelState = reconstructionGrid[minI, r, c];

            if (voxelState != null
                    && voxelState.HasRoomId(roomId)) {

                ResolveOverlapWithOtherRooms(
                    roomId,
                    r,
                    c,
                    minI,
                    maxI,
                    voxelState,
                    reconstructionGrid);
            }
            
            if (minI == 0) {
                minI++;
            }

            for (int i = minI; i <= maxI; i++) {

                UpdateVoxelState(
                    roomId,
                    i,
                    r,
                    c,
                    maxI,
                    normalGrid,
                    reconstructionGrid,
                    roomIdsWithoutFloor,
                    out stopSweep);

                if (stopSweep) {
                    break;
                }
            }
        }

        private static void ResolveOverlapWithOtherRooms(
                int roomId,
                int r,
                int c,
                int minI,
                int maxI,
                int[] voxelState,
                int[,,][] reconstructionGrid) {

            int[] voxelClassValues = voxelState.GetVoxelClassValues(roomId);

            if (voxelClassValues.Contains(VoxelClassValues.CEILING)
                    && voxelClassValues.Contains(VoxelClassValues.WALL)) {

                foreach (int otherRoomId in voxelState.GetRoomIds()) {

                    if (otherRoomId == roomId) {
                        continue;
                    }

                    voxelClassValues = voxelState.GetVoxelClassValues(otherRoomId);

                    if (!voxelClassValues.Contains(VoxelClassValues.WALL) 
                            && (voxelClassValues.Contains(VoxelClassValues.CEILING)
                                || voxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                                || voxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT))) {

                        reconstructionGrid[minI, r, c] =
                                reconstructionGrid[minI, r, c].CopyRemoveRoom(roomId);
                        reconstructionGrid[maxI, r, c] =
                                reconstructionGrid[maxI, r, c].CopyRemoveRoom(roomId);

                        return;
                    }
                }
            }
            else if (voxelClassValues.Contains(VoxelClassValues.CEILING)) {

                foreach (int otherRoomId in voxelState.GetRoomIds()) {

                    if (otherRoomId == roomId) {
                        continue;
                    }

                    voxelClassValues = voxelState.GetVoxelClassValues(otherRoomId);

                    if (voxelClassValues.Contains(VoxelClassValues.CEILING)
                            || voxelClassValues.Contains(VoxelClassValues.WALL)
                            || voxelClassValues.Contains(VoxelClassValues.WALL_OPENING)) {

                        reconstructionGrid[minI, r, c] =
                                reconstructionGrid[minI, r, c].CopyRemoveRoom(roomId);
                        reconstructionGrid[maxI, r, c] =
                                reconstructionGrid[maxI, r, c].CopyRemoveRoom(roomId);

                        return;
                    }
                }
            }
        }

        private static void UpdateVoxelState(
                int roomId,
                int i,
                int r, 
                int c,
                int maxI,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid,
                HashSet<int> roomIdsWithoutFloor,
                out bool stopSweep) {

            int[] voxelClassValues;
            int[] voxelClassValuesAbove;
            int[] voxelState;
            int[] voxelStateAbove;

            stopSweep = false;

            voxelState = reconstructionGrid[i, r, c];
            voxelStateAbove = reconstructionGrid[i - 1, r, c];

            // if NOTHING | *
            if (voxelStateAbove == null
                    || !voxelStateAbove.HasRoomId(roomId)) {
                // set NOTHING | *
                return;
            }

            if (voxelState != null) {

                ResolveOverlapWithOtherRooms(
                    roomId,
                    i,
                    r,
                    c,
                    maxI,
                    voxelState,
                    voxelStateAbove,
                    reconstructionGrid,
                    out stopSweep);

                if (stopSweep) {
                    return;
                }
            }

            voxelClassValues = voxelState.GetVoxelClassValues(roomId);
            voxelClassValuesAbove = voxelStateAbove.GetVoxelClassValues(roomId);

            // if FLOOR or WALL_FLOOR | *
            if (voxelClassValuesAbove.Contains(VoxelClassValues.FLOOR)) {
                // set FLOOR | *
                return;
            }

            // if CEILING | *
            if (voxelClassValuesAbove.Contains(VoxelClassValues.CEILING)
                    && !voxelClassValuesAbove.Contains(VoxelClassValues.WALL)) {

                // if CEILING WITHOUT FLOOR | *
                if (roomIdsWithoutFloor.Contains(roomId)) {
                    // set NOTHING | *
                    reconstructionGrid[i - 1, r, c] = reconstructionGrid[i - 1, r, c].CopyRemoveRoom(roomId);
                    return;
                }

                // if CEILING | FLOOR
                if (voxelClassValues.Contains(VoxelClassValues.FLOOR)) {
                    // set CEILING | FLOOR
                    return;
                }

                // if CEILING | other Room Stop
                if (voxelState != null
                        && voxelState
                            .GetOtherRoomIds(roomId)
                            .Select(otherRoomId => voxelState.GetVoxelClassValues(otherRoomId))
                            .Any(otherVoxelClassValues => otherVoxelClassValues.Contains(VoxelClassValues.CEILING)
                                || otherVoxelClassValues.Contains(VoxelClassValues.FLOOR)
                                || otherVoxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                                || otherVoxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT))) {
                    // set CEILING | same FLOOR)
                    reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyChangeVoxelClassValues(
                        roomId,
                        VoxelClassValues.FLOOR);
                    return;
                }

                // if CEILING | CEILING
                if (voxelClassValues.Contains(VoxelClassValues.CEILING)) {
                    // set CEILING | CEILING
                    return;
                }

                // if CEILING | EMPTY
                if (normalGrid[i, r, c] == NormalGridValues.EMPTY) {
                    // set CEILING | EMPTY_INTERIOR
                    reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyChangeVoxelClassValues(
                        roomId,
                        VoxelClassValues.EMPTY_INTERIOR);
                    return;
                }

                // else (CEILING | NOT EMPTY)
                // set CEILING | INTERIOR_OBJECT
                reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyChangeVoxelClassValues(
                    roomId,
                    VoxelClassValues.INTERIOR_OBJECT);
                return;
            }

            // if EMPTY_INTERIOR | *
            if (voxelClassValuesAbove.Contains(VoxelClassValues.EMPTY_INTERIOR)) {

                // if EMPTY_INTERIOR | FLOOR
                if (voxelClassValues.Contains(VoxelClassValues.FLOOR)) {
                    // set EMPTY_INTERIOR | same FLOOR
                    return;
                }

                // if EMPTY_INTERIOR | other Room Stop
                if (voxelState != null
                        && voxelState
                            .GetOtherRoomIds(roomId)
                            .Select(otherRoomId => voxelState.GetVoxelClassValues(otherRoomId))
                            .Any(otherVoxelClassValues => otherVoxelClassValues.Contains(VoxelClassValues.CEILING)
                                || otherVoxelClassValues.Contains(VoxelClassValues.FLOOR)
                                || otherVoxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                                || otherVoxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT))) {
                    // set EMPTY_INTERIOR | same FLOOR
                    reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyChangeVoxelClassValues(
                        roomId,
                        VoxelClassValues.FLOOR);
                    return;
                }

                // if EMPTY_INTERIOR | CEILING
                if (voxelClassValues.Contains(VoxelClassValues.CEILING)) {
                    reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyChangeVoxelClassValues(
                        roomId,
                        VoxelClassValues.EMPTY_INTERIOR);
                    return;
                }

                // if EMPTY_INTERIOR | EMPTY
                if (normalGrid[i, r, c] == NormalGridValues.EMPTY) {
                    // set EMPTY_INTERIOR | EMPTY_INTERIOR
                    reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyChangeVoxelClassValues(
                        roomId,
                        VoxelClassValues.EMPTY_INTERIOR);
                    return;
                }

                // else (EMPTY_INTERIOR | NOT EMPTY)
                // set EMPTY_INTERIOR | INTERIOR_OBJECT
                reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyChangeVoxelClassValues(
                    roomId,
                    VoxelClassValues.INTERIOR_OBJECT);
                return;
            }

            // if INTERIOR_OBJECT | *
            if (voxelClassValuesAbove.Contains(VoxelClassValues.INTERIOR_OBJECT)) {

                // if INTERIOR_OBJECT | FLOOR
                if (voxelClassValues.Contains(VoxelClassValues.FLOOR)) {
                    // set INTERIOR_OBJECT | FLOOR
                    return;
                }

                // if INTERIOR_OBJECT | other Room Stop
                if (voxelState != null
                        && voxelState
                            .GetOtherRoomIds(roomId)
                            .Select(otherRoomId => voxelState.GetVoxelClassValues(otherRoomId))
                            .Any(otherVoxelClassValues => otherVoxelClassValues.Contains(VoxelClassValues.CEILING)
                                || otherVoxelClassValues.Contains(VoxelClassValues.FLOOR)
                                || otherVoxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                                || otherVoxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT))) {
                    // set INTERIOR_OBJECT | same FLOOR)
                    reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyChangeVoxelClassValues(
                        roomId,
                        VoxelClassValues.FLOOR);
                    return;
                }

                // if INTERIOR_OBJECT | CEILING
                if (voxelClassValuesAbove.Contains(VoxelClassValues.CEILING)) {
                    // INTERIOR_OBJECT | CEILING
                    reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyChangeVoxelClassValues(
                        roomId,
                        VoxelClassValues.INTERIOR_OBJECT);
                    return;
                }

                // if INTERIOR_OBJECT | EMPTY
                if (normalGrid[i, r, c] == NormalGridValues.EMPTY) {
                    // set INTERIOR_OBJECT | EMPTY_INTERIOR
                    reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyChangeVoxelClassValues(
                        roomId,
                        VoxelClassValues.EMPTY_INTERIOR);
                    return;
                }

                // else (INTERIOR_OBJECT | NOT EMPTY)
                // set INTERIOR_OBJECT | INTERIOR_OBJECT
                reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyChangeVoxelClassValues(
                    roomId,
                    VoxelClassValues.INTERIOR_OBJECT);
                return;
            }

            // if WALL (or WALL_CEILING) | *
            if (voxelClassValuesAbove.Contains(VoxelClassValues.WALL)) {

                // if WALL (or WALL_CEILING) | WALL_FLOOR
                if (voxelClassValues.Contains(VoxelClassValues.FLOOR)
                        && voxelClassValues.Contains(VoxelClassValues.WALL)) {
                    // set WALL (or WALL_CEILING) | WALL_FLOOR
                    return;
                }

                // WALL (or WALL_CEILING) | FLOOR
                if (voxelClassValues.Contains(VoxelClassValues.FLOOR)) {
                    // set WALL (or WALL_CEILING) | same WALL_FLOOR)
                    reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyAddVoxelClassValue(
                        roomId,
                        VoxelClassValues.WALL);
                    return;
                }

                // if WALL (or WALL_CEILING) | CEILING (or WALL_CEILING)
                if (voxelClassValues.Contains(VoxelClassValues.CEILING)) {
                    // set WALL (or WALL_CEILING) | WALL
                    reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyChangeVoxelClassValues(
                        roomId,
                        VoxelClassValues.WALL);
                    return;
                }

                // if WALL (or WALL_CEILING) | NOT EMPTY
                if (normalGrid[i, r, c] != NormalGridValues.EMPTY) {
                    // set WALL (or WALL_CEILING) | WALL
                    reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyChangeVoxelClassValues(
                        roomId,
                        VoxelClassValues.WALL);
                    return;
                }

                // else (WALL (or WALL_CEILING) | EMPTY
                // set WALL (or WALL_CEILING) | WALL OPENING
                reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyChangeVoxelClassValues(
                        roomId,
                        VoxelClassValues.WALL_OPENING);
                return;
            }

            // if WALL_OPENING | *
            if (voxelClassValuesAbove.Contains(VoxelClassValues.WALL_OPENING)) {

                // if WALL (or WALL_CEILING) | WALL_FLOOR
                if (voxelClassValues.Contains(VoxelClassValues.FLOOR)
                        && voxelClassValues.Contains(VoxelClassValues.WALL)) {
                    // set WALL (or WALL_CEILING) | WALL_FLOOR
                    return;
                }

                // WALL_OPENING | FLOOR
                if (voxelClassValues.Contains(VoxelClassValues.FLOOR)) {
                    // set WALL_OPENING | same WALL_FLOOR)
                    reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyAddVoxelClassValue(
                        roomId,
                        VoxelClassValues.WALL);
                    return;
                }

                // if WALL_OPENING | other Room Stop
                if (voxelState != null
                        && voxelState
                            .GetOtherRoomIds(roomId)
                            .Select(otherRoomId => voxelState.GetVoxelClassValues(otherRoomId))
                            .Any(otherVoxelClassValues => otherVoxelClassValues.Contains(VoxelClassValues.CEILING)
                                || otherVoxelClassValues.Contains(VoxelClassValues.EMPTY_INTERIOR)
                                || otherVoxelClassValues.Contains(VoxelClassValues.INTERIOR_OBJECT)
                                || (otherVoxelClassValues.Contains(VoxelClassValues.FLOOR)
                                    && !otherVoxelClassValues.Contains(VoxelClassValues.WALL)))) {
                    // set WALL_OPENING | same WALL_FLOOR
                    reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyChangeVoxelClassValues(
                        roomId,
                        new int[] { 
                            VoxelClassValues.FLOOR,
                            VoxelClassValues.WALL
                        });
                    return;
                }

                // if WALL_OPENING | EMPTY
                if (normalGrid[i, r, c] == NormalGridValues.EMPTY) {
                    // set WALL_OPENING | WALL_OPENING
                    reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyChangeVoxelClassValues(
                        roomId,
                        VoxelClassValues.WALL_OPENING);
                    return;
                }

                // else (WALL_OPENING | NOT EMPTY)
                // set WALL_OPENING | WALL
                reconstructionGrid[i, r, c] = reconstructionGrid[i, r, c].CopyChangeVoxelClassValues(
                        roomId,
                        VoxelClassValues.WALL);
                return;
            }
        }

        private static void ResolveOverlapWithOtherRooms(
                int roomId,
                int i,
                int r,
                int c,
                int maxI,
                int[] voxelState,
                int[] voxelStateAbove,
                int[,,][] reconstructionGrid,
                out bool stopSweep) {

            int[] voxelClassValues;
            int[] voxelClassValuesAbove;

            stopSweep = false;

            foreach (int otherRoomId in voxelState.GetRoomIds()) {

                if (otherRoomId == roomId) {
                    continue;
                }

                voxelClassValues = voxelState.GetVoxelClassValues(otherRoomId);

                if (!voxelClassValues.Contains(VoxelClassValues.WALL)
                        && voxelClassValues.Contains(VoxelClassValues.CEILING)) {

                    voxelClassValuesAbove = voxelStateAbove.GetVoxelClassValues(roomId);

                    reconstructionGrid[i, r, c] = voxelState.CopyChangeVoxelClassValues(
                        roomId,
                        !voxelClassValuesAbove.Contains(VoxelClassValues.EMPTY_INTERIOR)
                                && !voxelClassValuesAbove.Contains(VoxelClassValues.EMPTY_INTERIOR) ?
                            new int[] {
                                VoxelClassValues.FLOOR,
                                VoxelClassValues.WALL
                            } : 
                            new int[] {
                                VoxelClassValues.FLOOR
                            });

                    if (i < maxI) {
                        reconstructionGrid[maxI, r, c] =
                            reconstructionGrid[maxI, r, c].CopyRemoveRoom(roomId);
                    }

                    stopSweep = true;
                    return;
                }
                else if (voxelClassValues.Contains(VoxelClassValues.WALL)
                        && voxelClassValues.Contains(VoxelClassValues.CEILING)) {

                    voxelClassValuesAbove = voxelStateAbove.GetVoxelClassValues(roomId);

                    if (voxelClassValuesAbove.Contains(VoxelClassValues.EMPTY_INTERIOR)
                            || voxelClassValuesAbove.Contains(VoxelClassValues.INTERIOR_OBJECT)
                            || ((voxelClassValuesAbove.Contains(VoxelClassValues.CEILING)
                                    || voxelClassValuesAbove.Contains(VoxelClassValues.FLOOR))
                                && !voxelClassValuesAbove.Contains(VoxelClassValues.WALL))) {

                        reconstructionGrid[i, r, c] = voxelState.CopyChangeVoxelClassValues(
                            roomId,
                            new int[] { 
                                VoxelClassValues.FLOOR,
                                VoxelClassValues.WALL
                            });

                        if (i < maxI) {
                            reconstructionGrid[maxI, r, c] =
                                reconstructionGrid[maxI, r, c].CopyRemoveRoom(roomId);
                        }
                    }

                    stopSweep = true;
                }
            }
        }
    }
}
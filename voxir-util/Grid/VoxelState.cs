using System;
using System.Collections.Generic;
using System.Linq;

namespace HuePat.VoxIR.Util.Grid {
    public static class VoxelState {

        public static int[] CreateVoxelState(
                int roomId,
                int voxelClassValue) {

            return CreateVoxelState(
                roomId,
                new int[] {
                    voxelClassValue
                });
        }

        public static int[] CreateVoxelState(
                int roomId,
                int[] voxelClassValues) {

            int[] voxelState;

            voxelState = new int[voxelClassValues.Length + 3];
            voxelState[0] = 1;
            voxelState[1] = roomId;
            voxelState[2] = voxelClassValues.Length;

            for (int j = 0; j < voxelClassValues.Length; j++) {
                voxelState[j + 3] = voxelClassValues[j];
            }

            return voxelState;
        }

        public static bool HasRoomId(
                this int[] voxelState,
                int roomId) {

            int j = 1;
            int roomIndex = 0;

            if (voxelState == null) {
                return false;
            }

            do {
                if (voxelState[j] == roomId) {
                    return true;
                }

                j += voxelState[j + 1] + 2;
                roomIndex++;

            } while (roomIndex < voxelState[0]);

            return false;
        }

        public static bool HasOtherRoomIds(
                this int[] voxelState,
                int roomId) {

            int j = 1;
            int roomIndex = 0;

            if (voxelState == null) {
                return false;
            }

            do {
                if (voxelState[j] != roomId) {
                    return true;
                }

                j += voxelState[j + 1] + 2;
                roomIndex++;

            } while (roomIndex < voxelState[0]);

            return false;
        }

        public static int GetRoomCount(
                this int[] voxelState) {

            return voxelState[0];
        }

        public static int[] GetRoomIds(
                this int[] voxelState) {

            int j = 1;
            int roomIndex = 0;
            int[] roomIds;

            if (voxelState == null) {
                return new int[0];
            }

            roomIds = new int[voxelState[0]];

            do {
                roomIds[roomIndex] = voxelState[j];
                j += voxelState[j + 1] + 2;
                roomIndex++;

            } while (roomIndex < voxelState[0]);

            return roomIds;
        }

        public static IEnumerable<int> GetOtherRoomIds(
                this int[] voxelState,
                int roomId) {

            return voxelState
                .GetRoomIds()
                .Where(otherRoomId => otherRoomId != roomId);
        }

        private static int GetVoxelClassValueCount(
                this int[] voxelState,
                int roomId) {

            int j = 1;
            int roomIndex = 0;

            if (voxelState == null) {
                throw new ArgumentException();
            }

            do {
                if (voxelState[j] == roomId) {
                    return voxelState[j + 1];
                }

                j += voxelState[j + 1] + 2;
                roomIndex++;

            } while (roomIndex < voxelState[0]);

            throw new ArgumentException();
        }

        public static int[] GetVoxelClassValues(
                this int[] voxelState,
                int roomId) {

            int j = 1;
            int k;
            int voxelClassValueCount;
            int voxelClassValueIndex = 0;
            int roomIndex = 0;
            int[] voxelClassValues = null;

            if (voxelState == null
                    || !voxelState.HasRoomId(roomId)) {
                return new int[0];
            }

            do {

                if (voxelState[j] == roomId) {

                    voxelClassValueCount = voxelState[j + 1];
                    voxelClassValues = new int[voxelClassValueCount];

                    for (k = j + 2; k <= j + voxelClassValueCount + 1; k++) {
                        voxelClassValues[voxelClassValueIndex++] = voxelState[k];
                    }

                    break;
                }

                j += voxelState[j + 1] + 2;
                roomIndex++;

            } while (roomIndex < voxelState[0]);

            return voxelClassValues;
        }

        public static int[] GetVoxelClassValues(
                this int[] voxelState) {

            if (voxelState == null) {
                return new int[0];
            }

            return voxelState
                .GetRoomIds()
                .SelectMany(roomId => voxelState.GetVoxelClassValues(roomId))
                .Distinct()
                .ToArray();
        }

        public static int[] CopyAddRoom(
                this int[] voxelState,
                int roomId,
                int voxelClassValue) {

            return voxelState.CopyAddRoom(
                roomId,
                new int[] { voxelClassValue });
        }

        public static int[] CopyAddRoom(
                this int[] voxelState,
                int roomId,
                int[] voxelClassValues) {

            int j = 1;
            int k;
            int roomIndex = 0;
            int[] copy;

            if (voxelState == null) {
                return CreateVoxelState(
                    roomId,
                    voxelClassValues);
            }

            if (voxelState.HasRoomId(roomId)) {
                return voxelState.CopyAddVoxelClassValues(
                    roomId,
                    voxelClassValues);
            }

            copy = new int[voxelState.Length + voxelClassValues.Length + 2];
            copy[0] = voxelState[0] + 1;
            do {
                for (k = j; k < j + voxelState[j + 1] + 2; k++) {
                    copy[k] = voxelState[k];
                }
                j = k;
                roomIndex++;

            } while (roomIndex < voxelState[0]);

            copy[j] = roomId;
            copy[j + 1] = voxelClassValues.Length;
            for (k = 0; k < voxelClassValues.Length; k++) {
                copy[k + j + 2] = voxelClassValues[k];
            }

            for (k = j + voxelClassValues.Length + 2; k < copy.Length; k++) {
                copy[k] = voxelState[k - voxelClassValues.Length - 2];
            }

            return copy;
        }

        public static int[] CopyRemoveRoom(
                this int[] voxelState,
                int roomId) {

            int k, j = 1;
            int voxelClassValueCount;
            int[] copy;
            int[] roomIds;

            if (voxelState == null) {
                return null;
            }

            roomIds = voxelState.GetRoomIds();

            if (!roomIds.Contains(roomId)) {
                return voxelState.Copy();
            }
            if (roomIds.Length == 1) {
                return null;
            }

            voxelClassValueCount = voxelState.GetVoxelClassValueCount(roomId);
            copy = new int[voxelState.Length - voxelClassValueCount - 2];

            do {
                if (voxelState[j] == roomId) {
                    break;
                }
                j += voxelState[j + 1] + 2;
            } while (j < voxelState.Length);

            copy[0] = roomIds.Length - 1;
            for (k = 1; k < j; k++) {
                copy[k] = voxelState[k];
            }
            for (k = j + voxelClassValueCount + 2; k < voxelState.Length; k++) {
                copy[k - voxelClassValueCount - 2] = voxelState[k];
            }

            return copy;
        }

        public static int[] CopyAddVoxelClassValue(
                this int[] voxelState,
                int roomId,
                int voxelClassValue) {

            return voxelState.CopyAddVoxelClassValues(
                roomId,
                new int[] { 
                    voxelClassValue
                });
        }

        public static int[] CopyAddVoxelClassValues(
                this int[] voxelState,
                int roomId,
                int[] voxelClassValues) {

            int k, j = 1;
            int[] values;
            int[] copy;
            HashSet<int> existingValues;

            if (!voxelState.HasRoomId(roomId)) {
                return voxelState.CopyAddRoom(
                    roomId,
                    voxelClassValues);
            }

            existingValues = voxelState
                .GetVoxelClassValues(roomId)
                .ToHashSet();
            values = voxelClassValues
                .Distinct()
                .Where(value => !existingValues.Contains(value))
                .ToArray();

            copy = new int[voxelState.Length + values.Length];
            do {
                if (voxelState[j] == roomId) {
                    break;
                }
                j += voxelState[j + 1] + 2;
            } while (j < voxelState.Length);
            for (k = 0; k <= j; k++) {
                copy[k] = voxelState[k];
            }

            copy[j + 1] = voxelState[j + 1] + values.Length;
            for (k = 0; k < values.Length; k++) {
                copy[j + k + 2] = values[k];
            }
            for (k = j + 2; k < voxelState.Length; k++) {
                copy[k + values.Length] = voxelState[k];
            }

            return copy;
        }

        public static int[] CopyChangeVoxelClassValues(
                this int[] voxelState,
                int roomId,
                int voxelClassValue) {

            return voxelState.CopyChangeVoxelClassValues(
                roomId,
                new int[] { 
                    voxelClassValue
                });
        }

        public static int[] CopyChangeVoxelClassValues(
                this int[] voxelState,
                int roomId,
                int[] voxelClassValues) {

            return voxelState
                .CopyRemoveRoom(roomId)
                .CopyAddRoom(
                    roomId,
                    voxelClassValues);
        }

        public static int[] CopyRemoveVoxelClassValue(
                this int[] voxelState,
                int roomId,
                int voxelClassValue) {

            int k, j = 1;
            int[] copy;
            int[] voxelClassValues;
            int[] newVoxelClassificationValues;

            voxelClassValues = voxelState.GetVoxelClassValues(roomId);

            if (!voxelClassValues.Contains(voxelClassValue)) {
                return voxelState.Copy();
            }
            if (voxelClassValues.Length == 1) {
                return voxelState.CopyRemoveRoom(roomId);
            }

            copy = new int[voxelState.Length - 1];
            newVoxelClassificationValues = voxelClassValues
                .Where(value => value != voxelClassValue)
                .ToArray();

            do {
                if (voxelState[j] == roomId) {
                    break;
                }
                j += voxelState[j + 1] + 2;
            } while (j < voxelState.Length);

            for (k = 0; k <= j; k++) {
                copy[k] = voxelState[k];
            }
            copy[j + 1] = newVoxelClassificationValues.Length;
            for (k = 0; k < newVoxelClassificationValues.Length; k++) {
                copy[j + k + 2] = newVoxelClassificationValues[k];
            }
            for (k = j + voxelClassValues.Length + 2; k < voxelState.Length; k++) {
                copy[k - 1] = voxelState[k];
            }

            return copy;
        }
    }
}
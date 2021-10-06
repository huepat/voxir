using HuePat.VoxIR.Util.Grid;
using System.Collections.Generic;
using System.Linq;

namespace HuePat.VoxIR.VoxelModelRefinement {
    public static class WallNormals {
        public static int[] GetBaseVoxelState(
                this int[] voxelState) {

            int j = 1;
            int roomIndex = 0;
            int[] result;

            if (voxelState == null) {
                return null;
            }

            do {
                j += voxelState[j + 1] + 2;
                roomIndex++;

            } while (roomIndex < voxelState[0]);

            result = new int[j];

            for (j = 0; j < result.Length; j++) {
                result[j] = voxelState[j];
            }

            return result;
        }

        public static int GetNormalCount(
                this int[] voxelState,
                int roomId) {

            bool isInNormalsSection = false;
            int j;
            int roomIndex = 0;

            if (voxelState == null) {
                return 0;
            }

            for (j = 1; j < voxelState.Length; j++) {

                if (isInNormalsSection) {

                    if (voxelState[j] == roomId) {
                        return voxelState[j + 1];
                    }

                    j += 2 * voxelState[j + 1] + 1;
                }
                else {

                    j += voxelState[j + 1] + 1;
                    roomIndex++;

                    if (roomIndex >= voxelState[0]) {
                        isInNormalsSection = true;
                    }
                }
            }

            return 0;
        }

        public static (int, int)[][] GetNormals(
                this int[] voxelState) {

            return voxelState
                .GetRoomIds()
                .Select(roomId => voxelState.GetNormals(roomId))
                .Where(normals => normals.Length > 0)
                .ToArray();
        }

        public static (int, int)[] GetNormals(
                this int[] voxelState,
                int roomId) {

            bool isInNormalsSection = false;
            int j, k;
            int roomIndex = 0;
            int normalCount;
            (int, int)[] normals;

            if (voxelState == null) {
                return new (int, int)[0];
            }

            for (j = 1; j < voxelState.Length; j++) {

                if (isInNormalsSection) {

                    if (voxelState[j] == roomId) {

                        normalCount = voxelState[j + 1];
                        normals = new (int, int)[normalCount];

                        for (k = 0; k < normalCount; k++) {
                            normals[k] = (
                                voxelState[j + 2 * k + 2],
                                voxelState[j + 2 * k + 3]
                            );
                        }

                        return normals;
                    }

                    j += 2 * voxelState[j + 1] + 1;
                }
                else {

                    j += voxelState[j + 1] + 1;
                    roomIndex++;

                    if (roomIndex >= voxelState[0]) {
                        isInNormalsSection = true;
                    }
                }
            }

            return new (int, int)[0];
        }

        public static int[] CopySetNormal(
                this int[] voxelState,
                int roomId,
                (int, int) normal) {

            return voxelState.CopySetNormals(
                roomId,
                new (int, int)[] { 
                    normal
                });
        }

        public static int[] CopySetNormals(
                this int[] voxelState,
                int roomId,
                (int, int)[] normals) {

            bool hasRoomId = false;
            int j, k;
            int[] baseVoxelState;
            List<int> result = new List<int>();

            baseVoxelState = voxelState.GetBaseVoxelState();
            result.AddRange(baseVoxelState);

            for (j = baseVoxelState.Length; j < voxelState.Length; j++) {

                if (voxelState[j] == roomId) {

                    hasRoomId = true;
                    result.Add(
                        roomId,
                        normals);
                    j += 2 * voxelState[j + 1] + 1;
                }
                else {

                    for (k = j; k <= j + 2 * voxelState[j + 1] + 1; k++) {
                        result.Add(voxelState[k]);
                    }
                    j = k - 1;
                }
            }

            if (!hasRoomId) {
                result.Add(
                    roomId,
                    normals);
            }

            return result.ToArray();
        }

        public static int[] CopyRemoveNormals(
                this int[] voxelState,
                int roomId) {

            int j, k;
            int[] baseVoxelState;
            List<int> result = new List<int>();

            if (voxelState == null) {
                return null;
            }

            baseVoxelState = voxelState.GetBaseVoxelState();
            result.AddRange(baseVoxelState);

            for (j = baseVoxelState.Length; j < voxelState.Length; j++) {

                if (voxelState[j] == roomId) {
                    j += 2 * voxelState[j + 1] + 1;
                }
                else {

                    for (k = j; k <= j + 2 * voxelState[j + 1] + 1; k++) {
                        result.Add(voxelState[k]);
                    }
                    j = k - 1;
                }
            }

            return result.ToArray();
        }

        private static void Add(
                this List<int> newVoxelState,
                int roomId,
                (int, int)[] normals) {

            newVoxelState.Add(roomId);
            newVoxelState.Add(normals.Length);
            foreach ((int, int) normal in normals) {
                newVoxelState.Add(normal.Item1);
                newVoxelState.Add(normal.Item2);
            }
        }
    }
}
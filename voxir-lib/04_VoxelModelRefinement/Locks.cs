using System;
using System.Collections.Generic;

namespace HuePat.VoxIR.VoxelModelRefinement {
    static class Locks {
        public static int[] GetSave(
                this int[,,][] reconstructionGrid,
                int i,
                int r,
                int c,
                Tuple<int, int> partition,
                object globalLock,
                Dictionary<int, object> locks) {

            int[] voxelState;

            if (i > partition.Item1 && i < partition.Item2 - 1) {
                return reconstructionGrid[i, r, c];
            }

            lock (locks.GetLock(i, globalLock)) {
                voxelState = reconstructionGrid[i, r, c];
                if (voxelState != null) {
                    voxelState = voxelState.Copy();
                }
            }

            return voxelState;
        }

        public static void SetSave(
                this int[,,][] reconstructionGrid,
                int i,
                int r,
                int c,
                int[] voxelState,
                Tuple<int, int> partition,
                object globalLock,
                Dictionary<int, object> locks) {

            if (i > partition.Item1 && i < partition.Item2 - 1) {
                reconstructionGrid[i, r, c] = voxelState;
                return;
            }

            lock (locks.GetLock(i, globalLock)) {
                reconstructionGrid[i, r, c] = voxelState;
            }
        }

        private static object GetLock(
                this Dictionary<int, object> locks,
                int index,
                object @lock) {

            object localLock;

            lock (@lock) {
                if (!locks.ContainsKey(index)) {
                    locks.Add(
                        index, 
                        new object());
                }
                localLock = locks[index];
            }

            return localLock;
        }
    }
}
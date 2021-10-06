using HuePat.VoxIR.Util.Grid;
using System.Collections.Generic;
using System.Linq;

namespace HuePat.VoxIR.VoxelClassification {
    public static class VoxelClassification {
        public static void InitializeWalls(
                int[,][] ceilingGrid,
                int[,][] floorGrid) {

            int wallHeight, r, c;

            for (r = 0; r < ceilingGrid.GetLength(0); r++) {
                for (c = 0; c < ceilingGrid.GetLength(1); c++) {

                    if (ceilingGrid[r, c] != null) {
                        continue;
                    }

                    if (IsWall(
                            r,
                            c,
                            ceilingGrid,
                            out wallHeight)) {

                        ceilingGrid[r, c] = PixelState.CreatePixelState(
                            PixelClassValues.WALL,
                            wallHeight);
                        InitializeFloorWall(
                            r, 
                            c, 
                            floorGrid);
                    }
                }
            }
        }

        public static void UpdateReconstructionGrid(
                int roomId,
                int maxCeilingHeight,
                (int, int, int) ceilingGridOffset,
                int[,][] ceilingGrid,
                int[,][] floorGrid,
                int[,,][] reconstructionGrid) {

            int r, r2, c, c2;
            int height;
            int pixelClassValue;
            int[] pixelState;

            for (r = 0; r < ceilingGrid.GetLength(0); r++) {
                for (c = 0; c < ceilingGrid.GetLength(1); c++) {

                    pixelState = ceilingGrid[r, c];
                    if (pixelState == null) {
                        continue;
                    }

                    r2 = r + ceilingGridOffset.Item2;
                    c2 = c + ceilingGridOffset.Item3;
                    if (r2 < 0 || c2 < 0
                            || r2 >= reconstructionGrid.GetLength(1)
                            || c2 >= reconstructionGrid.GetLength(2)) {
                        continue;
                    }

                    height = pixelState.GetPixelHeight();
                    pixelClassValue = pixelState.GetPixelClassValue();

                    Update(
                        false,
                        roomId,
                        height,
                        r2,
                        c2,
                        pixelClassValue,
                        reconstructionGrid);

                    Update(
                        true,
                        roomId,
                        floorGrid[r, c].GetPixelHeight(),
                        r2,
                        c2,
                        pixelClassValue,
                        reconstructionGrid);

                    RemoveCeilingVoxelsAboveMaxCeilingHeight(
                        roomId,
                        height,
                        r2,
                        c2,
                        maxCeilingHeight,
                        ceilingGridOffset,
                        reconstructionGrid);
                }
            }
        }

        public static void DoVerticalVoxelClassificationSweep(
                int roomId,
                (int, int, int) ceilingGridOffset,
                int[,][] ceilingGrid,
                int[,][] floorGrid,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid,
                HashSet<int> roomIdsWithoutFloor) {

            int r, r2, c, c2;
            int[] ceilingPixelState;

            for (r = 0; r < ceilingGrid.GetLength(0); r++) {
                for (c = 0; c < ceilingGrid.GetLength(1); c++) {

                    ceilingPixelState = ceilingGrid[r, c];
                    if (ceilingPixelState == null) {
                        continue;
                    }

                    r2 = r + ceilingGridOffset.Item2;
                    c2 = c + ceilingGridOffset.Item3;
                    if (r2 < 0 || c2 < 0
                            || r2 >= reconstructionGrid.GetLength(1)
                            || c2 >= reconstructionGrid.GetLength(2)) {
                        continue;
                    }

                    Util.DoVerticalVoxelClassificationSweep(
                        roomId,
                        r2,
                        c2,
                        ceilingPixelState.GetPixelHeight(),
                        floorGrid[r, c].GetPixelHeight(),
                        normalGrid,
                        reconstructionGrid,
                        roomIdsWithoutFloor);
                }
            }
        }

        private static void Update(
                bool isFloor,
                int roomId,
                int height,
                int r,
                int c,
                int pixelClassValue,
                int[,,][] reconstructionGrid) {

            int voxelClassValue = isFloor ? 
                VoxelClassValues.FLOOR : 
                VoxelClassValues.CEILING;

            if (reconstructionGrid[height, r, c] == null) {

                reconstructionGrid[height, r, c] = VoxelState.CreateVoxelState(
                    roomId,
                    voxelClassValue);
            }
            else {
                reconstructionGrid[height, r, c]
                    = reconstructionGrid[height, r, c]
                        .CopyAddVoxelClassValue(
                            roomId,
                            voxelClassValue);
            }

            if (pixelClassValue == PixelClassValues.WALL) {

                reconstructionGrid[height, r, c]
                    = reconstructionGrid[height, r, c]
                        .CopyAddVoxelClassValue(
                            roomId,
                            VoxelClassValues.WALL);
            }
        }

        private static void InitializeFloorWall(
                int r, 
                int c,
                int[,][] floorGrid) {

            int i2, r2, dr, c2, dc;
            int wallHeight = int.MaxValue;
            int[] pixelState;

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    if (dr == 0 && dc == 0) {
                        continue;
                    }

                    r2 = r + dr;
                    c2 = c + dc;
                    if (r2 < 0 || c2 < 0 
                            || r2 >= floorGrid.GetLength(0)
                            || c2 >= floorGrid.GetLength(1)) {
                        continue;
                    }

                    pixelState = floorGrid[r2, c2];
                    if (pixelState == null
                            || pixelState.GetPixelClassValue() == PixelClassValues.WALL) {
                        continue;
                    }

                    i2 = pixelState.GetPixelHeight();
                    if (i2 < wallHeight) {
                        wallHeight = i2;
                    }
                }
            }

            floorGrid[r, c] = PixelState.CreatePixelState(
                PixelClassValues.WALL,
                wallHeight);
        }

        private static bool IsWall(
                int r,
                int c,
                int[,][] ceilingGrid,
                out int wallHeight) {

            bool isWall = false;
            int i2, r2, dr, c2, dc;
            int[] pixelState;

            wallHeight = int.MinValue;

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    if (dr == 0 && dc == 0) {
                        continue;
                    }

                    r2 = r + dr;
                    c2 = c + dc;
                    if (r2 < 0 || r2 >= ceilingGrid.GetLength(0)
                            || c2 < 0 || c2 >= ceilingGrid.GetLength(1)) {
                        continue;
                    }

                    pixelState = ceilingGrid[r2, c2];
                    if (pixelState != null
                            && pixelState.GetPixelClassValue() == PixelClassValues.ROOM) {

                        isWall = true;
                        i2 = pixelState.GetPixelHeight();
                        if (i2 > wallHeight) {
                            wallHeight = i2;
                        }
                    }
                }
            }

            return isWall;
        }

        private static void RemoveCeilingVoxelsAboveMaxCeilingHeight(
                int roomId,
                int i,
                int r, 
                int c,
                int maxCeilingHeight,
                (int, int, int) ceilingGridOffset,
                int[,,][] reconstructionGrid) {

            int i2;
            int[] voxelState;
            int[] voxelClassValues;

            for (i2 = ceilingGridOffset.Item1; i2 <= maxCeilingHeight; i2++) {

                if (i2 == i) {
                    continue;
                }

                voxelState = reconstructionGrid[i2, r, c];
                if (voxelState == null
                        || !voxelState.HasRoomId(roomId)) {
                    continue;
                }

                voxelClassValues = voxelState.GetVoxelClassValues(roomId);

                if (voxelClassValues.Contains(VoxelClassValues.CEILING)
                        && !voxelClassValues.Contains(VoxelClassValues.WALL)) {

                    reconstructionGrid[i2, r, c] = voxelState.CopyRemoveRoom(roomId);
                }
            }
        }
    }
}
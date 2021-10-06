using HuePat.VoxIR.Util.Grid;
using System.Collections.Generic;

namespace HuePat.VoxIR.CeilingAndFloorReconstruction {
    public static class CeilingRefinement {
        public static int[,][] Create2DCeilingGrid(
                VoxelSegment ceilingSegment,
                out int maxCeilingHeight,
                out (int, int, int) ceilingGridOffset) {

            int r, c;
            int[] pixelState;
            int[,][] ceilingGrid = new int[
                ceilingSegment.BBox.Size.Item2 + 2,
                ceilingSegment.BBox.Size.Item3 + 2][];

            ceilingGridOffset = (
                ceilingSegment.BBox.Min.Item1,
                ceilingSegment.BBox.Min.Item2 - 1,
                ceilingSegment.BBox.Min.Item3 - 1
            );
            maxCeilingHeight = ceilingGridOffset.Item1 + ceilingSegment.BBox.Size.Item1 - 1;

            foreach ((int, int, int) voxel in ceilingSegment) {

                r = voxel.Item2 - ceilingSegment.BBox.Min.Item2 + 1;
                c = voxel.Item3 - ceilingSegment.BBox.Min.Item3 + 1;

                pixelState = ceilingGrid[r, c];
                if (pixelState == null) {

                    ceilingGrid[r, c] = PixelState.CreatePixelState(
                        PixelClassValues.ROOM,
                        voxel.Item1);
                    continue;
                }

                if (voxel.Item1 < pixelState.GetPixelHeight()) {
                    pixelState.SetPixelHeight(voxel.Item1);
                }
            }

            return ceilingGrid;
        }

        public static List<Hole> DetectCeilingHoles(
                int[,][] ceilingGrid) {

            DetectHolePixels(ceilingGrid);

            return Util.SegmentHoles(
                false,
                -1,
                ceilingGrid);
        }

        public static void InterpolateCeilingHolesHeight(
                int roomId,
                int maxCeilingHeight,
                double resolution,
                (int, int, int) ceilingGridOffset,
                int[,][] ceilingGrid,
                int[,,][] reconstructionGrid,
                List<Hole> ceilingHoles) {

            List<Hole> resultHoles = new List<Hole>();

            foreach (Hole hole in ceilingHoles) {

                // holes detached from room
                if (Util.IsHoleDetachedFromRoom(
                        ceilingGrid,
                        hole)) {

                    foreach ((int, int) position in hole) {
                        ceilingGrid[
                            position.Item1,
                            position.Item2] = null;
                    }
                    continue;
                }

                // interpolate height
                Util.InterpolateHoleHeight(
                    resolution,
                    (
                        ceilingGridOffset.Item1,
                        maxCeilingHeight
                    ),
                    ceilingGrid,
                    hole);

                resultHoles.Add(hole);
            }

            Util.SmoothHeightOverWholeGrid(
                roomId,
                resolution,
                Parameters.CEILING_GRID_SMOOTHING_DIAMETER,
                ceilingGridOffset,
                ceilingGrid,
                reconstructionGrid);
        }

        private static void DetectHolePixels(
                int[,][] ceilingGrid) {

            bool isInCeilingAgain, isOutsideOfCeiling;
            int r, r2, c, c2;
            int[] pixelState;

            for (r = 0; r < ceilingGrid.GetLength(0); r++) {
                for (c = 0; c < ceilingGrid.GetLength(1); c++) {

                    pixelState = ceilingGrid[r, c];
                    if (pixelState != null 
                            && pixelState.IsPixelClassValueRoom()) {
                        continue;
                    }

                    isInCeilingAgain = false;
                    isOutsideOfCeiling = false;

                    for (r2 = r - 1; r2 >= 0; r2--) {
                        pixelState = ceilingGrid[r2, c];
                        if (pixelState != null && pixelState.IsPixelClassValueRoom()) {
                            isOutsideOfCeiling = true;
                            break;
                        }
                    }

                    if (isOutsideOfCeiling) {

                        for (r2 = r + 1; r2 < ceilingGrid.GetLength(0); r2++) {
                            pixelState = ceilingGrid[r2, c];
                            if (pixelState != null && pixelState.IsPixelClassValueRoom()) {
                                isInCeilingAgain = true;
                                break;
                            }
                        }
                    }

                    if (!isInCeilingAgain) {

                        isOutsideOfCeiling = false;

                        for (c2 = c - 1; c2 >= 0; c2--) {
                            pixelState = ceilingGrid[r, c2];
                            if (pixelState != null && pixelState.IsPixelClassValueRoom()) {
                                isOutsideOfCeiling = true;
                                break;
                            }
                        }

                        if (isOutsideOfCeiling) {

                            for (c2 = c + 1; c2 < ceilingGrid.GetLength(1); c2++) {
                                pixelState = ceilingGrid[r, c2];
                                if (pixelState != null && pixelState.IsPixelClassValueRoom()) {
                                    isInCeilingAgain = true;
                                    break;
                                }
                            }
                        }
                    }

                    if (!isInCeilingAgain) {

                        isOutsideOfCeiling = false;

                        for (r2 = r - 1, c2 = c - 1;
                                r2 >= 0 && c2 >= 0;
                                r2--, c2--) {
                            pixelState = ceilingGrid[r2, c2];
                            if (pixelState != null && pixelState.IsPixelClassValueRoom()) {
                                isOutsideOfCeiling = true;
                                break;
                            }
                        }

                        if (isOutsideOfCeiling) {

                            for (r2 = r + 1, c2 = c + 1;
                                    r2 < ceilingGrid.GetLength(0) && c2 < ceilingGrid.GetLength(1);
                                    r2++, c2++) {
                                pixelState = ceilingGrid[r2, c2];
                                if (pixelState != null && pixelState.IsPixelClassValueRoom()) {
                                    isInCeilingAgain = true;
                                    break;
                                }
                            }
                        }
                    }

                    if (!isInCeilingAgain) {

                        isOutsideOfCeiling = false;

                        for (r2 = r - 1, c2 = c + 1;
                                r2 >= 0 && c2 < ceilingGrid.GetLength(1);
                                r2--, c2++) {
                            pixelState = ceilingGrid[r2, c2];
                            if (pixelState != null && pixelState.IsPixelClassValueRoom()) {
                                isOutsideOfCeiling = true;
                                break;
                            }
                        }

                        if (isOutsideOfCeiling) {

                            for (r2 = r + 1, c2 = c - 1;
                                    r2 < ceilingGrid.GetLength(0) && c2 >= 0;
                                    r2++, c2--) {
                                pixelState = ceilingGrid[r2, c2];
                                if (pixelState != null && pixelState.IsPixelClassValueRoom()) {
                                    isInCeilingAgain = true;
                                    break;
                                }
                            }
                        }
                    }

                    if (isInCeilingAgain) {
                        ceilingGrid[r, c] = PixelState.CreatePixelState(PixelClassValues.HOLE_INTERIOR);
                    }
                }
            }
        }
    }
}
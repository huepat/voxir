using HuePat.VoxIR.Util.Grid;
using System.Collections.Generic;

namespace HuePat.VoxIR.CeilingAndFloorReconstruction {
    public static class CeilingAndFloorFinalization {
        public static void InterpolateFloorHeight(
                int roomId,
                int dominantFloorHeight,
                double resolution,
                (int, int) floorHeightRange,
                (int, int, int) ceilingGridOffset,
                int[,][] floorGrid,
                int[,,][] reconstructionGrid) {

            List<List<(int, int)>> floorGridRefinementSegments = GetFloorGridRefinementSegments(floorGrid);

            RemoveSmallFloorGridRefinementSegments(
                resolution,
                floorGrid,
                floorGridRefinementSegments);

            foreach (Hole hole in Util.SegmentHoles(
                    true,
                    dominantFloorHeight,
                    floorGrid)) {

                Util.InterpolateHoleHeight(
                    resolution,
                    floorHeightRange,
                    floorGrid,
                    hole);
            }

            EnsureEveryFloorGridPositionHasHeightValue(floorGrid);

            Util.SmoothHeightOverWholeGrid(
                roomId,
                resolution,
                Parameters.FLOOR_GRID_SMOOTHING_DIAMETER,
                ceilingGridOffset,
                floorGrid,
                reconstructionGrid);
        }

        private static List<List<(int, int)>> GetFloorGridRefinementSegments(
                int[,][] floorGrid) {

            int r, r2, dr, c, c2, dc;
            int[] pixelState;
            bool[,] isSegmented = new bool[
                floorGrid.GetLength(0),
                floorGrid.GetLength(1)];
            Queue<(int, int)> candidates = new Queue<(int, int)>();
            List<List<(int, int)>> segments = new List<List<(int, int)>>();

            for (r = 0; r < floorGrid.GetLength(0); r++) {
                for (c = 0; c < floorGrid.GetLength(1); c++) {

                    if (isSegmented[r, c]) {
                        continue;
                    }

                    pixelState = floorGrid[r, c];
                    if (pixelState == null
                            || pixelState.GetPixelClassValue() != PixelClassValues.ROOM) {
                        continue;
                    }

                    candidates.Enqueue((r, c));
                    List<(int, int)> segment = new List<(int, int)>();

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
                        segment.Add(candidate);

                        for (dr = -1; dr <= 1; dr++) {
                            for (dc = -1; dc <= 1; dc++) {

                                if (dr.Abs() == dc.Abs()) {
                                    continue;
                                }

                                r2 = candidate.Item1 + dr;
                                c2 = candidate.Item2 + dc;
                                if (r2 < 0 || c2 < 0
                                        || r2 >= floorGrid.GetLength(0)
                                        || c2 >= floorGrid.GetLength(1)) {
                                    continue;
                                }

                                pixelState = floorGrid[r2, c2];
                                if (pixelState != null
                                        && pixelState.GetPixelClassValue() == PixelClassValues.ROOM
                                        && (pixelState.GetPixelHeight()
                                                - floorGrid[
                                                        candidate.Item1,
                                                        candidate.Item2]
                                                    .GetPixelHeight())
                                            .Abs() <= 1) {

                                    candidates.Enqueue((r2, c2));
                                }
                            }
                        }
                    }
                    segments.Add(segment);
                }
            }

            return segments;
        }

        private static void RemoveSmallFloorGridRefinementSegments(
                double resolution,
                int[,][] floorGrid,
                List<List<(int, int)>> floorGridRefinementSegments) {

            int minFloorArea = Parameters
                .ROOMLET_MIN_AREA
                .GetAreaInVoxels(resolution);

            foreach (List<(int, int)> segment in floorGridRefinementSegments) {

                if (segment.Count < minFloorArea) {

                    foreach ((int, int) pixel in segment) {

                        floorGrid[pixel.Item1, pixel.Item2]
                            .SetPixelClassValue(
                                PixelClassValues.HOLE_INTERIOR);
                    }
                }
            }
        }

        // ensure every position in floor has a height value
        // (interpolation in SegmentHoles is not guaranteed to give height to every hole position because holes are not detected in the same way as with ceilings)
        private static void EnsureEveryFloorGridPositionHasHeightValue(
                int[,][] floorGrid) {

            bool found;
            int dr, r, r2, dc, c, c2;
            int[] pixelState, pixelState2;

            do {
                found = false;

                for (r = 0; r < floorGrid.GetLength(0); r++) {
                    for (c = 0; c < floorGrid.GetLength(1); c++) {

                        pixelState = floorGrid[r, c];
                        if (pixelState == null || pixelState.HasPixelHeight()) {
                            continue;
                        }

                        found = true;

                        for (dr = -1; dr <= 1; dr++) {
                            for (dc = -1; dc <= 1; dc++) {

                                if (dr.Abs() == dc.Abs()) {
                                    continue;
                                }

                                r2 = r + dr;
                                c2 = c + dc;
                                if (r2 < 0 || c2 < 0 
                                        || r2 >= floorGrid.GetLength(0) 
                                        || c2 >= floorGrid.GetLength(1)) {
                                    continue;
                                }

                                pixelState2 = floorGrid[r2, c2];
                                if (pixelState2 != null && pixelState2.HasPixelHeight()) {
                                    pixelState.SetPixelHeight(
                                        pixelState2.GetPixelHeight());
                                }
                            }
                        }
                    }
                }
            } while (found);
        }
    }
}
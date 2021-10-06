using HuePat.VoxIR.Util.Grid;
using System;
using System.Collections.Generic;

namespace HuePat.VoxIR.CeilingAndFloorReconstruction {
    static class Util {
        public static List<Hole> SegmentHoles(
                bool isFloor,
                int dominantHeight,
                int[,][] grid) {

            List<Hole> holes = new List<Hole>();
            List<List<(int, int)>> holeSegments;
            
            holeSegments = SegmentHolePixels(grid);

            foreach (List<(int, int)> holeSegment in holeSegments) {

                SegmentHole(
                    isFloor,
                    grid,
                    holeSegment,
                    out List<(int, int)> border,
                    out List<(int, int)> interior);

                if (border.Count == 0) {

                    if (!isFloor) {
                        throw new ApplicationException(
                            "Something is wrong! A ceiling segment should not have wholes without border.");
                    }

                    TurnHoleToFloor(
                        dominantHeight,
                        grid,
                        interior);
                }
                else {
                    holes.Add(
                        new Hole(
                            interior, 
                            border));
                }
            }

            return holes;
        }

        public static void InterpolateHoleHeight(
                double resolution,
                (int, int) heightRange,
                int[,][] grid,
                Hole hole) {

            List<List<(int, int)>> mainBorderSegments;

            if (hole.Interior.Count == 0) {
                return;
            }

            // get main border segment
            mainBorderSegments = GetMainBorderSegments(
                resolution,
                grid,
                hole);

            InterpolateHeightAcrossHole(
                grid,
                mainBorderSegments);

            // move interpolated hole interior positions so that each voxel is nearest to its neighbours
            SmoothHeightAcrossHole(
                heightRange,
                grid);
        }

        public static void SmoothHeightOverWholeGrid(
                int roomId,
                double resolution,
                double smoothingDiameter,
                (int, int, int) ceilingGridOffset,
                int[,][] grid,
                int[,,][] reconstructionGrid) {

            bool found, anyChanges;
            int r, c, d1, d2, x1, x2, h, h1, h2;
            int radius = (int)((int)(smoothingDiameter / resolution) / 2.0).Ceil() - 1; ;
            int[] voxelState;
            long checkSum;
            HashSet<long> checkSums = new HashSet<long>();

            do {
                checkSum = 0;
                anyChanges = false;

                for (r = 0; r < grid.GetLength(0); r++) {
                    for (c = 0; c < grid.GetLength(1); c++) {

                        if (grid[r, c] == null) {
                            continue;
                        }

                        found = false;
                        h = grid[r, c].GetPixelHeight();

                        for (d1 = -1; d1 >= -radius; d1--) {
                            for (d2 = 1; d2 <= radius; d2++) {

                                x1 = r + d1;
                                x2 = r + d2;
                                if (x1 < 0 || x2 < 0
                                        || x1 >= grid.GetLength(0)
                                        || x2 >= grid.GetLength(0)
                                        || grid[x1, c] == null
                                        || grid[x2, c] == null) {
                                    continue;
                                }

                                h1 = grid[x1, c].GetPixelHeight();
                                h2 = grid[x2, c].GetPixelHeight();
                                if (h1 == h2 && h1 != h) {

                                    voxelState = reconstructionGrid[
                                        h1,
                                        r + ceilingGridOffset.Item2,
                                        c + ceilingGridOffset.Item3];

                                    if (!voxelState.HasOtherRoomIds(roomId)) {
                                        found = true;
                                        grid[r, c].SetPixelHeight(h1);
                                        break;
                                    }
                                }

                                x1 = c + d1;
                                x2 = c + d2;
                                if (x1 < 0 || x2 < 0 
                                        || x1 >= grid.GetLength(1)
                                        || x2 >= grid.GetLength(1)
                                        || grid[r, x1] == null 
                                        || grid[r, x2] == null) {
                                    continue;
                                }

                                h1 = grid[r, x1].GetPixelHeight();
                                h2 = grid[r, x2].GetPixelHeight();
                                if (h1 == h2 
                                        && h1 != h) {

                                    voxelState = reconstructionGrid[
                                        h1,
                                        r + ceilingGridOffset.Item2,
                                        c + ceilingGridOffset.Item3];

                                    if (!voxelState.HasOtherRoomIds(roomId)) {

                                        checkSum += r + c;
                                        found = anyChanges = true;
                                        grid[r, c].SetPixelHeight(h1);
                                        break;
                                    }
                                }
                            }
                            if (found) {
                                break;
                            }
                        }
                    }
                }

                if (checkSums.Contains(checkSum)) {
                    break;
                }
                checkSums.Add(checkSum);

            } while (anyChanges);
        }

        public static bool IsHoleDetachedFromRoom(
                int[,][] grid,
                Hole hole) {

            int r, dr, c, dc;
            int[] pixelState;

            foreach ((int, int) pixel in hole) {

                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (dr.Abs() == dc.Abs()) {
                            continue;
                        }

                        r = pixel.Item1 + dr;
                        c = pixel.Item2 + dc;
                        if (r < 0 || c < 0
                                || r >= grid.GetLength(0)
                                || c >= grid.GetLength(1)) {
                            continue;
                        }

                        pixelState = grid[r, c];
                        if (pixelState != null
                                && pixelState.IsPixelClassValueRoom()) {
                            return false;
                        }
                    }
                }
            }
            return true;
        }

        public static bool IsPixelClassValueRoom(
                this int[] horizontalGridInfo) {

            return horizontalGridInfo[0] == PixelClassValues.ROOM
                || horizontalGridInfo[0] == PixelClassValues.WALL;
        }

        public static bool IsPixelClassValueHole(
                this int[] horizontalGridInfo) {

            return horizontalGridInfo[0] == PixelClassValues.HOLE_INTERIOR
                || horizontalGridInfo[0] == PixelClassValues.HOLE_BORDER;
        }

        private static List<List<(int, int)>> GetMainBorderSegments(
                double resolution,
                int[,][] grid,
                Hole hole) {

            int r, r2, dr, c, c2, dc;
            int maxHeightDifference = Parameters
                .HOLE_BORDER_MAX_HEIGHT_DIFFERENCE
                .GetDistanceInVoxels(resolution);
            bool[,] isSegmented = new bool[
                grid.GetLength(0),
                grid.GetLength(1)];
            bool[,] isBorder = new bool[
                grid.GetLength(0),
                grid.GetLength(1)];
            Queue<(int, int)> candidates = new Queue<(int, int)>();
            List<List<(int, int)>> mainBorderSegments = new List<List<(int, int)>>();

            foreach ((int, int) pixe in hole.Border) {
                isBorder[pixe.Item1, pixe.Item2] = true;
            }

            for (r = hole.BBox.Min.Item1; r <= hole.BBox.Max.Item1; r++) {
                for (c = hole.BBox.Min.Item2; c <= hole.BBox.Max.Item2; c++) {

                    if (isSegmented[r, c]
                            || !isBorder[r, c]) {
                        continue;
                    }

                    candidates.Enqueue((r, c));
                    List<(int, int)> borderSegment = new List<(int, int)>();

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
                        borderSegment.Add(candidate);

                        for (dr = -1; dr <= 1; dr++) {
                            for (dc = -1; dc <= 1; dc++) {

                                if (dr == 0 && dc == 0) {
                                    continue;
                                }
                                r2 = candidate.Item1 + dr;
                                c2 = candidate.Item2 + dc;

                                if (r2 >= 0 && c2 >= 0
                                        && r2 < grid.GetLength(0)
                                        && c2 < grid.GetLength(1)
                                        && !isSegmented[r2, c2]
                                        && isBorder[r2, c2]
                                        && (grid[candidate.Item1, candidate.Item2].GetPixelHeight()
                                                - grid[r2, c2].GetPixelHeight()).Abs()
                                            <= maxHeightDifference) {
                                    candidates.Enqueue((r2, c2));
                                }
                            }
                        }
                    } while (candidates.Count > 0);

                    if (mainBorderSegments.Count == 0
                            || borderSegment.Count == mainBorderSegments[0].Count) {
                        mainBorderSegments.Add(borderSegment);
                    }
                    else if (borderSegment.Count > mainBorderSegments[0].Count) {
                        mainBorderSegments = new List<List<(int, int)>> {
                            borderSegment
                        };
                    }
                }
            }

            return mainBorderSegments;
        }

        private static List<List<(int, int)>> SegmentHolePixels(
                int[,][] grid) {

            int r, r2, dr, c, c2, dc;
            int[] pixelState;
            bool[,] isSegmented = new bool[
                grid.GetLength(0),
                grid.GetLength(1)];
            Queue<(int, int)> candidates = new Queue<(int, int)>();
            List<List<(int, int)>> holeSegments = new List<List<(int, int)>>();

            for (r = 0; r < grid.GetLength(0); r++) {
                for (c = 0; c < grid.GetLength(1); c++) {

                    if (isSegmented[r, c]) {
                        continue;
                    }

                    pixelState = grid[r, c];
                    if (pixelState == null
                            || !pixelState.IsPixelClassValueHole()) {
                        continue;
                    }

                    candidates.Enqueue((r, c));
                    List<(int, int)> holeSegment = new List<(int, int)>();
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
                        holeSegment.Add(candidate);

                        for (dr = -1; dr <= 1; dr++) {
                            for (dc = -1; dc <= 1; dc++) {

                                if (dr.Abs() == dc.Abs()) {
                                    continue;
                                }

                                r2 = candidate.Item1 + dr;
                                c2 = candidate.Item2 + dc;
                                if (r2 < 0 || c2 < 0
                                        || r2 >= grid.GetLength(0)
                                        || c2 >= grid.GetLength(1)
                                        || isSegmented[r2, c2]) {
                                    continue;
                                }

                                pixelState = grid[r2, c2];
                                if (pixelState == null
                                        || !pixelState.IsPixelClassValueHole()) {
                                    continue;
                                }

                                candidates.Enqueue((r2, c2));
                            }
                        }
                    } while (candidates.Count > 0);
                    holeSegments.Add(holeSegment);
                }
            }

            return holeSegments;
        }

        private static void SegmentHole(
                bool isFloor,
                int[,][] grid,
                List<(int, int)> holeSegment,
                out List<(int, int)> border,
                out List<(int, int)> interior) {

            bool isBorder;
            int i, height, r, r2, dr, c, c2, dc;
            int[] pixelState;
            border = new List<(int, int)>();
            interior = new List<(int, int)>();

            foreach ((int, int) pixel in holeSegment) {

                r = pixel.Item1;
                c = pixel.Item2;
                isBorder = false;

                if (isFloor) {
                    height = int.MinValue;
                }
                else {
                    height = int.MaxValue;
                }

                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (dr == 0 && dc == 0) {
                            continue;
                        }

                        r2 = r + dr;
                        c2 = c + dc;
                        if (r2 < 0 || c2 < 0
                                || r2 >= grid.GetLength(0)
                                || c2 >= grid.GetLength(1)) {
                            continue;
                        }

                        pixelState = grid[r2, c2];
                        if (pixelState == null) {
                            continue;
                        }

                        if (pixelState.IsPixelClassValueRoom()) {

                            isBorder = true;
                            i = pixelState.GetPixelHeight();
                            if (isFloor && i > height) {
                                height = i;
                            }
                            if (!isFloor && i < height) {
                                height = i;
                            }
                        }
                    }
                }

                if (isBorder) {
                    grid[r, c] = PixelState.CreatePixelState(
                        PixelClassValues.HOLE_BORDER,
                        height);
                    border.Add(pixel);
                }
                else {
                    interior.Add(pixel);
                }
            }
        }

        private static void TurnHoleToFloor(
                int dominantHeight,
                int[,][] grid,
                List<(int, int)> interior) {

            foreach ((int, int) pixel in interior) {
                grid[pixel.Item1, pixel.Item2]
                    = PixelState.CreatePixelState(
                        PixelClassValues.ROOM,
                        dominantHeight);
            }
        }

        private static void InterpolateHeightAcrossHole(
                int[,][] grid,
                List<List<(int, int)>> mainBorderSegments) {

            int startHeight, finalHeight;
            List<(int, int)> holePixelsToUpdateHeight;

            foreach (List<(int, int)> segment in mainBorderSegments) {
                foreach ((int, int) holeBorderPixel in segment) {

                    startHeight = grid[holeBorderPixel.Item1, holeBorderPixel.Item2]
                        .GetPixelHeight();

                    foreach ((int, int) direction in GetDirections(holeBorderPixel, grid)) {

                        // go across hole along inverted directions and find hole interior positions to update
                        holePixelsToUpdateHeight = GetHolePixelsToUpdateAlongDirection(
                            holeBorderPixel,
                            direction,
                            grid,
                            out finalHeight);

                        // interpolate along inverted direction across hole or constant between hole border and outside
                        InterpolateHeight(
                            startHeight,
                            finalHeight,
                            grid,
                            holePixelsToUpdateHeight);
                    }
                }
            }
        }

        private static List<(int, int)> GetDirections(
                (int, int) holeBorderPixel,
                int[,][] grid) {

            int r, dr, c, dc;
            int[] pixelState;
            List<(int, int)> directions = new List<(int, int)>();

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    if (dr == 0 && dc == 0) {
                        continue;
                    }

                    r = holeBorderPixel.Item1 + dr;
                    c = holeBorderPixel.Item2 + dc;
                    if (r < 0 || c < 0
                            || r >= grid.GetLength(0)
                            || c >= grid.GetLength(1)) {
                        continue;
                    }

                    pixelState = grid[r, c];
                    if (pixelState == null
                            || pixelState.GetPixelClassValue() == PixelClassValues.HOLE_INTERIOR) {
                        continue;
                    }

                    directions.Add((dr, dc));
                }
            }

            return directions;
        }

        private static List<(int, int)> GetHolePixelsToUpdateAlongDirection(
                (int, int) holeBorderPixel,
                (int, int) direction,
                int[,][] grid,
                out int finalHeight) {

            int r, c;
            int pixelClassValue;
            int[] pixelState;
            List<(int, int)> holePixelsToUpdateHeight = new List<(int, int)>();

            finalHeight = -1;
            r = holeBorderPixel.Item1;
            c = holeBorderPixel.Item2;

            do {

                r -= direction.Item1;
                c -= direction.Item2;
                if (r < 0 || c < 0
                        || r >= grid.GetLength(0)
                        || c >= grid.GetLength(1)) {
                    break;
                }

                pixelState = grid[r, c];
                if (pixelState == null) {
                    break;
                }

                pixelClassValue = pixelState.GetPixelClassValue();
                if (pixelClassValue == PixelClassValues.HOLE_INTERIOR) {
                    holePixelsToUpdateHeight.Add((r, c));
                }
                else if (pixelClassValue == PixelClassValues.HOLE_BORDER) {
                    finalHeight = pixelState.GetPixelHeight();
                    break;
                }
                else {
                    break;
                }
            } while (true);

            return holePixelsToUpdateHeight;
        }

        private static void InterpolateHeight(
                int startHeight,
                int finalHeight,
                int[,][] grid,
                List<(int, int)> holePixelsToUpdateHeight) {

            int j, r, c;
            int counter;
            int height, oldHeight;
            int[,] heightContributionCounter = new int[
                grid.GetLength(0),
                grid.GetLength(1)
            ];

            for (j = 0; j < holePixelsToUpdateHeight.Count; j++) {

                r = holePixelsToUpdateHeight[j].Item1;
                c = holePixelsToUpdateHeight[j].Item2;

                if (finalHeight < 0) {
                    grid[r, c].SetPixelHeight(startHeight);
                }
                else {

                    counter = ++heightContributionCounter[r, c];
                    height = (int)(startHeight + ((double)(finalHeight - startHeight) / holePixelsToUpdateHeight.Count) * j);

                    if (counter == 1) {

                        oldHeight = grid[r, c].GetPixelHeight();
                        grid[r, c].SetPixelHeight(
                            oldHeight + (height - oldHeight) / (counter));
                    }
                    else {
                        grid[r, c].SetPixelHeight(height);
                    }
                }
            }
        }

        private static void SmoothHeightAcrossHole(
                (int, int) heightRange,
                int[,][] grid) {

            int i, r, r2, dr, c, c2, dc;
            int pixelClassValue;
            int sumOfHeightDifferences, minSumOfHeightDifferences;
            int finalHeight, height;
            int[] pixelState;

            for (r = 0; r < grid.GetLength(0); r++) {
                for (c = 0; c < grid.GetLength(1); c++) {

                    pixelState = grid[r, c];
                    if (pixelState == null) {
                        continue;
                    }

                    pixelClassValue = pixelState.GetPixelClassValue();
                    if (pixelClassValue != PixelClassValues.HOLE_INTERIOR) {
                        continue;
                    }

                    minSumOfHeightDifferences = int.MaxValue;
                    height = pixelState.GetPixelHeight();
                    finalHeight = height;

                    for (i = heightRange.Item1; i <= heightRange.Item2; i++) {

                        sumOfHeightDifferences = int.MaxValue;

                        for (dr = -1; dr <= 1; dr++) {
                            for (dc = -1; dc <= 1; dc++) {

                                if (dr == 0 && dc == 0) {
                                    continue;
                                }

                                r2 = r + dr;
                                c2 = c + dc;
                                if (r2 < 0 || r2 >= grid.GetLength(0)
                                        || c2 < 0 || c2 >= grid.GetLength(1)) {
                                    continue;
                                }

                                pixelState = grid[r2, c2];
                                if (pixelState == null) {
                                    continue;
                                }

                                sumOfHeightDifferences += (pixelState.GetPixelHeight() - i).Abs();
                            }
                        }

                        if (sumOfHeightDifferences < minSumOfHeightDifferences) {

                            minSumOfHeightDifferences = sumOfHeightDifferences;
                            finalHeight = i;
                        }
                    }
                    grid[r, c].SetPixelHeight(finalHeight);
                }
            }
        }
    }
}
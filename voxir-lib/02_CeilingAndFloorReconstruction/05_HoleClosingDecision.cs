using HuePat.VoxIR.Util.Grid;
using System;
using System.Collections.Generic;

namespace HuePat.VoxIR.CeilingAndFloorReconstruction {
    public static class HoleClosingDecision {
        public static void RemoveCeilingHoleOverlapsWithOtherRooms(
                int roomId,
                (int, int, int) ceilingGridOffset,
                int[,][] ceilingGrid,
                int[,][] floorGrid,
                int[,,][] reconstructionGrid,
                ref List<Hole> ceilingHoles) {

            List<Hole> result = new List<Hole>();

            foreach (Hole hole in ceilingHoles) {

                result.AddRange(
                    RemoveOverlapWithOtherRooms(
                        roomId,
                        ceilingGridOffset,
                        ceilingGrid,
                        floorGrid,
                        reconstructionGrid,
                        hole));
            }

            ceilingHoles = result;
        }

        public static void CloseClosableCeilingHoles(
                double resolution,
                (int, int, int) ceilingGridOffset,
                int[,][] ceilingGrid,
                int[,][] floorGrid,
                byte[,,] normalGrid,
                List<Hole> holes) {

            foreach (Hole hole in holes) {

                if (CanHoleBeClosed(
                        resolution,
                        ceilingGridOffset,
                        ceilingGrid,
                        floorGrid,
                        normalGrid,
                        hole)) {

                    foreach ((int, int) pixel in hole) {

                        ceilingGrid[pixel.Item1, pixel.Item2]
                            .SetPixelClassValue(PixelClassValues.ROOM);
                        floorGrid[pixel.Item1, pixel.Item2]
                            .SetPixelClassValue(PixelClassValues.ROOM);
                    }
                }
                else {
                    foreach ((int, int) pixel in hole) {

                        ceilingGrid[pixel.Item1, pixel.Item2] = null;
                        floorGrid[pixel.Item1, pixel.Item2] = null;
                    }
                }
            }

            FinalizeFloor(floorGrid);
        }

        private static List<Hole> RemoveOverlapWithOtherRooms(
                int roomId,
                (int, int, int) ceilingGridOffset,
                int[,][] ceilingGrid,
                int[,][] floorGrid,
                int[,,][] reconstructionGrid,
                Hole hole) {

            List<Hole> connectedHoleFragments = new List<Hole>();
            HashSet<(int, int)> overlap;
            
            overlap = DetectOverlapWithOtherRooms(
                roomId,
                ceilingGridOffset,
                ceilingGrid,
                floorGrid,
                reconstructionGrid,
                hole);

            if (overlap.Count > 0) {

                hole.Remove(overlap);

                Remove(
                    ceilingGrid,
                    floorGrid,
                    overlap);

                foreach (Hole holeSegment in ResegmentHole(ceilingGrid, hole)) {

                    if (!Util.IsHoleDetachedFromRoom(
                            ceilingGrid,
                            holeSegment)) {

                        connectedHoleFragments.Add(holeSegment);
                    }
                    else {
                        Remove(
                            ceilingGrid,
                            floorGrid,
                            holeSegment);
                    }
                }
            }
            else {
                connectedHoleFragments.Add(hole);
            }

            return connectedHoleFragments;
        }

        private static void Remove(
                int[,][] ceilingGrid,
                int[,][] floorGrid,
                IEnumerable<(int, int)> positions) {

            foreach ((int, int) position in positions) {

                ceilingGrid[
                    position.Item1,
                    position.Item2] = null;

                floorGrid[
                    position.Item1,
                    position.Item2] = null;
            }
        }

        private static HashSet<(int,int)> DetectOverlapWithOtherRooms(
                int roomId,
                (int, int, int) ceilingGridOffset,
                int[,][] ceilingGrid,
                int[,][] floorGrid,
                int[,,][] reconstructionGrid,
                Hole hole) {

            int i, minI, maxI, r, c;
            int[] voxelState;
            HashSet<(int, int)> overlap = new HashSet<(int, int)>();

            foreach ((int, int) pixel in hole) {

                r = pixel.Item1;
                c = pixel.Item2;
                minI = ceilingGrid[r, c].GetPixelHeight();
                maxI = (int)(minI + (floorGrid[r, c].GetPixelHeight() - minI) / 2.0).Round();

                for (i = minI; i <= maxI; i++) {

                    voxelState = reconstructionGrid[
                        i,
                        r + ceilingGridOffset.Item2,
                        c + ceilingGridOffset.Item3];
                    if (voxelState == null) {
                        continue;
                    }

                    if (voxelState.GetRoomCount() == 1
                            && voxelState.HasRoomId(roomId)) {
                        continue;
                    }

                    ceilingGrid[r, c] = null;
                    floorGrid[r, c] = null;
                    overlap.Add((r, c));
                    break;
                }
            }

            return overlap;
        }

        private static List<Hole> ResegmentHole(
                int[,][] ceilingGrid,
                Hole hole) {

            bool[,] occupancyGrid;
            List<List<(int, int)>> segments;

            occupancyGrid = GetOccupancyGrid(
                ceilingGrid, 
                hole);
            segments = Resegment(occupancyGrid);

            return CreateNewHolesFromSegments(
                ceilingGrid, 
                segments);
        }

        private static bool[,] GetOccupancyGrid(
                int[,][] ceilingGrid,
                IEnumerable<(int, int)> pixels) {

            bool[,] occupancyGrid = new bool[
                ceilingGrid.GetLength(0),
                ceilingGrid.GetLength(1)];

            foreach ((int, int) pixel in pixels) {
                occupancyGrid[
                    pixel.Item1,
                    pixel.Item2] = true;
            }

            return occupancyGrid;
        }

        private static List<List<(int, int)>> Resegment(
                bool[,] occupancyGrid) {

            int dr, r, r2, dc, c, c2;
            Queue<(int, int)> candidates = new Queue<(int, int)>();
            List<List<(int, int)>> segments = new List<List<(int, int)>>();
            bool[,] isSegmented = new bool[
                occupancyGrid.GetLength(0),
                occupancyGrid.GetLength(1)];

            for (r = 0; r < occupancyGrid.GetLength(0); r++) {
                for (c = 0; c < occupancyGrid.GetLength(1); c++) {

                    if (!occupancyGrid[r, c]
                            || isSegmented[r, c]) {
                        continue;
                    }

                    candidates.Enqueue((r, c));
                    List<(int, int)> segment = new List<(int, int)>();

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
                        segment.Add(candidate);

                        for (dr = -1; dr <= 1; dr++) {
                            for (dc = -1; dc <= 1; dc++) {

                                if (dr.Abs() == dc.Abs()) {
                                    continue;
                                }

                                r2 = candidate.Item1 + dr;
                                c2 = candidate.Item2 + dc;
                                if (r2 >= 0 && c2 >= 0
                                        && r2 < occupancyGrid.GetLength(0)
                                        && c2 < occupancyGrid.GetLength(1)
                                        && occupancyGrid[r2, c2]
                                        && !isSegmented[r2, c2]) {
                                    candidates.Enqueue((r2, c2));
                                }
                            }
                        }

                    } while (candidates.Count > 0);

                    segments.Add(segment);
                }
            }

            return segments;
        }

        private static List<Hole> CreateNewHolesFromSegments(
                int[,][] ceilingGrid,
                List<List<(int, int)>> segments) {

            int pixelClassValue;
            List<Hole> holes = new List<Hole>();

            foreach (List<(int, int)> segment in segments) {

                List<(int, int)> interior = new List<(int, int)>();
                List<(int, int)> border = new List<(int, int)>();

                foreach ((int, int) position in segment) {

                    pixelClassValue = ceilingGrid[
                        position.Item1,
                        position.Item2].GetPixelClassValue();

                    if (pixelClassValue == PixelClassValues.HOLE_BORDER) {
                        border.Add(position);
                    }
                    else if (pixelClassValue == PixelClassValues.HOLE_INTERIOR) {
                        interior.Add(position);
                    }
                    else {
                        throw new ApplicationException();
                    }
                }

                holes.Add(
                    new Hole(
                        interior, 
                        border));
            }

            return holes;
        }

        private static bool CanHoleBeClosed(
                double resolution,
                (int, int, int) ceilingGridOffset,
                int[,][] ceilingGrid,
                int[,][] floorGrid,
                byte[,,] normalGrid,
                Hole hole) {

            return hole.Interior.Count == 0
                || GetHoleBorderCompletenessRatio(
                    resolution,
                    ceilingGridOffset,
                    ceilingGrid,
                    floorGrid,
                    normalGrid,
                    hole) < Parameters.HOLE_BORDER_WALL_MAX_COMPLETENESS;
        }

        private static double GetHoleBorderCompletenessRatio(
                double resolution,
                (int, int, int) ceilingGridOffset,
                int[,][] ceilingGrid,
                int[,][] floorGrid,
                byte[,,] normalGrid,
                Hole hole) {

            int counter = 0;
            int counterFilled = 0;
            List<(int, int)> normals;
            List<(int, int, int)> contour;

            contour = GetHoleContourInRoom(
                ceilingGrid, 
                hole);

            foreach ((int, int, int) voxel in contour) {

                normals = GetNormals(
                    voxel,
                    ceilingGrid);

                UpdateCounters(
                    ref counter,
                    ref counterFilled,
                    resolution,
                    voxel,
                    ceilingGridOffset,
                    floorGrid,
                    normalGrid,
                    normals);
            }

            return (double)counterFilled / counter;
        }

        private static List<(int, int, int)> GetHoleContourInRoom(
                int[,][] ceilingGrid,
                Hole hole) {

            bool[,] isContour = new bool[
                hole.BBox.Size.Item1 + 2,
                hole.BBox.Size.Item2 + 2];
            List<(int, int, int)> contour = new List<(int, int, int)>();

            foreach ((int, int) pixel in hole.Border) {

                if (DoesNeighbourInteriorPixel(
                        pixel, 
                        ceilingGrid)) {

                    contour.AddRange(
                        GetContourInRoom(
                            pixel,
                            hole.BBox.Min,
                            isContour,
                            ceilingGrid));
                }
            }

            return contour;
        }

        private static bool DoesNeighbourInteriorPixel(
                (int, int) pixel,
                int[,][] ceilingGrid) {

            int r, dr, c, dc;
            int[] pixelState;

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    if (dr == 0 && dc == 0) {
                        continue;
                    }

                    r = pixel.Item1 + dr;
                    c = pixel.Item2 + dc;
                    if (r < 0 || c < 0
                            || r >= ceilingGrid.GetLength(0)
                            || c >= ceilingGrid.GetLength(1)) {
                        continue;
                    }

                    pixelState = ceilingGrid[r, c];
                    if (pixelState == null) {
                        continue;
                    }

                    if (pixelState.GetPixelClassValue() == PixelClassValues.HOLE_INTERIOR) {
                        return true;
                    }
                }
            }

            return false;
        }

        private static List<(int, int, int)> GetContourInRoom(
                (int, int) pixel,
                (int, int) holeOffset,
                bool[,] isContour,
                int[,][] ceilingGrid) {

            int r, r2, dr, c, c2, dc;
            int[] pixelState;
            List<(int, int, int)> contour = new List<(int, int, int)>();

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    if (dr == 0 && dc == 0) {
                        continue;
                    }

                    r = pixel.Item1 + dr;
                    c = pixel.Item2 + dc;
                    r2 = r - holeOffset.Item1 + 1;
                    c2 = c - holeOffset.Item2 + 1;
                    if (r < 0 || c < 0
                            || r >= ceilingGrid.GetLength(0)
                            || c >= ceilingGrid.GetLength(1)
                            || isContour[r2, c2]) {
                        continue;
                    }

                    pixelState = ceilingGrid[r, c];
                    if (pixelState != null 
                            && pixelState.IsPixelClassValueRoom()) {

                        isContour[r2, c2] = true;
                        contour.Add(
                            new(
                                pixelState.GetPixelHeight(),
                                r,
                                c
                            ));
                    }
                }
            }

            return contour;
        }

        private static List<(int, int)> GetNormals(
                (int, int, int) voxel,
                int[,][] ceilingGrid) {

            int r, dr, c, dc;
            int[] pixelState;
            List<(int, int)> normals = new List<(int, int)>();

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    if (dr == 0 && dc == 0) {
                        continue;
                    }

                    r = voxel.Item2 + dr;
                    c = voxel.Item3 + dc;
                    if (r < 0 || c < 0
                            || r >= ceilingGrid.GetLength(0)
                            || c >= ceilingGrid.GetLength(1)) {
                        continue;
                    }

                    pixelState = ceilingGrid[r, c];
                    if (pixelState != null
                            && pixelState.IsPixelClassValueHole()) {
                        normals.Add((dr, dc));
                    }
                }
            }

            return normals;
        }

        private static void UpdateCounters(
                ref int counter,
                ref int counterFilled,
                double resolution,
                (int, int, int) voxel,
                (int, int, int) ceilingGridOffset,
                int[,][] floorGrid,
                byte[,,] normalGrid,
                List<(int, int)> normals) {

            bool found;
            int d, i, maxI, r, c;
            int radius = Parameters
                .HOLE_WALL_SEARCH_RADIUS
                .GetDistanceInVoxels(resolution) - 1;

            maxI = voxel.Item1 + (int)((floorGrid[
                voxel.Item2,
                voxel.Item3].GetPixelHeight() - voxel.Item1) * 0.25).Round();

            for (i = voxel.Item1; i <= maxI; i++) {

                counter++;
                found = false;

                foreach ((int, int) normal in normals) {
                    for (d = -radius; d <= radius; d++) {

                        r = voxel.Item2 + d * normal.Item1 + ceilingGridOffset.Item2;
                        c = voxel.Item3 + d * normal.Item2 + ceilingGridOffset.Item3;
                        if (r < 0 || c < 0
                                || r >= normalGrid.GetLength(1)
                                || c >= normalGrid.GetLength(2)) {
                            continue;
                        }

                        if (normalGrid[i, r, c] != NormalGridValues.EMPTY) {
                            counterFilled++;
                            found = true;
                            break;
                        }
                    }

                    if (found) {
                        break;
                    }
                }
            }
        }

        private static void FinalizeFloor(
                int[,][] floorGrid) {

            int r, c;
            int[] pixelState;

            for (r = 0; r < floorGrid.GetLength(0); r++) {
                for (c = 0; c < floorGrid.GetLength(1); c++) {

                    pixelState = floorGrid[r, c];
                    if (pixelState != null) {

                        pixelState.SetPixelClassValue(
                            PixelClassValues.ROOM);
                    }
                }
            }
        }
    }
}
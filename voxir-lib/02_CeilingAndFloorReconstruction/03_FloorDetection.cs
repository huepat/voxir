using HuePat.VoxIR.Util.Grid;
using System.Collections.Generic;
using System.Linq;

namespace HuePat.VoxIR.CeilingAndFloorReconstruction {
    public static class FloorDetection {
        public static int[,][] Create2DFloorGrid(
                int roomId,
                (int, int, int) ceilingGridOffset,
                int[,][] ceilingGrid,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid) {

            bool found;
            int i, r, r2, c, c2;
            (int, int, int) size = (
                normalGrid.GetLength(0),
                ceilingGrid.GetLength(0),
                ceilingGrid.GetLength(1)
            );
            int[] pixelState, voxelState;
            int[,][] floorGrid = new int[
                    ceilingGrid.GetLength(0),
                    ceilingGrid.GetLength(1)][];

            for (r = 0; r < size.Item2; r++) {
                for (c = 0; c < size.Item3; c++) {

                    pixelState = ceilingGrid[r, c];
                    if (pixelState == null) {
                        continue;
                    }

                    found = false;
                    r2 = r + ceilingGridOffset.Item2;
                    c2 = c + ceilingGridOffset.Item3;

                    for (i = pixelState.GetPixelHeight() + 1; i < size.Item1; i++) {

                        voxelState = reconstructionGrid[i, r2, c2];
                        if (voxelState != null
                                && !voxelState.HasRoomId(roomId)) {
                            break;
                        }

                        if (normalGrid[i, r2, c2] == NormalGridValues.NORMAL_UP) {
                            found = true;
                            floorGrid[r, c] = PixelState.CreatePixelState(
                                PixelClassValues.ROOM,
                                i);
                            break;
                        }
                    }

                    if (!found) {
                        floorGrid[r, c] = PixelState.CreatePixelState(
                            PixelClassValues.HOLE_INTERIOR);
                    }
                }
            }

            return floorGrid;
        }

        public static List<List<(int, int)>> DetectFloorCandidateSegments(
                double resolution,
                int[,][] floorGrid) {

            int dr, r, r2, dc, c, c2;
            int maxHeightDifference = Parameters
                .FLOOR_MAX_HEIGHT_DIFFERENCE
                .GetDistanceInVoxels(resolution);
            int[] pixelState;
            bool[,] isSegmented = new bool[
                floorGrid.GetLength(0),
                floorGrid.GetLength(1)];
            Queue<(int, int)> candidates = new Queue<(int, int)>();
            List<List<(int, int)>> floorCandidateSegments = new List<List<(int, int)>>();

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

                    List<(int, int)> floorCandidateSegment = new List<(int, int)>();
                    candidates.Enqueue((r, c));
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
                        floorCandidateSegment.Add(candidate);

                        for (dr = -1; dr <= 1; dr++) {
                            for (dc = -1; dc <= 1; dc++) {

                                if (dr == 0 && dc == 0) {
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
                                if (pixelState == null
                                        || pixelState.GetPixelClassValue() != PixelClassValues.ROOM) {
                                    continue;
                                }

                                if ((floorGrid[candidate.Item1, candidate.Item2].GetPixelHeight()
                                            - pixelState.GetPixelHeight()).Abs() 
                                        <= maxHeightDifference) {

                                    candidates.Enqueue((r2, c2));
                                }
                            }
                        }
                    } while (candidates.Count > 0);
                    floorCandidateSegments.Add(floorCandidateSegment);
                }
            }

            return floorCandidateSegments;
        }

        public static void RemoveRoomWithoutFloor(
                int roomId,
                int[,,][] reconstructionGrid,
                HashSet<int> roomIdsWithoutFloor,
                List<VoxelSegment> ceilingSegments) {

            roomIdsWithoutFloor.Add(roomId);

            foreach ((int, int, int) voxel in ceilingSegments[roomId]) {

                reconstructionGrid[
                        voxel.Item1,
                        voxel.Item2,
                        voxel.Item3]
                    = reconstructionGrid[
                        voxel.Item1,
                        voxel.Item2,
                        voxel.Item3].CopyRemoveRoom(roomId);
            }
        }

        public static void SelectDominantFloorSegment(
                long[] floorIncidence,
                int[,][] floorGrid,
                List<List<(int, int)>> floorCandidateSegments,
                out int dominantFloorHeight,
                out (int, int) floorHeightRange) {

            // find floor candidate from floor segments
            List<int> dominantFloorSegmentIndices = GetDominantFloorSegmentIndices(
                floorIncidence,
                floorGrid,
                floorCandidateSegments);

            // add floor positions to floorGrid
            UpdateFloorGrid(
                floorGrid,
                dominantFloorSegmentIndices,
                floorCandidateSegments,
                out dominantFloorHeight,
                out floorHeightRange);
        }

        private static List<int> GetDominantFloorSegmentIndices(
                long[] floorIncidence,
                int[,][] floorGrid,
                List<List<(int, int)>> floorCandidateSegments) {

            return Enumerable
                .Range(
                    0, 
                    floorCandidateSegments.Count)
                .WhereMax(floorSegmentIndex => floorCandidateSegments[floorSegmentIndex]
                    .Select(pixel => floorIncidence[
                        floorGrid[
                            pixel.Item1,    
                            pixel.Item2].GetPixelHeight()])
                    .Sum());
        }

        private static void UpdateFloorGrid(
                int[,][] floorGrid,
                List<int> dominantFloorSegmentIndices,
                List<List<(int, int)>> floorCandidateSegments,
                out int dominantFloorHeight,
                out (int, int) floorHeightRange) {

            int j, height;
            int maxHeight = int.MaxValue;
            int minHeight = int.MinValue;
            Dictionary<int, int> heightCounters = new Dictionary<int, int>();

            for (j = 0; j < floorCandidateSegments.Count; j++) {

                if (dominantFloorSegmentIndices.Contains(j)) {

                    foreach ((int, int) pixel in floorCandidateSegments[j]) {

                        height = floorGrid[pixel.Item1, pixel.Item2]
                            .GetPixelHeight();

                        if (height < maxHeight) {
                            maxHeight = height;
                        }
                        if (height > minHeight) {
                            minHeight = height;
                        }

                        heightCounters.BucketIncrement(height);
                        floorGrid[pixel.Item1, pixel.Item2] = PixelState.CreatePixelState(
                            PixelClassValues.ROOM,
                            height);
                    }
                }
                else {
                    foreach ((int, int) pixel in floorCandidateSegments[j]) {

                        floorGrid[pixel.Item1, pixel.Item2] = PixelState.CreatePixelState(
                            PixelClassValues.HOLE_INTERIOR);
                    }
                }
            }

            dominantFloorHeight = heightCounters
                .Keys
                .WhereMax(h => heightCounters[h])
                .First();
            floorHeightRange = (minHeight, maxHeight);
        }
    }
}
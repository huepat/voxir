using HuePat.VoxIR.Util.Grid;
using System.Collections.Generic;
using System.Linq;

namespace HuePat.VoxIR.RoomSegmentation {
    public static class RoomPartitioning2D {

        private const double RAD_ANGLE_360_DEGREE = 2.0 * System.Math.PI;
        private const double RAD_ANGLE_22_5_DEGREE = 22.5 * System.Math.PI / 180.0;
        private const double RAD_ANGLE_67_5_DEGREE = 67.5 * System.Math.PI / 180.0;
        private const double RAD_ANGLE_112_5_DEGREE = 112.5 * System.Math.PI / 180.0;
        private const double RAD_ANGLE_157_5_DEGREE = 157.5 * System.Math.PI / 180.0;
        private const double RAD_ANGLE_202_5_DEGREE = 202.5 * System.Math.PI / 180.0;
        private const double RAD_ANGLE_247_5_DEGREE = 247.5 * System.Math.PI / 180.0;
        private const double RAD_ANGLE_292_5_DEGREE = 292.5 * System.Math.PI / 180.0;
        private const double RAD_ANGLE_337_5_DEGREE = 337.5 * System.Math.PI / 180.0;

        public static void RepartitionInteriorSpace(
                double resolution,
                bool[,] interiorSpaceGrid,
                out bool[,] bottleNeckGrid,
                out int[,] spacePartitioningGrid) {

            int roomCount;
            int transitionZoneId = 1;
            int lastTransitionSpaceId;
            bool[,] erosionGrid;
            bool[,] transitionSpaceGrid = new bool[
                interiorSpaceGrid.GetLength(0),
                interiorSpaceGrid.GetLength(1)];
            int[,] roomDomainGrid;
            int[,] roomDomainContactZoneSegmentGrid; 
            Dictionary<int, List<(int, int)>> roomSeedSegmentContours;
            Dictionary<(int, int), List<List<(int, int)>>> roomDomainContactZoneSegments;

            bottleNeckGrid = new bool[
                interiorSpaceGrid.GetLength(0),
                interiorSpaceGrid.GetLength(1)];

            erosionGrid = Erode(
                resolution,
                interiorSpaceGrid);

            spacePartitioningGrid = InitializeRoomSeedSegments(
                erosionGrid,
                out roomCount,
                out roomSeedSegmentContours);

            roomDomainGrid = InitializeRoomDomainGrid(
                interiorSpaceGrid,
                erosionGrid,
                spacePartitioningGrid,
                roomSeedSegmentContours);

            roomDomainContactZoneSegments = DetectRoomDomainContactZoneSegments(
                roomDomainGrid);

            roomDomainContactZoneSegmentGrid = InitializeRoomDomainContactZoneSegmentGrid(
                roomDomainGrid,
                roomDomainContactZoneSegments);

            foreach ((int, int) roomConnectionId in roomDomainContactZoneSegments.Keys) {
                foreach (List<(int, int)> roomDomainContactZoneSegment in roomDomainContactZoneSegments[roomConnectionId]) {

                    DetectTransitionSpaces(
                        resolution,
                        transitionZoneId,
                        roomConnectionId,
                        transitionSpaceGrid,
                        bottleNeckGrid,
                        roomDomainGrid,
                        roomDomainContactZoneSegmentGrid,
                        roomSeedSegmentContours[roomConnectionId.Item1],
                        roomSeedSegmentContours[roomConnectionId.Item2],
                        roomDomainContactZoneSegment);

                    transitionZoneId++;
                }
            }

            SegmentRoomsAndTransitionSpaces(
                roomCount,
                interiorSpaceGrid,
                transitionSpaceGrid,
                bottleNeckGrid,
                roomDomainGrid,
                spacePartitioningGrid,
                out lastTransitionSpaceId);

            ResolveConnectedRooms(
                lastTransitionSpaceId,
                resolution,
                interiorSpaceGrid,
                roomDomainGrid,
                spacePartitioningGrid,
                roomSeedSegmentContours);

            MergeDeadEndTransitionSpacesWithRooms(
                roomDomainGrid,
                spacePartitioningGrid);
        }

        private static bool[,] Erode(
                double resolution,
                bool[,] interiorSpaceGrid) {

            int kernelSize1 = Parameters
                .INTERIOR_SPACE_PARTITIONING_2D_1ST_EROSION_KERNEL_SIZE
                .GetDistanceInVoxels(resolution);
            int kernelSize2 = Parameters
                .INTERIOR_SPACE_PARTITIONING_2D_2ND_EROSION_KERNEL_SIZE
                .GetDistanceInVoxels(resolution);

            bool[,] kernel1 = InitializeCircleKernel(kernelSize1);
            bool[,] kernel2 = InitializeCircleKernel(kernelSize2);

            bool[,] erosionGrid = Erode(
                interiorSpaceGrid,
                kernel1);

            bool[,] closingGrid = Dilate(
                erosionGrid,
                kernel1);

            bool[,] differenceGrid = Substract(
                interiorSpaceGrid,
                closingGrid);

            bool[,] erodedDifferenceGrid = Erode(
                differenceGrid,
                kernel2);

            List<List<(int, int)>> segments = Segment(erodedDifferenceGrid);

            Add(
                resolution,
                erosionGrid,
                segments);

            return erosionGrid;
        }

        private static bool[,] InitializeCircleKernel(
                int radius) {

            int r, c;
            double center = radius - 1;
            double threshold = ((double)radius).Squared();
            bool[,] kernel = new bool[
                2 * radius - 1,
                2 * radius - 1];

            for (r = 0; r < kernel.GetLength(0); r++) {
                for (c = 0; c < kernel.GetLength(1); c++) {
                    if ((center - r).Squared() + (center - c).Squared() <= threshold) {
                        kernel[r, c] = true;
                    }
                }
            }

            return kernel;
        }

        private static bool[,] Erode(
                bool[,] grid,
                bool[,] kernel) {
            
            bool erode;
            int r, dr, c, dc;
            int s_r = (kernel.GetLength(0) - 1) / 2;
            int s_c = (kernel.GetLength(1) - 1) / 2;
            bool[,] result = new bool[
                grid.GetLength(0),
                grid.GetLength(1)];

            for (r = s_r; r < grid.GetLength(0) - s_r; r++) {
                for (c = s_c; c < grid.GetLength(1) - s_c; c++) {

                    if (!grid[r, c]) {
                        continue;
                    }

                    erode = false;

                    for (dr = -s_r; dr <= s_r; dr++) {
                        for (dc = -s_c; dc <= s_c; dc++) {

                            if (kernel[dr + s_r, dc + s_r]
                                    && !grid[r + dr, c + dc]) {
                                erode = true;
                                break;
                            }
                        }

                        if (erode) {
                            break;
                        }
                    }

                    if (!erode) {
                        result[r, c] = true;
                    }
                }
            }

            return result;
        }

        private static bool[,] Dilate(
                bool[,] grid,
                bool[,] kernel) {
            
            int r, r2, dr, c, c2, dc;
            int s_r = (kernel.GetLength(0) - 1) / 2;
            int s_c = (kernel.GetLength(1) - 1) / 2;
            bool[,] result = new bool[
                grid.GetLength(0),
                grid.GetLength(1)];

            for (r = s_r; r < grid.GetLength(0) - s_r; r++) {
                for (c = s_c; c < grid.GetLength(1) - s_c; c++) {

                    if (!grid[r, c]) {
                        continue;
                    }

                    for (dr = -s_r; dr <= s_r; dr++) {
                        for (dc = -s_c; dc <= s_c; dc++) {

                            r2 = r + dr;
                            c2 = c + dc;
                            if (kernel[dr + s_r, dc + s_r]) {
                                result[r2, c2] = true;
                            }
                        }
                    }
                }
            }

            return result;
        }

        private static bool[,] Substract(
                bool[,] grid1,
                bool[,] grid2) {

            int r, c;
            bool[,] result = new bool[
                grid1.GetLength(0),
                grid1.GetLength(1)];

            for (r = 0; r < grid1.GetLength(0); r++) {
                for (c = 0; c < grid1.GetLength(1); c++) {

                    if (grid1[r, c] && !grid2[r, c]) {
                        result[r, c] = true;
                    }
                }
            }

            return result;
        }

        private static List<List<(int, int)>> Segment(
                bool[,] grid) {

            int dr, r, r2, dc, c, c2;
            (int, int) candidate;
            bool[,] isSegmented = new bool[
                grid.GetLength(0),
                grid.GetLength(1)];
            Queue<(int, int)> candidates = new Queue<(int, int)>();
            List<List<(int, int)>> segments = new List<List<(int, int)>>();

            for (r = 0; r < grid.GetLength(0); r++) {
                for (c = 0; c < grid.GetLength(1); c++) {

                    if (!grid[r, c]
                            || isSegmented[r, c]) {
                        continue;
                    }

                    candidates.Enqueue((r, c));
                    List<(int, int)> segment = new List<(int, int)>();
                    do {
                        candidate = candidates.Dequeue();
                        if (isSegmented[
                                candidate.Item1,
                                candidate.Item2]) {
                            continue;
                        }
                        segment.Add(candidate);
                        isSegmented[
                            candidate.Item1,
                            candidate.Item2] = true;

                        for (dr = -1; dr <= 1; dr++) {
                            for (dc = -1; dc <= 1; dc++) {

                                r2 = candidate.Item1 + dr;
                                c2 = candidate.Item2 + dc;
                                if (r2 >= 0 && c2 >= 0
                                        && r2 < grid.GetLength(0)
                                        && c2 < grid.GetLength(1)
                                        && grid[r2, c2]
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

        private static void Add(
                double resolution,
                bool[,] erosionGrid,
                List<List<(int, int)>> segments) {

            int minArea = Parameters
                .ROOM_MIN_AREA
                .GetAreaInVoxels(resolution);

            foreach (List<(int, int)> segment in segments) {
                if (segment.Count >= minArea) {
                    foreach ((int, int) pixel in segment) {
                        erosionGrid[
                            pixel.Item1,
                            pixel.Item2] = true;
                    }
                }
            }
        }

        private static int[,] InitializeRoomSeedSegments(
                bool[,] erosionGrid,
                out int roomCount,
                out Dictionary<int, List<(int, int)>> roomSeedSegmentContours) {

            int dr, r, r2, dc, c, c2;
            int[,] spacePartitioningGrid = new int[
                erosionGrid.GetLength(0),
                erosionGrid.GetLength(1)];
            Queue<(int, int)> candidates = new Queue<(int, int)>();

            roomCount = 0;
            roomSeedSegmentContours = new Dictionary<int, List<(int, int)>>();

            for (r = 0; r < erosionGrid.GetLength(0); r++) {
                for (c = 0; c < erosionGrid.GetLength(1); c++) {

                    if (!erosionGrid[r, c]
                            || spacePartitioningGrid[r, c] != 0
                            || !IsContour(
                                    r,
                                    c,
                                    erosionGrid)) {
                        continue;
                    }

                    roomCount++;
                    roomSeedSegmentContours.Add(
                        roomCount,
                        new List<(int, int)>());
                    candidates.Enqueue((r, c));

                    do {
                        (int, int) candidate = candidates.Dequeue();
                        if (spacePartitioningGrid[
                                candidate.Item1,
                                candidate.Item2] != 0) {
                            continue;
                        }
                        roomSeedSegmentContours[roomCount].Add(candidate);
                        spacePartitioningGrid[
                            candidate.Item1,
                            candidate.Item2] = roomCount;

                        for (dr = -1; dr <= 1; dr++) {
                            for (dc = -1; dc <= 1; dc++) {

                                if (dr == 0 && dc == 0) {
                                    continue;
                                }

                                r2 = candidate.Item1 + dr;
                                c2 = candidate.Item2 + dc;
                                if (spacePartitioningGrid[r2, c2] != 0
                                        || !erosionGrid[r2, c2]
                                        || !IsContour(
                                            candidate.Item1,
                                            candidate.Item2,
                                            erosionGrid)) {
                                    continue;
                                }
                                candidates.Enqueue((r2, c2));
                            }
                        }
                    } while (candidates.Count > 0);
                }
            }

            return spacePartitioningGrid;
        }

        private static bool IsContour(
                int r,
                int c,
                bool[,] erosionGrid) {

            int dr, dc;

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    if (dr.Abs() == dc.Abs()) {
                        continue;
                    }

                    if (!erosionGrid[
                            r + dr,
                            c + dc]) {
                        return true;
                    }
                }
            }
            return false;
        }

        private static int[,] InitializeRoomDomainGrid(
                bool[,] interiorSpaceGrid,
                bool[,] erosionGrid,
                int[,] spacePartitioningGrid,
                Dictionary<int, List<(int, int)>> roomSeedSegmentContours) {

            int[,] roomDomainGrid = new int[
                erosionGrid.GetLength(0),
                erosionGrid.GetLength(1)];

            InitializeRoomsDomainsWithRoomSeedSegmentContours(
                roomDomainGrid,
                roomSeedSegmentContours);

            GrowRoomDomainsLayerwise(
                interiorSpaceGrid,
                erosionGrid,
                roomDomainGrid);
            
            CompleteRoomDomainGrid(
                erosionGrid, 
                roomDomainGrid,
                spacePartitioningGrid);

            return roomDomainGrid;
        }

        private static void InitializeRoomsDomainsWithRoomSeedSegmentContours(
                int[,] roomDomainGrid,
                Dictionary<int, List<(int, int)>> roomSeedSegmentContours) {

            foreach (int roomId in roomSeedSegmentContours.Keys) {
                foreach ((int, int) pixel in roomSeedSegmentContours[roomId]) {
                    roomDomainGrid[
                        pixel.Item1,
                        pixel.Item2] = roomId;
                }
            }
        }

        private static void GrowRoomDomainsLayerwise(
                bool[,] interiorSpaceGrid,
                bool[,] erosionGrid,
                int[,] roomDomainGrid) {

            Dictionary<int, List<(int, int)>> nextLayer;

            do {

                nextLayer = GetNextLayer(
                    interiorSpaceGrid,
                    erosionGrid,
                    roomDomainGrid);

                Grow(
                    roomDomainGrid,
                    nextLayer);
                
            } while (nextLayer.Count > 0);
        }

        private static Dictionary<int, List<(int, int)>> GetNextLayer(
                bool[,] interiorSpaceGrid,
                bool[,] erosionGrid,
                int[,] roomDomainGrid) {

            bool found;
            int dr, r, r2, dc, c, c2;
            Dictionary<int, List<(int, int)>> nextLayer = new Dictionary<int, List<(int, int)>>();

            for (r = 0; r < erosionGrid.GetLength(0); r++) {
                for (c = 0; c < erosionGrid.GetLength(1); c++) {

                    if (roomDomainGrid[r, c] != 0
                            || !(!erosionGrid[r, c] && interiorSpaceGrid[r, c])) {
                        continue;
                    }

                    found = false;
                    for (dr = -1; dr <= 1; dr++) {
                        for (dc = -1; dc <= 1; dc++) {

                            r2 = r + dr;
                            c2 = c + dc;
                            if (r2 >= 0 && c2 >= 0
                                    && r2 < erosionGrid.GetLength(0)
                                    && c2 < erosionGrid.GetLength(1)
                                    && roomDomainGrid[r2, c2] != 0) {
                                found = true;
                                nextLayer.BucketAdd(
                                    roomDomainGrid[r2, c2],
                                    (r, c));
                                break;
                            }
                        }
                        if (found) {
                            break;
                        }
                    }
                }
            }

            return nextLayer;
        }

        private static void Grow(
                int[,] roomDomainGrid,
                Dictionary<int, List<(int, int)>> nextLayer) {

            foreach (int roomId in nextLayer.Keys) {
                foreach ((int, int) pixel in nextLayer[roomId]) {
                    roomDomainGrid[
                        pixel.Item1,
                        pixel.Item2] = roomId;
                }
            }
        }

        private static void CompleteRoomDomainGrid(
                bool[,] erosionGrid,
                int[,] roomDomainGrid,
                int[,] spacePartitioningGrid) {

            int dr, r, r2, dc, c, c2, roomId;
            (int, int) candidate;
            Queue<(int, int)> candidates = new Queue<(int, int)>();

            for (r = 0; r < erosionGrid.GetLength(0); r++) {
                for (c = 0; c < erosionGrid.GetLength(1); c++) {

                    if (roomDomainGrid[r, c] != 0
                            || !erosionGrid[r, c]) {
                        continue;
                    }

                    roomId = GetAdjacentRoomId(
                        r,
                        c,
                        roomDomainGrid);
                    if (roomId == 0) {
                        continue;
                    }

                    candidates.Enqueue((r, c));
                    do {

                        candidate = candidates.Dequeue();
                        if (roomDomainGrid[
                                candidate.Item1,
                                candidate.Item2] != 0) {
                            continue;
                        }

                        roomDomainGrid[
                            candidate.Item1,
                            candidate.Item2] = roomId;
                        spacePartitioningGrid[
                            candidate.Item1,
                            candidate.Item2] = roomId;

                        for (dr = -1; dr <= 1; dr++) {
                            for (dc = -1; dc <= 1; dc++) {

                                r2 = r + dr;
                                c2 = c + dc;
                                if (roomDomainGrid[r2, c2] == 0
                                        && erosionGrid[r, c]) {
                                    candidates.Enqueue((r2, c2));
                                }
                            }
                        }
                    } while (candidates.Count > 0);
                }
            }
        }

        private static int GetAdjacentRoomId(
                int r,
                int c,
                int[,] roomDomainGrid) {

            int dr, dc;
            int roomId = 0;

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    roomId = roomDomainGrid[
                        r + dr,
                        c + dc];
                    if (roomId > 0) {
                        return roomId;
                    }
                }
            }

            return roomId;
        }

        private static Dictionary<(int, int), List<List<(int, int)>>> DetectRoomDomainContactZoneSegments(
                int[,] roomDomainGrid) {

            Dictionary<(int, int), HashSet<(int, int)>> roomDomainContactZones 
                = DetectRoomDomainContactZones(
                    roomDomainGrid);

            Dictionary<(int, int), List<List<(int, int)>>> roomDomainContactZoneSegments 
                = SegmentRoomDomainContactZones(
                    roomDomainContactZones);

            OrderRoomDomainContactZoneSegmentsDescendingBySize(
                roomDomainContactZoneSegments);

            return roomDomainContactZoneSegments;
        }

        private static Dictionary<(int, int), HashSet<(int, int)>> DetectRoomDomainContactZones(
                int[,] roomDomainGrid) {

            int dr, r, r2, dc, c, c2, roomId1, roomId2;
            Dictionary<(int, int), HashSet<(int, int)>> roomDomainContactZones
                = new Dictionary<(int, int), HashSet<(int, int)>>();

            for (r = 0; r < roomDomainGrid.GetLength(0); r++) {
                for (c = 0; c < roomDomainGrid.GetLength(1); c++) {

                    roomId1 = roomDomainGrid[r, c];
                    if (roomId1 == 0) {
                        continue;
                    }

                    for (dr = -1; dr <= 1; dr++) {
                        for (dc = -1; dc <= 1; dc++) {

                            r2 = r + dr;
                            c2 = c + dc;
                            if (r2 < 0 || c2 < 0
                                    || r2 >= roomDomainGrid.GetLength(0)
                                    || c2 >= roomDomainGrid.GetLength(1)) {
                                continue;
                            }

                            roomId2 = roomDomainGrid[r2, c2];
                            if (roomId2 != 0
                                    && roomId1 != roomId2) {
                                roomDomainContactZones.Add(
                                    roomId1,
                                    roomId2,
                                    (r, c),
                                    (r2, c2));
                            }
                        }
                    }
                }
            }

            return roomDomainContactZones;
        }

        private static void Add(
                this Dictionary<(int, int), HashSet<(int, int)>> roomDomainContactZones,
                int roomId1,
                int roomId2,
                (int, int) pixel1,
                (int, int) pixel2) {

            (int, int) id = Util.GetUnambiguousOrder(
                roomId1, 
                roomId2);

            if (!roomDomainContactZones.ContainsKey(id)) {
                roomDomainContactZones.Add(
                    id,
                    new HashSet<(int, int)>());
            }

            roomDomainContactZones[id].Add(pixel1);
            roomDomainContactZones[id].Add(pixel2);
        }

        private static Dictionary<(int, int), List<List<(int, int)>>> SegmentRoomDomainContactZones(
                Dictionary<(int, int), HashSet<(int, int)>> roomDomainContactZones) {

            int dr, dc;
            (int, int) candidate, neighbour;
            HashSet<(int, int)> segmentedPixels;
            Queue<(int, int)> candidates = new Queue<(int, int)>();
            Dictionary<(int, int), List<List<(int, int)>>> roomDomainContactZoneSegments
                = new Dictionary<(int, int), List<List<(int, int)>>>();

            foreach ((int, int) id in roomDomainContactZones.Keys) {

                segmentedPixels = new HashSet<(int, int)>();
                List<List<(int, int)>> segments = new List<List<(int, int)>>();

                foreach ((int, int) pixel in roomDomainContactZones[id]) {

                    if (segmentedPixels.Contains(pixel)) {
                        continue;
                    }

                    List<(int, int)> segment = new List<(int, int)>();
                    candidates.Enqueue(pixel);

                    do {

                        candidate = candidates.Dequeue();
                        if (segmentedPixels.Contains(candidate)) {
                            continue;
                        }

                        segment.Add(candidate);
                        segmentedPixels.Add(candidate);

                        for (dr = -1; dr <= 1; dr++) {
                            for (dc = -1; dc <= 1; dc++) {

                                if (dr == 0 && dc == 0) {
                                    continue;
                                }

                                neighbour = (
                                    candidate.Item1 + dr,
                                    candidate.Item2 + dc);

                                if (!segmentedPixels.Contains(neighbour)
                                        && roomDomainContactZones[id].Contains(neighbour)) {

                                    candidates.Enqueue(neighbour);
                                }
                            }
                        }
                    } while (candidates.Count > 0);

                    segments.Add(segment);
                }

                roomDomainContactZoneSegments.Add(id, segments);
            }

            return roomDomainContactZoneSegments;
        }

        private static void OrderRoomDomainContactZoneSegmentsDescendingBySize(
                Dictionary<(int, int), List<List<(int, int)>>> roomDomainContactZoneSegments) {

            foreach ((int, int) id in roomDomainContactZoneSegments.Keys) {
                roomDomainContactZoneSegments[id] = roomDomainContactZoneSegments[id]
                    .OrderByDescending(segment => segment.Count)
                    .ToList();
            }
        }

        private static (int, int) GetDirectionOfTransit(
                List<(int, int)> roomSeedSegment1,
                List<(int, int)> roomSeedSegment2,
                List<(int, int)> roomDomainContactZoneSegment,
                out (int, int) roomSeedSegment1Pixel,
                out (int, int) roomSeedSegment2Pixel) {

            double angle;
            (int, int) roomDomainContactZoneSegmentCentroid = roomDomainContactZoneSegment.GetCentroid();

            roomSeedSegment1Pixel = roomSeedSegment1
                .GetMinDistancePixel(roomDomainContactZoneSegmentCentroid);
            roomSeedSegment2Pixel = roomSeedSegment2
                .GetMinDistancePixel(roomDomainContactZoneSegmentCentroid);
            angle = GetAngle(roomSeedSegment1Pixel, roomSeedSegment2Pixel);

            if (angle >= RAD_ANGLE_337_5_DEGREE || angle <= RAD_ANGLE_22_5_DEGREE) {
                return (0, 1);
            }
            if (angle > RAD_ANGLE_22_5_DEGREE && angle < RAD_ANGLE_67_5_DEGREE) {
                return (1, 1);
            }
            if (angle >= RAD_ANGLE_67_5_DEGREE && angle <= RAD_ANGLE_112_5_DEGREE) {
                return (1, 0);
            }
            if (angle > RAD_ANGLE_112_5_DEGREE && angle < RAD_ANGLE_157_5_DEGREE) {
                return (1, -1);
            }
            if (angle >= RAD_ANGLE_157_5_DEGREE && angle <= RAD_ANGLE_202_5_DEGREE) {
                return (0, -1);
            }
            if (angle > RAD_ANGLE_202_5_DEGREE && angle < RAD_ANGLE_247_5_DEGREE) {
                return (-1, -1);
            }
            if (angle >= RAD_ANGLE_247_5_DEGREE && angle <= RAD_ANGLE_292_5_DEGREE) {
                return (1, 1);
            }
            return (-1, 1);
        }

        private static (int, int) GetCentroid(
                this List<(int, int)> segment) {

            (int, int) sum = (0, 0);

            foreach ((int, int) pixel in segment) {
                sum.Item1 += pixel.Item1;
                sum.Item2 += pixel.Item2;
            }

            return (
                (int)((double)sum.Item1 / segment.Count).Round(),
                (int)((double)sum.Item2 / segment.Count).Round());
        }

        private static (int, int) GetMinDistancePixel(
                this List<(int, int)> segment,
                (int, int) pixel) {

            double distance;
            double minDistance = double.MaxValue;
            (int, int) minDistancePixel = (-1, -1);

            foreach ((int, int) roomPixel in segment) {

                distance = PixelUtils.GetDistance(
                    roomPixel, 
                    pixel);

                if (distance < minDistance) {
                    minDistance = distance;
                    minDistancePixel = roomPixel;
                }
            }

            return minDistancePixel;
        }

        private static double GetAngle(
                (int, int) pixel1,
                (int, int) pixel2) {

            double angle = System.Math.Atan2(
                pixel2.Item1 - pixel1.Item1,
                pixel2.Item2 - pixel1.Item2);

            if (angle < 0.0) {
                angle += RAD_ANGLE_360_DEGREE;
            }

            return angle;
        }

        private static void DetectTransitionSpaces(
                double resolution,
                int transitionZoneId,
                (int, int) roomConnectionId,
                bool[,] transitionSpaceGrid,
                bool[,] bottleNeckGrid,
                int[,] roomDomainGrid,
                int[,] roomDomainContactZoneSegmentGrid,
                List<(int, int)> roomSeedSegment1,
                List<(int, int)> roomSeedSegment2,
                List<(int, int)> roomDomainContactZoneSegment) {

            (int, int) directionOfTransit;
            (int, int) roomSeedSegment1Pixel;
            (int, int) roomSeedSegment2Pixel;
            bool[,] transitionZoneGrid;
            bool[,] transitionZoneBorderGrid;
            List<List<HashSet<(int, int)>>> transitionZoneSections;

            directionOfTransit = GetDirectionOfTransit(
                roomSeedSegment1,
                roomSeedSegment2,
                roomDomainContactZoneSegment,
                out roomSeedSegment1Pixel,
                out roomSeedSegment2Pixel);

            transitionZoneSections = InitializeTransitionZoneSections(
                transitionZoneId,
                roomSeedSegment1Pixel,
                roomSeedSegment2Pixel,
                directionOfTransit,
                roomDomainGrid,
                roomDomainContactZoneSegmentGrid);

            InitializeTransitionZoneGrids(
                transitionSpaceGrid,
                transitionZoneSections,
                out transitionZoneGrid,
                out transitionZoneBorderGrid);

            RemoveBottleNecks(
                resolution,
                directionOfTransit,
                transitionZoneGrid,
                bottleNeckGrid,
                roomDomainGrid,
                transitionZoneSections);

            if (!IsTransitionZoneDisconnected(
                    roomConnectionId,
                    directionOfTransit,
                    transitionSpaceGrid,
                    transitionZoneGrid,
                    bottleNeckGrid,
                    roomDomainGrid,
                    transitionZoneSections)) {

                LocalizeTransitionSpacesWithinTransitionZone(
                    directionOfTransit,
                    transitionSpaceGrid,
                    transitionZoneGrid,
                    transitionZoneBorderGrid,
                    transitionZoneSections);
            }
        }

        private static int[,] InitializeRoomDomainContactZoneSegmentGrid(
                int[,] roomDomainGrid,
                Dictionary<(int, int), List<List<(int, int)>>> roomDomainContactZoneSegments) {

            int roomDomainContactZoneSegmentId = 1;
            int[,] roomDomainContactZoneSegmentGrid = new int[
                roomDomainGrid.GetLength(0),
                roomDomainGrid.GetLength(1)];

            foreach ((int, int) id in roomDomainContactZoneSegments.Keys) {

                foreach (List<(int, int)> contactSurfaceSegment in roomDomainContactZoneSegments[id]) {

                    foreach ((int, int) pixel in contactSurfaceSegment) {
                        roomDomainContactZoneSegmentGrid[
                            pixel.Item1,
                            pixel.Item2] = roomDomainContactZoneSegmentId;
                    }

                    roomDomainContactZoneSegmentId++;
                }
            }

            return roomDomainContactZoneSegmentGrid;
        }

        private static List<List<HashSet<(int, int)>>> InitializeTransitionZoneSections(
                int transitionZoneId,
                (int, int) startPixel,
                (int, int) stopPixel,
                (int, int) directionOfTransit,
                int[,] roomDomainGrid,
                int[,] roomDomainContactZoneSegmentGrid) {

            (int, int)[] perpendicularDirections = directionOfTransit.GetPerpendicularDirections();

            List<List<HashSet<(int, int)>>> transitionZoneSections = new List<List<HashSet<(int, int)>>> {
                new List<HashSet<(int, int)>> {
                    GrowTransitionZoneSection(
                        transitionZoneId,
                        startPixel,
                        perpendicularDirections,
                        roomDomainGrid,
                        roomDomainContactZoneSegmentGrid)
                }
            };

            while (GetNextTransitionZoneSection(
                    transitionZoneId,
                    stopPixel,
                    directionOfTransit,
                    perpendicularDirections,
                    roomDomainGrid,
                    roomDomainContactZoneSegmentGrid,
                    transitionZoneSections.Last(),
                    out List<HashSet<(int, int)>> nextSection)) {

                transitionZoneSections.Add(nextSection);
            }

            RemoveDeadEndsInTransitionZone(
                directionOfTransit, 
                stopPixel,
                transitionZoneSections);

            return transitionZoneSections;
        }

        private static HashSet<(int, int)> GrowTransitionZoneSection(
                int transitionZoneId,
                (int, int) startPixel,
                (int, int)[] perpendicularDirections,
                int[,] roomDomainGrid,
                int[,] roomDomainContactZoneSegmentGrid) {

            int j, r, c, d;
            int roomDomainContactZoneSegmentId;
            HashSet<(int, int)> transitionZoneSectionSegment = new HashSet<(int, int)> {
                startPixel
            };

            for (j = 0; j <= 1; j++) {

                d = 1;

                while (true) {

                    r = startPixel.Item1 + d * perpendicularDirections[j].Item1;
                    c = startPixel.Item2 + d * perpendicularDirections[j].Item2;
                    if (r < 0 || c < 0
                            || r >= roomDomainGrid.GetLength(0)
                            || c >= roomDomainGrid.GetLength(1)
                            || roomDomainGrid[r, c] == 0) {
                        break;
                    }

                    roomDomainContactZoneSegmentId = roomDomainContactZoneSegmentGrid[r, c];
                    if (roomDomainContactZoneSegmentId != 0 
                            && roomDomainContactZoneSegmentId != transitionZoneId) {
                        break;
                    }

                    transitionZoneSectionSegment.Add((r, c));
                    d++;
                }
            }

            return transitionZoneSectionSegment;
        }

        private static bool GetNextTransitionZoneSection(
                int transitionZoneId,
                (int, int) stopPixel,
                (int, int) directionOfTransit,
                (int, int)[] perpendicularDirections,
                int[,] roomDomainGrid,
                int[,] roomDomainContactZoneSegmentGrid,
                List<HashSet<(int, int)>> sectionBefore,
                out List<HashSet<(int, int)>> nextSection) {

            bool stop = false;
            HashSet<(int, int)> nextSectionPixels = new HashSet<(int, int)>();

            foreach (HashSet<(int, int)> sectionSegment in sectionBefore) {

                AddNextTransitionZoneSectionPixels(
                    false,
                    transitionZoneId,
                    directionOfTransit,
                    perpendicularDirections,
                    roomDomainGrid,
                    roomDomainContactZoneSegmentGrid,
                    sectionSegment,
                    nextSectionPixels);

                AddNextTransitionZoneSectionPixels(
                    true,
                    transitionZoneId,
                    directionOfTransit,
                    perpendicularDirections,
                    roomDomainGrid,
                    roomDomainContactZoneSegmentGrid,
                    sectionSegment,
                    nextSectionPixels);
            }

            if (nextSectionPixels.Contains(stopPixel)) {
                stop = true;
            }

            RemoveOvertakingPixels(
                directionOfTransit,
                perpendicularDirections,
                nextSectionPixels);

            nextSection = SegmentTransitionZoneSection(
                nextSectionPixels);

            return !stop 
                && nextSection.Count > 0;
        }

        private static void AddNextTransitionZoneSectionPixels(
                bool advanceDiagonally,
                int transitionZoneId,
                (int, int) directionOfTransit,
                (int, int)[] perpendicularDirections,
                int[,] roomDomainGrid,
                int[,] roomDomainContactZoneSegmentGrid,
                HashSet<(int, int)> transitionZoneSectionSegmentBefore,
                HashSet<(int, int)> nextTransitionZoneSectionPixels) {

            int r, c;
            int roomDomainContactZoneSegmentId;
            (int, int) candidatePixel;
            IEnumerable<(int, int)> advanceDirections = advanceDiagonally ?
                directionOfTransit.SplitDirectionDiagonal() :
                directionOfTransit.SplitDirectionStraight();

            foreach ((int, int) pixel in transitionZoneSectionSegmentBefore) {
                foreach ((int, int) advanceDirection in directionOfTransit.SplitDirectionStraight()) {

                    r = pixel.Item1 + advanceDirection.Item1;
                    c = pixel.Item2 + advanceDirection.Item2;
                    if (r < 0 || c < 0
                            || r >= roomDomainContactZoneSegmentGrid.GetLength(0)
                            || c >= roomDomainContactZoneSegmentGrid.GetLength(1)) {
                        continue;
                    }

                    roomDomainContactZoneSegmentId = roomDomainContactZoneSegmentGrid[r, c];

                    if (roomDomainGrid[r, c] != 0
                            && (roomDomainContactZoneSegmentId == 0 
                                || roomDomainContactZoneSegmentId == transitionZoneId)) {

                        candidatePixel = (r, c);

                        if (!nextTransitionZoneSectionPixels.Contains(candidatePixel)
                                && (!advanceDiagonally
                                    || candidatePixel
                                        .GetPixelsStraightBefore(directionOfTransit)
                                        .All(pixelAbove => roomDomainGrid[
                                            pixelAbove.Item1,
                                            pixelAbove.Item2] == 0))) {

                            nextTransitionZoneSectionPixels.AddRange(
                                GrowTransitionZoneSection(
                                    transitionZoneId,
                                    candidatePixel,
                                    perpendicularDirections,
                                    roomDomainGrid,
                                    roomDomainContactZoneSegmentGrid));
                        }
                    }
                }
            }
        }

        private static IEnumerable<(int, int)> SplitDirectionDiagonal(
                this (int, int) direction) {

            if (direction.IsDirectionDiagonal()) {
                yield return direction;
            }
            else {
                yield return (
                    direction.Item1 == 0 ? 1 : direction.Item1,
                    direction.Item2 == 0 ? 1 : direction.Item2
                );
                yield return (
                    direction.Item1 == 0 ? -1 : direction.Item1,
                    direction.Item2 == 0 ? -1 : direction.Item2
                );
            }
        }

        private static IEnumerable<(int, int)> SplitDirectionStraight(
                this (int, int) direction) {

            if (direction.IsDirectionDiagonal()) {
                yield return (direction.Item1, 0);
                yield return (0, direction.Item2);
            }
            else {
                yield return direction;
            }
        }

        private static IEnumerable<(int, int)> GetPixelsStraightBefore(
                this (int, int) pixel,
                (int, int) direction) {

            if (direction.IsDirectionDiagonal()) {
                yield return (
                    pixel.Item1 - direction.Item1,
                    pixel.Item2
                );
                yield return (
                    pixel.Item1,
                    pixel.Item2 - direction.Item2
                );
            }
            else {
                yield return (
                    pixel.Item1 - direction.Item1,
                    pixel.Item2 - direction.Item2
                );
            }
        }

        private static void RemoveOvertakingPixels(
                (int, int) direction,
                (int, int)[] perpendicularDirections,
                HashSet<(int, int)> nextSectionPixels) {

            (int, int) overtakingPixel;
            List<(int, int)> pixelsToRemove;

            do {
                if (!FindAnOvertakingPixel(
                        nextSectionPixels,
                        direction,
                        out overtakingPixel)) {
                    break;
                }

                pixelsToRemove = GrowOvertakingPixels(
                    overtakingPixel,
                    perpendicularDirections,
                    nextSectionPixels);

                nextSectionPixels.Remove(pixelsToRemove);
            } while (true);
        }

        private static bool FindAnOvertakingPixel(
                HashSet<(int, int)> nextSectionPixels,
                (int, int) direction,
                out (int, int) overtakingPixel) {

            overtakingPixel = (0, 0);

            foreach ((int, int) pixel in nextSectionPixels) {
                foreach ((int, int) pixelBefore in pixel.GetPixelsStraightBefore(direction)) {
                    if (nextSectionPixels.Contains(pixelBefore)) {
                        overtakingPixel = pixel;
                        return true;
                    }
                }
            }
            return false;
        }

        private static List<(int, int)> GrowOvertakingPixels(
                (int, int) seed,
                (int, int)[] perpendicularDirections,
                HashSet<(int, int)> nextSectionPixels) {

            int j, d;
            (int, int) candidate;
            List<(int, int)> overtakingPixels = new List<(int, int)> {
                seed
            };

            for (j = 0; j <= 1; j++) {

                d = 1;
                while (true) {

                    candidate = (
                        seed.Item1 + d * perpendicularDirections[j].Item1,
                        seed.Item2 + d * perpendicularDirections[j].Item2
                    );

                    if (nextSectionPixels.Contains(candidate)) {
                        d++;
                        overtakingPixels.Add(candidate);
                        continue;
                    }

                    break;
                }
            }

            return overtakingPixels;
        }

        private static List<HashSet<(int, int)>> SegmentTransitionZoneSection(
                HashSet<(int, int)> transitionZoneSectionPixels) {

            int dr, dc;
            (int, int) candidate;
            (int, int) neighbour;
            Queue<(int, int)> candidates = new Queue<(int, int)>();
            HashSet<(int, int)> segmentedPixels = new HashSet<(int, int)>();
            List<HashSet<(int, int)>> transitionZoneSectionSegments = new List<HashSet<(int, int)>>();

            foreach ((int, int) pixel in transitionZoneSectionPixels) {

                if (segmentedPixels.Contains(pixel)) {
                    continue;
                }

                HashSet<(int, int)> segment = new HashSet<(int, int)>();
                candidates.Enqueue(pixel);

                do {

                    candidate = candidates.Dequeue();
                    if (segmentedPixels.Contains(candidate)) {
                        continue;
                    }

                    segment.Add(candidate);
                    segmentedPixels.Add(candidate);

                    for (dr = -1; dr <= 1; dr++) {
                        for (dc = -1; dc <= 1; dc++) {

                            neighbour = (
                                candidate.Item1 + dr,
                                candidate.Item2 + dc);

                            if (transitionZoneSectionPixels.Contains(neighbour)
                                    && !segmentedPixels.Contains(neighbour)) {
                                candidates.Enqueue(neighbour);
                            }
                        }
                    }
                } while (candidates.Count > 0);

                transitionZoneSectionSegments.Add(segment);
            }

            return transitionZoneSectionSegments;
        }

        private static void RemoveDeadEndsInTransitionZone(
                (int, int) direction,
                (int, int) stopPixel,
                List<List<HashSet<(int, int)>>> transitionZoneSections) {

            int j, k;
            List<int> transitionZoneSectionIdsToRemove = new List<int>();
            List<int> transitionZoneSectionSegmentIdsToRemove = new List<int>();

            for (k = 0; k < transitionZoneSections[transitionZoneSections.Count - 1].Count; k++) {
                if (!transitionZoneSections[transitionZoneSections.Count - 1][k]
                        .Any(pixel => pixel.Neighbours(stopPixel))) {
                    transitionZoneSectionSegmentIdsToRemove.Add(k);
                }
            }

            transitionZoneSections[transitionZoneSections.Count - 1].RemoveAt(transitionZoneSectionSegmentIdsToRemove);
            if (transitionZoneSections[transitionZoneSections.Count - 1].Count == 0) {
                return;
            }

            for (j = transitionZoneSections.Count - 2; j >= 0; j--) {

                transitionZoneSectionSegmentIdsToRemove = new List<int>();

                for (k = 0; k < transitionZoneSections[j].Count; k++) {
                    if (!transitionZoneSections[j][k]
                            .Any(pixel => pixel.GetPixelsAfter(direction)
                                .Any(pixelAfter => transitionZoneSections[j + 1]
                                    .Any(sectionSegmentAfter => sectionSegmentAfter.Contains(pixelAfter))))) {
                        transitionZoneSectionSegmentIdsToRemove.Add(k);
                    }
                }

                transitionZoneSections[j].RemoveAt(transitionZoneSectionSegmentIdsToRemove);
                if (transitionZoneSections[j].Count == 0) {
                    transitionZoneSectionIdsToRemove.Add(j);
                }
            }

            transitionZoneSections.RemoveAt(transitionZoneSectionIdsToRemove);
        }

        private static bool Neighbours(
                this (int, int) pixel,
                (int, int) otherPixel) {

            int dr, dc;

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {
                    if (otherPixel.Item1 == pixel.Item1 + dr
                            && otherPixel.Item2 == pixel.Item2 + dc) {
                        return true;
                    }
                }
            }

            return false;
        }

        private static IEnumerable<(int, int)> GetPixelsAfter(
                this (int, int) pixel,
                (int, int) direction) {

            return pixel.GetPixelsStraightAfter(direction)
                .Concat(pixel.GetPixelsDiagonallyAfter(direction));
        }

        private static IEnumerable<(int, int)> GetPixelsStraightAfter(
                this (int, int) pixel,
                (int, int) direction) {

            if (direction.IsDirectionDiagonal()) {
                yield return (
                    pixel.Item1 + direction.Item1,
                    pixel.Item2
                );
                yield return (
                    pixel.Item1,
                    pixel.Item2 + direction.Item2
                );
            }
            else {
                yield return (
                    pixel.Item1 + direction.Item1,
                    pixel.Item2 + direction.Item2
                );
            }
        }

        private static IEnumerable<(int, int)> GetPixelsDiagonallyAfter(
                this (int, int) pixel,
                (int, int) direction) {

            if (direction.IsDirectionDiagonal()) {
                yield return (
                    pixel.Item1 + direction.Item1,
                    pixel.Item2 + direction.Item2
                );
            }
            else {
                yield return (
                    pixel.Item1 + (direction.Item1 == 0 ? 1 : direction.Item1),
                    pixel.Item2 + (direction.Item2 == 0 ? 1 : direction.Item2)
                );
                yield return (
                    pixel.Item1 + (direction.Item1 == 0 ? -1 : direction.Item1),
                    pixel.Item2 + (direction.Item2 == 0 ? -1 : direction.Item2)
                );
            }
        }

        private static void InitializeTransitionZoneGrids(
                bool[,] transitionSpaceGrid,
                List<List<HashSet<(int, int)>>> transitionZoneSections,
                out bool[,] transitionZoneGrid,
                out bool[,] transitionZoneBorderGrid) {

            transitionZoneGrid = new bool[
                transitionSpaceGrid.GetLength(0),
                transitionSpaceGrid.GetLength(1)];
            transitionZoneBorderGrid = new bool[
                transitionSpaceGrid.GetLength(0),
                transitionSpaceGrid.GetLength(1)];

            foreach (List<HashSet<(int, int)>> section in transitionZoneSections) {
                foreach (HashSet<(int, int)> segment in section) {
                    foreach ((int, int) pixel in segment) {
                        transitionZoneGrid[
                            pixel.Item1,
                            pixel.Item2] = true;
                    }
                }
            }

            foreach (HashSet<(int, int)> segment in transitionZoneSections.First()) {
                foreach ((int, int) pixel in segment) {
                    transitionZoneBorderGrid[
                        pixel.Item1,
                        pixel.Item2] = true;
                }
            }

            foreach (HashSet<(int, int)> segment in transitionZoneSections.Last()) {
                foreach ((int, int) pixel in segment) {
                    transitionZoneBorderGrid[
                        pixel.Item1,
                        pixel.Item2] = true;
                }
            }
        }

        private static void RemoveBottleNecks(
                double resolution,
                (int, int) directionOfTransit,
                bool[,] transitionZoneGrid,
                bool[,] bottleNeckGrid,
                int[,] roomDomainGrid,
                List<List<HashSet<(int, int)>>> transitionZoneSections) {

            List<(int, int)> bottleNeckPixels = RemoveBottleNeckSections(
                resolution,
                directionOfTransit,
                transitionZoneGrid,
                bottleNeckGrid,
                roomDomainGrid,
                transitionZoneSections);

            if (bottleNeckPixels.Count > 0
                    && directionOfTransit.IsDirectionDiagonal()) {

                DilateDiagonalBottleNeckSections(
                    transitionZoneGrid,
                    bottleNeckGrid,
                    bottleNeckPixels);
            }
        }

        private static List<(int, int)> RemoveBottleNeckSections(
                double resolution,
                (int, int) directionOfTransit,
                bool[,] transitionZoneGrid,
                bool[,] bottleNeckGrid,
                int[,] roomDomainGrid,
                List<List<HashSet<(int, int)>>> transitionZoneSections) {

            int j, k;
            int bottleNeckThreshold = directionOfTransit.IsDirectionDiagonal() ?
                Parameters.TRANSITION_SPACE_MIN_WIDTH
                    .GetDistanceInVoxels(
                        resolution.GetVoxelSizeDiagonal()) :
                Parameters.TRANSITION_SPACE_MIN_WIDTH
                    .GetDistanceInVoxels(resolution);
            List<int> transitionZoneSectionSegmentIdsToRemove;
            List<int> transitionZoneSectionIdsToRemove = new List<int>();
            List<(int, int)> bottleNeckPixels = new List<(int, int)>();

            for (j = 0; j < transitionZoneSections.Count; j++) {

                transitionZoneSectionSegmentIdsToRemove = new List<int>();

                for (k = 0; k < transitionZoneSections[j].Count; k++) {

                    if (transitionZoneSections[j][k].Count < bottleNeckThreshold) {

                        transitionZoneSectionSegmentIdsToRemove.Add(k);

                        foreach ((int, int) pixel in transitionZoneSections[j][k]) {

                            roomDomainGrid[
                                pixel.Item1,
                                pixel.Item2] = 0;
                            transitionZoneGrid[
                                pixel.Item1,
                                pixel.Item2] = false;
                            bottleNeckGrid[
                                pixel.Item1,
                                pixel.Item2] = true;
                            bottleNeckPixels.Add(pixel);
                        }
                    }
                }

                transitionZoneSections[j].RemoveAt(transitionZoneSectionSegmentIdsToRemove);

                if (transitionZoneSections[j].Count == 0) {
                    transitionZoneSectionIdsToRemove.Add(j);
                }
            }

            transitionZoneSections.RemoveAt(transitionZoneSectionIdsToRemove);

            return bottleNeckPixels;
        }

        private static void DilateDiagonalBottleNeckSections(
                bool[,] transitionZoneGrid,
                bool[,] bottleNeckGrid,
                List<(int, int)> bottleNeckPixels) {

            int dr, r, dc, c;

            foreach ((int, int) pixel in bottleNeckPixels) {

                if (!IsSingleBottleNeckSection(
                        pixel,
                        bottleNeckGrid)) {
                    continue;
                }

                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (dr.Abs() == dc.Abs()) {
                            continue;
                        }

                        r = pixel.Item1 + dr;
                        c = pixel.Item2 + dc;
                        if (r >= 0 && c >= 0
                                && r < bottleNeckGrid.GetLength(0)
                                && c < bottleNeckGrid.GetLength(1)
                                && transitionZoneGrid[r, c]) {
                            transitionZoneGrid[r, c] = false;
                            bottleNeckGrid[r, c] = true;
                        }
                    }
                }
            }
        }

        private static bool IsSingleBottleNeckSection(
                (int, int) pixel,
                bool[,] bottleNeckGrid) {

            int dr, r, dc, c;

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    if (dr.Abs() == dc.Abs()) {
                        continue;
                    }

                    r = pixel.Item1 + dr;
                    c = pixel.Item2 + dc;
                    if (r >= 0 && c >= 0
                            && r < bottleNeckGrid.GetLength(0)
                            && c < bottleNeckGrid.GetLength(1)
                            && bottleNeckGrid[r, c]) {
                        return false;
                    }
                }
            }

            return true;
        }

        private static bool IsTransitionZoneDisconnected(
                (int, int) roomConnectionId,
                (int, int) directionOfTransit,
                bool[,] transitionSpaceGrid,
                bool[,] transitionZoneGrid,
                bool[,] bottleNeckGrid,
                int[,] roomDomainGrid,
                List<List<HashSet<(int, int)>>> transitionZoneSections) {

            if (!IsTransitionZoneConnectedToRoom(
                        roomConnectionId.Item1,
                        transitionSpaceGrid,
                        bottleNeckGrid,
                        roomDomainGrid,
                        transitionZoneSections)
                    || !IsTransitionZoneConnectedToRoom(
                        roomConnectionId.Item2,
                        transitionSpaceGrid,
                        bottleNeckGrid,
                        roomDomainGrid,
                        transitionZoneSections)) {
                return true;
            }

            return !AreTransitionZoneSectionsConnectedToSectionsBeforeAndAfter(
                directionOfTransit,
                transitionZoneGrid,
                transitionZoneSections);
        }

        private static bool IsTransitionZoneConnectedToRoom(
                int roomId,
                bool[,] transitionSpaceGrid,
                bool[,] bottleNeckGrid,
                int[,] roomDomainGrid,
                List<List<HashSet<(int, int)>>> transitionZoneSections) {

            return transitionZoneSections.Any(
                transitionZoneSection => transitionZoneSection.Any(
                    transitionZoneSectionSegment => transitionZoneSectionSegment.Any(
                        pixel => roomDomainGrid[pixel.Item1, pixel.Item2] == roomId
                            && !transitionSpaceGrid[pixel.Item1, pixel.Item2]
                            && !bottleNeckGrid[pixel.Item1, pixel.Item2])));
        }

        private static bool AreTransitionZoneSectionsConnectedToSectionsBeforeAndAfter(
                (int, int) directionOfTransit,
                bool[,] transitionZoneGrid,
                List<List<HashSet<(int, int)>>> transitionZoneSections) {

            for (int j = transitionZoneSections.Count - 2; j >= 1; j--) {
                if (!IsTransitionZoneSectionConnectedToSectionsBeforeAndAfter(
                        directionOfTransit,
                        transitionZoneGrid,
                        transitionZoneSections[j])) {
                    return false;
                }
            }

            return true;
        }

        private static bool IsTransitionZoneSectionConnectedToSectionsBeforeAndAfter(
                (int, int) directionOfTransit,
                bool[,] transitionZoneGrid,
                List<HashSet<(int, int)>> transitionZoneSection) {

            return transitionZoneSection
                .Any(transitionZoneSectionSegment => transitionZoneSectionSegment
                    .Any(pixel => pixel.GetPixelsStraightAfter(directionOfTransit)
                            .Any(pixelAfter => pixelAfter.Item1 >= 0
                                && pixelAfter.Item2 >= 0
                                && pixelAfter.Item1 < transitionZoneGrid.GetLength(0)
                                && pixelAfter.Item2 < transitionZoneGrid.GetLength(1)
                                && transitionZoneGrid[
                                    pixelAfter.Item1,
                                    pixelAfter.Item2])
                        && pixel.GetPixelsStraightBefore(directionOfTransit)
                            .Any(pixelBefore => pixelBefore.Item1 >= 0
                                && pixelBefore.Item2 >= 0
                                && pixelBefore.Item1 < transitionZoneGrid.GetLength(0)
                                && pixelBefore.Item2 < transitionZoneGrid.GetLength(1)
                                && transitionZoneGrid[
                                    pixelBefore.Item1,
                                    pixelBefore.Item2])));
        }

        private static void LocalizeTransitionSpacesWithinTransitionZone(
                (int, int) directionOfTransit,
                bool[,] transitionSpaceGrid,
                bool[,] transitionZoneGrid,
                bool[,] transitionZoneBorderGrid,
                List<List<HashSet<(int, int)>>> transitionZoneSections) {

            int stopIndex, startIndex;

            List<(int, int)> transitionSpacePixels = DetectTransitionSpacePixels(
                directionOfTransit,
                transitionSpaceGrid,
                transitionZoneGrid,
                transitionZoneSections);

            GetDisconnectedTransitionZoneSectionsInbetweenTransitionSpaces(
                directionOfTransit,
                transitionZoneGrid,
                transitionZoneSections,
                out startIndex,
                out stopIndex);

            ConvertTransitionZoneSectionsToTransitionSpace(
                startIndex,
                stopIndex,
                transitionSpaceGrid,
                transitionZoneGrid,
                transitionSpacePixels,
                transitionZoneSections);

            RemoveEmptyTransitionZoneSectionsAndSegments(
                transitionZoneSections);

            RemovePixelsOnLateralSidesOfTransitionSpaces(
                directionOfTransit,
                transitionSpaceGrid,
                transitionZoneGrid,
                transitionZoneBorderGrid,
                transitionSpacePixels);

            if (directionOfTransit.IsDirectionDiagonal()) {
                DilateDiagonalTransitionSpacesOfOnePixelWidth(
                    transitionSpaceGrid,
                    transitionZoneGrid,
                    transitionSpacePixels);
            }
        }

        private static List<(int, int)> DetectTransitionSpacePixels(
                (int, int) directionOfTransit,
                bool[,] transitionSpaceGrid,
                bool[,] transitionZoneGrid,
                List<List<HashSet<(int, int)>>> transitionZoneSections) {

            int minWidth;
            Dictionary<int, List<(int, int)>> transitionZoneSectionSegmentWidths;
            List<(int, int)> transitionSpacePixels = new List<(int, int)>();

            do {

                if (transitionZoneSections.Count == 0) {
                    break;
                }

                transitionZoneSectionSegmentWidths = GetTransitionZoneSectionSegmentWidths(
                    transitionZoneSections);

                if (transitionZoneSectionSegmentWidths.Keys.All(width => width == 0)) {
                    break;
                }

                minWidth = transitionZoneSectionSegmentWidths.Keys.Where(width => width > 0).Min();

                ConvertMinWidthTransitionZoneSectionSegmentsToTransitionSpace(
                    minWidth,
                    transitionSpaceGrid,
                    transitionZoneGrid,
                    transitionSpacePixels,
                    transitionZoneSections,
                    transitionZoneSectionSegmentWidths);

                if (!AreTransitionZoneSectionsConnectedToSectionsBeforeAndAfter(
                        directionOfTransit,
                        transitionZoneGrid,
                        transitionZoneSections)) {
                    break;
                }
            } while (true);

            return transitionSpacePixels;
        }

        private static Dictionary<int, List<(int, int)>> GetTransitionZoneSectionSegmentWidths(
                List<List<HashSet<(int, int)>>> transitionZoneSections) {

            int j, k;
            Dictionary<int, List<(int, int)>> transitionZoneSectionSegmentWidths 
                = new Dictionary<int, List<(int, int)>>();

            for (j = 0; j < transitionZoneSections.Count; j++) {
                for (k = 0; k < transitionZoneSections[j].Count; k++) {
                    transitionZoneSectionSegmentWidths.BucketAdd(
                        transitionZoneSections[j][k].Count,
                        (j, k));
                }
            }

            return transitionZoneSectionSegmentWidths;
        }

        private static void ConvertMinWidthTransitionZoneSectionSegmentsToTransitionSpace(
                int minWidth,
                bool[,] transitionSpaceGrid,
                bool[,] transitionZoneGrid,
                List<(int, int)> transitionSpacePixels,
                List<List<HashSet<(int, int)>>> transitionZoneSections,
                Dictionary<int, List<(int, int)>> transitionZoneSectionSegmentWidths) {

            foreach ((int, int) segmentId in transitionZoneSectionSegmentWidths[minWidth]) {

                foreach ((int, int) pixel in transitionZoneSections[segmentId.Item1][segmentId.Item2]) {

                    transitionZoneGrid[
                        pixel.Item1,
                        pixel.Item2] = false;
                    transitionSpaceGrid[
                        pixel.Item1,
                        pixel.Item2] = true;
                    transitionSpacePixels.Add(pixel);
                }

                transitionZoneSections[segmentId.Item1][segmentId.Item2].Clear();
            }
        }

        private static void GetDisconnectedTransitionZoneSectionsInbetweenTransitionSpaces(
            (int, int) directionOfTransit,
                bool[,] transitionZoneGrid,
                List<List<HashSet<(int, int)>>> transitionZoneSections,
                out int startIndex,
                out int stopIndex) {

            for (startIndex = 1; 
                    startIndex < transitionZoneSections.Count - 1; 
                    startIndex++) {

                if (transitionZoneSections[startIndex][0].Count == 0
                        || !IsTransitionZoneSectionConnectedToSectionsBeforeAndAfter(
                            directionOfTransit,
                            transitionZoneGrid,
                            transitionZoneSections[startIndex])) {
                    break;
                }
            }

            for (stopIndex = transitionZoneSections.Count 
                        - (transitionZoneSections[transitionZoneSections.Count - 1][0].Count == 0 ? 
                            1 : 
                            2);
                    stopIndex >= 1;
                    stopIndex--) {

                if (transitionZoneSections[stopIndex][0].Count == 0
                        || !IsTransitionZoneSectionConnectedToSectionsBeforeAndAfter(
                            directionOfTransit,
                            transitionZoneGrid,
                            transitionZoneSections[stopIndex])) {
                    break;
                }
            }
        }

        private static void ConvertTransitionZoneSectionsToTransitionSpace(
                int startIndex,
                int stopIndex,
                bool[,] transitionSpaceGrid,
                bool[,] transitionZoneGrid,
                List<(int, int)> transitionSpacePixels,
                List<List<HashSet<(int, int)>>> transitionZoneSections) {

            for (int j = startIndex + 1; j < stopIndex; j++) {

                foreach (HashSet<(int, int)> segment in transitionZoneSections[j]) {

                    foreach ((int, int) pixel in segment) {

                        transitionZoneGrid[
                            pixel.Item1,
                            pixel.Item2] = false;
                        transitionSpaceGrid[
                            pixel.Item1,
                            pixel.Item2] = true;
                        transitionSpacePixels.Add(pixel);
                    }
                }
            }
        }

        private static void RemoveEmptyTransitionZoneSectionsAndSegments(
                List<List<HashSet<(int, int)>>> transitionZoneSections) {

            foreach (List<HashSet<(int, int)>> section in transitionZoneSections) {

                section.RemoveAt(
                    Enumerable
                        .Range(0, section.Count)
                        .Where(segmentId => section[segmentId].Count == 0));
            }

            transitionZoneSections.RemoveAt(
                Enumerable
                    .Range(0, transitionZoneSections.Count)
                    .Where(sectionId => transitionZoneSections[sectionId].Count == 0));
        }

        private static void RemovePixelsOnLateralSidesOfTransitionSpaces(
                (int, int) directionOfTransit,
                bool[,] transitionSpaceGrid,
                bool[,] transitionZoneGrid,
                bool[,] transitionZoneBorderGrid,
                List<(int, int)> transitionSpacePixels) {

            bool removedPixels = false;
            int orientation, r, c, d, d2;

            foreach ((int, int) pixel in transitionSpacePixels) {

                if (transitionZoneBorderGrid[
                        pixel.Item1,
                        pixel.Item2]) {
                    continue;
                }

                for (orientation = -1; orientation <= 1; orientation += 2) {

                    r = pixel.Item1 - orientation * directionOfTransit.Item1;
                    c = pixel.Item2 - orientation * directionOfTransit.Item2;
                    if (r < 0 || c < 0
                            || r >= transitionSpaceGrid.GetLength(0)
                            || c >= transitionSpaceGrid.GetLength(1)
                            || (!transitionZoneGrid[r, c]
                                && !transitionZoneBorderGrid[r, c])) {
                        continue;
                    }

                    d = GetDistanceForPixelRemovalFromLateralSidesOfTransitionSpace(
                        orientation,
                        pixel,
                        directionOfTransit,
                        transitionSpaceGrid,
                        transitionZoneGrid,
                        transitionZoneBorderGrid);

                    if (d > 0) {
                        removedPixels = true;
                        for (d2 = d - 1; d2 >= 0; d2--) {
                            transitionSpaceGrid[
                                pixel.Item1 + orientation * d2 * directionOfTransit.Item1,
                                pixel.Item2 + orientation * d2 * directionOfTransit.Item2] = false;
                        }
                    }
                }
            }

            if (removedPixels) {
                UpdateTransitionSpacePixels(
                    transitionSpaceGrid,
                    transitionSpacePixels);
            }
        }

        private static int GetDistanceForPixelRemovalFromLateralSidesOfTransitionSpace(
                int orientation,
                (int, int) pixel,
                (int, int) directionOfTransit,
                bool[,] transitionSpaceGrid,
                bool[,] transitionZoneGrid,
                bool[,] transitionZoneBorderGrid) {

            int d = 1;
            int r, r2, c, c2;

            while (true) {

                r = pixel.Item1 + orientation * d * directionOfTransit.Item1;
                c = pixel.Item2 + orientation * d * directionOfTransit.Item2;
                if (r < 0 || c < 0
                        || r >= transitionSpaceGrid.GetLength(0)
                        || c >= transitionSpaceGrid.GetLength(1)) {
                    return d;
                }

                if (!transitionSpaceGrid[r, c]
                        && !transitionZoneGrid[r, c]
                        && !transitionZoneBorderGrid[r, c]) {
                    if (directionOfTransit.IsDirectionDiagonal()) {
                        r2 = r - orientation * directionOfTransit.Item1;
                        if (r2 >= 0
                                && r2 < transitionSpaceGrid.GetLength(0)
                                && (transitionZoneGrid[r2, c]
                                    || transitionZoneBorderGrid[r2, c])) {
                            return -1;
                        }
                        c2 = c - orientation * directionOfTransit.Item2;
                        if (c2 >= 0
                                && c2 < transitionSpaceGrid.GetLength(1)
                                && (transitionZoneGrid[r, c2]
                                    || transitionZoneBorderGrid[r, c2])) {
                            return -1;
                        }
                    }
                    return d;
                }
                if (transitionZoneGrid[r, c]
                        || transitionZoneBorderGrid[r, c]) {
                    return -1;
                }
                d++;
            }
        }

        private static void UpdateTransitionSpacePixels(
                bool[,] transitionSpaceGrid,
                List<(int, int)> transitionSpacePixels) {

            List<(int, int)> pixels = new List<(int, int)>();

            foreach ((int, int) pixel in transitionSpacePixels) {
                if (transitionSpaceGrid[
                        pixel.Item1,
                        pixel.Item2]) {
                    pixels.Add(pixel);
                }
            }
            transitionSpacePixels.Clear();
            transitionSpacePixels.AddRange(pixels);
        }

        private static void DilateDiagonalTransitionSpacesOfOnePixelWidth(
                bool[,] transitionSpaceGrid,
                bool[,] transitionZoneGrid,
                List<(int, int)> transitionSpacePixels) {

            int dr, r, dc, c;
            List<(int, int)> newPixels = new List<(int, int)>();

            foreach ((int, int) pixel in transitionSpacePixels) {

                if (IsSingleSection(
                        pixel,
                        transitionSpaceGrid)) {

                    for (dr = -1; dr <= 1; dr++) {
                        for (dc = -1; dc <= 1; dc++) {

                            if (dr.Abs() == dc.Abs()) {
                                continue;
                            }

                            r = pixel.Item1 + dr;
                            c = pixel.Item2 + dc;
                            if (r >= 0 && c >= 0
                                    && r < transitionSpaceGrid.GetLength(0)
                                    && c < transitionSpaceGrid.GetLength(1)
                                    && transitionZoneGrid[r, c]) {
                                newPixels.Add((r, c));
                            }
                        }
                    }
                }   
            }

            foreach ((int, int) pixel in newPixels) {
                transitionZoneGrid[
                    pixel.Item1,
                    pixel.Item2] = false;
                transitionSpaceGrid[
                    pixel.Item1,
                    pixel.Item2] = true;
            }
        }

        private static bool IsSingleSection(
                (int, int) pixel,
                bool[,] transitionSpaceGrid) {

            int dr, r, dc, c;

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    if (dr.Abs() == dc.Abs()) {
                        continue;
                    }

                    r = pixel.Item1 + dr;
                    c = pixel.Item2 + dc;
                    if (r >= 0 && c >= 0
                            && r < transitionSpaceGrid.GetLength(0)
                            && c < transitionSpaceGrid.GetLength(1)
                            && transitionSpaceGrid[r, c]) {
                        return false;
                    }
                }
            }
            return true;
        }

        private static void SegmentRoomsAndTransitionSpaces(
                int roomCount,
                bool[,] interiorSpaceGrid,
                bool[,] transitionSpaceGrid,
                bool[,] bottleNeckGrid,
                int[,] roomDomainGrid,
                int[,] spacePartitioningGrid,
                out int lastTransitionSpaceId) {

            int r, c;
            int roomId;
            int transitionSpaceId = -1;

            for (r = 0; r < spacePartitioningGrid.GetLength(0); r++) {
                for (c = 0; c < spacePartitioningGrid.GetLength(1); c++) {

                    if (transitionSpaceGrid[r, c]) {
                        roomDomainGrid[r, c] = 0;
                    }

                    if (spacePartitioningGrid[r, c] != 0) {
                        continue;
                    }

                    if (transitionSpaceGrid[r, c]) {

                        SegmentTransitionSpace(
                            r,
                            c,
                            transitionSpaceId--,
                            transitionSpaceGrid,
                            spacePartitioningGrid);
                    }
                    else if (roomDomainGrid[r, c] != 0) {

                        roomId = GetNeighbouringRoomId(
                            r,
                            c,
                            spacePartitioningGrid);

                        if (roomId > 0) {

                            SegmentRoomFromRoomDomain(
                                r,
                                c,
                                roomId,
                                transitionSpaceGrid,
                                roomDomainGrid,
                                spacePartitioningGrid);
                        }
                        
                    }
                    else if (interiorSpaceGrid[r, c]
                            && !bottleNeckGrid[r, c]) {

                        SegmentNewRoom(
                            r,
                            c,
                            ++roomCount,
                            interiorSpaceGrid,
                            transitionSpaceGrid,
                            bottleNeckGrid,
                            roomDomainGrid,
                            spacePartitioningGrid);
                    }
                }
            }

            lastTransitionSpaceId = transitionSpaceId;
        }

        private static void SegmentTransitionSpace(
                int r,
                int c,
                int transitionSpaceId,
                bool[,] transitionSpaceGrid,
                int[,] spacePartitioningGrid) {

            int dr, r2, dc, c2;
            (int, int) candidate;
            Queue<(int, int)> candidates = new Queue<(int, int)>();

            candidates.Enqueue((r, c));

            do {

                candidate = candidates.Dequeue();
                if (spacePartitioningGrid[
                        candidate.Item1,
                        candidate.Item2] != 0) {
                    continue;
                }
                spacePartitioningGrid[
                    candidate.Item1,
                    candidate.Item2] = transitionSpaceId;

                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        r2 = candidate.Item1 + dr;
                        c2 = candidate.Item2 + dc;
                        if (r2 >= 0 && c2 >= 0
                                && r2 < spacePartitioningGrid.GetLength(0)
                                && c2 < spacePartitioningGrid.GetLength(1)
                                && transitionSpaceGrid[r2, c2]
                                && spacePartitioningGrid[r2, c2] == 0) {
                            candidates.Enqueue((r2, c2));
                        }
                    }
                }
            } while (candidates.Count > 0);
        }

        private static int GetNeighbouringRoomId(
                int r,
                int c,
                int[,] spacePartitioningGrid) {

            int dr, r2, dc, c2;
            int roomId = 0;

            for (dr = -1; dr <= 1; dr++) {
                for (dc = -1; dc <= 1; dc++) {

                    if (dr.Abs() == dc.Abs()) {
                        continue;
                    }

                    r2 = r + dr;
                    c2 = c + dc;
                    if (r2 >= 0 && c2 >= 0
                            && r2 < spacePartitioningGrid.GetLength(0)
                            && c2 < spacePartitioningGrid.GetLength(1)) {
                        roomId = spacePartitioningGrid[r2, c2];

                        if (roomId > 0) {
                            return roomId;
                        }
                    }
                }
            }

            return roomId;
        }

        private static void SegmentRoomFromRoomDomain(
                int r,
                int c,
                int roomId,
                bool[,] transitionSpaceGrid,
                int[,] roomDomainGrid,
                int[,] spacePartitioningGrid) {

            int dr, r2, dc, c2;
            (int, int) candidate;
            Queue<(int, int)> candidates = new Queue<(int, int)>();

            candidates.Enqueue((r, c));

            do {

                candidate = candidates.Dequeue();
                if (spacePartitioningGrid[
                        candidate.Item1,
                        candidate.Item2] != 0) {
                    continue;
                }
                spacePartitioningGrid[
                    candidate.Item1,
                    candidate.Item2] = roomId;

                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (dr.Abs() == dc.Abs()) {
                            continue;
                        }

                        r2 = candidate.Item1 + dr;
                        c2 = candidate.Item2 + dc;
                        if (r2 >= 0 && c2 >= 0
                                && r2 < spacePartitioningGrid.GetLength(0)
                                && c2 < spacePartitioningGrid.GetLength(1)
                                && roomDomainGrid[r2, c2] != 0
                                && !transitionSpaceGrid[r2, c2]
                                && spacePartitioningGrid[r2, c2] == 0) {
                            candidates.Enqueue((r2, c2));
                        }
                    }
                }
            } while (candidates.Count > 0);
        }

        private static void SegmentNewRoom(
                int r,
                int c,
                int roomId,
                bool[,] interiorSpaceGrid,
                bool[,] transitionSpaceGrid,
                bool[,] bottleNeckGrid,
                int[,] roomDomainGrid,
                int[,] spacePartitioningGrid) {

            int dr, r2, dc, c2;
            (int, int) candidate;
            Queue<(int, int)> candidates = new Queue<(int, int)>();

            candidates.Enqueue((r, c));
            do {

                candidate = candidates.Dequeue();
                if (spacePartitioningGrid[
                        candidate.Item1,
                        candidate.Item2] != 0) {
                    continue;
                }
                spacePartitioningGrid[
                    candidate.Item1,
                    candidate.Item2] = roomId;

                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (dr.Abs() == dc.Abs()) {
                            continue;
                        }
                        r2 = candidate.Item1 + dr;
                        c2 = candidate.Item2 + dc;

                        if (r2 >= 0 && c2 >= 0
                                && r2 < spacePartitioningGrid.GetLength(0)
                                && c2 < spacePartitioningGrid.GetLength(1)
                                && interiorSpaceGrid[r2, c2]
                                && roomDomainGrid[r2, c2] == 0
                                && !transitionSpaceGrid[r2, c2]
                                && spacePartitioningGrid[r2, c2] == 0
                                && !bottleNeckGrid[r2, c2]) {
                            candidates.Enqueue((r2, c2));
                        }
                    }
                }
            } while (candidates.Count > 0);
        }

        private static void ResolveConnectedRooms(
                int lastTransitionSpaceId,
                double resolution,
                bool[,] interiorSpaceGrid,
                int[,] roomDomainGrid,
                int[,] spacePartitioningGrid,
                Dictionary<int, List<(int, int)>> roomSeedSegmentContours) {

            HashSet<(int, int)> roomConnectionIds = GetRoomConnectionIds(spacePartitioningGrid);

            if (roomConnectionIds.Count == 0) {
                return;
            }

            ReevaluateRoomConnections(
                lastTransitionSpaceId,
                resolution,
                interiorSpaceGrid,
                roomDomainGrid,
                spacePartitioningGrid,
                roomConnectionIds,
                roomSeedSegmentContours);

            roomConnectionIds = GetRoomConnectionIds(spacePartitioningGrid);

            if (roomConnectionIds.Count == 0) {
                return;
            }

            MergeConnectedRooms(
                roomDomainGrid,
                spacePartitioningGrid,
                roomConnectionIds);
        }

        private static HashSet<(int, int)> GetRoomConnectionIds(
                this int[,] spacePartitioningGrid) {

            int dr, r, r2, dc, c, c2;
            int roomId, roomId2;
            HashSet<(int, int)> roomConnectionIds = new HashSet<(int, int)>();

            for (r = 0; r < spacePartitioningGrid.GetLength(0); r++) {
                for (c = 0; c < spacePartitioningGrid.GetLength(1); c++) {

                    roomId = spacePartitioningGrid[r, c];
                    if (roomId <= 0) {
                        continue;
                    }

                    for (dr = -1; dr <= 1; dr++) {
                        for (dc = -1; dc <= 1; dc++) {

                            if (dr.Abs() == dc.Abs()) {
                                continue;
                            }

                            r2 = r + dr;
                            c2 = c + dc;
                            if (r2 >= 0 && c2 >= 0
                                    && r2 < spacePartitioningGrid.GetLength(0)
                                    && c2 < spacePartitioningGrid.GetLength(1)) {
                                roomId2 = spacePartitioningGrid[r2, c2];

                                if (roomId2 > 0
                                        && roomId2 != roomId) {

                                    roomConnectionIds.Add(
                                        Util.GetUnambiguousOrder(
                                            roomId, 
                                            roomId2));
                                }
                            }
                        }
                    }
                }
            }

            return roomConnectionIds;
        }

        private static void ReevaluateRoomConnections(
                int lastTransitionSpaceId,
                double resolution,
                bool[,] interiorSpaceGrid,
                int[,] roomDomainGrid,
                int[,] spacePartitioningGrid,
                HashSet<(int, int)> roomConnectionIds,
                Dictionary<int, List<(int, int)>> roomSeedSegmentContours) {

            int transitionZoneId = 1;
            bool[,] transitionSpaceGrid = new bool[
                spacePartitioningGrid.GetLength(0),
                spacePartitioningGrid.GetLength(1)];
            bool[,] bottleNeckGrid = new bool[
                spacePartitioningGrid.GetLength(0),
                spacePartitioningGrid.GetLength(1)];
            int[,] roomDomainContactZoneSegmentGrid;
            int[,] newRoomDomainGrid;
            HashSet<int> roomIdsToReevaluate;
            Dictionary<(int, int), List<List<(int, int)>>> roomDomainContactZoneSegments;

            roomIdsToReevaluate = roomConnectionIds.GetRoomIds();

            newRoomDomainGrid = GetRoomDomainsForReevaluationOfRoomConnections(
               interiorSpaceGrid,
               spacePartitioningGrid,
               roomIdsToReevaluate,
               roomSeedSegmentContours);

            roomDomainContactZoneSegments = DetectRoomDomainContactZoneSegments(
                newRoomDomainGrid);

            roomDomainContactZoneSegmentGrid = InitializeRoomDomainContactZoneSegmentGrid(
                newRoomDomainGrid,
                roomDomainContactZoneSegments);

            foreach ((int, int) roomConnectionId in roomConnectionIds) {

                if (!roomDomainContactZoneSegments.ContainsKey(roomConnectionId)) {
                    continue;
                }

                foreach (List<(int, int)> roomDomainContactZoneSegment in roomDomainContactZoneSegments[roomConnectionId]) {

                    DetectTransitionSpaces(
                        resolution,
                        transitionZoneId,
                        roomConnectionId,
                        transitionSpaceGrid,
                        bottleNeckGrid,
                        newRoomDomainGrid,
                        roomDomainContactZoneSegmentGrid,
                        roomSeedSegmentContours[roomConnectionId.Item1],
                        roomSeedSegmentContours[roomConnectionId.Item2],
                        roomDomainContactZoneSegment);

                    transitionZoneId++;
                }
            }

            ResegmentSpacePartitioning(
                lastTransitionSpaceId,
                bottleNeckGrid,
                transitionSpaceGrid,
                roomDomainGrid,
                spacePartitioningGrid,
                roomIdsToReevaluate,
                roomSeedSegmentContours);
        }

        private static HashSet<int> GetRoomIds(
                this HashSet<(int, int)> roomConnectionIds) {

            HashSet<int> roomIds = new HashSet<int>();

            foreach ((int, int) roomConnectionId in roomConnectionIds) {
                roomIds.Add(roomConnectionId.Item1);
                roomIds.Add(roomConnectionId.Item2);
            }

            return roomIds;
        }

        private static int[,] GetRoomDomainsForReevaluationOfRoomConnections(
                bool[,] interiorSpaceGrid,
                int[,] spacePartitioningGrid,
                HashSet<int> roomIdsToReevaluate,
                Dictionary<int, List<(int, int)>> roomSeedSegmentContours) {

            bool[,] forbidden = new bool[
                spacePartitioningGrid.GetLength(0),
                spacePartitioningGrid.GetLength(1)];
            int[,] newRoomDomainGrid = new int[
                spacePartitioningGrid.GetLength(0),
                spacePartitioningGrid.GetLength(1)];
            Dictionary<int, List<(int, int)>> growPixels;

            InitializeNewRoomDomainGrid(
                newRoomDomainGrid,
                roomIdsToReevaluate,
                roomSeedSegmentContours);
            
            do {

                growPixels = GrowNewLayerOfRoomsToReevaluate(
                    interiorSpaceGrid,
                    forbidden,
                    spacePartitioningGrid,
                    newRoomDomainGrid,
                    roomIdsToReevaluate);

                if (growPixels.Count == 0) {
                    break;
                }

                AddNewLayerOfRoomsToReevaluate(
                    newRoomDomainGrid,
                    growPixels);
                
            } while (true);

            return newRoomDomainGrid;
        }

        private static void InitializeNewRoomDomainGrid(
                int[,] newRoomDomainGrid,
                HashSet<int> roomIdsToReevaluate,
                Dictionary<int, List<(int, int)>> roomSeedSegmentContours) {

            foreach (int roomId in roomIdsToReevaluate) {
                foreach ((int, int) pixel in roomSeedSegmentContours[roomId]) {
                    newRoomDomainGrid[
                        pixel.Item1,
                        pixel.Item2] = roomId;
                }
            }
        } 

        private static Dictionary<int, List<(int, int)>> GrowNewLayerOfRoomsToReevaluate(
                bool[,] interiorSpaceGrid,
                bool[,] forbidden,
                int[,] spacePartitioningGrid,
                int[,] newRoomDomainGrid,
                HashSet<int> roomIdsToReevaluate) {

            bool changedPixel;
            int segmentId;
            int dr, r, r2, dc, c, c2;
            Dictionary<int, List<(int, int)>> growPixels 
                = new Dictionary<int, List<(int, int)>>();

            for (r = 0; r < interiorSpaceGrid.GetLength(0); r++) {
                for (c = 0; c < interiorSpaceGrid.GetLength(1); c++) {

                    if (newRoomDomainGrid[r, c] != 0
                            || !interiorSpaceGrid[r, c]
                            || spacePartitioningGrid[r, c] < 0
                            || forbidden[r, c]) {
                        continue;
                    }

                    changedPixel = false;

                    for (dr = -1; dr <= 1; dr++) {
                        for (dc = -1; dc <= 1; dc++) {

                            r2 = r + dr;
                            c2 = c + dc;

                            if (r2 >= 0 && c2 >= 0
                                    && r2 < interiorSpaceGrid.GetLength(0)
                                    && c2 < interiorSpaceGrid.GetLength(1)
                                    && newRoomDomainGrid[r2, c2] != 0) {

                                segmentId = spacePartitioningGrid[r, c];

                                if (segmentId > 0
                                        && !roomIdsToReevaluate.Contains(segmentId)) {

                                    forbidden[r2, c2] = true;
                                    newRoomDomainGrid[r2, c2] = 0;
                                }
                                else {
                                    changedPixel = true;
                                    growPixels.BucketAdd(
                                        newRoomDomainGrid[r2, c2],
                                        (r, c));
                                    break;
                                }
                            }
                        }
                        if (changedPixel) {
                            break;
                        }
                    }
                }
            }

            return growPixels;
        }

        private static void AddNewLayerOfRoomsToReevaluate(
                int[,] newRoomDomainGrid,
                Dictionary<int, List<(int, int)>> growPixels) {

            foreach (int roomId in growPixels.Keys) {
                foreach ((int, int) pixel in growPixels[roomId]) {
                    newRoomDomainGrid[
                        pixel.Item1,
                        pixel.Item2] = roomId;
                }
            }
        }

        private static void ResegmentSpacePartitioning(
                int lastTransitionSpaceId,
                bool[,] bottleNeckGrid,
                bool[,] transitionSpaceGrid,
                int[,] roomDomainGrid,
                int[,] spacePartitioningGrid,
                HashSet<int> roomIdsToReevaluate,
                Dictionary<int, List<(int, int)>> roomSeedSegmentContours) {

            ResegmentTransitionSpaces(
                lastTransitionSpaceId,
                transitionSpaceGrid,
                bottleNeckGrid,
                spacePartitioningGrid,
                roomDomainGrid);

            ResegmentRooms(
                roomDomainGrid,
                spacePartitioningGrid,
                roomIdsToReevaluate,
                roomSeedSegmentContours);
        }

        private static void ResegmentTransitionSpaces(
                int lastTransitionSpaceId,
                bool[,] transitionSpaceGrid,
                bool[,] bottleNeckGrid,
                int[,] spacePartitioningGrid,
                int[,] roomDomainGrid) {

            int r, c;
            bool[,] isSegmented = new bool[
                spacePartitioningGrid.GetLength(0),
                spacePartitioningGrid.GetLength(1)];
            List<List<(int, int)>> transitionSpaceSegments = new List<List<(int, int)>>();

            for (r = 0; r < spacePartitioningGrid.GetLength(0); r++) {
                for (c = 0; c < spacePartitioningGrid.GetLength(1); c++) {

                    if (bottleNeckGrid[r, c]) {
                        spacePartitioningGrid[r, c] = 0;
                        roomDomainGrid[r, c] = 0;
                        continue;
                    }

                    if (!transitionSpaceGrid[r, c]
                            || isSegmented[r, c]) {
                        continue;
                    }

                    transitionSpaceSegments.Add(
                        GrowTransitionSpaceSegment(
                            r,
                            c,
                            isSegmented,
                            transitionSpaceGrid,
                            spacePartitioningGrid));
                }
            }

            foreach (List<(int, int)> segment in transitionSpaceSegments) {

                lastTransitionSpaceId--;

                foreach ((int, int) pixel in segment) {
                    spacePartitioningGrid[
                        pixel.Item1,
                        pixel.Item2] = lastTransitionSpaceId;
                }
            }
        }

        private static List<(int, int)> GrowTransitionSpaceSegment(
                int r,
                int c,
                bool[,] isSegmented,
                bool[,] transitionSpaceGrid,
                int[,] spacePartitioningGrid) {

            int dr, r2, dc, c2;
            List<(int, int)> segment = new List<(int, int)>();
            Queue<(int, int)> candidates = new Queue<(int, int)>();

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
                segment.Add(candidate);

                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (dr.Abs() == dc.Abs()) {
                            continue;
                        }

                        r2 = candidate.Item1 + dr;
                        c2 = candidate.Item2 + dc;
                        if (r2 >= 0 && c2 >= 0
                                && r2 < spacePartitioningGrid.GetLength(0)
                                && c2 < spacePartitioningGrid.GetLength(1)
                                && transitionSpaceGrid[r2, c2]
                                && !isSegmented[r2, c2]) {
                            candidates.Enqueue((r2, c2));
                        }
                    }
                }
            } while (candidates.Count > 0);

            return segment;
        }

        private static void ResegmentRooms(
                int[,] roomDomainGrid,
                int[,] spacePartitioningGrid,
                HashSet<int> roomIdsToReevaluate,
                Dictionary<int, List<(int, int)>> roomSeedSegmentContours) {

            int dr, r2, dc, c2;
            bool[,] isSegmented = new bool[
                spacePartitioningGrid.GetLength(0),
                spacePartitioningGrid.GetLength(1)];
            Queue<(int, int)> candidates = new Queue<(int, int)>();
            
            foreach (int roomId in roomIdsToReevaluate) {

                candidates.Enqueue(
                    roomSeedSegmentContours[roomId].First());

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
                    spacePartitioningGrid[
                        candidate.Item1,
                        candidate.Item2] = roomId;
                    roomDomainGrid[
                        candidate.Item1,
                        candidate.Item2] = roomId;

                    for (dr = -1; dr <= 1; dr++) {
                        for (dc = -1; dc <= 1; dc++) {

                            if (dr.Abs() == dc.Abs()) {
                                continue;
                            }

                            r2 = candidate.Item1 + dr;
                            c2 = candidate.Item2 + dc;
                            if (r2 >= 0 && c2 >= 0
                                    && r2 < spacePartitioningGrid.GetLength(0)
                                    && c2 < spacePartitioningGrid.GetLength(1)
                                    && spacePartitioningGrid[r2, c2] > 0
                                    && !isSegmented[r2, c2]) {
                                candidates.Enqueue((r2, c2));
                            }
                        }
                    }
                } while (candidates.Count > 0);
            }
        }

        private static void MergeConnectedRooms(
                int[,] roomDomainGrid,
                int[,] spacePartitioningGrid,
                HashSet<(int, int)> roomConnectionIds) {

            List<HashSet<int>> mergeClusters = ClusterRoomIdsForMerging(roomConnectionIds);

            Dictionary<int, int> mergeMapping = GetMergeMapping(mergeClusters);

            MergeRooms(
                roomDomainGrid,
                spacePartitioningGrid,
                mergeMapping);
        }

        private static List<HashSet<int>> ClusterRoomIdsForMerging(
                HashSet<(int, int)> roomConnectionIds) {

            List<int> matchClusterIndices;
            List<HashSet<int>> clusters = new List<HashSet<int>>();

            foreach ((int, int) roomConnectionId in roomConnectionIds) {

                matchClusterIndices = GetMatchClusterIndices(
                    roomConnectionId,
                    clusters);

                if (matchClusterIndices.Count == 0) {
                    clusters.Add(new HashSet<int> {
                        roomConnectionId.Item1,
                        roomConnectionId.Item2
                    });
                }
                else if (matchClusterIndices.Count == 1) {
                    clusters[matchClusterIndices[0]].Add(roomConnectionId.Item1);
                    clusters[matchClusterIndices[0]].Add(roomConnectionId.Item2);
                }
                else {
                    MergeClusters(
                        matchClusterIndices,
                        clusters);
                }
            }

            return clusters;
        }

        private static List<int> GetMatchClusterIndices(
                (int, int) roomConnectionId,
                List<HashSet<int>> clusters) {

            List<int> matchClusterIndices = new List<int>();

            for (int i = 0; i < clusters.Count; i++) {
                if (clusters[i].Contains(roomConnectionId.Item1)
                        || clusters[i].Contains(roomConnectionId.Item2)) {
                    matchClusterIndices.Add(i);
                }
            }

            return matchClusterIndices;
        }

        private static void MergeClusters(
                List<int> matchClusterIndices,
                List<HashSet<int>> clusters) {

            HashSet<int> newCluster = new HashSet<int>();

            foreach (int clusterIndex in matchClusterIndices) {
                newCluster.AddRange(clusters[clusterIndex]);
            }

            clusters.Add(newCluster);

            foreach (int clusterIndex in matchClusterIndices.OrderDescending()) {
                clusters.RemoveAt(clusterIndex);
            }
        }

        private static Dictionary<int, int> GetMergeMapping(
                List<HashSet<int>> clusters) {

            int destinationRoomId;
            Dictionary<int, int> mergeMapping = new Dictionary<int, int>();

            foreach (HashSet<int> cluster in clusters) {

                destinationRoomId = cluster.First();

                foreach (int roomId in cluster) {

                    if (roomId != destinationRoomId) {
                        mergeMapping.Add(
                            roomId,
                            destinationRoomId);
                    }
                }
            }

            return mergeMapping;
        }

        private static void MergeRooms(
                int[,] roomDomainGrid,
                int[,] spacePartitioningGrid,
                Dictionary<int, int> mergeMapping) {

            int r, c, roomId;

            for (r = 0; r < spacePartitioningGrid.GetLength(0); r++) {
                for (c = 0; c < spacePartitioningGrid.GetLength(1); c++) {

                    roomId = spacePartitioningGrid[r, c];
                    if (mergeMapping.ContainsKey(roomId)) {
                        spacePartitioningGrid[r, c] = mergeMapping[roomId];
                    }

                    roomId = roomDomainGrid[r, c];
                    if (mergeMapping.ContainsKey(roomId)) {
                        roomDomainGrid[r, c] = mergeMapping[roomId];
                    }
                }
            }
        }

        private static void MergeDeadEndTransitionSpacesWithRooms(
                int[,] roomDomainGrid,
                int[,] spacePartitioningGrid) {

            HashSet<int> deadEndTransitionSpaceIds = GetDeadEndTransitionSpaceIds(
                spacePartitioningGrid);

            if (deadEndTransitionSpaceIds.Count > 0) {
                MergeDeadEndTransitionSpacesWithRooms(
                    roomDomainGrid,
                    spacePartitioningGrid,
                    deadEndTransitionSpaceIds);
            }
        }

        private static HashSet<int> GetDeadEndTransitionSpaceIds(
                int[,] spacePartitioningGrid) {

            Dictionary<int, HashSet<int>> transitionSpaceAdjacency 
                = GetTransitionSpaceAdjacency(
                    spacePartitioningGrid);

            return transitionSpaceAdjacency
                .Keys
                .Where(id => transitionSpaceAdjacency[id].Count < 2)
                .ToHashSet();
        }

        private static Dictionary<int, HashSet<int>> GetTransitionSpaceAdjacency(
                int[,] spacePartitioningGrid) {

            int dr, r, r2, dc, c, c2;
            int transitionSpaceId, roomId;
            Dictionary<int, HashSet<int>> transitionSpaceAdjacency = new Dictionary<int, HashSet<int>>();

            for (r = 0; r < spacePartitioningGrid.GetLength(0); r++) {
                for (c = 0; c < spacePartitioningGrid.GetLength(1); c++) {

                    transitionSpaceId = spacePartitioningGrid[r, c];
                    if (transitionSpaceId >= 0) {
                        continue;
                    }

                    for (dr = -1; dr <= 1; dr++) {
                        for (dc = -1; dc <= 1; dc++) {

                            if (dr.Abs() == dc.Abs()) {
                                continue;
                            }

                            r2 = r + dr;
                            c2 = c + dc;
                            if (r2 >= 0 && c2 >= 0
                                    && r2 < spacePartitioningGrid.GetLength(0)
                                    && c2 < spacePartitioningGrid.GetLength(1)) {

                                roomId = spacePartitioningGrid[r2, c2];
                                if (roomId > 0) {

                                    transitionSpaceAdjacency.BucketAdd(
                                        transitionSpaceId,
                                        roomId);
                                }
                            }
                        }
                    }
                }
            }

            return transitionSpaceAdjacency;
        }

        private static void MergeDeadEndTransitionSpacesWithRooms(
                int[,] roomDomainGrid,
                int[,] spacePartitioningGrid,
                HashSet<int> deadEndTransitionSpaceIds) {

            bool stop;
            int dr, dr2, r, r2, r3, dc, dc2, c, c2, c3;
            int transitionSpaceId, roomId, segmentId;
            (int, int) candidate;
            Queue<(int, int)> candidates = new Queue<(int, int)>();

            for (r = 0; r < spacePartitioningGrid.GetLength(0); r++) {
                for (c = 0; c < spacePartitioningGrid.GetLength(1); c++) {

                    transitionSpaceId = spacePartitioningGrid[r, c];
                    if (!deadEndTransitionSpaceIds.Contains(transitionSpaceId)) {
                        continue;
                    }

                    roomId = GetNeighbouringRoomId(
                        r,
                        c,
                        spacePartitioningGrid);
                    if (roomId <= 0) {
                        continue;
                    }

                    candidates.Enqueue((r, c));

                    do {

                        candidate = candidates.Dequeue();
                        if (spacePartitioningGrid[
                                candidate.Item1,
                                candidate.Item2] > 0) {
                            continue;
                        }
                        spacePartitioningGrid[
                            candidate.Item1,
                            candidate.Item2] = roomId;

                        for (dr = -1; dr <= 1; dr++) {
                            for (dc = -1; dc <= 1; dc++) {

                                if (dr.Abs() == dc.Abs()) {
                                    continue;
                                }

                                r2 = candidate.Item1 + dr;
                                c2 = candidate.Item2 + dc;
                                if (r2 >= 0 && c2 >= 0
                                        && r2 < spacePartitioningGrid.GetLength(0)
                                        && c2 < spacePartitioningGrid.GetLength(1)) {

                                    segmentId = spacePartitioningGrid[r2, c2];
                                    if (segmentId < 0) {
                                        candidates.Enqueue((r2, c2));
                                        continue;
                                    }

                                    if (segmentId > 0
                                            || (segmentId == 0
                                                && roomDomainGrid[r2, c2] == 0)) {
                                        continue;
                                    }

                                    stop = false;

                                    for (dr2 = -1; dr2 <= 1; dr2++) {
                                        for (dc2 = -1; dc2 <= 1; dc2++) {

                                            r3 = r2 + dr2;
                                            c3 = c2 + dc2;
                                            if (r3 >= 0 && c3 >= 0
                                                    && r3 < spacePartitioningGrid.GetLength(0)
                                                    && c3 < spacePartitioningGrid.GetLength(1)) {

                                                segmentId = spacePartitioningGrid[r3, c3];

                                                if (segmentId > 0
                                                        && segmentId != roomId) {
                                                    stop = true;
                                                    break;
                                                }
                                            }
                                        }

                                        if (stop) {
                                            break;
                                        }
                                    }

                                    if (!stop) {
                                        candidates.Enqueue((r2, c2));
                                    }
                                }
                            }
                        }
                    } while (candidates.Count > 0);
                }
            }
        }
    }
}
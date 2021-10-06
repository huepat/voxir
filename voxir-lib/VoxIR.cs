using HuePat.VoxIR.Util.Geometry;
using HuePat.VoxIR.Util.Grid;
using System.Collections.Generic;

namespace HuePat.VoxIR {
    public static class Parameters {
        public const int NORMAL_SMOOTHING_MIN_NEIGHBOUR_NORMAL_COUNT = 2;
        public static readonly double DIRECTION_DOWN_ANGLE_THRESHOLD = 60.0.DegreeToRadian();
        public static readonly double DIRECTION_UP_ANGLE_THRESHOLD = 45.0.DegreeToRadian();
        public const double ROOM_MIN_AREA = 0.5;
        public const double HOLE_BORDER_MAX_HEIGHT_DIFFERENCE = 0.2;
        public const double CEILING_GRID_SMOOTHING_DIAMETER = 0.5;
        public const double FLOOR_GRID_SMOOTHING_DIAMETER = 0.5;
        public const double FLOOR_MAX_HEIGHT_DIFFERENCE = 0.18;
        public const double HOLE_BORDER_WALL_MAX_COMPLETENESS = 0.75;
        public const double HOLE_WALL_SEARCH_RADIUS = 0.2;
        public const double FLOOR_CEILING_COMPLETION_SEARCH_DISTANCE = 0.15;
        public const double ROOMLET_FLOOR_HOLE_CLOSING_MAX_DISTANCE = 0.15;
        public const double ROOMLET_MIN_HEIGHT = 0.5;
        public const double ROOMLET_MIN_AREA = 0.02;
        public const double ROOMLET_MIN_CEILING_GUESS_HEIGHT = 1.5;
        public const double ROOMLET_MIN_CEILING_GUESS_FLOOR_AREA = 3.0;
        public const double ROOMLET_MIN_CEILING_COMPLETENESS = 0.75;
        public const double ROOMLET_MIN_FLOOR_SEGMENT_AREA = 0.5;
        public const double MIN_WALL_HEIGHT = 0.2;
        public const double HORIZONTAL_SURFACE_REFINEMENT_DISTANCE = 0.10;
        public const double WALL_REFINEMENT_RADIUS_INSIDE = 0.15;
        public const double WALL_REFINEMENT_RADIUS_OUTSIDE = 0.15;
        public const double MAX_WALL_THICKNESS = 0.5;
        public const double WALL_OPENING_OCCLUSION_SEARCH_DISTANCE = 0.7;
        public const double WALL_GRID_MIN_WALL_HEIGHT = 1.0;
        public const double SLOPE_DETERMINATION_RADIUS = 0.5;
        public const double MIN_RAMP_SLOPE_DEGREES = 20;
        public const double MAX_RAMP_SLOPE_DEGREES = 50;
        public const double RAMP_MIN_HEIGHT = 1.2;
        public const double RAMP_SPACE_CLOSING_RADIUS = 0.25;
        public const double HORIZONTAL_SURFACE_REFINEMENT_MIN_AREA = 1.0;
        public const double HORIZONTAL_SURFACE_REFINEMENT_MIN_RAMP_RATIO = 0.25;
        public const double INTERIOR_SPACE_PARTITIONING_2D_1ST_EROSION_KERNEL_SIZE = 0.55;
        public const double INTERIOR_SPACE_PARTITIONING_2D_2ND_EROSION_KERNEL_SIZE = 0.2;
        public const double TRANSITION_SPACE_MIN_WIDTH = 0.5;
        public const double TRANSITION_SPACE_MIN_HEIGHT = 1.2;
        public const double ROOM_SPACE_MIN_HEIGHT = 1.0;
        public const double AUTONOMOUS_RAMP_SPACE_NEIGHBOUR_ROOM_MIN_AREA = 3.0;
        public const double ROOM_CONTACT_SURFACE_MIN_OPENING_AREA = 0.5;
        public const double ROOM_CONTACT_SURFACE_MIN_OPENING_RATIO = 0.75;
        public const double MIN_WALL_OPENING_HEIGHT = 0.5;
        public const double MAX_WALL_OPENING_JUMP_SIZE = 0.1;
        public const double TRANSITION_SPACE_REFINEMENT_MIN_SURFACE_WIDTH = 0.5;
        public const double TRANSITION_SPACE_REFINEMENT_SEARCH_DISTANCE = 0.5;
        public const double TRANSITION_SPACE_REFINEMENT_MIN_DIRECTION_RATIO = 0.5;
        public const double TRANSITION_SPACE_REFINEMENT_HEIGHT_CLUSTER_RATIO_THRESHOLD = 0.2;
        public const double TRANSITION_SPACE_REFINEMENT_VERTICAL_RESOLUTION = 0.2;
        public const double TRANSITION_SPACE_REFINEMENT_SURE_MERGE_MIN_WIDTH = 2.0;
        public const double TRANSITION_SPACE_REFINEMENT_MERGE_WIDTH_RATIO = 0.5;
        public const double FINAL_ROOM_REFINEMENT_MIN_ROOM_AREA = 1.0;
        public const double INDEPENDENT_RAMP_NEIGHBOUR_MIN_AREA = 3.0;
        public const double ROOM_SPACE_REFINEMENT_FLOOR_SEARCH_HEIGHT_THRESHOLD = 1.0;
        public const double POST_SPACE_PARTITIONING_REFINEMENT_WALL_OPENING_MIN_WIDTH = 0.5;
        public const double POST_SPACE_PARTITIONING_REFINEMENT_WALL_OPENING_MAX_CLOSED_RATIO = 0.5;
        public const double POST_SPACE_PARTITIONING_REFINEMENT_MIN_WALL_HEIGHT = 1.5;
    }

    public static class VoxIR {
        public static int[,,][] Process(
                double resolution,
                AABox gridExtent,
                Mesh mesh,
                out HashSet<int> rampSpaceIds) {

            int roomCount;
            byte[,,] normalGrid;
            int[,,][] reconstructionGrid;
            HashSet<int> roomIdsWithoutFloor;
            HashSet<int> roomletIds;

            normalGrid = DataPreparation.Voxelization.Voxelize(
                resolution,
                gridExtent,
                mesh);

            reconstructionGrid = CreateVoxelReconstructionGrid(
                resolution,
                normalGrid,
                out roomCount,
                out roomIdsWithoutFloor);

            CompleteVoxelModel(
                roomCount,
                resolution,
                normalGrid,
                reconstructionGrid,
                roomIdsWithoutFloor,
                out roomletIds);

            RefineVoxelModel(
                resolution,
                normalGrid,
                reconstructionGrid,
                roomletIds);

            RepartitionIndoorSpace(
                resolution,
                normalGrid,
                reconstructionGrid,
                out rampSpaceIds);

            RoomSegmentation
                .PostRoomPartitioningRefinement
                .RefineReconstructionGridAfterSpacePartitioning(
                    resolution,
                    normalGrid,
                    reconstructionGrid);

            return reconstructionGrid;
        }

        private static int[,,][] CreateVoxelReconstructionGrid(
                double resolution,
                byte[,,] normalGrid,
                out int roomCount,
                out HashSet<int> roomIdsWithoutFloor) {

            int roomId;
            int dominantFloorHeight;
            int maxCeilingHeight;
            (int, int) floorHeightRange;
            (int, int, int) ceilingGridOffset;
            long[] floorIncidence;
            int[,][] ceilingGrid, floorGrid;
            int[,,][] reconstructionGrid = new int[
                normalGrid.GetLength(0),
                normalGrid.GetLength(1),
                normalGrid.GetLength(2)][];
            List<Hole> ceilingHoles;
            List<VoxelSegment> ceilingSegments;
            List<List<(int, int)>> floorCandidateSegments;

            roomIdsWithoutFloor = new HashSet<int>();

            CeilingAndFloorReconstruction
                .CeilingDetection
                .TraceWallThroughCeilings(
                    normalGrid,
                    out floorIncidence);

            ceilingSegments = CeilingAndFloorReconstruction
                .CeilingDetection
                .SegmentCeilings(
                    resolution,
                    normalGrid,
                    reconstructionGrid);

            roomCount = ceilingSegments.Count;

            for (roomId = 0; roomId < roomCount; roomId++) {

                ceilingGrid = CeilingAndFloorReconstruction
                    .CeilingRefinement
                    .Create2DCeilingGrid(
                        ceilingSegments[roomId],
                        out maxCeilingHeight,
                        out ceilingGridOffset);

                ceilingHoles = CeilingAndFloorReconstruction
                    .CeilingRefinement
                    .DetectCeilingHoles(ceilingGrid);

                CeilingAndFloorReconstruction
                    .CeilingRefinement
                    .InterpolateCeilingHolesHeight(
                        roomId,
                        maxCeilingHeight,
                        resolution,
                        ceilingGridOffset,
                        ceilingGrid,
                        reconstructionGrid,
                        ceilingHoles);

                floorGrid = CeilingAndFloorReconstruction
                    .FloorDetection
                    .Create2DFloorGrid(
                        roomId,
                        ceilingGridOffset,
                        ceilingGrid,
                        normalGrid,
                        reconstructionGrid);

                floorCandidateSegments = CeilingAndFloorReconstruction
                    .FloorDetection
                    .DetectFloorCandidateSegments(
                        resolution,
                        floorGrid);

                if (floorCandidateSegments.Count == 0) {
                    CeilingAndFloorReconstruction
                        .FloorDetection
                        .RemoveRoomWithoutFloor(
                            roomId,
                            reconstructionGrid,
                            roomIdsWithoutFloor,
                            ceilingSegments);
                    continue;
                }

                CeilingAndFloorReconstruction
                    .FloorDetection
                    .SelectDominantFloorSegment(
                        floorIncidence,
                        floorGrid,
                        floorCandidateSegments,
                        out dominantFloorHeight,
                        out floorHeightRange);

                CeilingAndFloorReconstruction
                    .CeilingAndFloorFinalization
                    .InterpolateFloorHeight(
                        roomId,
                        dominantFloorHeight,
                        resolution,
                        floorHeightRange,
                        ceilingGridOffset,
                        floorGrid,
                        reconstructionGrid);

                CeilingAndFloorReconstruction
                    .HoleClosingDecision
                    .RemoveCeilingHoleOverlapsWithOtherRooms(
                        roomId,
                        ceilingGridOffset,
                        ceilingGrid,
                        floorGrid,
                        reconstructionGrid,
                        ref ceilingHoles);

                CeilingAndFloorReconstruction
                    .HoleClosingDecision
                    .CloseClosableCeilingHoles(
                        resolution,
                        ceilingGridOffset,
                        ceilingGrid,
                        floorGrid,
                        normalGrid,
                        ceilingHoles);

                VoxelClassification
                    .VoxelClassification
                    .InitializeWalls(
                        ceilingGrid,
                        floorGrid);

                VoxelClassification
                    .VoxelClassification
                    .UpdateReconstructionGrid(
                        roomId,
                        maxCeilingHeight,
                        ceilingGridOffset,
                        ceilingGrid,
                        floorGrid,
                        reconstructionGrid);

                VoxelClassification
                    .VoxelClassification
                    .DoVerticalVoxelClassificationSweep(
                        roomId,
                        ceilingGridOffset,
                        ceilingGrid,
                        floorGrid,
                        normalGrid,
                        reconstructionGrid,
                        roomIdsWithoutFloor);
            }

            return reconstructionGrid;
        }

        private static void CompleteVoxelModel(
                int roomCount,
                double resolution,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid,
                HashSet<int> roomIdsWithoutFloor,
                out HashSet<int> roomletIds) {

            VoxelClassification
                .VoxelModelCompletion
                .AssignRoomlessVoxelsToCeilingsAndFloors(
                    resolution,
                    normalGrid,
                    reconstructionGrid);

            VoxelClassification
                .VoxelModelCompletion
                .DetectMissingIndoorSpaces(
                    roomCount,
                    resolution,
                    normalGrid,
                    reconstructionGrid,
                    roomIdsWithoutFloor,
                    out roomletIds);
        }

        private static void RefineVoxelModel(
                double resolution,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid,
                HashSet<int> roomletIds) {

            VoxelModelRefinement
                .VoxelGeometryRefinement
                .AddMissingWalls(
                    resolution,
                    normalGrid,
                    reconstructionGrid);

            VoxelModelRefinement
                .VoxelGeometryRefinement
                .RefineCeilingAndFloorGeometry(
                    resolution,
                    reconstructionGrid);

            VoxelModelRefinement
                .VoxelGeometryRefinement
                .RefineWallGeometry(
                    resolution,
                    normalGrid,
                    reconstructionGrid);

            VoxelModelRefinement
                .WallOpeningRefinement
                .InitializeWallNormals(
                    normalGrid,
                    reconstructionGrid,
                    roomletIds);

            VoxelModelRefinement
                .WallOpeningRefinement
                .CloseHolesInWalls(
                    resolution,
                    reconstructionGrid);

            VoxelModelRefinement
                .WallOpeningRefinement
                .RefineWallOpenings(
                    resolution,
                    normalGrid,
                    reconstructionGrid);

            VoxelModelRefinement
                .WallOpeningRefinement
                .RemoveWallNormals(
                    reconstructionGrid);
        }

        private static void RepartitionIndoorSpace(
                double resolution,
                byte[,,] normalGrid,
                int[,,][] reconstructionGrid,
                out HashSet<int> rampSpaceIds) {

            bool rampSpacesDetected;
            int transitionSpaceCount;
            int[,,] slopeGrid;
            bool[,,] wallGrid;
            bool[,,] rampSpaceGrid;
            bool[,,] interiorSpaceGrid;
            int[,,] spacePartitioningGrid;

            RoomSegmentation
                .RoomMerging
                .MergeIndoorSpace(
                    resolution,
                    normalGrid,
                    reconstructionGrid);

            wallGrid = RoomSegmentation
                .RampSpaceDetection
                .InitializeWallGrid(
                    resolution,
                    reconstructionGrid);

            slopeGrid = RoomSegmentation
                .RampSpaceDetection
                .InitializeSlopeGrid(
                    resolution,
                    wallGrid,
                    reconstructionGrid);

            rampSpaceGrid = RoomSegmentation
                .RampSpaceDetection
                .InitializeRampSpaceGrid(
                    resolution,
                    wallGrid,
                    slopeGrid,
                    reconstructionGrid,
                    out rampSpacesDetected);

            if (rampSpacesDetected) {
                RoomSegmentation
                    .RampSpaceDetection
                    .RefineHorizontalRampSurfaces(
                        resolution,
                        wallGrid,
                        rampSpaceGrid,
                        normalGrid,
                        reconstructionGrid);
            }

            interiorSpaceGrid = RoomSegmentation
                .RoomPartitioning3D
                .InitializeInteriorSpaceGrid(
                    reconstructionGrid);

            spacePartitioningGrid = RoomSegmentation
                .RoomPartitioning3D
                .RepartitionInteriorSpace(
                    resolution,
                    wallGrid,
                    rampSpaceGrid,
                    interiorSpaceGrid,
                    reconstructionGrid,
                    out transitionSpaceCount,
                    out rampSpaceIds);

            RoomSegmentation
                .RoomPartitioning3D
                .RefineSpacePartitioning(
                    transitionSpaceCount,
                    resolution,
                    wallGrid,
                    normalGrid,
                    spacePartitioningGrid,
                    reconstructionGrid,
                    rampSpaceIds);

            RoomSegmentation
                .RoomPartitioning3D
                .ApplySpacePartitioningToReconstructionGrid(
                    spacePartitioningGrid,
                    reconstructionGrid);
        }
    }
}
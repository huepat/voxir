using HuePat.VoxIR.Datasets;
using HuePat.VoxIR.Evaluation.IO;
using HuePat.VoxIR.IO;
using HuePat.VoxIR.IO.PLY.Reading;
using HuePat.VoxIR.IO.PLY.Writing;
using HuePat.VoxIR.IO.Visualization;
using HuePat.VoxIR.Util.Geometry;
using HuePat.VoxIR.Util.Grid;
using OpenTK.Mathematics;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using Visualizer = HuePat.VoxIR.Evaluation.IO.Visualizer;

namespace HuePat.VoxIR.Evaluation {
    public static class Evaluation {
        private static readonly Vector3d UP_AXIS = new Vector3d(0.0, -1.0, 0.0);

        public static void Evaluate(
                bool redetermineNormalGrid,
                IList<(string, string)> testFilesWithGroundTruthDirectories,
                IList<(double, double)> resolutionAndHorizontalRotationValues,
                OutputConfig outputConfig) {

            string timestamp = Util.CreateTimestampLabel();

            foreach ((string, string) fileInfo in testFilesWithGroundTruthDirectories) {

                Evaluate(
                    redetermineNormalGrid,
                    fileInfo.Item1,
                    fileInfo.Item2,
                    resolutionAndHorizontalRotationValues,
                    outputConfig.CopyWithSubDirectory(
                        true,
                        $"{timestamp}_{Path.GetFileNameWithoutExtension(fileInfo.Item1)}_EvaluationResults"));
            }
        }

        public static void Evaluate(
                bool redetermineNormalGrid,
                string testFile,
                string groundTruthDirectory,
                IList<(double, double)> resolutionAndHorizontalRotationValues,
                OutputConfig outputConfig) {

            Util.Evaluate(
                testFile,
                outputConfig,
                resolutionAndHorizontalRotationValues,
                (resolution, rotationAroundHorizontalAxis, outputConfig) => {

                    return Evaluate(
                        redetermineNormalGrid,
                        testFile,
                        groundTruthDirectory,
                        resolution,
                        rotationAroundHorizontalAxis,
                        outputConfig);

                });
        }

        public static Result Evaluate(
                bool redetermineNormalGrid,
                string testFile,
                string groundTruthDirectory,
                double resolution,
                double rotationAngle,
                OutputConfig outputConfig) {

            bool[,,] occupancyGrid;
            byte[,,] normalGrid;
            int[,,][] reconstructionGrid;
            int[,,][] groundTruthGrid;
            AABox extent;
            Mesh testMesh;
            Result result;
            HashSet<int> testRampSpaceIds;
            HashSet<int> groundTruthRampSpaceIds;
            Dictionary<int, List<(Mesh, GroundTruthInfo)>> groundTruthMeshes;

            using (InvariantCultureBlock block = new InvariantCultureBlock()) {

                testMesh = new PLYReader().ReadMesh(testFile);
                groundTruthMeshes = GroundTruthReader.LoadGroundTruthMeshes(groundTruthDirectory);

                extent = GetExtent(
                    testMesh,
                    groundTruthMeshes);

                if (!rotationAngle.ApproximateEquals(0.0)) {
                    
                    Rotate(
                        rotationAngle,
                        ref extent,
                        testMesh,
                        groundTruthMeshes);

                    if (outputConfig.ExportPLY) {
                        using (PLYWriter writer = new PLYWriter($"{outputConfig.OutputDirectory}/RotatedTestMesh.ply")) {
                            writer.Write(
                                testMesh,
                                Color.Gray);
                        }
                    }
                }

                if (redetermineNormalGrid) {

                    normalGrid = DataPreparation.Voxelization.Voxelize(
                        resolution,
                        extent,
                        testMesh);

                    occupancyGrid = Util.GetOccupancyGrid(normalGrid);

                    normalGrid = HuePat.VoxIR.NormalGridDetermination
                        .NormalGridDetermination
                        .DetermineNormalGrid(
                            resolution,
                            occupancyGrid);

                    reconstructionGrid = VoxIR.Process(
                        resolution,
                        normalGrid,
                        out testRampSpaceIds);
                }
                else {
                    reconstructionGrid = VoxIR.Process(
                        resolution,
                        extent,
                        testMesh,
                        out testRampSpaceIds);
                }

                groundTruthRampSpaceIds = groundTruthMeshes
                    .Keys
                    .Where(roomId => groundTruthMeshes[roomId]
                        .Any(groundTruthMesh => groundTruthMesh.Item2.IsRampSpace))
                    .ToHashSet();

                groundTruthGrid = GroundTruth.CreateGroundTruthGrid(
                    resolution,
                    extent,
                    groundTruthMeshes);

                result = Evaluate(
                    reconstructionGrid,
                    groundTruthGrid,
                    outputConfig,
                    testRampSpaceIds,
                    groundTruthRampSpaceIds);
            }

            return result;
        }

        private static Result Evaluate(
                int[,,][] reconstructionGrid,
                int[,,][] groundTruthGrid,
                OutputConfig outputConfig,
                HashSet<int> reconstructionRampSpaceIds,
                HashSet<int> groundTruthRampSpaceIds) {

            int totalReconstructionVolume;
            int totalGroundTruthVolume;
            int reconstructionCorrectRoomMappingVolume;
            int groundTruthCorrectRoomMappingVolume;
            double roomMappingError;
            double roomSegmentationPrecision;
            double roomSegmentationRecall;
            double roomSegmentationF1Score;
            HashSet<int> reconstructionRoomMappingErrorRoomIds;
            HashSet<int> groundTruthRoomMappingErrorRoomIds;
            Dictionary<int, int> reconstructionToGroundTruthRoomMapping;
            Dictionary<int, int> groundTruthToReconstructionRoomMapping;
            Dictionary<int, double> voxelClassificationPrecision;
            Dictionary<int, double> voxelClassificationRecall;
            Dictionary<int, double> voxelClassificationNeighbourhoodPrecision;
            Dictionary<int, double> voxelClassificationNeighbourhoodRecall;
            Dictionary<int, double> voxelClassificationF1Score;
            Dictionary<int, Dictionary<int, int>> weightedReconstructionToGroundTruthRoomMapping;
            Dictionary<int, Dictionary<int, int>> weightedGroundTruthToReconstructionRoomMapping;

            if (outputConfig.ExportPLY) {

                Visualizer.ExportPLY(
                    reconstructionGrid,
                    groundTruthGrid,
                    outputConfig);
            }

            if (outputConfig.ExportImages) {

                Visualizer.ExportImages(
                    reconstructionGrid,
                    groundTruthGrid,
                    outputConfig);
            }

            totalReconstructionVolume = GetVolume(
                reconstructionGrid,
                voxelState => voxelState != null);
            totalGroundTruthVolume = GetVolume(
                groundTruthGrid,
                voxelState => voxelState != null);

            GetWeightedRoomMappings(
                reconstructionGrid,
                groundTruthGrid,
                out weightedReconstructionToGroundTruthRoomMapping,
                out weightedGroundTruthToReconstructionRoomMapping);

            if (outputConfig.ExportExtendedLogs) {

                TextOutput.WriteWeightedRoomMappings(
                    outputConfig.OutputDirectory,
                    reconstructionRampSpaceIds,
                    groundTruthRampSpaceIds,
                    weightedReconstructionToGroundTruthRoomMapping,
                    weightedGroundTruthToReconstructionRoomMapping);
            }

            EvaluateRoomMapping(
                totalReconstructionVolume,
                "Test to GroundTruth",
                reconstructionGrid,
                outputConfig,
                reconstructionRampSpaceIds,
                groundTruthRampSpaceIds,
                weightedReconstructionToGroundTruthRoomMapping,
                weightedGroundTruthToReconstructionRoomMapping,
                out reconstructionCorrectRoomMappingVolume,
                out reconstructionRoomMappingErrorRoomIds,
                out reconstructionToGroundTruthRoomMapping);

            EvaluateRoomMapping(
                totalGroundTruthVolume,
                "GroundTruth to Test",
                groundTruthGrid,
                outputConfig,
                groundTruthRampSpaceIds,
                reconstructionRampSpaceIds,
                weightedGroundTruthToReconstructionRoomMapping,
                weightedReconstructionToGroundTruthRoomMapping,
                out groundTruthCorrectRoomMappingVolume,
                out groundTruthRoomMappingErrorRoomIds,
                out groundTruthToReconstructionRoomMapping);

            EvaluateRoomSegmentation(
                reconstructionGrid,
                groundTruthGrid,
                outputConfig,
                reconstructionRampSpaceIds,
                reconstructionRoomMappingErrorRoomIds,
                groundTruthRoomMappingErrorRoomIds,
                reconstructionToGroundTruthRoomMapping,
                groundTruthToReconstructionRoomMapping,
                out roomSegmentationPrecision,
                out roomSegmentationRecall);

            EvaluateVoxelClassification(
                reconstructionGrid,
                groundTruthGrid,
                outputConfig,
                out voxelClassificationPrecision,
                out voxelClassificationRecall,
                out voxelClassificationNeighbourhoodPrecision,
                out voxelClassificationNeighbourhoodRecall);

            roomMappingError = GetRoomMappingError(
                totalReconstructionVolume,
                totalGroundTruthVolume,
                reconstructionCorrectRoomMappingVolume,
                groundTruthCorrectRoomMappingVolume);

            roomSegmentationF1Score = GetF1Score(
                roomSegmentationPrecision,
                roomSegmentationRecall);

            voxelClassificationF1Score = GetVoxelClassificationF1Score(
                voxelClassificationPrecision,
                voxelClassificationRecall);

            return new Result(
                roomMappingError,
                roomSegmentationPrecision,
                roomSegmentationRecall,
                roomSegmentationF1Score,
                voxelClassificationPrecision,
                voxelClassificationRecall,
                voxelClassificationF1Score,
                voxelClassificationNeighbourhoodPrecision,
                voxelClassificationNeighbourhoodRecall);
        }

        public static AABox GetExtent(
                Mesh testMesh,
                Dictionary<int, List<(Mesh, GroundTruthInfo)>> groundTruthMeshes) {

            AABox bBox;
            Vector3d min = testMesh.BBox.Min;
            Vector3d max = testMesh.BBox.Max;

            foreach ((Mesh, GroundTruthInfo) groundTruthMesh in groundTruthMeshes.UnwrapValues()) {
                
                bBox = groundTruthMesh.Item1.BBox;

                if (bBox.Min.X < min.X) {
                    min.X = bBox.Min.X;
                }
                if (bBox.Min.Y < min.Y) {
                    min.Y = bBox.Min.Y;
                }
                if (bBox.Min.Z < min.Z) {
                    min.Z = bBox.Min.Z;
                }

                if (bBox.Max.X > max.X) {
                    max.X = bBox.Max.X;
                }
                if (bBox.Max.Y > max.Y) {
                    max.Y = bBox.Max.Y;
                }
                if (bBox.Max.Z > max.Z) {
                    max.Z = bBox.Max.Z;
                }
            }

            return new AABox(
                min,
                max);
        }

        private static void Rotate(
                double rotationAngle,
                ref AABox extent,
                Mesh testMesh,
                Dictionary<int, List<(Mesh, GroundTruthInfo)>> groundTruthMeshes) {

            Vector3d anchor = (extent.Min + extent.Max) / 2.0;

            testMesh.Rotate(
                rotationAngle,
                UP_AXIS,
                anchor);

            foreach ((Mesh, GroundTruthInfo) groundTruthMesh in groundTruthMeshes.UnwrapValues()) {
                groundTruthMesh
                    .Item1
                    .Rotate(
                        rotationAngle,
                        UP_AXIS,
                        anchor);
            }

            extent = GetExtent(
                testMesh,
                groundTruthMeshes);
        }

        private static int GetVolume(
                int[,,][] grid,
                Predicate<int[]> voxelFilter) {

            object @lock = new object();
            int volume = 0;

            Parallel.ForEach(
                Partitioner.Create(
                    0,
                    grid.GetLength(0)),
                () => 0,
                (loopState, partition, localVolume) => {

                    int i, r, c;

                    for (i = 0; i < grid.GetLength(0); i++) {
                        for (r = 0; r < grid.GetLength(1); r++) {
                            for (c = 0; c < grid.GetLength(2); c++) {
                                if (voxelFilter(grid[i, r, c])) {
                                    localVolume++;
                                }
                            }
                        }
                    }

                    return localVolume;
                },
                localVolume => {

                    lock (@lock) {
                        volume += localVolume;
                    }
                });

            return volume;
        }

        private static void GetWeightedRoomMappings(
                int[,,][] reconstructionGrid,
                int[,,][] groundTruthGrid,
                out Dictionary<int, Dictionary<int, int>> weightedReconstructionToGroundTruthRoomMapping,
                out Dictionary<int, Dictionary<int, int>> weightedGroundTruthToReconstructionRoomMapping) {

            object @lock = new object();
            Dictionary<int, Dictionary<int, int>> _weightedReconstructionToGroundTruthRoomMapping 
                = new Dictionary<int, Dictionary<int, int>>();
            Dictionary<int, Dictionary<int, int>> _weightedGroundTruthToReconstructionRoomMapping 
                = new Dictionary<int, Dictionary<int, int>>();

            Parallel.ForEach(
                Partitioner.Create(
                    0,
                    reconstructionGrid.GetLength(0)),
                () => (
                    new Dictionary<int, Dictionary<int, int>>(),
                    new Dictionary<int, Dictionary<int, int>>()
                ),
                (partition, loopState, localRoomMappings) => {

                    for (int i = partition.Item1; i < partition.Item2; i++) {

                        GetWeightedRoomMappings(
                            i,
                            reconstructionGrid,
                            groundTruthGrid,
                            localRoomMappings.Item1,
                            localRoomMappings.Item2);
                    }

                    return localRoomMappings;
                },
                localRoomMappings => {

                    lock (@lock) {
                        _weightedReconstructionToGroundTruthRoomMapping.Update(localRoomMappings.Item1);
                        _weightedGroundTruthToReconstructionRoomMapping.Update(localRoomMappings.Item2);
                    }
                });

            weightedReconstructionToGroundTruthRoomMapping = _weightedReconstructionToGroundTruthRoomMapping;
            weightedGroundTruthToReconstructionRoomMapping = _weightedGroundTruthToReconstructionRoomMapping;
        }

        private static void GetWeightedRoomMappings(
                int i,
                int[,,][] reconstructionGrid,
                int[,,][] groundTruthGrid,
                Dictionary<int, Dictionary<int, int>> weightedReconstructionToGroundTruthRoomMapping,
                Dictionary<int, Dictionary<int, int>> weightedGroundTruthToReconstructionRoomMapping) {

            int r, c;
            int[] reconstructionRoomIds;
            int[] groundTruthRoomIds;

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                    reconstructionRoomIds = reconstructionGrid[i, r, c].GetRoomIds();
                    groundTruthRoomIds = groundTruthGrid[i, r, c].GetRoomIds();

                    weightedReconstructionToGroundTruthRoomMapping.Update(
                        reconstructionRoomIds,
                        groundTruthRoomIds);
                    weightedGroundTruthToReconstructionRoomMapping.Update(
                        groundTruthRoomIds,
                        reconstructionRoomIds);
                }
            }
        }

        private static void EvaluateRoomMapping(
                int totalVolume,
                string title,
                int[,,][] fromGrid,
                OutputConfig outputConfig,
                HashSet<int> fromRampSpaceIds,
                HashSet<int> toRampSpaceIds,
                Dictionary<int, Dictionary<int, int>> weightedFromRoomMapping,
                Dictionary<int, Dictionary<int, int>> weightedToRoomMapping,
                out int correctRoomMappingVolume,
                out HashSet<int> roomMappingErrorRoomIds,
                out Dictionary<int, int> fromRoomMapping) {

            int? toRoomId, backMappingId;
            List<int> rampSpaceErrorRoomIds = new List<int>();

            correctRoomMappingVolume = 0;
            roomMappingErrorRoomIds = new HashSet<int>();
            fromRoomMapping = new Dictionary<int, int>();

            foreach (int fromRoomId in weightedFromRoomMapping.Keys) {

                toRoomId = weightedFromRoomMapping.Get(fromRoomId);
                if (!toRoomId.HasValue) {
                    roomMappingErrorRoomIds.Add(fromRoomId);
                    continue;
                }

                backMappingId = weightedToRoomMapping.Get(toRoomId.Value);
                if (!backMappingId.HasValue
                        || backMappingId.Value != fromRoomId) {
                    roomMappingErrorRoomIds.Add(fromRoomId);
                    continue;
                }

                fromRoomMapping.Add(
                    fromRoomId,
                    toRoomId.Value);

                if (fromRampSpaceIds.Contains(fromRoomId) != toRampSpaceIds.Contains(toRoomId.Value)) {
                    rampSpaceErrorRoomIds.Add(fromRoomId);
                }
            }

            foreach (int roomId in roomMappingErrorRoomIds) {
                correctRoomMappingVolume += GetVolume(
                    fromGrid,
                    voxelState => voxelState != null
                        && voxelState
                            .GetRoomIds()
                            .Contains(roomId));
            }

            if (outputConfig.ExportExtendedLogs) {

                TextOutput.WriteRoomMappingEvaluationResults(
                    totalVolume,
                    correctRoomMappingVolume,
                    outputConfig.OutputDirectory,
                    title,
                    fromRampSpaceIds,
                    roomMappingErrorRoomIds,
                    rampSpaceErrorRoomIds);
            }
        }

        private static void EvaluateRoomSegmentation(
                int[,,][] reconstructionGrid,
                int[,,][] groundTruthGrid,
                OutputConfig outputConfig,
                HashSet<int> reconstructionRampSpaceIds,
                HashSet<int> reconstructionRoomMappingErrorIds,
                HashSet<int> groundTruthRoomMappingErrorIds,
                Dictionary<int, int> reconstructionToGroundTruthRoomMapping,
                Dictionary<int, int> groundTruthToReconstructionRoomMapping,
                out double precision,
                out double recall) {

            object @lock = new object();
            Color backgroundColor = outputConfig.BackgroundColor == BackgroundColor.BLACK ?
                Color.Black :
                Color.White;
            Dictionary<int, int> reconstructionVolume = new Dictionary<int, int>();
            Dictionary<int, int> groundTruthVolume = new Dictionary<int, int>();
            Dictionary<int, int> truePositiveVolume = new Dictionary<int, int>();
            Dictionary<int, PLYWriter> falsePositivePLYWriters = null;
            Dictionary<int, PLYWriter> falseNegativePLYWriters = null;

            if (outputConfig.ExportPLY) {
                falsePositivePLYWriters = new Dictionary<int, PLYWriter>();
                falseNegativePLYWriters = new Dictionary<int, PLYWriter>();
            }

            Parallel.ForEach(
                Partitioner.Create(
                    0,
                    reconstructionGrid.GetLength(0)),
                () => (
                    new Dictionary<int, int>(),
                    new Dictionary<int, int>(),
                    new Dictionary<int, int>(),
                    new Dictionary<int, List<(int, int, int)>>(),
                    new Dictionary<int, List<(int, int, int)>>()
                ),
                (partition, loopState, localData) => {

                    for (int i = partition.Item1; i < partition.Item2; i++) {

                        EvaluateRoomSegmentation(
                            i,
                            reconstructionGrid,
                            groundTruthGrid,
                            backgroundColor,
                            outputConfig,
                            reconstructionRoomMappingErrorIds,
                            groundTruthRoomMappingErrorIds,
                            reconstructionToGroundTruthRoomMapping,
                            groundTruthToReconstructionRoomMapping,
                            localData.Item1,
                            localData.Item2,
                            localData.Item3,
                            localData.Item4,
                            localData.Item5);
                    }

                    return localData;
                },
                localData => {

                    lock (@lock) {

                        reconstructionVolume.BucketAdd(localData.Item1);
                        groundTruthVolume.BucketAdd(localData.Item2);
                        truePositiveVolume.BucketAdd(localData.Item3);

                        if (outputConfig.ExportPLY) {

                            falsePositivePLYWriters.Write(
                                    Definitions.EvaluationErrorColors.FALSE_POSITIVE_COLOR,
                                    outputConfig,
                                    localData.Item4,
                                    roomId => $"RoomSegmentation_FalsePositives_{TextOutput.FormatRoomLabel(roomId, reconstructionRampSpaceIds)}");

                            falseNegativePLYWriters.Write(
                                    Definitions.EvaluationErrorColors.FALSE_NEGATIVE_COLOR,
                                    outputConfig,
                                    localData.Item5,
                                    roomId => $"RoomSegmentation_FalseNegatives_{TextOutput.FormatRoomLabel(roomId, reconstructionRampSpaceIds)}");
                        }
                    }
                });

            precision = 100.0 * truePositiveVolume.Values.Sum() / reconstructionVolume.Values.Sum();
            recall = 100.0 * truePositiveVolume.Values.Sum() / groundTruthVolume.Values.Sum();

            if (outputConfig.ExportPLY) {
                falsePositivePLYWriters.Dispose();
                falseNegativePLYWriters.Dispose();
            }

            if (outputConfig.ExportExtendedLogs) {

                TextOutput.WriteRoomSegmentationEvaluationResults(
                    outputConfig.OutputDirectory,
                    precision,
                    recall,
                    reconstructionRampSpaceIds,
                    reconstructionRoomMappingErrorIds,
                    truePositiveVolume,
                    reconstructionVolume,
                    groundTruthVolume,
                    reconstructionToGroundTruthRoomMapping);
            }
        }

        private static void EvaluateRoomSegmentation(
                int i,
                int[,,][] reconstructionGrid,
                int[,,][] groundTruthGrid,
                Color backgroundColor,
                OutputConfig outputConfig,
                HashSet<int> reconstructionRoomMappingErrorIds,
                HashSet<int> groundTruthRoomMappingErrorIds,
                Dictionary<int, int> reconstructionToGroundTruthRoomMapping,
                Dictionary<int, int> groundTruthToReconstructionRoomMapping,
                Dictionary<int, int> reconstructionVolume,
                Dictionary<int, int> groundTruthVolume,
                Dictionary<int, int> truePositiveVolume,
                Dictionary<int, List<(int, int, int)>> falsePositiveVoxels,
                Dictionary<int, List<(int, int, int)>> falseNegativeVoxels) {

            bool foundFalsePositive;
            bool foundFalseNegative;
            int r, c;
            ImageWriter imageWriter = null;

            if (outputConfig.ExportImages) {
                imageWriter = new ImageWriter(
                    reconstructionGrid.GetLength(1),
                    reconstructionGrid.GetLength(2),
                    $"{outputConfig.OutputDirectory}/RoomSegmentation_EvaluationResults_i{i}.{outputConfig.OutputImageFileType}");
            }

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                    foundFalsePositive = foundFalseNegative = false;

                    foreach (int reconstructionRoomId in reconstructionGrid[i, r, c].GetRoomIds()) {

                        if (reconstructionRoomMappingErrorIds.Contains(reconstructionRoomId)) {
                            continue;
                        }

                        reconstructionVolume.BucketIncrement(reconstructionRoomId);

                        if (groundTruthGrid[i, r, c]
                                .GetRoomIds()
                                .Contains(reconstructionToGroundTruthRoomMapping[reconstructionRoomId])) {

                            truePositiveVolume.BucketIncrement(reconstructionRoomId);
                        }
                        else {

                            foundFalsePositive = true;
                            if (outputConfig.ExportPLY) {

                                falsePositiveVoxels.BucketAdd(
                                    reconstructionRoomId,
                                    (i, r, c));   
                            }
                        }
                    }

                    foreach (int groundTruthRoomId in groundTruthGrid[i, r, c].GetRoomIds()) {

                        if (groundTruthRoomMappingErrorIds.Contains(groundTruthRoomId)) {
                            continue;
                        }

                        groundTruthVolume.BucketIncrement(groundTruthToReconstructionRoomMapping[groundTruthRoomId]);

                        if (!reconstructionGrid[i, r, c]
                                .GetRoomIds()
                                .Contains(groundTruthToReconstructionRoomMapping[groundTruthRoomId])) {

                            foundFalseNegative = true;
                            if (outputConfig.ExportPLY) {

                                falseNegativeVoxels.BucketAdd(
                                    groundTruthToReconstructionRoomMapping[groundTruthRoomId],
                                    (i, r, c));
                            }
                        }
                    }

                    if (outputConfig.ExportImages) {

                        imageWriter.Write(
                            foundFalsePositive,
                            foundFalseNegative,
                            r,
                            c,
                            reconstructionGrid[i, r, c].GetVoxelClassValues(),
                            backgroundColor);
                    }
                }
            }

            if (outputConfig.ExportImages) {
                imageWriter.Dispose();
            }
        }

        private static void EvaluateVoxelClassification(
                int[,,][] reconstructionGrid,
                int[,,][] groundTruthGrid,
                OutputConfig outputConfig,
                out Dictionary<int, double> precision,
                out Dictionary<int, double> recall,
                out Dictionary<int, double> neighbourhoodPrecision,
                out Dictionary<int, double> neighbourhoodRecall) {

            object @lock = new object();
            double reconstructionVolumForVoxelClassSubset;
            Color backgroundColor = outputConfig.BackgroundColor == BackgroundColor.BLACK ?
                Color.Black :
                Color.White;
            Dictionary<int, int> reconstructionVolume = new Dictionary<int, int>();
            Dictionary<int, int> groundTruthVolume = new Dictionary<int, int>();
            Dictionary<int, int> truePositiveVolume = new Dictionary<int, int>();
            Dictionary<int, int> truePositiveVolumeForNeighbourhood = new Dictionary<int, int>();
            Dictionary<int, int> recallVolumeForNeighbourhood = new Dictionary<int, int>();
            Dictionary<int, PLYWriter> falsePositivePLYWriters = null;
            Dictionary<int, PLYWriter> falseNegativePLYWriters = null;
            Dictionary<int, PLYWriter> falsePositivePLYWritersForNeighbourhood = null;
            Dictionary<int, PLYWriter> falseNegativePLYWritersForNeighbourhood = null;

            precision = new Dictionary<int, double>();
            recall = new Dictionary<int, double>();
            neighbourhoodPrecision = new Dictionary<int, double>();
            neighbourhoodRecall = new Dictionary<int, double>();

            if (outputConfig.ExportPLY) {
                falsePositivePLYWriters = new Dictionary<int, PLYWriter>();
                falseNegativePLYWriters = new Dictionary<int, PLYWriter>();
                falsePositivePLYWritersForNeighbourhood = new Dictionary<int, PLYWriter>();
                falseNegativePLYWritersForNeighbourhood = new Dictionary<int, PLYWriter>();
            }

            Parallel.ForEach(
                Partitioner.Create(
                    0,
                    reconstructionGrid.GetLength(0)),
                () => (
                    new Dictionary<int, int>(),
                    new Dictionary<int, int>(),
                    new Dictionary<int, int>(),
                    new Dictionary<int, int>(),
                    new Dictionary<int, int>(),
                    new Dictionary<int, List<(int, int, int)>>(),
                    new Dictionary<int, List<(int, int, int)>>(),
                    new Dictionary<int, List<(int, int, int)>>(),
                    new Dictionary<int, List<(int, int, int)>>()
                ),
                (partition, loopState, localData) => {

                    for (int i = partition.Item1; i < partition.Item2; i++) {

                        EvaluateVoxelClassification(
                            i,
                            reconstructionGrid,
                            groundTruthGrid,
                            backgroundColor,
                            outputConfig,
                            localData.Item1,
                            localData.Item2,
                            localData.Item3,
                            localData.Item4,
                            localData.Item5,
                            localData.Item6,
                            localData.Item7,
                            localData.Item8,
                            localData.Item9);
                    }

                    return localData;
                },
                localData => {

                    lock (@lock) {

                        reconstructionVolume.BucketAdd(localData.Item1);
                        groundTruthVolume.BucketAdd(localData.Item2);
                        truePositiveVolume.BucketAdd(localData.Item3);
                        truePositiveVolumeForNeighbourhood.BucketAdd(localData.Item4);
                        recallVolumeForNeighbourhood.BucketAdd(localData.Item5);

                        if (outputConfig.ExportPLY) {

                            falsePositivePLYWriters.Write(
                                    Definitions.EvaluationErrorColors.FALSE_POSITIVE_COLOR,
                                    outputConfig,
                                    localData.Item6,
                                    voxelClassValue => $"VoxelClassification_FalsePositives_{VoxelClassValues.Labels[voxelClassValue]}");

                            falseNegativePLYWriters.Write(
                                    Definitions.EvaluationErrorColors.FALSE_NEGATIVE_COLOR,
                                    outputConfig,
                                    localData.Item7,
                                    voxelClassValue => $"VoxelClassification_FalseNegatives_{VoxelClassValues.Labels[voxelClassValue]}");

                            falsePositivePLYWritersForNeighbourhood.Write(
                                    Definitions.EvaluationErrorColors.FALSE_POSITIVE_COLOR,
                                    outputConfig,
                                    localData.Item8,
                                    voxelClassValue => $"VoxelClassification_FalsePositives_Neighbourhood_{VoxelClassValues.Labels[voxelClassValue]}");

                            falseNegativePLYWritersForNeighbourhood.Write(
                                    Definitions.EvaluationErrorColors.FALSE_NEGATIVE_COLOR,
                                    outputConfig,
                                    localData.Item9,
                                    voxelClassValue => $"VoxelClassification_FalseNegatives_Neighbourhood_{VoxelClassValues.Labels[voxelClassValue]}");
                        }
                    }
                });

            foreach (int voxelClassValue in reconstructionVolume.Keys.Order()) {
                if (!truePositiveVolume.ContainsKey(voxelClassValue)) {
                    truePositiveVolume.Add(
                        voxelClassValue, 
                        0);
                }
                if (!truePositiveVolumeForNeighbourhood.ContainsKey(voxelClassValue)) {
                    truePositiveVolumeForNeighbourhood.Add(
                        voxelClassValue, 
                        0);
                }
            }

            foreach (int voxelClassValue in reconstructionVolume.Keys.Order()) {
                precision.Add(
                    voxelClassValue,
                    100.0 * truePositiveVolume.Get(voxelClassValue) / reconstructionVolume.Get(voxelClassValue));
                recall.Add(
                    voxelClassValue,
                    100.0 * truePositiveVolume.Get(voxelClassValue) / groundTruthVolume.Get(voxelClassValue));
                neighbourhoodPrecision.Add(
                    voxelClassValue,
                    100.0 * truePositiveVolumeForNeighbourhood.Get(voxelClassValue) / reconstructionVolume.Get(voxelClassValue));
                neighbourhoodRecall.Add(
                    voxelClassValue,
                    100.0 * recallVolumeForNeighbourhood.Get(voxelClassValue) / groundTruthVolume.Get(voxelClassValue));
            }

            if (outputConfig.ExportPLY) {
                falsePositivePLYWriters.Dispose();
                falseNegativePLYWriters.Dispose();
                falsePositivePLYWritersForNeighbourhood.Dispose();
                falseNegativePLYWritersForNeighbourhood.Dispose();
            }

            if (outputConfig.ExportExtendedLogs) {

                reconstructionVolumForVoxelClassSubset = reconstructionVolume.Keys
                    .Where(voxelClassVoxel => voxelClassVoxel != VoxelClassValues.EMPTY_INTERIOR
                        && voxelClassVoxel != VoxelClassValues.WALL_OPENING)
                    .Select(voxelClassValue => reconstructionVolume[voxelClassValue])
                    .Sum();

                TextOutput.WriteVoxelClassificationClassDistribution(
                    outputConfig.OutputDirectory,
                    reconstructionVolume,
                    groundTruthVolume);

                TextOutput.WriteVoxelClassificationEvaluationResults(
                    outputConfig.OutputDirectory,
                    reconstructionVolumForVoxelClassSubset,
                    reconstructionVolume,
                    precision,
                    recall,
                    neighbourhoodPrecision,
                    neighbourhoodRecall);
            }
        }

        private static int Get(
                this Dictionary<int, int> volume,
                int key) {

            return volume.ContainsKey(key) ?
                volume[key] :
                0;
        }

        private static void EvaluateVoxelClassification(
                int i,
                int[,,][] reconstructionGrid,
                int[,,][] groundTruthGrid,
                Color backgroundColor,
                OutputConfig outputConfig,
                Dictionary<int, int> reconstructionVolume,
                Dictionary<int, int> groundTruthVolume,
                Dictionary<int, int> truePositiveVolume,
                Dictionary<int, int> truePositiveVolumeForNeighbourhood,
                Dictionary<int, int> recallVolumeForNeighbourhood,
                Dictionary<int, List<(int, int, int)>> falsePositiveVoxels,
                Dictionary<int, List<(int, int, int)>> falseNegativeVoxels,
                Dictionary<int, List<(int, int, int)>> falsePositiveVoxelsForNeighbourhood,
                Dictionary<int, List<(int, int, int)>> falseNegativeVoxelsForNeighbourhood) {

            int r, c;
            bool foundFalsePositive;
            bool foundFalseNegative;
            bool foundNeighbourhoodFalsePositive;
            bool foundNeighbourhoodFalseNegative;
            int[] reconstructionVoxelClassValues;
            int[] groundTruthVoxelClassValues;
            ImageWriter imageWriter = null;
            ImageWriter neighbourhoodImageWriter = null;
            List<int> reconstructionVoxelClassValuesInNeighbourhood;
            List<int> groundTruthVoxelClassValuesInNeighbourhood;

            if (outputConfig.ExportImages) {

                imageWriter = new ImageWriter(
                    reconstructionGrid.GetLength(1),
                    reconstructionGrid.GetLength(2),
                    $"{outputConfig.OutputDirectory}/VoxelClassification_EvaluationResults_i{i}.{outputConfig.OutputImageFileType}");

                neighbourhoodImageWriter = new ImageWriter(
                    reconstructionGrid.GetLength(1),
                    reconstructionGrid.GetLength(2),
                    $"{outputConfig.OutputDirectory}/VoxelClassification_EvaluationResults_Neighbourhood_i{i}.{outputConfig.OutputImageFileType}");
            }

            for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                    foundFalsePositive = foundNeighbourhoodFalsePositive = false;
                    foundFalseNegative = foundNeighbourhoodFalseNegative = false;

                    reconstructionVoxelClassValues = reconstructionGrid[i, r, c].GetVoxelClassValues();
                    groundTruthVoxelClassValues = groundTruthGrid[i, r, c].GetVoxelClassValues();

                    GetVoxelClassValuesInNeighbourhood(
                        i,
                        r,
                        c,
                        reconstructionGrid,
                        groundTruthGrid,
                        out reconstructionVoxelClassValuesInNeighbourhood,
                        out groundTruthVoxelClassValuesInNeighbourhood);

                    foreach (int reconstructionVoxelClassValue in reconstructionVoxelClassValues) {

                        reconstructionVolume.BucketIncrement(reconstructionVoxelClassValue);

                        if (reconstructionVoxelClassValue == VoxelClassValues.WALL_OPENING
                                && groundTruthVoxelClassValues.Length != 1) {
                            continue;
                        }

                        if (groundTruthVoxelClassValues.Contains(reconstructionVoxelClassValue)) {
                            truePositiveVolume.BucketIncrement(reconstructionVoxelClassValue);
                        }
                        else {

                            foundFalsePositive = true;
                            if (outputConfig.ExportPLY) {

                                falsePositiveVoxels.BucketAdd(
                                    reconstructionVoxelClassValue,
                                    (i, r, c));
                            }

                        }

                        if (groundTruthVoxelClassValuesInNeighbourhood.Contains(reconstructionVoxelClassValue)) {
                            truePositiveVolumeForNeighbourhood.BucketIncrement(reconstructionVoxelClassValue);
                        }
                        else {

                            foundNeighbourhoodFalsePositive = true;
                            if (outputConfig.ExportPLY) {

                                falsePositiveVoxelsForNeighbourhood.BucketAdd(
                                    reconstructionVoxelClassValue,
                                    (i, r, c));
                            }

                        }
                    }

                    foreach (int groundTruthVoxelClassValue in groundTruthVoxelClassValues) {

                        if (groundTruthVoxelClassValue == VoxelClassValues.WALL_OPENING
                                && groundTruthVoxelClassValues.Length != 1) {
                            continue;
                        }

                        groundTruthVolume.BucketIncrement(groundTruthVoxelClassValue);

                        if (!reconstructionVoxelClassValues.Contains(groundTruthVoxelClassValue)) {

                            foundFalseNegative = true;
                            if (outputConfig.ExportPLY) {

                                falseNegativeVoxels.BucketAdd(
                                    groundTruthVoxelClassValue,
                                    (i, r, c));
                            }
                        }

                        if (!reconstructionVoxelClassValuesInNeighbourhood.Contains(groundTruthVoxelClassValue)) {
                            foundNeighbourhoodFalseNegative = true;
                            if (outputConfig.ExportPLY) {

                                falseNegativeVoxelsForNeighbourhood.BucketAdd(
                                    groundTruthVoxelClassValue,
                                    (i, r, c));
                            }
                        }
                        else {
                            recallVolumeForNeighbourhood.BucketIncrement(groundTruthVoxelClassValue);
                        }
                    }

                    if (outputConfig.ExportImages) {

                        imageWriter.Write(
                            foundFalsePositive,
                            foundFalseNegative,
                            r,
                            c,
                            reconstructionVoxelClassValues,
                            backgroundColor);

                        neighbourhoodImageWriter.Write(
                            foundNeighbourhoodFalsePositive,
                            foundNeighbourhoodFalseNegative,
                            r,
                            c,
                            reconstructionVoxelClassValues,
                            backgroundColor);
                    }
                }
            }

            if (outputConfig.ExportImages) {
                imageWriter.Dispose();
                neighbourhoodImageWriter.Dispose();
            }
        }

        private static void GetVoxelClassValuesInNeighbourhood(
                int i, 
                int r,
                int c,
                int[,,][] reconstructionGrid,
                int[,,][] groundTruthGrid,
                out List<int> reconstructionVoxelClassValuesInNeighbourhood,
                out List<int> groundTruthVoxelClassValuesInNeighbourhood) {

            int di, i2, dr, r2, dc, c2;

            reconstructionVoxelClassValuesInNeighbourhood = new List<int>();
            groundTruthVoxelClassValuesInNeighbourhood = new List<int>();

            for (di = -1; di <= 1; di++) {
                for (dr = -1; dr <= 1; dr++) {
                    for (dc = -1; dc <= 1; dc++) {

                        if (di.Abs() + dr.Abs() + dc.Abs() > 1) {
                            continue;
                        }

                        i2 = i + di;
                        r2 = r + dr;
                        c2 = c + dc;
                        if (i2 >= 0 && r2 >= 0 && c2 >= 0
                                && i2 < reconstructionGrid.GetLength(0)
                                && r2 < reconstructionGrid.GetLength(1)
                                && c2 < reconstructionGrid.GetLength(2)) {

                            reconstructionVoxelClassValuesInNeighbourhood.AddRange(
                                reconstructionGrid[i2, r2, c2].GetVoxelClassValues());
                            groundTruthVoxelClassValuesInNeighbourhood.AddRange(
                                groundTruthGrid[i2, r2, c2].GetVoxelClassValues());
                        }
                    }
                }
            }

            reconstructionVoxelClassValuesInNeighbourhood = reconstructionVoxelClassValuesInNeighbourhood
                .Distinct()
                .ToList();
            groundTruthVoxelClassValuesInNeighbourhood = groundTruthVoxelClassValuesInNeighbourhood
                .Distinct()
                .ToList();
        }

        private static double GetRoomMappingError(
                int totalReconstructionVolume,
                int totalGroundTruthVolume,
                int reconstructionCorrectRoomMappingVolume,
                int groundTruthCorrectRoomMappingVolume) {

            return Math.Max(
                100.0 * reconstructionCorrectRoomMappingVolume / totalReconstructionVolume,
                100.0 * groundTruthCorrectRoomMappingVolume / totalGroundTruthVolume);
        }

        private static Dictionary<int, double> GetVoxelClassificationF1Score(
                Dictionary<int, double> voxelClassificationPrecision,
                Dictionary<int, double> voxelClassificationRecall) {

            Dictionary<int, double> voxelClassificationF1Score = new Dictionary<int, double>();

            foreach (int voxelClassValue in voxelClassificationPrecision.Keys) {
                if (voxelClassificationRecall.ContainsKey(voxelClassValue)) {
                    voxelClassificationF1Score.Add(
                        voxelClassValue,
                        GetF1Score(
                            voxelClassificationPrecision[voxelClassValue],
                            voxelClassificationRecall[voxelClassValue]));
                }
            }

            return voxelClassificationF1Score;
        }

        private static double GetF1Score(
                double precision,
                double recall) {

            double f1Score = 2.0 * precision * recall / (precision + recall);

            return double.IsNaN(f1Score) ?
                0.0 :
                f1Score;
        }
    }
}
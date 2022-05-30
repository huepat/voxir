using HuePat.VoxIR.Datasets;
using HuePat.VoxIR.IO;
using HuePat.VoxIR.IO.ISPRS;
using HuePat.VoxIR.IO.PLY.Reading;
using HuePat.VoxIR.IO.PLY.Writing;
using HuePat.VoxIR.IO.Visualization;
using HuePat.VoxIR.Util.Geometry;
using HuePat.VoxIR.Util.Grid;
using OpenTK.Mathematics;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Threading.Tasks;

namespace HuePat.VoxIR.Evaluation.ISPRS {
    public enum InputType {
        POINT_CLOUD,
        MESH
    }

    public static class Evaluation {
        public static void Evaluate(
                bool exportPLY,
                bool redetermineMeshNormals,
                InputType inputType,
                IList<double> resolutions,
                IList<double> maxBufferSizes,
                IList<string> files,
                IList<string> groundTruthDirectories,
                string outputDirectory) {

            string fileName;

            if (resolutions.Count != maxBufferSizes.Count
                    || resolutions.Count != files.Count
                    || resolutions.Count != groundTruthDirectories.Count) {

                throw new ArgumentException(
                    "Input needs to have equal number of resolutions, max buffer sizes, input files and ground truth directories");
            }

            if (!Directory.Exists(outputDirectory)) {
                Directory.CreateDirectory(outputDirectory);
            }

            outputDirectory = $"{outputDirectory}/{Util.CreateTimestampLabel()}";
            Directory.CreateDirectory(outputDirectory);

            for (int j = 0; j < files.Count; j++) {

                fileName = Path.GetFileNameWithoutExtension(files[j]);

                using (InvariantCultureBlock block = new InvariantCultureBlock()) {
                    Log($"Evaluating {fileName} [{resolutions[j]:0.00}m Resolution]");
                }

                if (inputType == InputType.POINT_CLOUD) {

                    EvaluatePointCloud(
                        exportPLY,
                        resolutions[j],
                        maxBufferSizes[j],
                        files[j],
                        groundTruthDirectories[j],
                        $"{outputDirectory}/{fileName}");
                }
                else {
                    EvaluateMesh(
                        exportPLY,
                        redetermineMeshNormals,
                        resolutions[j],
                        maxBufferSizes[j],
                        files[j],
                        groundTruthDirectories[j],
                        $"{outputDirectory}/{fileName}");
                }

                Log("");
            }
        }

        public static void EvaluatePointCloud(
                bool exportPLY,
                double resolution,
                double maxBufferSize,
                string file,
                string groundTruthDirectory,
                string outputDirectory) {

            (int, int, int) gridSize;
            bool[,,] occupancyGrid;
            int[,,][] resultGrid;
            int[,,][] groundTruthGrid;
            AABox extent;
            Vector3d centroid;
            PointCloud mesh;
            Dictionary<int, List<Mesh>> groundTruth;

            if (!Directory.Exists(outputDirectory)) {
                Directory.CreateDirectory(outputDirectory);
            }

            mesh = ISPRSPointCloudReader.Read(
                file,
                false);

            centroid = mesh.GetCentroid();

            ISPRSPointCloudReader.Rotate(
                centroid,
                mesh);

            groundTruth = GroundTruthReader.ReadGroundTruthMeshes(
                groundTruthDirectory,
                centroid,
                out extent);

            extent = GetCommonBBox(
                extent,
                mesh.BBox);

            gridSize = Voxelizer.DetermineGridSize(
                resolution,
                ref extent);

            occupancyGrid = Voxelizer.VoxelizePointCloud(
                resolution,
                gridSize,
                extent.Min,
                mesh);

            resultGrid = VoxIR.Process(
                resolution,
                occupancyGrid,
                out _);

            groundTruthGrid = GroundTruthVoxelizer.Voxelize(
                resolution,
                gridSize,
                extent.Min,
                groundTruth);

            if (exportPLY) {

                PostProcessResultGridForVisualization(resultGrid);

                Visualizer.VisualizeVoxelClassificationAsPLY(
                    $"{outputDirectory}/result.ply",
                    resultGrid);

                Visualizer.VisualizeVoxelClassificationAsPLY(
                    $"{outputDirectory}/reference.ply",
                    groundTruthGrid);
            }

            Evaluate(
                resolution,
                maxBufferSize,
                $"{outputDirectory}/evaluation.csv",
                resultGrid.ToOccupancyGrid(),
                groundTruthGrid.ToOccupancyGrid());
        }

        public static void EvaluateMesh(
                bool exportPLY,
                bool redetermineMeshNormals,
                double resolution,
                double maxBufferSize,
                string file,
                string groundTruthDirectory,
                string outputDirectory) {

            bool[,,] occupancyGrid;
            byte[,,] normalGrid;
            int[,,][] resultGrid;
            int[,,][] groundTruthGrid;
            AABox extent;
            Mesh mesh;
            Dictionary<int, List<(Mesh, GroundTruthInfo)>> groundTruth;

            using (InvariantCultureBlock block = new InvariantCultureBlock()) {

                if (!Directory.Exists(outputDirectory)) {
                    Directory.CreateDirectory(outputDirectory);
                }

                mesh = new PLYReader().ReadMesh(file);
                groundTruth = Datasets.GroundTruthReader.LoadGroundTruthMeshes(groundTruthDirectory);

                extent = HuePat.VoxIR.Evaluation.Evaluation.GetExtent(
                    mesh,
                    groundTruth);

                if (redetermineMeshNormals) {

                    normalGrid = DataPreparation.Voxelization.Voxelize(
                        resolution,
                        extent,
                        mesh);

                    occupancyGrid = Util.GetOccupancyGrid(normalGrid);

                    normalGrid = HuePat.VoxIR.NormalGridDetermination
                        .NormalGridDetermination
                        .DetermineNormalGrid(
                            resolution,
                            occupancyGrid);

                    resultGrid = VoxIR.Process(
                        resolution,
                        normalGrid,
                        out _);
                }
                else {
                    resultGrid = VoxIR.Process(
                        resolution,
                        extent,
                        mesh,
                        out _);
                }

                groundTruthGrid = GroundTruth.CreateGroundTruthGrid(
                    resolution,
                    extent,
                    groundTruth);

                if (exportPLY) {

                    PostProcessResultGridForVisualization(resultGrid);

                    Visualizer.VisualizeVoxelClassificationAsPLY(
                        $"{outputDirectory}/result.ply",
                        resultGrid);

                    Visualizer.VisualizeVoxelClassificationAsPLY(
                        $"{outputDirectory}/reference.ply",
                        groundTruthGrid);
                }

                Evaluate(
                    resolution,
                    maxBufferSize,
                    $"{outputDirectory}/evaluation.csv",
                    resultGrid.ToOccupancyGrid(),
                    groundTruthGrid.ToOccupancyGrid());
            }
        }

        private static void Log(
                string message) {

            Trace.WriteLine(message);
            Console.WriteLine(message);
        }

        private static AABox GetCommonBBox(
                AABox bBox1,
                AABox bBox2) {

            return new AABox(
                new Vector3d(
                    Math.Min(
                        bBox1.Min.X,
                        bBox2.Min.X),
                    Math.Min(
                        bBox1.Min.Y,
                        bBox2.Min.Y),
                    Math.Min(
                        bBox1.Min.Z,
                        bBox2.Min.Z)),
                new Vector3d(
                    Math.Max(
                        bBox1.Max.X,
                        bBox2.Max.X),
                    Math.Max(
                        bBox1.Max.Y,
                        bBox2.Max.Y),
                    Math.Max(
                        bBox1.Max.Z,
                        bBox2.Max.Z)));
        }

        private static void PostProcessResultGridForVisualization(
                int[,,][] resultGrid) {

            Parallel.For(
                0,
                resultGrid.GetLength(0),
                i => {

                    int r, c;
                    int[] voxelClassValues;

                    for (r = 0; r < resultGrid.GetLength(1); r++) {
                        for (c = 0; c < resultGrid.GetLength(2); c++) {

                            if (resultGrid[i, r, c] == null) {
                                continue;
                            }

                            voxelClassValues = resultGrid[i, r, c]
                                .GetVoxelClassValues()
                                .Where(voxelClassValue => voxelClassValue != VoxelClassValues.EMPTY_INTERIOR
                                    && voxelClassValue != VoxelClassValues.INTERIOR_OBJECT
                                    && voxelClassValue != VoxelClassValues.WALL_OPENING)
                                .ToArray();

                            resultGrid[i, r, c] = voxelClassValues.Length == 0 ?
                                null :
                                VoxelState.CreateVoxelState(
                                    0,
                                    voxelClassValues);
                        }
                    }
                });
        }

        private static bool[,,] ToOccupancyGrid(
                this int[,,][] reconstructionGrid) {

            bool[,,] occupancyGrid = new bool[
                reconstructionGrid.GetLength(0),
                reconstructionGrid.GetLength(1),
                reconstructionGrid.GetLength(2)];

            Parallel.For(
                0,
                reconstructionGrid.GetLength(0),
                i => {

                    int r, c;

                    for (r = 0; r < reconstructionGrid.GetLength(1); r++) {
                        for (c = 0; c < reconstructionGrid.GetLength(2); c++) {

                            if (reconstructionGrid[i, r, c] != null
                                    && reconstructionGrid[i, r, c]
                                        .GetVoxelClassValues()
                                        .Any(voxelClassValue => voxelClassValue == VoxelClassValues.CEILING
                                            || voxelClassValue == VoxelClassValues.FLOOR
                                            || voxelClassValue == VoxelClassValues.WALL)) {

                                occupancyGrid[i, r, c] = true;
                            }
                        }
                    }
                });

            return occupancyGrid;
        }

        private static void Evaluate(
                double resolution,
                double maxBufferSize,
                string outputFile,
                bool[,,] resultGrid,
                bool[,,] groundTruthGrid) {

            object @lock = new object();
            int stepCount = (int)(maxBufferSize / resolution).Round();
            long resultCounter = 0;
            long groundTruthCounter = 0;
            long[] truePositiveCounter = new long[stepCount];
            List<double>[] distances = new List<double>[stepCount];

            for (int d = 0; d < stepCount; d++) {
                distances[d] = new List<double>();
            }

            Parallel.ForEach(
                Partitioner.Create(
                    0,
                    resultGrid.GetLength(0)),
                () => {

                    List<double>[] localDistances = new List<double>[stepCount];

                    for (int d = 0; d < stepCount; d++) {
                        localDistances[d] = new List<double>();
                    }

                    return (
                        0,
                        0,
                        new long[stepCount],
                        localDistances
                    );
                },
                (partition, loopState, localResults) => {

                    int localResultCounter;
                    int localGroundTruthCounter;

                    for (int i = partition.Item1; i < partition.Item2; i++) {

                        Evaluate(
                            i,
                            resolution,
                            localResults.Item3,
                            resultGrid,
                            groundTruthGrid,
                            localResults.Item4,
                            out localResultCounter,
                            out localGroundTruthCounter);

                        localResults.Item1 += localResultCounter;
                        localResults.Item2 += localGroundTruthCounter;
                    }

                    return localResults;
                },
                localResults => {

                    lock (@lock) {

                        resultCounter += localResults.Item1;
                        groundTruthCounter += localResults.Item2;

                        for (int d = 0; d < stepCount; d++) {
                            truePositiveCounter[d] += localResults.Item3[d];
                            distances[d].AddRange(localResults.Item4[d]);
                        }
                    }
                });

            //int localResultCounter;
            //int localGroundTruthCounter;

            //for (int i = 0; i < resultGrid.GetLength(0); i++) {
            //    Evaluate(
            //        i,
            //        resolution,
            //        truePositiveCounter,
            //        resultGrid,
            //        groundTruthGrid,
            //        distances,
            //        out localResultCounter,
            //        out localGroundTruthCounter);

            //    resultCounter += localResultCounter;
            //    groundTruthCounter += localGroundTruthCounter;
            //}

            Evaluate(
                outputFile,
                resultCounter,
                groundTruthCounter,
                resolution,
                truePositiveCounter,
                distances);
        }

        private static void Evaluate(
                int i,
                double resolution,
                long[] truePositiveCounter,
                bool[,,] resultGrid,
                bool[,,] groundTruthGrid,
                List<double>[] distances,
                out int resultCounter,
                out int groundTruthCounter) {

            int r, c;

            resultCounter = 0;
            groundTruthCounter = 0;

            for (r = 0; r < resultGrid.GetLength(1); r++) {
                for (c = 0; c < groundTruthGrid.GetLength(2); c++) {

                    if (groundTruthGrid[i, r, c]) {
                        groundTruthCounter++;
                    }

                    if (resultGrid[i, r, c]) {

                        resultCounter++;

                        Evaluate(
                            resolution,
                            (i, r, c),
                            truePositiveCounter,
                            groundTruthGrid,
                            distances);
                    }
                }
            }
        }

        private static void Evaluate(
                double resolution,
                (int, int, int) evaluationVoxel,
                long[] truePositiveCounter,
                bool[,,] groundTruthGrid,
                List<double>[] distances) {

            int d, d2, di, i, dr, r, dc, c;
            (int, int, int) neighbour;
            bool[] found = new bool[truePositiveCounter.Length];
            Vector3d evaluationVoxelPosition = new Vector3d(
                evaluationVoxel.Item1,
                evaluationVoxel.Item2,
                evaluationVoxel.Item3);
            HashSet<(int, int, int)> voxels = new HashSet<(int, int, int)>();
            List<(int, int, int)> currentLevelVoxels = new List<(int, int, int)> { 
                evaluationVoxel
            };
            HashSet<(int, int, int)> nextLevelVoxels;

            for (d = 0; d < truePositiveCounter.Length; d++) {

                nextLevelVoxels = new HashSet<(int, int, int)>();
                voxels.AddRange(currentLevelVoxels);

                foreach ((int, int, int) voxel in currentLevelVoxels) {

                    if (groundTruthGrid[
                            voxel.Item1,
                            voxel.Item2,
                            voxel.Item3]) {

                        for (d2 = d; d2 < truePositiveCounter.Length; d2++) {

                            found[d2] = true;
                            truePositiveCounter[d2]++;
                            distances[d2].Add(
                                evaluationVoxelPosition
                                    .DistanceTo(
                                        new Vector3d(
                                            voxel.Item1,
                                            voxel.Item2,
                                            voxel.Item3))
                                    * resolution);
                        }

                        return;
                    }

                    for (di = -1; di <= 1; di++) {
                        for (dr = -1; dr <= 1; dr++) {
                            for (dc = -1; dc <= 1; dc++) {

                                i = voxel.Item1 + di;
                                r = voxel.Item2 + dr;
                                c = voxel.Item3 + dc;

                                if (i < 0 || r < 0 || c < 0
                                        || i >= groundTruthGrid.GetLength(0)
                                        || r >= groundTruthGrid.GetLength(1)
                                        || c >= groundTruthGrid.GetLength(2)) {

                                    continue;
                                }

                                neighbour = (i, r, c);
                                if (!nextLevelVoxels.Contains(neighbour)
                                        && !voxels.Contains(neighbour)) {

                                    nextLevelVoxels.Add(neighbour);
                                }
                            }
                        }
                    }
                }

                currentLevelVoxels.Clear();
                currentLevelVoxels.AddRange(nextLevelVoxels);
            }
        }

        private static void Evaluate(
                string outputFile,
                long resultCounter,
                long groundTruthCounter,
                double resolution,
                long[] truePositiveCounter,
                List<double>[] distances) {

            int d;
            double completeness;
            double correctness;
            double accuracy;

            using (InvariantCultureBlock block = new InvariantCultureBlock()) {

                using (StreamWriter writer = new StreamWriter(outputFile)) {

                    writer.WriteLine($"bufferStep [{resolution:0.00}m voxel resolution]; completeness; correctness; accuracy [m]");

                    for (d = 0; d < truePositiveCounter.Length; d++) {

                        completeness = (double)truePositiveCounter[d] / groundTruthCounter;
                        correctness = (double)truePositiveCounter[d] / resultCounter;
                        accuracy = distances[d].Average();

                        writer.WriteLine($"{d}; {completeness:0.00}; {correctness:0.00}; {accuracy:0.00}");
                        Log($"  BufferStep {d}:");
                        Log($"      Completeness: {completeness:0.00}");
                        Log($"      Correctness: {correctness:0.00}");
                        Log($"      Accuracy: {accuracy:0.00} m");
                    }
                }
            }
        }
    }
}
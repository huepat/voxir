using HuePat.VoxIR.DataPreparation;
using HuePat.VoxIR.Evaluation.IO;
using HuePat.VoxIR.IO.PLY.Reading;
using HuePat.VoxIR.Util.Geometry;
using OpenTK.Mathematics;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Threading.Tasks;

namespace HuePat.VoxIR.Evaluation.NormalGridDetermination {
    public static class Evaluation {
        private static readonly Vector3d UP_AXIS = new Vector3d(0.0, -1.0, 0.0);

        public static void Evaluate(
                IList<string> files,
                IList<(double, double)> resolutionAndHorizontalRotationValues,
                OutputConfig outputConfig) {

            string timestamp = Util.CreateTimestampLabel();

            foreach (string file in files) {

                Evaluate(
                    file,
                    resolutionAndHorizontalRotationValues,
                    outputConfig.CopyWithSubDirectory(
                        true,
                        $"{timestamp}_{Path.GetFileNameWithoutExtension(file)}_EvaluationResults"));
            }
        }

        public static void Evaluate(
                string file,
                IList<(double, double)> resolutionAndHorizontalRotationValues,
                OutputConfig outputConfig) {

            Util.Evaluate(
                file,
                outputConfig,
                resolutionAndHorizontalRotationValues,
                (resolution, rotationAroundVerticalAxis, outputConfig) => {

                    return Evaluate(
                        file,
                        resolution,
                        rotationAroundVerticalAxis,
                        outputConfig);

                });
        }

        public static Result Evaluate(
                string file,
                double resolution,
                double rotationAngle,
                OutputConfig outputConfig) {

            bool[,,] occupancyGrid;
            byte[,,] testNormalGrid;
            byte[,,] groundTruthNormalGrid;
            Mesh mesh;

            mesh = new PLYReader().ReadMesh(file);

            if (!rotationAngle.ApproximateEquals(0.0)) {

                mesh.Rotate(
                    rotationAngle,
                    UP_AXIS,
                    mesh.GetCentroid());
            }

            groundTruthNormalGrid = Voxelization.Voxelize(
                resolution,
                null,
                mesh);

            occupancyGrid = Util.GetOccupancyGrid(groundTruthNormalGrid);

            testNormalGrid = HuePat.VoxIR.NormalGridDetermination
                .NormalGridDetermination
                .DetermineNormalGrid(
                    resolution,
                    occupancyGrid);

            if (outputConfig.ExportPLY) {

                HuePat.VoxIR.IO.Visualization.Visualizer.VisualizeNormalGridAsPly(
                    $"{outputConfig.OutputDirectory}/Test_NormalGrid.ply",
                    testNormalGrid,
                    outputConfig.VoxelMesher);

                HuePat.VoxIR.IO.Visualization.Visualizer.VisualizeNormalGridAsPly(
                    $"{outputConfig.OutputDirectory}/GroundTruth_NormalGrid.ply",
                    groundTruthNormalGrid,
                    outputConfig.VoxelMesher);
            }

            if (outputConfig.ExportImages) {

                HuePat.VoxIR.IO.Visualization.Visualizer.VisualizeNormalGridAsSections(
                    $"{outputConfig.OutputDirectory}/Test_NormalGrid.{outputConfig.OutputImageFileType}",
                    testNormalGrid,
                    outputConfig.BackgroundColor);

                HuePat.VoxIR.IO.Visualization.Visualizer.VisualizeNormalGridAsSections(
                    $"{outputConfig.OutputDirectory}/GroundTruth_NormalGrid.{outputConfig.OutputImageFileType}",
                    groundTruthNormalGrid,
                    outputConfig.BackgroundColor);
            }

            return Evaluate(
                testNormalGrid,
                groundTruthNormalGrid);
        }

        private static Result Evaluate(
                byte[,,] testNormalGrid,
                byte[,,] groundTruthNormalGrid) {

            object @lock = new object();
            int truePositiveVoxelCount = 0;
            int nonEmptyTestVoxelCount = 0;
            int nonEmptyGroundTruthVoxelCount = 0;

            Parallel.ForEach(
                Partitioner.Create(
                    0,
                    testNormalGrid.GetLength(0)),
                () => (0, 0, 0),
                (partition, loopState, localCounters) => {

                    for (int i = partition.Item1; i < partition.Item2; i++) {

                        Evaluate(
                            i,
                            ref localCounters,
                            testNormalGrid,
                            groundTruthNormalGrid);
                    }

                    return localCounters;

                },
                localCounters => {

                    lock (@lock) {
                        truePositiveVoxelCount += localCounters.Item1;
                        nonEmptyTestVoxelCount += localCounters.Item2;
                        nonEmptyGroundTruthVoxelCount += localCounters.Item3;
                    }

                });

            return new Result(
                (double)truePositiveVoxelCount / nonEmptyTestVoxelCount,
                (double)truePositiveVoxelCount / nonEmptyGroundTruthVoxelCount);
        }

        private static void Evaluate(
                int i,
                ref (int, int, int) counters,
                byte[,,] testNormalGrid,
                byte[,,] groundTruthNormalGrid) {

            int r, c;
            byte testNormalGridValue;
            byte groundTruthNormalGridValue;

            for (r = 0; r < testNormalGrid.GetLength(1); r++) {
                for (c = 0; c < testNormalGrid.GetLength(2); c++) {

                    testNormalGridValue = testNormalGrid[i, r, c];
                    groundTruthNormalGridValue = groundTruthNormalGrid[i, r, c];

                    if (testNormalGridValue == groundTruthNormalGridValue
                            && (testNormalGridValue != NormalGridValues.EMPTY
                                || groundTruthNormalGridValue != NormalGridValues.EMPTY)) {

                        counters.Item1++;
                    }
                    if (testNormalGridValue != NormalGridValues.EMPTY) {
                        counters.Item2++;
                    }
                    if (groundTruthNormalGridValue != NormalGridValues.EMPTY) {
                        counters.Item3++;
                    }
                }
            }
        }
    }
}
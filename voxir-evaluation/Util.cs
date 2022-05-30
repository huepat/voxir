using HuePat.VoxIR.Evaluation.IO;
using HuePat.VoxIR.IO;
using System;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace HuePat.VoxIR.Evaluation {
    static class Util {
        public static string CreateTimestampLabel() {
            return DateTime
                .Now
                .ToString("yyyyMMdd_HHmmss");
        }

        public static OutputConfig CopyWithSubDirectory(
                this OutputConfig outputConfig,
                bool createDirectory,
                string subDirectory) {

            return new OutputConfig(
                    $"{outputConfig.OutputDirectory}/{subDirectory}",
                    createDirectory) {
                ExportExtendedLogs = outputConfig.ExportExtendedLogs,
                ExportPLY = outputConfig.ExportPLY,
                ExportImages = outputConfig.ExportImages,
                BackgroundColor = outputConfig.BackgroundColor,
                OutputImageFileType = outputConfig.OutputImageFileType,
                VoxelMesher = outputConfig.VoxelMesher
            };
        }

        public static string GetParameterTableColumnHeader(
                (double, double) parameters) {

            return $"Res. {parameters.Item1:0.00}m" +
                $" | Rot. {parameters.Item2.RadianToDegree():0.00}°";
        }

        public static void Evaluate<T>(
                string testFile,
                OutputConfig outputConfig,
                IList<(double, double)> resolutionAndHorizontalRotationValues,
                Func<double, double, OutputConfig, T> callback)
                    where T : class, IResult {

            List<T> results = new List<T>();

            using (InvariantCultureBlock block = new InvariantCultureBlock()) {

                using (CSVWriter writer = new CSVWriter(
                        $"{outputConfig.OutputDirectory}/EvaluationResults.csv")) {

                    writer.WriteHeaderLine(resolutionAndHorizontalRotationValues);

                    foreach ((double, double) parameters in resolutionAndHorizontalRotationValues) {

                        Console.WriteLine($"Evaluating {testFile} " +
                            $"[{GetParameterTableColumnHeader(parameters)}]");

                        results.Add(callback(
                            parameters.Item1,
                            parameters.Item2,
                            outputConfig.CopyWithSubDirectory(
                                outputConfig.ExportExtendedLogs
                                    || outputConfig.ExportPLY
                                    || outputConfig.ExportImages,
                                $"Resolution {parameters.Item1:0.00}m, " +
                                    $"Horizontal Rotation {parameters.Item2.RadianToDegree():0.00}°")));
                    }

                    writer.WriteResults(results);
                }
            }
        }

        public static bool[,,] GetOccupancyGrid(
                byte[,,] normalGrid) {

            bool[,,] occupancyGrid = new bool[
                normalGrid.GetLength(0),
                normalGrid.GetLength(1),
                normalGrid.GetLength(2)];

            Parallel.For(
                0,
                normalGrid.GetLength(0),
                i => {

                    int r, c;

                    for (r = 0; r < normalGrid.GetLength(1); r++) {
                        for (c = 0; c < normalGrid.GetLength(2); c++) {

                            if (normalGrid[i, r, c] != NormalGridValues.EMPTY) {
                                occupancyGrid[i, r, c] = true;
                            }
                        }
                    }
                });

            return occupancyGrid;
        }
    }
}
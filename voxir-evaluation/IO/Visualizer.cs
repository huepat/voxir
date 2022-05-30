using HuePat.VoxIR.IO;
using HuePat.VoxIR.IO.PLY.Writing;
using HuePat.VoxIR.IO.Visualization;
using System;
using System.Collections.Generic;

namespace HuePat.VoxIR.Evaluation.IO {
    public static class Visualizer {
        public static void ExportPLY(
                int[,,][] reconstructionGrid,
                int[,,][] groundTruthGrid,
                OutputConfig outputConfig) {

            HuePat.VoxIR.IO.Visualization.Visualizer.VisualizeVoxelClassificationAsPLY(
                $"{outputConfig.OutputDirectory}/Test_VoxelClasses.ply",
                reconstructionGrid,
                outputConfig.VoxelMesher);

            HuePat.VoxIR.IO.Visualization.Visualizer.VisualizeVoxelClassificationAsPLY(
                $"{outputConfig.OutputDirectory}/GroundTruth_VoxelClasses.ply",
                groundTruthGrid,
                outputConfig.VoxelMesher);

            HuePat.VoxIR.IO.Visualization.Visualizer.VisualizeSpacePartitioningAsPLY(
                $"{outputConfig.OutputDirectory}/Test_RoomPartitioning.ply",
                reconstructionGrid,
                outputConfig.VoxelMesher);

            HuePat.VoxIR.IO.Visualization.Visualizer.VisualizeSpacePartitioningAsPLY(
                $"{outputConfig.OutputDirectory}/GroundTruth_RoomPartitioning.ply",
                groundTruthGrid,
                outputConfig.VoxelMesher); 
        }

        public static void ExportImages(
                int[,,][] reconstructionGrid,
                int[,,][] groundTruthGrid,
                OutputConfig outputConfig) {

            HuePat.VoxIR.IO.Visualization.Visualizer.VisualizeVoxelClassificationAsSections(
                $"{outputConfig.OutputDirectory}/Test_VoxelClasses.{outputConfig.OutputImageFileType}",
                reconstructionGrid,
                outputConfig.BackgroundColor);

            HuePat.VoxIR.IO.Visualization.Visualizer.VisualizeVoxelClassificationAsSections(
                $"{outputConfig.OutputDirectory}/GroundTruth_VoxelClasses.{outputConfig.OutputImageFileType}",
                groundTruthGrid,
                outputConfig.BackgroundColor);

            HuePat.VoxIR.IO.Visualization.Visualizer.VisualizeSpacePartitioningAsSections(
                $"{outputConfig.OutputDirectory}/Test_RoomPartitioning.{outputConfig.OutputImageFileType}",
                reconstructionGrid,
                outputConfig.BackgroundColor);

            HuePat.VoxIR.IO.Visualization.Visualizer.VisualizeSpacePartitioningAsSections(
                $"{outputConfig.OutputDirectory}/GroundTruth_RoomPartitioning.{outputConfig.OutputImageFileType}",
                groundTruthGrid,
                outputConfig.BackgroundColor);
        }

        public static void Write(
                this Dictionary<int, PLYWriter> writers,
                Color color,
                OutputConfig outputConfig,
                Dictionary<int, List<(int, int, int)>> voxels,
                Func<int, string> getFileNameCallback) {

            foreach (int key in voxels.Keys) {

                if (!writers.ContainsKey(key)) {
                    writers.Add(
                        key,
                        new PLYWriter($"{outputConfig.OutputDirectory}/{getFileNameCallback(key)}.ply"));
                }

                foreach ((int, int, int) voxel in voxels[key]) {
                    writers[key].Write(
                        voxel.Item1,
                        voxel.Item2,
                        voxel.Item3,
                        color,
                        outputConfig.VoxelMesher);
                }
            }
        }

        public static void Dispose(
                this Dictionary<int, PLYWriter> writers) {

            foreach (PLYWriter writer in writers.Values) {
                writer.Dispose();
            }
        }

        public static void Write(
                this ImageWriter writer,
                bool foundFalsePositive,
                bool foundFalseNegative,
                int r,
                int c,
                int[] voxelClassValues,
                Color backgroundColor) {

            Color color;

            if (foundFalsePositive && foundFalseNegative) {
                color = Definitions.EvaluationErrorColors.BOTH_ERRORS_COLOR;
            }
            else if (foundFalsePositive) {
                color = Definitions.EvaluationErrorColors.FALSE_POSITIVE_COLOR;
            }
            else if (foundFalsePositive) {
                color = Definitions.EvaluationErrorColors.FALSE_NEGATIVE_COLOR;
            }
            else if(voxelClassValues.Length == 0) {
                color = backgroundColor;
            }
            else {
                color = Definitions.VOXEL_CLASS_COLORS[
                    voxelClassValues.ToVisualizationVoxelClassValue()];
            }

            writer.Write(
                r,
                c,
                color);
        }
    }
}
using HuePat.VoxIR.IO;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace HuePat.VoxIR.Evaluation.IO {
    public static class TextOutput {
        public static void WriteWeightedRoomMappings(
                string directory,
                HashSet<int> reconstructionRampSpaceIds,
                HashSet<int> groundTruthRampSpaceIds,
                Dictionary<int, Dictionary<int, int>> weightedReconstructionToGroundTruthRoomMapping,
                Dictionary<int, Dictionary<int, int>> weightedGroundTruthToReconstructionRoomMapping) {

            using (InvariantCultureBlock block = new InvariantCultureBlock()) {

                WriteWeightedRoomMapping(
                    $"{directory}/WeightedRoomMapping_Test2GroundTruth.txt",
                    reconstructionRampSpaceIds,
                    groundTruthRampSpaceIds,
                    weightedReconstructionToGroundTruthRoomMapping);

                WriteWeightedRoomMapping(
                    $"{directory}/WeightedRoomMapping_GroundTruth2Test.txt",
                    groundTruthRampSpaceIds,
                    reconstructionRampSpaceIds,
                    weightedGroundTruthToReconstructionRoomMapping);
            }
        }

        public static void WriteRoomMappingEvaluationResults(
                int totalVolume,
                int correctRoomMappingVolume,
                string directory,
                string title,
                HashSet<int> fromRampSpaceIds,
                HashSet<int> roomMappingErrorRoomIds,
                List<int> rampSpaceErrorRoomIds) {

            using (InvariantCultureBlock block = new InvariantCultureBlock()) {
                using (StreamWriter writer = new StreamWriter($"{directory}/RoomMapping_Evaluation_{title}")) {       

                    writer.WriteLine("");
                    writer.WriteLine($"ROOM MAPPING EVALUATION {title}:");
                    writer.WriteLine($"    {roomMappingErrorRoomIds.Count} Room Mapping Errors:");
                    writer.WriteLine($"        (={100.0 * correctRoomMappingVolume / totalVolume:0.00}% of total volume)");

                    foreach (int id in roomMappingErrorRoomIds.Order()) {
                        writer.WriteLine($"        {FormatRoomLabel(id, fromRampSpaceIds)}");
                    }

                    writer.WriteLine($"    {rampSpaceErrorRoomIds.Count} Ramp Space Errors:");

                    foreach (int id in rampSpaceErrorRoomIds.Order()) {
                        writer.WriteLine($"        {FormatRoomLabel(id, fromRampSpaceIds)}");
                    }
                }
            }
        }

        public static string FormatRoomLabel(
                int roomId,
                HashSet<int> rampSpaceIds) {

            string s = roomId < 0 ? $"T{-roomId}" : $"R{roomId}";

            if (rampSpaceIds.Contains(roomId)) {
                s += "R";
            }

            return s;
        }

        public static void WriteRoomSegmentationEvaluationResults(
                string directory,
                double precision,
                double recall,
                HashSet<int> reconstructionRampSpaceIds,
                HashSet<int> reconstructionRoomMappingErrorIds,
                Dictionary<int, int> truePositiveVolume,
                Dictionary<int, int> reconstructionVolume,
                Dictionary<int, int> groundTruthVolume,
                Dictionary<int, int> reconstructionToGroundTruthRoomMapping) {

            using (InvariantCultureBlock block = new InvariantCultureBlock()) {
                using (StreamWriter writer = new StreamWriter($"{directory}/RoomSegmentation_EvaluationResults.txt")) {

                    writer.WriteLine($"ROOM SEGMENTATION PRECISION");

                    foreach (int roomId in reconstructionToGroundTruthRoomMapping.Keys.Order()) {
                        if (!reconstructionRoomMappingErrorIds.Contains(roomId)) {
                            writer.WriteLine($"{FormatRoomLabel(roomId, reconstructionRampSpaceIds)}: " +
                                $"{100.0 * truePositiveVolume[roomId] / reconstructionVolume[roomId]:0.##}%");
                        }
                    }

                    writer.WriteLine($"____________________________");
                    writer.WriteLine($"Total: {precision:0.00}%");
                    writer.WriteLine("");

                    writer.WriteLine($"ROOM SEGMENTATION RECALL");

                    foreach (int roomId in reconstructionToGroundTruthRoomMapping.Keys.Order()) {
                        if (!reconstructionRoomMappingErrorIds.Contains(roomId)) {
                            writer.WriteLine($"{FormatRoomLabel(roomId, reconstructionRampSpaceIds)}: " +
                                $"{100.0 * truePositiveVolume[roomId] / groundTruthVolume[roomId]:0.##}%");
                        }
                    }

                    writer.WriteLine($"____________________________");
                    writer.WriteLine($"Total: {recall:0.00}%");
                }
            }
        }

        public static void WriteVoxelClassificationClassDistribution(
                string directory,
                Dictionary<int, int> reconstructionVolume,
                Dictionary<int, int> groundTruthVolume) {

            int totalReconstructionVolume = reconstructionVolume.Values.Sum();
            int totalGroundTruthVolume = groundTruthVolume.Values.Sum();

            using (InvariantCultureBlock block = new InvariantCultureBlock()) {

                using (StreamWriter writer = new StreamWriter($"{directory}/VoxelClassification_ClassDistribution.txt")) {

                    writer.WriteLine($"VOXEL CLASSES");

                    foreach (int voxelClassValue in reconstructionVolume.Keys.Order()) {

                        writer.WriteLine($"  {VoxelClassValues.Labels[voxelClassValue]}: " +
                            $"{100.0 * reconstructionVolume[voxelClassValue] / totalReconstructionVolume:0.00}% of Test, " +
                            $"{100.0 * groundTruthVolume[voxelClassValue] / totalGroundTruthVolume:0.00}% of GroundTruth");
                    }
                }
            }
        }

        public static void WriteVoxelClassificationEvaluationResults(
                string directory,
                double reconstructionVolumeForVoxelClassSubset,
                Dictionary<int, int> reconstructionVolume,
                Dictionary<int, double> precision,
                Dictionary<int, double> recall,
                Dictionary<int, double> neighbourhoodPrecision,
                Dictionary<int, double> neighbourhoodRecall) {

            using (InvariantCultureBlock block = new InvariantCultureBlock()) {
                using (StreamWriter writer = new StreamWriter($"{directory}/VoxelClassification_EvaluationResults.txt")) {

                    WriteVoxelClassificationEvaluationResults(
                        reconstructionVolumeForVoxelClassSubset,
                        "PRECISION",
                        writer,
                        reconstructionVolume,
                        precision);

                    WriteVoxelClassificationEvaluationResults(
                        reconstructionVolumeForVoxelClassSubset,
                        "RECALL",
                        writer,
                        reconstructionVolume,
                        recall);

                    WriteVoxelClassificationEvaluationResults(
                        reconstructionVolumeForVoxelClassSubset,
                        "NEIGHBOURHOOD PRECISION (FROM 3D-6-NEIGHBOURHOOD)",
                        writer,
                        reconstructionVolume,
                        neighbourhoodPrecision);

                    WriteVoxelClassificationEvaluationResults(
                        reconstructionVolumeForVoxelClassSubset,
                        "NEIGHBOURHOOD RECALL (FROM 3D-6-NEIGHBOURHOOD)",
                        writer,
                        reconstructionVolume,
                        neighbourhoodRecall);
                }
            }
        }

        private static void WriteWeightedRoomMapping(
                string file,
                HashSet<int> fromRampSpaceIds,
                HashSet<int> toRampSpaceIds,
                Dictionary<int, Dictionary<int, int>> weightedRoomMapping) {

            using (StreamWriter writer = new StreamWriter(file)) {

                writer.WriteLine($"WEIGHTED ROOM MAPPING [from {weightedRoomMapping.Count} space ids]:");

                foreach (int fromId in weightedRoomMapping.Keys.Order()) {

                    writer.WriteLine($"{FormatRoomLabel(fromId, fromRampSpaceIds)}");

                    foreach (int toId in weightedRoomMapping[fromId].Keys.Order()) {
                        writer.WriteLine($"    -> {FormatRoomLabel(toId, toRampSpaceIds)} ({weightedRoomMapping[fromId][toId]})");
                    }
                }
            }
        }

        private static void WriteVoxelClassificationEvaluationResults(
                double reconstructionVolumForVoxelClassSubset,
                string label,
                StreamWriter writer,
                Dictionary<int, int> reconstructionVolume,
                Dictionary<int, double> resultsValues) {

            
            List<double> weightedResultValuesPerVoxelClass = new List<double>();
            List<double> weightedResultValuesPerVoxelClassSubset = new List<double>();

            writer.WriteLine(label);

            foreach (int voxelClassValue in reconstructionVolume.Keys.Order()) {

                writer.WriteLine($"    {VoxelClassValues.Labels[voxelClassValue]}: {resultsValues[voxelClassValue]:0.00}%");

                weightedResultValuesPerVoxelClass.Add(
                    (double)reconstructionVolume[voxelClassValue] / reconstructionVolume.Values.Sum() * resultsValues[voxelClassValue]);

                if (voxelClassValue != VoxelClassValues.EMPTY_INTERIOR
                        && voxelClassValue != VoxelClassValues.WALL_OPENING) {

                    weightedResultValuesPerVoxelClassSubset.Add(
                        reconstructionVolume[voxelClassValue] / reconstructionVolumForVoxelClassSubset * resultsValues[voxelClassValue]);
                }
            }

            writer.WriteLine($"    ____________________________________");
            writer.WriteLine(
                $"    Mean (weighted by voxel count per class): {weightedResultValuesPerVoxelClass.Sum():0.00}% " +
                $"(without EI & WO: {weightedResultValuesPerVoxelClassSubset.Sum():0.00}%)");
        }
    }
}
using System.Collections.Generic;

namespace HuePat.VoxIR.Evaluation {
    public class Result : IResult {
        public double RoomMappingError { get; private set; }
        public double RoomSegmentationPrecision { get; private set; }
        public double RoomSegmentationRecall { get; private set; }
        public double RoomSegmentationF1Score { get; private set; }
        public Dictionary<int, double> VoxelClassificationPrecision { get; private set; }
        public Dictionary<int, double> VoxelClassificationRecall { get; private set; }
        public Dictionary<int, double> VoxelClassificationF1Score { get; private set; }
        public Dictionary<int, double> VoxelClassificationNeighbourhoodPrecision { get; private set; }
        public Dictionary<int, double> VoxelClassificationNeighbourhoodRecall { get; private set; }

        public IEnumerable<string> Labels {
            get {

                yield return "Room Mapping Error (%)";
                yield return "Room Segmentation Precision (%)";
                yield return "Room Segmentation Recall (%)";
                yield return "Room Segmentation F1-Score (%)";

                foreach (string label in GetVoxelClassificationResultLabels("Precision")) {
                    yield return label;
                }
                foreach (string label in GetVoxelClassificationResultLabels("Recall")) {
                    yield return label;
                }
                foreach (string label in GetVoxelClassificationResultLabels("F1-Score")) {
                    yield return label;
                }
                foreach (string label in GetVoxelClassificationResultLabels("Neighbourhood Precision")) {
                    yield return label;
                }
                foreach (string label in GetVoxelClassificationResultLabels("Neighbourhood Recall")) {
                    yield return label;
                }
            }
        }

        public IEnumerable<double> Values {
            get {

                yield return RoomMappingError;
                yield return RoomSegmentationPrecision;
                yield return RoomSegmentationRecall;
                yield return RoomSegmentationF1Score;

                foreach (double value in GetVoxelClassificationResultValues(VoxelClassificationPrecision)) {
                    yield return value;
                }
                foreach (double value in GetVoxelClassificationResultValues(VoxelClassificationRecall)) {
                    yield return value;
                }
                foreach (double value in GetVoxelClassificationResultValues(VoxelClassificationF1Score)) {
                    yield return value;
                }
                foreach (double value in GetVoxelClassificationResultValues(VoxelClassificationNeighbourhoodPrecision)) {
                    yield return value;
                }
                foreach (double value in GetVoxelClassificationResultValues(VoxelClassificationNeighbourhoodRecall)) {
                    yield return value;
                }
            }
        }

        public Result(
                double roomMappingError,
                double roomSegmentationPrecision,
                double roomSegmentationRecall,
                double roomSegmentationF1Score,
                Dictionary<int, double> voxelClassificationPrecision,
                Dictionary<int, double> voxelClassificationRecall,
                Dictionary<int, double> voxelClassificationF1Score,
                Dictionary<int, double> voxelClassificationNeighbourhoodPrecision,
                Dictionary<int, double> voxelClassificationNeighbourhoodRecall) {

            RoomMappingError = roomMappingError;
            RoomSegmentationPrecision = roomSegmentationPrecision;
            RoomSegmentationRecall = roomSegmentationRecall;
            RoomSegmentationF1Score = roomSegmentationF1Score;
            VoxelClassificationPrecision = voxelClassificationPrecision;
            VoxelClassificationRecall = voxelClassificationRecall;
            VoxelClassificationF1Score = voxelClassificationF1Score;
            VoxelClassificationNeighbourhoodPrecision = voxelClassificationNeighbourhoodPrecision;
            VoxelClassificationNeighbourhoodRecall = voxelClassificationNeighbourhoodRecall;
        }

        private IEnumerable<string> GetVoxelClassificationResultLabels(
                string metricLabel) {

            yield return $"{metricLabel} Ceiling (%)";
            yield return $"{metricLabel} Floor (%)";
            yield return $"{metricLabel} Wall (%)";
            yield return $"{metricLabel} Interior Object (%)";
            yield return $"{metricLabel} Empty Interior (%)";
            yield return $"{metricLabel} Wall Opening (%)";
        }

        private IEnumerable<double> GetVoxelClassificationResultValues(
                Dictionary<int, double> voxelClassificationResults) {

            yield return voxelClassificationResults[VoxelClassValues.CEILING];
            yield return voxelClassificationResults[VoxelClassValues.FLOOR];
            yield return voxelClassificationResults[VoxelClassValues.WALL];
            yield return voxelClassificationResults[VoxelClassValues.INTERIOR_OBJECT];
            yield return voxelClassificationResults[VoxelClassValues.EMPTY_INTERIOR];
            yield return voxelClassificationResults[VoxelClassValues.WALL_OPENING];
        }
    }
}
using System.Collections.Generic;

namespace HuePat.VoxIR.Evaluation {
    public class Result {
        public double RoomMappingError { get; private set; }
        public double RoomSegmentationPrecision { get; private set; }
        public double RoomSegmentationRecall { get; private set; }
        public double RoomSegmentationF1Score { get; private set; }
        public Dictionary<int, double> VoxelClassificationPrecision { get; private set; }
        public Dictionary<int, double> VoxelClassificationRecall { get; private set; }
        public Dictionary<int, double> VoxelClassificationF1Score { get; private set; }
        public Dictionary<int, double> VoxelClassificationNeighbourhoodPrecision { get; private set; }
        public Dictionary<int, double> VoxelClassificationNeighbourhoodRecall { get; private set; }

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
    }
}
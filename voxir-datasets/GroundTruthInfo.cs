namespace HuePat.VoxIR.Datasets {
    public class GroundTruthInfo {
        public bool IsRampSpace { get; private set; }
        public int ClassValue { get; private set; }
        public int RoomId { get; private set; }

        public GroundTruthInfo(
                bool isRampSpace,
                int roomId,
                int classValue) {

            IsRampSpace = isRampSpace;
            RoomId = roomId;
            ClassValue = classValue;
        }
    }
}
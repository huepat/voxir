namespace HuePat.VoxIR.VoxelModelRefinement {
    class WallOccurence {
        public int RoomId { get; private set; }
        public int Distance { get; private set; }
        public int[] VoxelClassValues { get; private set; }
        public (int, int) Direction { get; private set; }

        public WallOccurence(
                int roomId,
                int distance,
                (int, int) direction,
                int[] voxelClassificationValues) {

            RoomId = roomId;
            Distance = distance;
            Direction = direction;
            VoxelClassValues = voxelClassificationValues;
        }
    }
}
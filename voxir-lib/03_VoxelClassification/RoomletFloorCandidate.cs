using System.Collections.Generic;

namespace HuePat.VoxIR.VoxelClassification {
    class RoomletFloorCandidate : List<(int, int, int)> {
        public int Id { get; private set; }

        public RoomletFloorCandidate(
                int id,
                List<(int, int, int)> voxels) :
                    base(voxels) {

            Id = id;
        }
    }
}
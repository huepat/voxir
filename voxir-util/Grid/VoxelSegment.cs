using System.Collections.Generic;

namespace HuePat.VoxIR.Util.Grid {
    public class VoxelSegment : List<(int, int, int)> {
        public GridBBox3D BBox { get; private set; }

        public VoxelSegment(
                GridBBox3D bBox,
                IEnumerable<(int, int, int)> voxels) :
                    base(voxels) {

            BBox = bBox;
        }
    }
}
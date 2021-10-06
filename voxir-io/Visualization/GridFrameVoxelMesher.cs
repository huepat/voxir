using HuePat.VoxIR.Util.Geometry;
using OpenTK.Mathematics;

namespace HuePat.VoxIR.IO.Visualization {
    public class GridFrameVoxelMesher : IVoxelMesher {
        public Mesh Mesh(
                int i, 
                int r, 
                int c) {

            return AABox.FromCenterAndSize(
                    new Vector3d(r, c, -i),
                    new Vector3d(1.0))
                .Mesh;
        }
    }
}
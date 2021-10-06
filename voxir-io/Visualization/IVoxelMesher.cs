using HuePat.VoxIR.Util.Geometry;

namespace HuePat.VoxIR.IO.Visualization {
    public interface IVoxelMesher {
        Mesh Mesh(
            int i,
            int r,
            int c);
    }
}
using HuePat.VoxIR.Util.Geometry;
using OpenTK.Mathematics;

namespace HuePat.VoxIR.IO.Visualization {
    public class MeshFrameVoxelMesher : IVoxelMesher {
        private double resolution;
        private Vector3d offset;

        public MeshFrameVoxelMesher(
                double resolution,
                Mesh mesh) {

            this.resolution = resolution;
            offset = mesh.BBox.Min - new Vector3d(resolution);
        }

        public virtual Mesh Mesh(
                int i, 
                int r, 
                int c) {

            return AABox.FromCenterAndSize(
                    new Vector3d(
                        offset.X + r * resolution,
                        offset.Y + i * resolution,
                        offset.Z + c * resolution),
                    new Vector3d(resolution))
                .Mesh;
        }
    }
}
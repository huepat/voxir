using HuePat.VoxIR.IO.Visualization;
using HuePat.VoxIR.Util.Geometry;
using OpenTK.Mathematics;

namespace HuePat.VoxIR.IO.Matterport3D {
    public class Matterport3DVoxelMesher : MeshFrameVoxelMesher {
        private Vector3d anchor;

        public Matterport3DVoxelMesher(
                double resolution,
                Mesh mesh) :
                    base(
                        resolution,
                        mesh){

            anchor = mesh.GetCentroid();
        }

        public override Mesh Mesh(
                int i, 
                int r, 
                int c) {

            Mesh mesh = base.Mesh(i, r, c);

            mesh.Rotate(
                90.0.DegreeToRadian(),
                new Vector3d(1.0, 0.0, 0.0),
                anchor);

            return mesh;
        }
    }
}
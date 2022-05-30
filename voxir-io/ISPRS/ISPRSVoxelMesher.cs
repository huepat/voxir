using HuePat.VoxIR.IO.Visualization;
using HuePat.VoxIR.Util.Geometry;
using OpenTK.Mathematics;

namespace HuePat.VoxIR.IO.ISPRS {
    public class ISPRSVoxelMesher : GeometryFrameVoxelMesher {
        private Vector3d anchor;

        public ISPRSVoxelMesher(
                double resolution,
                PointCloud pointCloud) :
                    base(
                        resolution,
                        pointCloud) {

            anchor = pointCloud.GetCentroid();
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
using HuePat.VoxIR.IO.PLY.Reading;
using HuePat.VoxIR.Util.Geometry;
using OpenTK.Mathematics;

namespace HuePat.VoxIR.IO.Matterport3D {
    public static class Matterport3DReader {
        public static Mesh LoadHouse(
                string file) {

            Mesh mesh;
            PLYReader reader;

            reader = new PLYReader {
                InvertNormals = true
            };

            mesh = reader.ReadMesh(file);

            mesh.Rotate(
                90.0.DegreeToRadian(),
                new Vector3d(1.0, 0.0, 0.0),
                mesh.GetCentroid());

            return mesh;
        }
    }
}
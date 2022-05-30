using OpenTK.Mathematics;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

namespace HuePat.VoxIR.Util.Geometry {
    public class Mesh: IReadOnlyList<Face>, IGeometrySet {

        public static Mesh Merge(IList<Mesh> meshes) {

            int offset = 0;
            List<Point> vertices;
            List<Face> faces = new List<Face>();

            vertices = meshes
                .SelectMany(mesh => mesh.Vertices)
                .ToList();

            foreach (Mesh mesh in meshes) {

                faces.AddRange(
                    mesh.Select(face => {
                        return new Face(
                            face.VertexIndex1 + offset,
                            face.VertexIndex2 + offset,
                            face.VertexIndex3 + offset,
                            vertices);
                    }));

                offset += mesh.Vertices.Count;
            }

            return new Mesh(
                vertices,
                faces);
        }

        private readonly Face[] faces;

        public PointCloud Vertices { get; private set; }

        public int Count {
            get {
                return faces.Length;
            }
        }

        public AABox BBox {
            get {
                return Vertices.BBox;
            }
        }

        Mesh IFiniteGeometry.Mesh { 
            get {
                return this;
            }
        }

        public Face this[int i] {
            get {
                return faces[i];
            }
        }

        public Mesh(
                IReadOnlyList<Point> vertices,
                IEnumerable<Face> faces,
                bool createBBox = true,
                bool useParallelForBBox = false) {

            this.faces = faces.ToArray();
            Vertices = new PointCloud(
                vertices,
                createBBox,
                useParallelForBBox);
        }

        public IEnumerator<Face> GetEnumerator() {

            return faces
                .AsEnumerable()
                .GetEnumerator();
        }

        IEnumerator IEnumerable.GetEnumerator() {

            return GetEnumerator();
        }

        public bool Intersects(AABox box) {

            return this.Any(face => face.Intersects(box));
        }

        public void Rotate(
                double angle,
                Vector3d axis,
                Vector3d anchor) {

            Vertices.Rotate(
                angle,
                axis,
                anchor);
        }

        public Vector3d GetCentroid() {

            return Vertices.GetCentroid();
        }
    }
}
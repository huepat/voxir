using OpenTK.Mathematics;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace HuePat.VoxIR.Util.Geometry {
    public class Mesh: IReadOnlyList<Face>, IFiniteGeometry {
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

            Matrix3d rotation = GetRotation(
                angle,
                axis);

            Parallel.For(
                0,
                Vertices.Count,
                j => {
                    Vertices[j].Position = rotation.Multiply(Vertices[j].Position - anchor) + anchor;
                });

            Vertices.UpdateBBox(true);
        }

        public Vector3d GetCentroid() {

            object @lock = new object();
            (Vector3d, long) centroid = (new Vector3d(0.0), 0);

            Parallel.ForEach(
                Vertices,
                () => (new Vector3d(0.0), 0),
                (point, loopState, localCentroid) => {

                    localCentroid.Item1 += (point.Position - localCentroid.Item1) / ++localCentroid.Item2;

                    return localCentroid;
                },
                localCentroid => {

                    lock (@lock) {
                        long newCount = centroid.Item2 + localCentroid.Item2;
                        centroid.Item1 = (double)centroid.Item2 / newCount * centroid.Item1
                            + (double)localCentroid.Item2 / newCount * localCentroid.Item1;
                        centroid.Item2 = newCount;
                    }
                });

            return centroid.Item1;
        }

        private Matrix3d GetRotation(
                double angle,
                Vector3d axis) {

            double c = angle.Cos();
            double s = angle.Sin();
            double t = 1.0 - c;
            Vector3d axisNormalized = axis.Normalized();
            double tmp1 = axisNormalized.X * axisNormalized.Y * t;
            double tmp2 = axisNormalized.Z * s;
            double tmp5 = axisNormalized.Y * axisNormalized.Z * t;
            double tmp6 = axisNormalized.X * s;
            double tmp3 = axisNormalized.X * axisNormalized.Z * t;
            double tmp4 = axisNormalized.Y * s;

            return new Matrix3d(
                c + axisNormalized.X * axisNormalized.X * t, tmp1 - tmp2, tmp3 + tmp4,
                tmp1 + tmp2, c + axisNormalized.Y * axisNormalized.Y * t, tmp5 - tmp6,
                tmp3 - tmp4, tmp5 + tmp6, c + axisNormalized.Z * axisNormalized.Z * t);
        }
    }
}
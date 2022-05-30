using OpenTK.Mathematics;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace HuePat.VoxIR.Util.Geometry {
    public class PointCloud: IReadOnlyList<Point>, IGeometrySet {
        private readonly Point[] points;

        public AABox BBox { get; protected set; }

        public Point this[int i] {
            get {
                return points[i];
            }
        }

        public int Count {
            get {
                return points.Length;
            }
        }

        public Mesh Mesh {
            get {
                return new Mesh(
                    this,
                    new Face[0],
                    false);
            }
        }

        public PointCloud(
                IEnumerable<Point> points,
                bool createBBox,
                bool useParallelForBBox) {

            this.points = points.ToArray();

            if (createBBox) {
                UpdateBBox(useParallelForBBox);
            }
        }

        public IEnumerator<Point> GetEnumerator() {

            return points
                .AsEnumerable()
                .GetEnumerator();
        }

        IEnumerator IEnumerable.GetEnumerator() {

            return GetEnumerator();
        }

        public bool Intersects(AABox box) {

            return this.Any(point => point.Intersects(box));
        }

        public void UpdateBBox(
                bool useParallel) {

            BBox = AABox.FromContainedGeometries(
                points,
                useParallel);
        }

        public Vector3d GetCentroid() {

            return points
                    .AsParallel()
                    .Select(point => point.Position)
                    .Aggregate((position1, position2) => position1 + position2)
                / points.Length;
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
                points.Length,
                j => {
                    points[j].Position = rotation.Multiply(points[j].Position - anchor) + anchor;
                });

            UpdateBBox(true);
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
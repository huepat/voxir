using System.Collections;
using System.Collections.Generic;
using System.Linq;

namespace HuePat.VoxIR.Util.Geometry {
    public class PointCloud: IReadOnlyList<Point>, IFiniteGeometry {
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
    }
}
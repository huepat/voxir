using OpenTK.Mathematics;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace HuePat.VoxIR.Util.Geometry {
    public class AABox : IFiniteGeometry {
        public static AABox FromContainedGeometries<T>(
                IList<T> geometries,
                bool useParallel = false)
                    where T : IFiniteGeometry {

            return useParallel ?
                FromContainedGeometries_Parallel(geometries) :
                FromContainedGeometries_Sequential(geometries);
        }

        public static AABox FromCenterAndSize(
                Vector3d center,
                Vector3d size) {

            Vector3d s = size / 2;

            return new AABox(center - s, center + s);
        }

        private static AABox FromContainedGeometries_Sequential<T>(
                IList<T> geometries)
                    where T : IFiniteGeometry {

            geometries.SelectMany(geometry => {

                AABox bbox = geometry.BBox;

                return new Vector3d[] { 
                    bbox.Min, 
                    bbox.Max 
                };

            }).GetMinMax(
                out Vector3d min, 
                out Vector3d max);

            return new AABox(min, max);
        }

        private static AABox FromContainedGeometries_Parallel<T>(
                IList<T> geometries)
                    where T : IFiniteGeometry {

            object @lock = new object();

            (Vector3d, Vector3d) bounds = (
                new Vector3d(double.MaxValue),
                new Vector3d(double.MinValue));

            Parallel.ForEach(
                Partitioner.Create(
                    0,
                    geometries.Count),
                () => (
                    new Vector3d(double.MaxValue),
                    new Vector3d(double.MinValue)),
                (partition, loopState, localBounds) => {

                    AABox bBox;

                    for (int i = partition.Item1; i < partition.Item2; i++) {
                        bBox = geometries[i].BBox;
                        if (bBox.Min.X < localBounds.Item1.X) {
                            localBounds.Item1.X = bBox.Min.X;
                        }
                        if (bBox.Min.Y < localBounds.Item1.Y) {
                            localBounds.Item1.Y = bBox.Min.Y;
                        }
                        if (bBox.Min.Z < localBounds.Item1.Z) {
                            localBounds.Item1.Z = bBox.Min.Z;
                        }
                        if (bBox.Max.X > localBounds.Item2.X) {
                            localBounds.Item2.X = bBox.Max.X;
                        }
                        if (bBox.Max.Y > localBounds.Item2.Y) {
                            localBounds.Item2.Y = bBox.Max.Y;
                        }
                        if (bBox.Max.Z > localBounds.Item2.Z) {
                            localBounds.Item2.Z = bBox.Max.Z;
                        }
                    }

                    return localBounds;
                },
                localBounds => {

                    lock (@lock) {
                        if (localBounds.Item1.X < bounds.Item1.X) {
                            bounds.Item1.X = localBounds.Item1.X;
                        }
                        if (localBounds.Item1.Y < bounds.Item1.Y) {
                            bounds.Item1.Y = localBounds.Item1.Y;
                        }
                        if (localBounds.Item1.Z < bounds.Item1.Z) {
                            bounds.Item1.Z = localBounds.Item1.Z;
                        }
                        if (localBounds.Item2.X > bounds.Item2.X) {
                            bounds.Item2.X = localBounds.Item2.X;
                        }
                        if (localBounds.Item2.Y > bounds.Item2.Y) {
                            bounds.Item2.Y = localBounds.Item2.Y;
                        }
                        if (localBounds.Item2.Z > bounds.Item2.Z) {
                            bounds.Item2.Z = localBounds.Item2.Z;
                        }
                    }
                });

            return new AABox(
                bounds.Item1,
                bounds.Item2);
        }

        public Vector3d Min { get; private set; }
        public Vector3d Max { get; private set; }

        public Vector3d Center {
            get {
                return (Max + Min) / 2;
            }
        }

        public Vector3d Size {
            get {
                return Max - Min;
            }
        }

        public AABox BBox {
            get {
                return this;
            }
        }

        public virtual Mesh Mesh {
            get {

                Point[] vertices = new Point[] {
                    new Point(Min),
                    new Point(new Vector3d(Min.X, Max.Y, Min.Z)),
                    new Point(new Vector3d(Min.X, Max.Y, Max.Z)),
                    new Point(new Vector3d(Min.X, Min.Y, Max.Z)),
                    new Point(new Vector3d(Max.X, Min.Y, Min.Z)),
                    new Point(new Vector3d(Max.X, Max.Y, Min.Z)),
                    new Point(Max),
                    new Point(new Vector3d(Max.X, Min.Y, Max.Z))
                };

                Mesh mesh = new Mesh(
                    vertices,
                    new Face[] {
                        new Face(0, 3, 1, vertices),
                        new Face(1, 3, 2, vertices),
                        new Face(5, 2, 6, vertices),
                        new Face(1, 2, 5, vertices),
                        new Face(2, 3, 6, vertices),
                        new Face(3, 7, 6, vertices),
                        new Face(4, 6, 7, vertices),
                        new Face(4, 5, 6, vertices),
                        new Face(0, 7, 3, vertices),
                        new Face(0, 4, 7, vertices),
                        new Face(0, 1, 4, vertices),
                        new Face(1, 5, 4, vertices)
                    },
                    false
                );

                return mesh;
            }
        }

        public AABox(
                Vector3d min, 
                Vector3d max) {

            Min = min;
            Max = max;
        }

        public AABox(
                IEnumerable<Vector3d> positions) {

            positions.GetMinMax(
                out Vector3d min,
                out Vector3d max);

            Min = min;
            Max = max;
        }

        protected AABox(AABox box) : 
                this(
                    box.Min, 
                    box.Max) {
        }

        public void UpdateBBox() {
            // nothing to to
        }

        public bool Contains(
                Vector3d position) {

            return position.X >= Min.X && position.X <= Max.X
                && position.Y >= Min.Y && position.Y <= Max.Y
                && position.Z >= Min.Z && position.Z <= Max.Z;
        }

        public bool Intersects(
                AABox box) {

            return Min.X <= box.Max.X && Max.X >= box.Min.X
                && Min.Y <= box.Max.Y && Max.Y >= box.Min.Y
                && Min.Z <= box.Max.Z && Max.Z >= box.Min.Z;
        }

        public double DistanceTo(
                Vector3d position) {

            if (Contains(position)) {
                return 0.0;
            }

            return (Vector3d.Clamp(position, Min, Max) - position).Length;
        }

        public bool ApproximateEquals(
                AABox box) {

            return Min.ApproximateEquals(box.Min) 
                && Max.ApproximateEquals(box.Max);
        }
    }
}
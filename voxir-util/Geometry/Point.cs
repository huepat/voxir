using OpenTK.Mathematics;

namespace HuePat.VoxIR.Util.Geometry {
    public class Point : IFiniteGeometry {

        private Vector3d position;

        public Vector3d Position {
            get { return position; }
            set { position = value; }
        }

        public double X {
            get { return position.X; }
            set { position.X = value; }
        }

        public double Y {
            get { return position.Y; }
            set { position.Y = value; }
        }

        public double Z {
            get { return position.Z; }
            set { position.Z = value; }
        }

        public AABox BBox {
            get {
                return new AABox(Position, Position);
            }
        }

        public Mesh Mesh {
            get {
                return new Mesh(
                    new Point[] { this },
                    new Face[0],
                    false);
            }
        }

        public Point(
                double x, 
                double y, 
                double z) :
                    this(new Vector3d(x, y, z)) {
        }

        public Point(
                Vector3d position) {

            this.position = position;
        }

        public bool Intersects(
                AABox box) {

            return
                X >= box.Min.X && X <= box.Max.X &&
                Y >= box.Min.Y && Y <= box.Max.Y &&
                Z >= box.Min.Z && Z <= box.Max.Z;
        }
    }
}
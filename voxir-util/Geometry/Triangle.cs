using OpenTK.Mathematics;

namespace HuePat.VoxIR.Util.Geometry {
    public class Triangle : IFiniteGeometry {
        public Vector3d Corner1 { get; private set; }
        public Vector3d Corner2 { get; private set; }
        public Vector3d Corner3 { get; private set; }

        public double Area {
            get {

                double a = Corner1.DistanceTo(Corner2);
                double b = Corner1.DistanceTo(Corner3);
                double c = Corner2.DistanceTo(Corner3);
                double s = (a + b + c) / 2.0;

                return (s * (s - a) * (s - b) * (s - c)).Sqrt();
            }
        }

        public Vector3d Normal {
            get {
                return Vector3d.Normalize(
                    Vector3d.Cross(
                        Corner2 - Corner1,
                        Corner3 - Corner1));
            }
        }

        public AABox BBox {
            get {
                return new AABox(
                    new Vector3d[] {
                        Corner1,
                        Corner2,
                        Corner3
                    });
            }
        }

        public Vector3d Centroid {
            get {
                return new Vector3d(
                    (Corner1.X + Corner2.X + Corner3.X) / 3.0,
                    (Corner1.Y + Corner2.Y + Corner3.Y) / 3.0,
                    (Corner1.Z + Corner2.Z + Corner3.Z) / 3.0);
            }
        }

        public Mesh Mesh {
            get {

                Point[] vertices = new Point[] {
                    new Point(Corner1),
                    new Point(Corner2),
                    new Point(Corner3)
                };

                Mesh mesh = new Mesh(
                    vertices,
                    new Face[] {
                        new Face(
                            0, 
                            1, 
                            2, 
                            vertices)
                    },
                    false);

                return mesh;
            }
        }

        public Triangle(
                Vector3d corner1,
                Vector3d corner2,
                Vector3d corner3) {

            Corner1 = corner1;
            Corner2 = corner2;
            Corner3 = corner3;
        }

        // Adapted to C# from:
        // Tomas Akenine-Möller. Triangle-Box Overlap Test
        // https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/
        public bool Intersects(AABox box) {

            if (box.Contains(Corner1)
                    || box.Contains(Corner2)
                    || box.Contains(Corner3)) {
                return true;
            }

            double rad, min, max, p0, p1, p2, fex, fey, fez;
            Vector3d boxCenter = box.Center;
            Vector3d boxhalfsize = 0.5 * box.Size;
            Vector3d v0 = Corner1 - boxCenter;
            Vector3d v1 = Corner2 - boxCenter;
            Vector3d v2 = Corner3 - boxCenter;
            Vector3d e0 = v1 - v0;
            Vector3d e1 = v2 - v1;
            Vector3d e2 = v0 - v2;

            fex = e0.X.Abs();
            fey = e0.Y.Abs();
            fez = e0.Z.Abs();

            p0 = e0.Z * v0.Y - e0.Y * v0.Z;
            p2 = e0.Z * v2.Y - e0.Y * v2.Z;
            if (p0 < p2) {
                min = p0;
                max = p2;
            }
            else {
                min = p2;
                max = p0;
            }
            rad = fez * boxhalfsize.Y + fey * boxhalfsize.Z;
            if (min > rad || max < -rad) {
                return false;
            }

            p0 = -e0.Z * v0.X + e0.X * v0.Z;
            p2 = -e0.Z * v2.X + e0.X * v2.Z;
            if (p0 < p2) {
                min = p0;
                max = p2;
            }
            else {
                min = p2;
                max = p0;
            }
            rad = fez * boxhalfsize.X + fex * boxhalfsize.Z;
            if (min > rad || max < -rad) {
                return false;
            }

            p1 = e0.Y * v1.X - e0.X * v1.Y;
            p2 = e0.Y * v2.X - e0.X * v2.Y;
            if (p2 < p1) {
                min = p2;
                max = p1;
            }
            else {
                min = p1;
                max = p2;
            }
            rad = fey * boxhalfsize.X + fex * boxhalfsize.Y;
            if (min > rad || max < -rad) {
                return false;
            }

            fex = e1.X.Abs();
            fey = e1.Y.Abs();
            fez = e1.Z.Abs();

            p0 = e1.Z * v0.Y - e1.Y * v0.Z;
            p2 = e1.Z * v2.Y - e1.Y * v2.Z;
            if (p0 < p2) {
                min = p0;
                max = p2;
            }
            else {
                min = p2;
                max = p0;
            }
            rad = fez * boxhalfsize.Y + fey * boxhalfsize.Z;
            if (min > rad || max < -rad) {
                return false;
            }

            p0 = -e1.Z * v0.X + e1.X * v0.Z;
            p2 = -e1.Z * v2.X + e1.X * v2.Z;
            if (p0 < p2) {
                min = p0;
                max = p2;
            }
            else {
                min = p2;
                max = p0;
            }
            rad = fez * boxhalfsize.X + fex * boxhalfsize.Z;
            if (min > rad || max < -rad) {
                return false;
            }

            p0 = e1.Y * v0.X - e1.X * v0.Y;
            p1 = e1.Y * v1.X - e1.X * v1.Y;
            if (p0 < p1) {
                min = p0;
                max = p1;
            }
            else {
                min = p1;
                max = p0;
            }
            rad = fey * boxhalfsize.X + fex * boxhalfsize.Y;
            if (min > rad || max < -rad) {
                return false;
            }

            fex = e2.X.Abs();
            fey = e2.Y.Abs();
            fez = e2.Z.Abs();

            p0 = e2.Z * v0.Y - e2.Y * v0.Z;
            p1 = e2.Z * v1.Y - e2.Y * v1.Z;
            if (p0 < p1) {
                min = p0;
                max = p1;
            }
            else {
                min = p1;
                max = p0;
            }
            rad = fez * boxhalfsize.Y + fey * boxhalfsize.Z;
            if (min > rad || max < -rad) {
                return false;
            }

            p0 = -e2.Z * v0.X + e2.X * v0.Z;
            p1 = -e2.Z * v1.X + e2.X * v1.Z;
            if (p0 < p1) {
                min = p0;
                max = p1;
            }
            else {
                min = p1;
                max = p0;
            }
            rad = fez * boxhalfsize.X + fex * boxhalfsize.Z;
            if (min > rad || max < -rad) {
                return false;
            }

            p1 = e2.Y * v1.X - e2.X * v1.Y;
            p2 = e2.Y * v2.X - e2.X * v2.Y;
            if (p2 < p1) {
                min = p2;
                max = p1;
            }
            else {
                min = p1;
                max = p2;
            }
            rad = fey * boxhalfsize.X + fex * boxhalfsize.Y;
            if (min > rad || max < -rad) {
                return false;
            }

            min = max = v0.X;
            if (v1.X < min) {
                min = v1.X;
            }
            if (v1.X > max) {
                max = v1.X;
            }
            if (v2.X < min) {
                min = v2.X;
            }
            if (v2.X > max) {
                max = v2.X;
            }
            if (min > boxhalfsize.X || max < -boxhalfsize.X) {
                return false;
            }

            min = max = v0.Y;
            if (v1.Y < min) {
                min = v1.Y;
            }
            if (v1.Y > max) {
                max = v1.Y;
            }
            if (v2.Y < min) {
                min = v2.Y;
            }
            if (v2.Y > max) {
                max = v2.Y;
            }
            if (min > boxhalfsize.Y || max < -boxhalfsize.Y) {
                return false;
            }

            min = max = v0.Z;
            if (v1.Z < min) {
                min = v1.Z;
            }
            if (v1.Z > max) {
                max = v1.Z;
            }
            if (v2.Z < min) {
                min = v2.Z;
            }
            if (v2.Z > max) {
                max = v2.Z;
            }
            if (min > boxhalfsize.Z || max < -boxhalfsize.Z) {
                return false;
            }

            if (!PlaneBoxOverlap(v0, boxhalfsize)) {
                return false;
            }
            return true;
        }

        // Adapted to C# from:
        // David Eberly. GeometricTools. GteDistPointTriangleExact
        // https://github.com/davideberly/GeometricTools
        public double DistanceTo(Vector3d position) {

            Vector3d D = position - Corner1;
            Vector3d E0 = Corner2 - Corner1;
            Vector3d E1 = Corner3 - Corner1;
            double a = Vector3d.Dot(E0, E0);
            double b = Vector3d.Dot(E0, E1);
            double c = Vector3d.Dot(E1, E1);
            double d = -Vector3d.Dot(D, E0);
            double e = -Vector3d.Dot(D, E1);
            double det = a * c - b * b;
            double s = b * e - c * d;
            double t = b * d - a * e;

            if (s + t <= det) {
                if (s < 0.0) {
                    if (t < 0.0) {
                        if (d < 0.0) {
                            t = 0.0;
                            if (-d >= a) {
                                s = 1.0;
                            }
                            else {
                                s = -d / a;
                            }
                        }
                        else {
                            s = 0.0;
                            if (e >= 0.0) {
                                t = 0.0;
                            }
                            else if (-e >= c) {
                                t = 1.0;
                            }
                            else {
                                t = -e / c;
                            }
                        }
                    }
                    else {
                        s = 0.0;
                        if (e >= 0.0) {
                            t = 0.0;
                        }
                        else if (-e >= c) {
                            t = 1.0;
                        }
                        else {
                            t = -e / c;
                        }
                    }
                }
                else if (t < 0.0) {
                    t = 0.0;
                    if (d >= 0.0) {
                        s = 0.0;
                    }
                    else if (-d >= a) {
                        s = 1.0;
                    }
                    else {
                        s = -d / a;
                    }
                }
                else {
                    double invDet = 1.0 / det;
                    s *= invDet;
                    t *= invDet;
                }
            }
            else {
                double tmp0, tmp1, numer, denom;
                if (s < 0.0) {
                    tmp0 = b + d;
                    tmp1 = c + e;
                    if (tmp1 > tmp0) {
                        numer = tmp1 - tmp0;
                        denom = a - 2.0 * b + c;
                        if (numer >= denom) {
                            s = 1.0;
                            t = 0.0;
                        }
                        else {
                            s = numer / denom;
                            t = 1.0 - s;
                        }
                    }
                    else {
                        s = 0.0;
                        if (tmp1 <= 0.0) {
                            t = 1.0;
                        }
                        else if (e >= 0.0) {
                            t = 0.0;
                        }
                        else {
                            t = -e / c;
                        }
                    }
                }
                else if (t < 0.0) {
                    tmp0 = b + e;
                    tmp1 = a + d;
                    if (tmp1 > tmp0) {
                        numer = tmp1 - tmp0;
                        denom = a - 2.0 * b + c;
                        if (numer >= denom) {
                            t = 1.0;
                            s = 0.0;
                        }
                        else {
                            t = numer / denom;
                            s = 1.0 - t;
                        }
                    }
                    else {
                        t = 0.0;
                        if (tmp1 <= 0.0) {
                            s = 1.0;
                        }
                        else if (d >= 0.0) {
                            s = 0.0;
                        }
                        else {
                            s = -d / a;
                        }
                    }
                }
                else {
                    numer = c + e - b - d;
                    if (numer <= 0.0) {
                        s = 0.0;
                        t = 1.0;
                    }
                    else {
                        denom = a - 2.0 * b + c;
                        if (numer >= denom) {
                            s = 1.0;
                            t = 0.0;
                        }
                        else {
                            s = numer / denom;
                            t = 1.0 - s;
                        }
                    }
                }
            }

            return (position - Corner1 + s * E0 + t * E1).Length;
        }

        // Adapted to C# from:
        // Tomas Akenine-Möller. Triangle-Box Overlap Test
        // https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/
        private bool PlaneBoxOverlap(
                Vector3d vertex, 
                Vector3d maxbox) {

            double v;
            Vector3d vmin = new Vector3d();
            Vector3d vmax = new Vector3d();

            for (int i = 0; i <= 2; i++) {
                v = vertex[i];
                if (Normal[i] > 0.0) {
                    vmin[i] = -maxbox[i] - v;
                    vmax[i] = maxbox[i] - v;
                }
                else {
                    vmin[i] = maxbox[i] - v;
                    vmax[i] = -maxbox[i] - v;
                }
            }

            if (Vector3d.Dot(Normal, vmin) > 0.0) {
                return false;
            }

            if (Vector3d.Dot(Normal, vmax) >= 0.0) {
                return true;
            }

            return false;
        }
    }
}
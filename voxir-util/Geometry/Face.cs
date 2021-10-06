using System.Collections.Generic;

namespace HuePat.VoxIR.Util.Geometry {
    public class Face: IFiniteGeometry {
        public int VertexIndex1 { get; private set; }
        public int VertexIndex2 { get; private set; }
        public int VertexIndex3 { get; private set; }
        public Point Vertex1 { get; private set; }
        public Point Vertex2 { get; private set; }
        public Point Vertex3 { get; private set; }

        public Triangle Geometry {
            get {
                return new Triangle(
                    Vertex1.Position,
                    Vertex2.Position,
                    Vertex3.Position);
            }
        }

        public Mesh Mesh {
            get {
                return Geometry.Mesh;
            }
        }

        public AABox BBox { 
            get {
                return Geometry.BBox;
            }
        }

        public Face(
                int vertexIndex1,
                int vertexIndex2,
                int vertexIndex3,
                IReadOnlyList<Point> vertices) {
            VertexIndex1 = vertexIndex1;
            VertexIndex2 = vertexIndex2;
            VertexIndex3 = vertexIndex3;
            Vertex1 = vertices[VertexIndex1];
            Vertex2 = vertices[VertexIndex2];
            Vertex3 = vertices[VertexIndex3];
        }

        public bool Intersects(
                AABox box) {

            return Geometry.Intersects(box);
        }
    }
}
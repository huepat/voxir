using HuePat.VoxIR.Util.Geometry;
using System.IO;

namespace HuePat.VoxIR.IO.PLY.Writing {
    class AsciiEncoder : IEncoder {
        private StreamWriter writer;

        public AsciiEncoder(
                string file) {

            writer = new StreamWriter(
                file, 
                true);
        }

        public void Dispose() {

            writer.Dispose();
        }

        public void Encode(
                Point point) {

            writer.WriteLine(
                $"{(float)point.X} {(float)point.Y} {(float)point.Z}");
        }

        public void Encode(
                Point point, 
                Color color) {

            writer.WriteLine(
                $"{(float)point.X} {(float)point.Y} {(float)point.Z} " +
                $"{color.R} {color.G} {color.B}");
        }

        public void Encode(
                int offset,
                Face face) {

            writer.WriteLine(
                $"3 {face.VertexIndex1 + offset} " +
                $"{face.VertexIndex2 + offset} " +
                $"{face.VertexIndex3 + offset}");
        }
    }
}
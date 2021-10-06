using HuePat.VoxIR.Util.Geometry;
using System.IO;
using System.Text;

namespace HuePat.VoxIR.IO.PLY.Writing {
    class BinaryEncoder : IEncoder {
        private bool coordinatesAsFloat;
        private BinaryWriter writer;

        public BinaryEncoder(
                bool littleEndian,
                bool coordinatesAsFloat,
                string file) {

            this.coordinatesAsFloat = coordinatesAsFloat;
            writer = new BinaryWriter(
                File.Open(
                    file,
                    FileMode.Append,
                    FileAccess.Write),
                littleEndian ?
                    Encoding.Unicode :
                    Encoding.BigEndianUnicode);
        }

        public void Dispose() {

            writer.Dispose();
        }

        public void Encode(
                Point point) {

            if (coordinatesAsFloat) {
                writer.Write((float)point.X);
                writer.Write((float)point.Y);
                writer.Write((float)point.Z);
            }
            else {
                writer.Write(point.X);
                writer.Write(point.Y);
                writer.Write(point.Z);
            }
        }

        public void Encode(
                Point point,
                Color color) {

            Encode(point);
            writer.Write(color.R);
            writer.Write(color.G);
            writer.Write(color.B);
        }

        public void Encode(
                int offset,
                Face face) {

            writer.Write((byte)3);
            writer.Write(face.VertexIndex1 + offset);
            writer.Write(face.VertexIndex2 + offset);
            writer.Write(face.VertexIndex3 + offset);
        }
    }
}
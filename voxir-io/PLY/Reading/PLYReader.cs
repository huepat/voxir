using System.IO;
using System;
using HuePat.VoxIR.Util.Geometry;

namespace HuePat.VoxIR.IO.PLY.Reading {
    public class PLYReader {
        public const int DEFAULT_LINEBREAK_BYTE_SIZE = 1;

        private bool areVertexCoordinatesFloat;
        private HeaderParser headerParser;

        public bool InvertNormals { private get; set; }
        public (string, string, string) CoordinateIndentifiers { private get; set; }

        public bool AreVertexCoordinatesFloat {
            set {
                areVertexCoordinatesFloat = value;
                headerParser.AreVertexCoordinatesFloat = value;
            }
        }

        public int LineBreakByteSize {
            set {
                headerParser.LineBreakByteSize = value;
            }
        }

        public PLYReader() {

            headerParser = new HeaderParser() {
                LineBreakByteSize = DEFAULT_LINEBREAK_BYTE_SIZE
            };
            AreVertexCoordinatesFloat = true;
            CoordinateIndentifiers = ("x", "y", "z");
        }

        public PointCloud ReadPointCloud(
                string file,
                bool createBBox = true,
                bool useParallelForBBox = true) {

            Header header;
            PointCloud pointCloud;
            IDecoder decoder;

            using (InvariantCultureBlock block = new InvariantCultureBlock()) {

                header = ReadHeader(file);
                decoder = GetDecoder(header);

                pointCloud = new PointCloud( 
                    decoder.ReadPoints(
                        file,
                        header),
                    createBBox,
                    useParallelForBBox);
            }

            return pointCloud;
        }

        public Mesh ReadMesh(
                string file,
                bool createBBox = true,
                bool useParallelForBBox = true) {

            Mesh mesh;
            Header header;
            IDecoder decoder;

            using (InvariantCultureBlock block = new InvariantCultureBlock()) {

                header = ReadHeader(file);
                decoder = GetDecoder(header);
                mesh = decoder.ReadMesh(
                    InvertNormals,
                    createBBox,
                    useParallelForBBox,
                    file,
                    header);
            }

            return mesh;
        }

        private Header ReadHeader(
                string file) {

            Header header;

            headerParser.Initialize(CoordinateIndentifiers);

            foreach (string line in File.ReadLines(file)) {

                header = headerParser.ParseHeaderLine(line);

                if (header != null) {
                    return header;
                }
            }

            return null;
        }

        private IDecoder GetDecoder(
                Header header) {

            IDecoder decoder;

            switch (header.Encoding) {
                case PLYEncoding.BINARY_LITTLE_ENDIAN:
                    decoder = new BinaryDecoder(
                        true,
                        areVertexCoordinatesFloat);
                    break;
                case PLYEncoding.BINARY_BIG_ENDIAN:
                    decoder = new BinaryDecoder(
                        false,
                        areVertexCoordinatesFloat);
                    break;
                case PLYEncoding.ASCII:
                    decoder = new AsciiDecoder();
                    break;
                default:
                    throw new ApplicationException();
            }

            return decoder;
        }
    }
}
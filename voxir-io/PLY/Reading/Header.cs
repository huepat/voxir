namespace HuePat.VoxIR.IO.PLY.Reading {
    class Header {
        public PLYEncoding Encoding { get; private set; }
        public int HeaderLineCount { get; private set; }
        public int VertexSectionStartPosition { get; private set; }
        public VertexSection VertexSection { get; private set; }
        public FaceSection FaceSection { get; private set; }

        public Header(
                int headerLineCount, 
                int vertexSectionStartPosition,
                PLYEncoding encoding,
                VertexSection vertexSection,
                FaceSection faceSection) {

            Encoding = encoding;
            HeaderLineCount = headerLineCount;
            VertexSectionStartPosition = vertexSectionStartPosition;
            VertexSection = vertexSection;
            FaceSection = faceSection;
        }
    }
}
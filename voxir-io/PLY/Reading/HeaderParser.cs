namespace HuePat.VoxIR.IO.PLY.Reading {
    class HeaderParser {
        private const string VERTEX_COUNT_POSTFIX = "element vertex ";
        private const string FACE_COUNT_POSTFIX = "element face ";
        private const string FORMAT_POSTFIX = "format ";
        private const string FORMAT_BINARY_LITTLE_ENDIAN_POSTFIX = "format binary_little_endian";
        private const string FORMAT_BINARY_BIG_ENDIAN_POSTFIX = "format binary_big_endian";
        private const string FORMAT_ASCII_POSTFIX = "format ascii";

        private enum HeaderSection {
            START, VERTICES, FACES
        }

        private PLYEncoding encoding;
        private HeaderSection section = HeaderSection.START;
        private int headerLineCount;
        private int positionCount;
        private int vertexCount;
        private int faceCount;
        private VertexSectionParser vertexSectionParser;
        private FaceSectionParser faceSectionParser;

        public bool AreVertexCoordinatesFloat {
            set {
                vertexSectionParser.AreCoordinatesFloat = value;
            }
        }

        public int LineBreakByteSize { private get; set; }

        public HeaderParser() {

            vertexSectionParser = new VertexSectionParser();
            faceSectionParser = new FaceSectionParser();
        }

        public void Initialize(
                (string, string, string) coordinateIdentifiers) {

            headerLineCount = 0;
            positionCount = 0;
            vertexCount = 0;
            faceCount = 0;
            vertexSectionParser.Initialize(coordinateIdentifiers);
            faceSectionParser.Initialize();
        }

        public Header ParseHeaderLine(
                string line) {

            headerLineCount++;
            positionCount += line.Length + LineBreakByteSize;

            if (line.StartsWith(FORMAT_POSTFIX)) {
                if (line.StartsWith(FORMAT_ASCII_POSTFIX)) {
                    encoding = PLYEncoding.ASCII;
                }
                else if(line.StartsWith(FORMAT_BINARY_LITTLE_ENDIAN_POSTFIX)) {
                    encoding = PLYEncoding.BINARY_LITTLE_ENDIAN;
                }
                else if (line.StartsWith(FORMAT_BINARY_BIG_ENDIAN_POSTFIX)) {
                    encoding = PLYEncoding.BINARY_BIG_ENDIAN;
                }
            }
            else if (line.StartsWith(VERTEX_COUNT_POSTFIX)) {
                section = HeaderSection.VERTICES;
                vertexCount = ParseCount(
                    line, 
                    VERTEX_COUNT_POSTFIX);
            }
            else if (section == HeaderSection.VERTICES 
                    && line.StartsWith("property")) {
                string[] propertyParts = line.Split(' ');
                vertexSectionParser.ParsePropertyIndex(
                    propertyParts[1], 
                    propertyParts[2]);
            }
            else if (line.StartsWith(FACE_COUNT_POSTFIX)) {
                section = HeaderSection.FACES;
                faceCount = ParseCount(line, FACE_COUNT_POSTFIX);
            }
            else if (section == HeaderSection.FACES && line.StartsWith("property") 
                    && !line.StartsWith("property list")) {
                string[] propertyParts = line.Split(' ');
                faceSectionParser.ParsePropertyIndex(
                    propertyParts[1], 
                    propertyParts[2]);
            }
            else if (line.Equals("end_header")) {
                return new Header(
                    headerLineCount,
                    positionCount,
                    encoding,
                    vertexSectionParser.Create(vertexCount),
                    faceSectionParser.Create(faceCount));
            }

            return null;
        }

        private int ParseCount(
                string line, 
                string postfix) {

            return int.Parse(
                line.Substring(
                    postfix.Length));
        }
    }
}
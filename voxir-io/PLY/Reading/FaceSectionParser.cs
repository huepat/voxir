namespace HuePat.VoxIR.IO.PLY.Reading {
    class FaceSectionParser : HeaderSectionParser {
        private const int INDEX_OFFSET = 4;

        public override string SectionName {
            get {
                return "face";
            }
        }

        public FaceSection Create(
                int count) {

            return new FaceSection(
                Create(
                    count, 
                    INDEX_OFFSET));
        }
    }
}
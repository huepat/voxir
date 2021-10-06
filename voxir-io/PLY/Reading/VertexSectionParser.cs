using System.Collections.Generic;

namespace HuePat.VoxIR.IO.PLY.Reading {
    class VertexSectionParser : HeaderSectionParser {
        private (string, string, string) coordinateIdentifiers;
        private Dictionary<string, int> coordinateIndices;

        public bool AreCoordinatesFloat { private get; set; }

        public override string SectionName {
            get {
                return "vertex";
            }
        }

        public void Initialize(
                (string, string, string) coordinateIdentifiers) {

            this.coordinateIdentifiers = coordinateIdentifiers;
            Initialize();
        }

        public override void Initialize() {

            base.Initialize();
            InitializeCoordinateIndices();
        }

        public override void ParsePropertyIndex(
                string propertyType, 
                string propertyName) {

            if (coordinateIndices.ContainsKey(propertyName)
                    && (propertyType == FLOAT_TYPE || propertyType == DOUBLE_TYPE)) {

                coordinateIndices[propertyName] = propertyIndex++;
            }
            else {
                base.ParsePropertyIndex(propertyType, propertyName);
            }
        }

        public new VertexSection Create(
                int count, 
                int indexOffset = 0) {

            VertexSection section = new VertexSection(
                AreCoordinatesFloat,
                base.Create(
                    count, 
                    indexOffset));

            section.CoordinateIndices = new int[] {
                coordinateIndices[coordinateIdentifiers.Item1],
                coordinateIndices[coordinateIdentifiers.Item2],
                coordinateIndices[coordinateIdentifiers.Item3]
            };

            return section;
        }

        protected override void Check() {

            base.Check();

            foreach (string identifier in coordinateIndices.Keys) {
                if (coordinateIndices[identifier] == NOT_SET_INDEX) {
                    ReportUnsetPropertyIndex("coordinate component", identifier);
                }
            }
        }

        private void InitializeCoordinateIndices() {

            coordinateIndices = new Dictionary<string, int>();
            coordinateIndices.Add(
                coordinateIdentifiers.Item1, 
                NOT_SET_INDEX);
            coordinateIndices.Add(
                coordinateIdentifiers.Item2, 
                NOT_SET_INDEX);
            coordinateIndices.Add(
                coordinateIdentifiers.Item3, 
                NOT_SET_INDEX);
        }
    }
}
using System.Collections.Generic;

namespace HuePat.VoxIR.IO.PLY.Reading {
    class VertexSection : HeaderSection {
        private bool areCoordinatesFloat;
        public int[] CoordinateIndices { get; set; }

        public override Dictionary<int, PropertyType> PropertyTypes {
            get {

                Dictionary<int, PropertyType> types = base.PropertyTypes;

                if (areCoordinatesFloat) {
                    types.Add(
                        CoordinateIndices[0], 
                        PropertyType.FLOAT);
                    types.Add(
                        CoordinateIndices[1], 
                        PropertyType.FLOAT);
                    types.Add(
                        CoordinateIndices[2], 
                        PropertyType.FLOAT);
                }
                else {
                    types.Add(
                        CoordinateIndices[0], 
                        PropertyType.DOUBLE);
                    types.Add(
                        CoordinateIndices[1], 
                        PropertyType.DOUBLE);
                    types.Add(
                        CoordinateIndices[2], 
                        PropertyType.DOUBLE);
                }

                return types;
            }
        }

        public VertexSection(
                bool areCoordinatesFloat,
                HeaderSection headerSection) : 
                    base(headerSection) {

            this.areCoordinatesFloat = areCoordinatesFloat;
        }
    }
}
using System;
using System.Collections.Generic;

namespace HuePat.VoxIR.IO.PLY.Reading {
    abstract class HeaderSectionParser {
        protected const int NOT_SET_INDEX = -1;
        protected const string INTEGER_TYPE = "int";
        protected const string FLOAT_TYPE = "float";
        protected const string DOUBLE_TYPE = "double";
        protected const string BYTE_TYPE = "uchar";

        protected int propertyIndex;
        private Dictionary<int, PropertyType> indicesNotInFormat;

        public abstract string SectionName { get; }

        public virtual void Initialize() {

            propertyIndex = 0;
            indicesNotInFormat = new Dictionary<int, PropertyType>();
        }

        public virtual void ParsePropertyIndex(
                string propertyTypeIdentifier, 
                string propertyName) {

            PropertyType propertyType;

            switch (propertyTypeIdentifier) {
                case BYTE_TYPE:
                    propertyType = PropertyType.BYTE;
                    break;
                case INTEGER_TYPE:
                    propertyType = PropertyType.INTEGER;
                    break;
                case FLOAT_TYPE:
                    propertyType = PropertyType.FLOAT;
                    break;
                case DOUBLE_TYPE:
                    propertyType = PropertyType.DOUBLE;
                    break;
                default:
                    throw new ArgumentException();
            }

            indicesNotInFormat.Add(
                propertyIndex++,
                propertyType);
        }

        protected HeaderSection Create(
                int count, 
                int indexOffset = 0) {

            HeaderSection section = new HeaderSection(count);

            section.IndicesNotInFormat = new Dictionary<int, PropertyType>();

            foreach (int index in indicesNotInFormat.Keys) {
                section.IndicesNotInFormat.Add(
                    index + indexOffset,
                    indicesNotInFormat[index]);
            }

            return section;
        }

        protected virtual void Check() {
        }

        protected void ReportUnsetPropertyIndex(
                string identifier, 
                string type) {

            throw new ArgumentException(
                $"Header does not contain expected {SectionName} {type} identifier '{identifier}'.");
        }
    }
}
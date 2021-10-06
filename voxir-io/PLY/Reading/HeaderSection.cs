using System.Collections.Generic;

namespace HuePat.VoxIR.IO.PLY.Reading {
    class HeaderSection {
        public int Count { get; private set; }
        public Dictionary<int, PropertyType> IndicesNotInFormat { get; set; }

        public virtual Dictionary<int, PropertyType> PropertyTypes {
            get {

                Dictionary<int, PropertyType> types = new Dictionary<int, PropertyType>();

                foreach (int index in IndicesNotInFormat.Keys) {
                    types.Add(
                        index,
                        IndicesNotInFormat[index]);
                }

                return types;
            }
        }

        public HeaderSection(
                int count) {

            Count = count;
            IndicesNotInFormat = new Dictionary<int, PropertyType>();
        }

        public HeaderSection(
                HeaderSection headerSection) {

            Count = headerSection.Count;
            IndicesNotInFormat = headerSection.IndicesNotInFormat;
        }
    }
}
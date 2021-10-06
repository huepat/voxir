using System.Collections.Generic;
using System.Linq;

namespace HuePat.VoxIR.RoomSegmentation {
    class IdSet {
        private HashSet<int> ids;

        public IdSet(HashSet<int> ids) {
            this.ids = ids;
        }

        public override int GetHashCode() {

            int hashCode = 13;

            foreach (int id in ids) {
                hashCode = (hashCode * 7) + id;
            }

            return hashCode;
        }

        public override bool Equals(object obj) {

            if (obj == null
                    || !(obj is IdSet)) {
                return false;
            }

            IdSet other = obj as IdSet;

            if (ids.Count != other.ids.Count) {
                return false;
            }

            return other
                .ids
                .All(id => ids.Contains(id));
        }
    }
}
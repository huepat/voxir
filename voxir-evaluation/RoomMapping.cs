using System.Collections.Generic;
using System.Linq;

namespace HuePat.VoxIR.Evaluation {
    static class RoomMapping {
        public static void Update(
                this Dictionary<int, Dictionary<int, int>> roomMapping,
                Dictionary<int, Dictionary<int, int>> roomMappingUpdate) {

            foreach (int fromRoomId in roomMappingUpdate.Keys) {

                if (!roomMapping.ContainsKey(fromRoomId)) {
                    roomMapping.Add(
                        fromRoomId,
                        new Dictionary<int, int>());
                }

                foreach (int toRoomId in roomMappingUpdate[fromRoomId].Keys) {
                    roomMapping[fromRoomId].BucketAdd(
                        toRoomId,
                        roomMappingUpdate[fromRoomId][toRoomId]);
                }
            }
        }

        public static void Update(
                this Dictionary<int, Dictionary<int, int>> roomMapping,
                int[] fromRoomIds,
                int[] toRoomIds) {

            foreach (int fromRoomId in fromRoomIds) {

                if (!roomMapping.ContainsKey(fromRoomId)) {
                    roomMapping.Add(
                        fromRoomId,
                        new Dictionary<int, int>());
                }

                foreach (int toRoomId in toRoomIds) {
                    roomMapping[fromRoomId].BucketIncrement(toRoomId);
                }
            }
        }

        public static int? Get(
                this Dictionary<int, Dictionary<int, int>> roomMapping,
                int fromId) {

            if (!roomMapping.ContainsKey(fromId)) {
                return null;
            }

            List<int> result = roomMapping[fromId]
                .Keys
                .Where(toId => fromId * toId > 0)
                .WhereMax(toId => roomMapping[fromId][toId])
                .Distinct()
                .ToList();

            return result.Count == 1 ?
                result[0] :
                null;
        }
    }
}
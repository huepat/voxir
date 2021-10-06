using HuePat.VoxIR.IO.PLY.Reading;
using HuePat.VoxIR.Util.Geometry;
using System;
using System.Collections.Generic;
using System.IO;

namespace HuePat.VoxIR.Datasets {
    public static class GroundTruthReader {
        public static Dictionary<int, List<(Mesh, GroundTruthInfo)>> LoadGroundTruthMeshes(
                string directory) {

            int roomId;
            string fileName;
            PLYReader reader = new PLYReader();
            Dictionary<int, List<(Mesh, GroundTruthInfo)>> groundTruthMeshes
                = new Dictionary<int, List<(Mesh, GroundTruthInfo)>>();

            foreach (string filePath in Directory.GetFiles(directory)) {

                fileName = Path.GetFileNameWithoutExtension(filePath);
                if (!fileName.StartsWith("R")
                        && !fileName.StartsWith("T")) {
                    continue;
                }

                roomId = GetRoomId(fileName);

                groundTruthMeshes.BucketAdd(
                    roomId,
                    (
                        reader.ReadMesh(filePath),
                        new GroundTruthInfo(
                            IsRampSpace(fileName),
                            roomId,
                            GetClassValue(fileName))
                    ));
            }

            return groundTruthMeshes;
        }

        private static int GetRoomId(
                string fileName) {

            int j = 1;
            int roomId;

            while (true) {
                if (!char.IsDigit(fileName[j])) {
                    break;
                }
                j++;
            }

            if (j == 1) {
                throw new ApplicationException();
            }

            roomId = int.Parse(
                fileName.Substring(1, j - 1));

            if (fileName.StartsWith("T")) {
                roomId *= -1;
            }

            return roomId;
        }

        private static int GetClassValue(
                string fileName) {

            string[] substrings = fileName.Split('_');

            if (substrings[1].StartsWith("C")) {
                return VoxelClassValues.CEILING;
            }
            if (substrings[1].StartsWith("F")) {
                return VoxelClassValues.FLOOR;
            }
            if (substrings[1].StartsWith("WO")) {
                return VoxelClassValues.WALL_OPENING;
            }
            if (substrings[1].StartsWith("W")) {
                return VoxelClassValues.WALL;
            }
            if (substrings[1].StartsWith("IO")) {
                return VoxelClassValues.INTERIOR_OBJECT;
            }

            throw new ApplicationException();
        }

        private static bool IsRampSpace(
                string fileName) {

            return fileName
                .Split('_')[0]
                .EndsWith("R");
        }
    }
}
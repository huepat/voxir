using HuePat.VoxIR.IO.ISPRS;
using HuePat.VoxIR.IO.PLY.Reading;
using HuePat.VoxIR.Util.Geometry;
using OpenTK.Mathematics;
using System.Collections.Generic;
using System.IO;

namespace HuePat.VoxIR.Evaluation.ISPRS {
    static class GroundTruthReader {

        public static Dictionary<int, List<Mesh>> ReadGroundTruthMeshes(
                string directory,
                Vector3d rotationAnchorPoint,
                out AABox extent) {

            string fileName;
            int groundTruthClassValue;
            Mesh mesh;
            PLYReader reader = new PLYReader();
            Dictionary<int, List<Mesh>> groundTruth = new Dictionary<int, List<Mesh>>();

            foreach (string file in Directory.EnumerateFiles(directory)) {

                fileName = Path.GetFileName(file);

                if (fileName.StartsWith("C")) {
                    groundTruthClassValue = GroundTruthClassValues.CEILING;
                }
                else if (fileName.StartsWith("F")) {
                    groundTruthClassValue = GroundTruthClassValues.FLOOR;
                }
                else if (fileName.StartsWith("W")) {
                    groundTruthClassValue = GroundTruthClassValues.WALL;
                }
                else if (fileName.StartsWith("NC")) {
                    groundTruthClassValue = GroundTruthClassValues.NOT_CEILING;
                }
                else if (fileName.StartsWith("NF")) {
                    groundTruthClassValue = GroundTruthClassValues.NOT_FLOOR;
                }
                else if (fileName.StartsWith("OC")) {
                    groundTruthClassValue = GroundTruthClassValues.OPENING_CEILING;
                }
                else {
                    continue;
                }

                mesh = reader.ReadMesh(
                    file,
                    false,
                    false);

                groundTruth.BucketAdd(
                    groundTruthClassValue,
                    mesh);
            }

            Rotate(
                rotationAnchorPoint,
                groundTruth,
                out extent);

            return groundTruth;
        }

        private static void Rotate(
                Vector3d rotationAnchorPoint,
                Dictionary<int, List<Mesh>> groundTruth,
                out AABox extent) {

            Vector3d min = new Vector3d(double.MaxValue);
            Vector3d max = new Vector3d(double.MinValue);

            foreach (List<Mesh> meshes in groundTruth.Values) {
                foreach (Mesh mesh in meshes) {

                    ISPRSPointCloudReader.Rotate(
                        rotationAnchorPoint,
                        mesh.Vertices);

                    if (mesh.BBox.Min.X < min.X) {
                        min.X = mesh.BBox.Min.X;
                    }
                    if (mesh.BBox.Min.Y < min.Y) {
                        min.Y = mesh.BBox.Min.Y;
                    }
                    if (mesh.BBox.Min.Z < min.Z) {
                        min.Z = mesh.BBox.Min.Z;
                    }
                    if (mesh.BBox.Max.X > max.X) {
                        max.X = mesh.BBox.Max.X;
                    }
                    if (mesh.BBox.Max.Y > max.Y) {
                        max.Y = mesh.BBox.Max.Y;
                    }
                    if (mesh.BBox.Max.Z > max.Z) {
                        max.Z = mesh.BBox.Max.Z;
                    }
                }
            }

            extent = new AABox(
                min,
                max);
        }
    }
}
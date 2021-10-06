using System.IO;

namespace HuePat.VoxIR.IO.PLY.Writing {
    static class HeaderWriter {
        public static void Write(
                bool coordinatesAsFloat,
                long vertexCount,
                long faceCount,
                string file,
                PLYEncoding encoding) {

            using (StreamWriter writer = new StreamWriter(file)) {

                writer.WriteLine("ply");
                writer.WriteLine($"format {encoding.GetString()} 1.0");
                writer.WriteLine($"element vertex {vertexCount}");

                if (coordinatesAsFloat) {
                    writer.WriteLine("property float x");
                    writer.WriteLine("property float y");
                    writer.WriteLine("property float z");
                }
                else {
                    writer.WriteLine("property double x");
                    writer.WriteLine("property double y");
                    writer.WriteLine("property double z");
                }

                writer.WriteLine("property uchar red");
                writer.WriteLine("property uchar green");
                writer.WriteLine("property uchar blue");
                writer.WriteLine($"element face {faceCount}");
                writer.WriteLine("property list uchar int vertex_indices");
                writer.WriteLine("end_header");
            }
        }
    }
}
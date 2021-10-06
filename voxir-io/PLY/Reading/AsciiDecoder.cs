using HuePat.VoxIR.Util.Geometry;
using OpenTK.Mathematics;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace HuePat.VoxIR.IO.PLY.Reading {
    class AsciiDecoder : IDecoder {
        public Mesh ReadMesh(
                bool switchNormals,
                string file, 
                Header header) {

            string[] values;
            List<Face> faces = new List<Face>();
            List<Point> vertices;

            vertices = ReadPoints(
                file,
                header);

            ReadLines(
                header.HeaderLineCount + header.VertexSection.Count,
                header.HeaderLineCount + header.VertexSection.Count + header.FaceSection.Count,
                file,
                line => {

                    values = SplitValues(line);

                    if (int.Parse(values[0]) != 3) {
                        throw new ArgumentException(
                            "FaceParser currently only parses faces with three indices.");
                    }

                    faces.Add(
                        new Face(
                            int.Parse(values[1]),
                            int.Parse(switchNormals ? values[3] : values[2]),
                            int.Parse(switchNormals ? values[2] : values[3]),
                            vertices));
                });

            return new Mesh(
                vertices, 
                faces,
                true,
                true);
        }

        private List<Point> ReadPoints(
                string file,
                Header header) {

            List<Point> points = new List<Point>();

            ReadLines(
                header.HeaderLineCount,
                header.HeaderLineCount + header.VertexSection.Count,
                file,
                line => {

                    points.Add(
                        new Point(
                            ParseVector3d(
                                header.VertexSection.CoordinateIndices,
                                SplitValues(line))));
                });

            return points;
        }

        private static void ReadLines(
                int startIndex,
                int stopIndex,
                string file,
                Action<string> callback) {

            int index = 0;

            foreach (string line in File.ReadLines(file)) {

                if (index >= stopIndex) {
                    return;
                }

                if (index >= startIndex) {
                    callback(line);
                }

                index++;
            }
        }

        private string[] SplitValues(
                string line) {

            return line
                .Split(' ')
                .Where(v => v != "")
                .ToArray();
        }

        private Vector3d ParseVector3d(
                int[] indices,
                string[] values) {

            return new Vector3d(
                double.Parse(values[indices[0]]),
                double.Parse(values[indices[1]]),
                double.Parse(values[indices[2]]));
        }
    }
}
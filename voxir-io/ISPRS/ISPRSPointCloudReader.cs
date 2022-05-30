using HuePat.VoxIR.IO.PLY.Reading;
using HuePat.VoxIR.Util.Geometry;
using laszip.net;
using OpenTK.Mathematics;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace HuePat.VoxIR.IO.ISPRS {
    public static class ISPRSPointCloudReader {

        public static PointCloud Read(
                string file,
                bool doRotate = true) {

            PointCloud pointCloud;

            using (InvariantCultureBlock block = new InvariantCultureBlock()) {

                if (file.EndsWith(".ply")) {
                    pointCloud = ReadPLY(file);
                }
                else if (file.EndsWith(".txt")) {
                    pointCloud = ReadTXT(file);
                }
                else if (file.EndsWith(".las")) {
                    pointCloud = ReadLAS(file);
                }
                else {
                    throw new ArgumentException("Unknown file format.");
                }
            }

            if (doRotate) {

                Rotate(
                    pointCloud.GetCentroid(),
                    pointCloud);
            }

            return pointCloud;
        }

        public static void Rotate(
                Vector3d anchorPoint,
                PointCloud pointCloud) {

            pointCloud.Rotate(
                90.0.DegreeToRadian(),
                new Vector3d(1.0, 0.0, 0.0),
                anchorPoint);
        }

        private static PointCloud ReadPLY(
                string file) {

            return new PLYReader()
                .ReadPointCloud(
                    file,
                    false,
                    false);
        }

        private static PointCloud ReadTXT(
                string file) {

            return new PointCloud(
                    File
                        .ReadLines(file)
                        .Select(line => line.Split(" "))
                        .Select(values => new Point(
                            double.Parse(values[0]),
                            double.Parse(values[1]),
                            double.Parse(values[2]))),
                    false,
                    false);
        }

        private static PointCloud ReadLAS(
                string file) {

            bool isCompressed = false;
            long pointIndex = 0;
            long pointCount;
            double[] coordinates = new double[3];
            laszip_dll reader;
            List<Point> points = new List<Point>();

            reader = laszip_dll.laszip_create();
            reader.laszip_open_reader(
                file,
                ref isCompressed);
            pointCount = reader.header.number_of_point_records;

            while (pointIndex++ < pointCount) {

                reader.laszip_read_point();
                reader.laszip_get_coordinates(coordinates);
                points.Add(
                    new Point(
                        coordinates[0],
                        coordinates[1],
                        coordinates[2]));
            }

            reader.laszip_close_reader();

            return new PointCloud(
                    points,
                    false,
                    false);
        }
    }
}
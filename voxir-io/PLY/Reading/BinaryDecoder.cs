using HuePat.VoxIR.Util.Geometry;
using OpenTK.Mathematics;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;

namespace HuePat.VoxIR.IO.PLY.Reading {
    class BinaryDecoder : IDecoder {
        private bool littleEndian;
        private bool areVertexCoordinatesFloat;

        public BinaryDecoder(
                bool littleEndian,
                bool areVertexCoordinatesFloat) {

            this.littleEndian = littleEndian;
            this.areVertexCoordinatesFloat = areVertexCoordinatesFloat;
        }

        public Mesh ReadMesh(
                bool switchNormals,
                string file, 
                Header header) {

            int propertyIndex;
            int maxPropertyIndex;
            object[] properties;
            BinaryReader reader;
            List<Point> vertices;
            List<Face> faces = new List<Face>();
            Dictionary<int, PropertyType> propertyTypes;

            reader = CreateReader(file, header);
            vertices = ReadPoints(reader, header);
            propertyTypes = header.FaceSection.PropertyTypes;
            maxPropertyIndex = propertyTypes.Keys.Max();
            properties = new object[maxPropertyIndex + 1];

            while (faces.Count < header.FaceSection.Count) {

                propertyIndex = 0;

                while (propertyIndex <= maxPropertyIndex) {

                    properties[propertyIndex] = ReadProperty(
                        reader,
                        propertyTypes[propertyIndex]);

                    propertyIndex++;
                }

                faces.Add(
                    ParseFace(
                        switchNormals,
                        properties,
                        header.FaceSection,
                        vertices));
            }

            return new Mesh(
                vertices, 
                faces,
                true,
                true);
        }

        private BinaryReader CreateReader(
                string file,
                Header header) {

            BinaryReader reader = new BinaryReader(
                File.Open(
                    file, 
                    FileMode.Open, 
                    FileAccess.Read),
                littleEndian ?
                    Encoding.Unicode :
                    Encoding.BigEndianUnicode);

            reader.BaseStream.Position = header.VertexSectionStartPosition;

            return reader;
        }

        private List<Point> ReadPoints(
                BinaryReader reader,
                Header header) {

            int propertyIndex;
            int maxPropertyIndex;
            object[] properties;
            List<Point> points = new List<Point>();
            Dictionary<int, PropertyType> propertyTypes;

            propertyTypes = header.VertexSection.PropertyTypes;
            maxPropertyIndex = propertyTypes.Keys.Max();
            properties = new object[maxPropertyIndex + 1];

            while (points.Count < header.VertexSection.Count) {

                propertyIndex = 0;

                while (propertyIndex <= maxPropertyIndex) {

                    properties[propertyIndex] = ReadProperty(
                        reader, 
                        propertyTypes[propertyIndex]);

                    propertyIndex++;
                }

                points.Add(
                    new Point(
                        ParseVector3d(
                            areVertexCoordinatesFloat,
                            properties,
                            header.VertexSection.CoordinateIndices)));
            }

            return points;
        }

        private object ReadProperty(
                BinaryReader reader,
                PropertyType propertyType) {

            switch (propertyType) {
                case PropertyType.BYTE:
                    return reader.ReadByte();
                case PropertyType.INTEGER:
                    return reader.ReadInt32();
                case PropertyType.LONG:
                    return reader.ReadInt64();
                case PropertyType.FLOAT:
                    return reader.ReadSingle();
                case PropertyType.DOUBLE:
                    return reader.ReadDouble();
                case PropertyType.VECTOR3D:
                case PropertyType.COLOR:
                case PropertyType.BOOL:
                default:
                    break;
            }

            throw new ArgumentException();
        }

        private Face ParseFace(
                bool switchNormals,
                object[] properties,
                FaceSection faceSection,
                List<Point> vertices) {

            byte vertexIndexCount;
            int[] vertexIndices;

            vertexIndexCount = ParseByte(
                faceSection.VertexCountIndex,
                properties);

            if (vertexIndexCount != 3) {
                throw new ArgumentException(
                    "BinaryDecorder currently only parses faces with three indices.");
            }

            vertexIndices = new int[] {
                ParseInteger(
                    faceSection.VertexIndexIndices[0],
                    properties),
                ParseInteger(
                    faceSection.VertexIndexIndices[1],
                    properties),
                ParseInteger(
                    faceSection.VertexIndexIndices[2],
                    properties)
            };

            return new Face(
                vertexIndices[0],
                switchNormals ? vertexIndices[2] : vertexIndices[1],
                switchNormals ? vertexIndices[1] : vertexIndices[2],
                vertices);
        }

        private byte ParseByte(
                int index,
                object[] properties) {

            return (byte)properties[index];
        }

        private int ParseInteger(
                int index,
                object[] properties) {

            return (int)properties[index];
        }

        private Vector3d ParseVector3d(
                bool isFloat,
                object[] properties,
                int[] indices) {

            if (isFloat) {
                return new Vector3d(
                    (float)properties[indices[0]],
                    (float)properties[indices[1]],
                    (float)properties[indices[2]]);
            }

            return new Vector3d(
                (double)properties[indices[0]],
                (double)properties[indices[1]],
                (double)properties[indices[2]]);
        }
    }
}
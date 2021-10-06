using HuePat.VoxIR.Util.Geometry;
using System;
using System.IO;

namespace HuePat.VoxIR.IO.PLY.Writing {
    public class PLYWriter: IDisposable {
        private bool isInitialized;
        private int vertexCount;
        private int faceCount;
        private string file;
        private string vertexTempFile;
        private string faceTempFile;
        private PLYEncoding encoding;
        private IEncoder vertexTempFileEncoder;
        private IEncoder faceTempFileEncoder;

        public bool WriteCoordinatesAsFloat { private get; set; }

        public PLYEncoding Encoding {
            private get {
                return encoding;
            }
            set {
                encoding = value;
                OnConfigUpdate();
            }
        }

        public PLYWriter(
                string file) {

            WriteCoordinatesAsFloat = true;
            Encoding = PLYEncoding.BINARY_LITTLE_ENDIAN;
            isInitialized = true;
            vertexCount = faceCount = 0;

            this.file = file;
            if (File.Exists(file)) {
                File.Delete(file);
            }

            vertexTempFile = FileSystemUtils.GetTempFile(
                FileSystemUtils.GetFileWithPostfix(
                    file, 
                    "_vertices"));
            faceTempFile = FileSystemUtils.GetTempFile(
                FileSystemUtils.GetFileWithPostfix(
                    file, 
                    "_faces"));

            OnConfigUpdate();
        }

        public void Dispose() {

            vertexTempFileEncoder.Dispose();
            faceTempFileEncoder.Dispose();

            HeaderWriter.Write(
                WriteCoordinatesAsFloat,
                vertexCount,
                faceCount,
                file,
                Encoding);

            using (Stream output = new FileStream(
                    file, 
                    FileMode.Append, 
                    FileAccess.Write, 
                    FileShare.None)) {

                using (Stream points = File.OpenRead(vertexTempFile)) {
                    points.CopyTo(output);
                }
                using (Stream faces = File.OpenRead(faceTempFile)) {
                    faces.CopyTo(output);
                }
            }

            File.Delete(vertexTempFile);
            File.Delete(faceTempFile);
        }

        public void Write(
                Mesh mesh,
                Color color) {

            int offset = vertexCount;

            using (InvariantCultureBlock block = new InvariantCultureBlock()) {

                for (int i = 0; i < mesh.Vertices.Count; i++) {
                    vertexCount++;
                    vertexTempFileEncoder.Encode(
                        mesh.Vertices[i],
                        color);
                }

                for (int i = 0; i < mesh.Count; i++) {
                    faceCount++;
                    faceTempFileEncoder.Encode(
                        offset,
                        mesh[i]);
                }
            }
        }

        private void OnConfigUpdate() {
            
            if (isInitialized) {
                vertexTempFileEncoder?.Dispose();
                faceTempFileEncoder?.Dispose();
                vertexTempFileEncoder = GetEncoder(vertexTempFile);
                faceTempFileEncoder = GetEncoder(faceTempFile);
            }
        }

        private IEncoder GetEncoder(
                string file) {

            switch (Encoding) {
                case PLYEncoding.BINARY_LITTLE_ENDIAN:
                    return new BinaryEncoder(
                        true,
                        WriteCoordinatesAsFloat,
                        file);
                case PLYEncoding.BINARY_BIG_ENDIAN:
                    return new BinaryEncoder(
                        false,
                        WriteCoordinatesAsFloat,
                        file);
                case PLYEncoding.ASCII:
                    return new AsciiEncoder(file);
            }

            throw new ApplicationException();
        }
    }
}
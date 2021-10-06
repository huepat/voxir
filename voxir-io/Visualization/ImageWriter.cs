using OpenCvSharp;
using System;

namespace HuePat.VoxIR.IO.Visualization {
    public class ImageWriter : IDisposable {
        private string file;
        private Mat image;

        public ImageWriter(
                int height,
                int width,
                string file) {

            this.file = file;
            image = new Mat(
                height,
                width,
                MatType.CV_8UC3);
        }

        public void Write(
                int r,
                int c,
                Color color) {

            image.Set(
                r,
                c,
                new Vec3b(
                    color.B,
                    color.G,
                    color.R));
        }

        public void Dispose() {

            Cv2.ImWrite(
                file,
                image);
            image.Dispose();
        }
    }
}
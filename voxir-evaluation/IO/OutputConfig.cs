using HuePat.VoxIR.IO.Visualization;
using System.IO;

namespace HuePat.VoxIR.Evaluation.IO {
    public class OutputConfig {
        public bool ExportExtendedLogs { get; set; }
        public bool ExportPLY { get; set; }
        public bool ExportImages { get; set; }
        public string OutputDirectory { get; private set; }
        public string OutputImageFileType { get; set; }
        public BackgroundColor BackgroundColor { get; set; }
        public IVoxelMesher VoxelMesher { get; set; }

        public OutputConfig(
                string outputDirectory,
                bool createDirectory = true) {

            OutputDirectory = outputDirectory;
            OutputImageFileType = "png";
            VoxelMesher = new GridFrameVoxelMesher();
            BackgroundColor = BackgroundColor.BLACK;

            if (createDirectory 
                    && !Directory.Exists(outputDirectory)) {
                Directory.CreateDirectory(outputDirectory);
            }
        }
    }
}
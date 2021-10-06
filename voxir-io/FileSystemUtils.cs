using System.IO;

namespace HuePat.VoxIR.IO {
    public static class FileSystemUtils {
        private const string TEMP_FILE_POSTFIX = "_temp";

        public static string GetFileWithPostfix(
                string file, 
                string postfix) {

            return
                $"{Path.GetDirectoryName(file)}/" +
                $"{Path.GetFileNameWithoutExtension(file)}{postfix}{Path.GetExtension(file)}";
        }

        public static string GetTempFile(
                string file) {

            string tempFile = GetFileWithPostfix(
                file, 
                TEMP_FILE_POSTFIX);

            while (File.Exists(tempFile)) {
                tempFile = GetFileWithPostfix(
                    tempFile, 
                    TEMP_FILE_POSTFIX);
            }

            return tempFile;
        }
    }
}
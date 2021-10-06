namespace HuePat.VoxIR.Util.Grid {
    public static class PixelState {
        public static int[] CreatePixelState(
                int pixelClassValue) {

            return CreatePixelState(
                pixelClassValue, 
                -1);
        }

        public static int[] CreatePixelState(
                int pixelClassValue,
                int height) {

            return new int[] {
                pixelClassValue,
                height
            };
        }

        public static int GetPixelClassValue(
                this int[] pixelState) {

            return pixelState[0];
        }

        public static int GetPixelHeight(
                this int[] pixelState) {

            return pixelState[1];
        }

        public static void SetPixelHeight(
                this int[] pixelState,
                int height) {

            pixelState[1] = height;
        }

        public static void SetPixelClassValue(
                this int[] pixelState,
                int pixelClassValue) {

            pixelState[0] = pixelClassValue;
        }

        public static bool HasPixelHeight(
                this int[] pixelState) {

            return pixelState[1] != -1;
        }
    }
}
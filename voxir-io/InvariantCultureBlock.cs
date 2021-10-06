using System;
using System.Globalization;

namespace HuePat.VoxIR.IO {
    public class InvariantCultureBlock : IDisposable {
        private CultureInfo originalCulture;

        public InvariantCultureBlock() {

            originalCulture = CultureInfo.CurrentCulture;
            CultureInfo.CurrentCulture = CultureInfo.InvariantCulture;
        }

        public void Dispose() {

            CultureInfo.CurrentCulture = originalCulture;
        }
    }
}
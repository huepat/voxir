using System.Collections.Generic;

namespace HuePat.VoxIR.Evaluation {
    public interface IResult {
        IEnumerable<string> Labels { get; }
        IEnumerable<double> Values { get; }
    }
}
using System.Collections.Generic;

namespace HuePat.VoxIR.Evaluation.NormalGridDetermination {
    public class Result : IResult {
        public double Precision { get; private set; }
        public double Recall { get; private set; }

        public IEnumerable<string> Labels { 
            get {
                yield return "Precision";
                yield return "Recall";
            }
        }

        public IEnumerable<double> Values {
            get {
                yield return Precision;
                yield return Recall;
            }
        }

        public Result(
                double precision,
                double recall) {

            Precision = precision;
            Recall = recall;
        }
    }
}
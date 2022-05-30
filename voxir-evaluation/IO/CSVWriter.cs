using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace HuePat.VoxIR.Evaluation.IO {
    public class CSVWriter : IDisposable {
        private StreamWriter writer;

        public CSVWriter(string file) {

            writer = new StreamWriter(file);
        }

        public void Dispose() {

            writer.Flush();
            writer.Dispose();
        }

        public void WriteHeaderLine(
                IList<(double, double)> resolutionAndHorizontalRotationValues) {

            string line = "";

            foreach ((double, double) parameters in resolutionAndHorizontalRotationValues) {

                line += $"; {Util.GetParameterTableColumnHeader(parameters)}";
            }

            writer.WriteLine(line);
        }

        public void WriteResults<T>(
                IList<T> results) 
                    where T : class, IResult {

            if (results.Count == 0) {
                return;
            }

            string line;
            IEnumerator<double>[] valueEnumerators = results
                .Select(result => {

                    IEnumerator<double> valueEnumarator = result.Values.GetEnumerator();

                    valueEnumarator.MoveNext();

                    return valueEnumarator;

                })
                .ToArray();

            foreach (string label in results[0].Labels) {

                line = label;

                foreach (IEnumerator<double> valueEnumerator in valueEnumerators) {
                    
                    line += $"; {valueEnumerator.Current:0.00}";
                    valueEnumerator.MoveNext();
                }

                writer.WriteLine(line);
            }
        }
    }
}
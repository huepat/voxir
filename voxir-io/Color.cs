using System;
using System.Collections.Generic;
using System.Linq;

namespace HuePat.VoxIR.IO {
    public class Color {
        private const byte COLOR_COMPARISON_DELTA = 10;
        private const int MAX_DISTINCT_COLOR_COUNT = 1000;

        public static readonly Color Black = new Color(0, 0, 0);
        public static readonly Color White = new Color(255, 255, 255);
        public static readonly Color Red = new Color(255, 0, 0);
        public static readonly Color Green = new Color(0, 255, 0);
        public static readonly Color Blue = new Color(0, 0, 255);
        public static readonly Color Yellow = new Color(255, 255, 0);
        public static readonly Color Magenta = new Color(255, 0, 255);
        public static readonly Color Cyan = new Color(0, 255, 255);
        public static readonly Color LightGray = new Color(211, 211, 211);
        public static readonly Color Gray = new Color(169, 169, 169);
        public static readonly Color DarkGray = new Color(105, 105, 105);
        public static readonly Color Orange = new Color(255, 140, 0);
        public static readonly Color LightRed = new Color(250, 128, 114);
        public static readonly Color DarkRed = new Color(139, 0, 0);
        public static readonly Color LightBlue = new Color(100, 149, 237);
        public static readonly Color DarkBlue = new Color(0, 0, 128);
        public static readonly Color LightGreen = new Color(152, 251, 152);
        public static readonly Color ForestGreen = new Color(34, 139, 34);
        public static readonly Color DarkGreen = new Color(0, 90, 0);
        public static readonly Color Olive = new Color(128, 128, 0);
        public static readonly Color DarkOlive = new Color(85, 107, 47);
        public static readonly Color DarkCyan = new Color(0, 139, 139);
        public static readonly Color Pink = new Color(255, 165, 210);
        public static readonly Color Purple = new Color(128, 0, 128);
        public static readonly Color Violet = new Color(138, 43, 226);
        public static readonly Color Indigo = new Color(75, 0, 130);
        public static readonly Color LightBrown = new Color(205, 133, 63);
        public static readonly Color Brown = new Color(139, 69, 19);
        public static readonly Color Beige = new Color(255, 222, 173);

        private static Random randomGenerator = new Random(DateTime.Now.Millisecond);

        public static Dictionary<int, Color> GetRandomColorMapping(
                HashSet<int> labels) {

            return GetRandomColorMapping(
                labels.ToList());
        }

        private static Dictionary<int, Color> GetRandomColorMapping(
                List<int> labels) {

            bool useDistinctColors = labels.Count < MAX_DISTINCT_COLOR_COUNT;
            byte[] randomBytes = new byte[3];
            List<Color> colors = new List<Color>();
            Dictionary<int, Color> colorMapping = new Dictionary<int, Color>();

            for (int i = 0; i < labels.Count; i++) {

                Color candidateColor;

                do {
                    randomGenerator.NextBytes(randomBytes);
                    candidateColor = new Color(
                        randomBytes[0], 
                        randomBytes[1], 
                        randomBytes[2]);
                } while (
                    ApproximateEquals(candidateColor, Black) 
                        || ApproximateEquals(candidateColor, White)
                        || (useDistinctColors
                            && colors.Any(color => ApproximateEquals(color, candidateColor))));

                colors.Add(candidateColor);
            }

            for (int i = 0; i < labels.Count; i++) {
                colorMapping.Add(
                    labels[i], 
                    colors[i]);
            }

            return colorMapping;
        }

        private static bool ApproximateEquals(
                Color color1,
                Color color2) {

            return (color1.R - color2.R).Abs() < COLOR_COMPARISON_DELTA
                && (color1.G - color2.G).Abs() < COLOR_COMPARISON_DELTA
                && (color1.B - color2.B).Abs() < COLOR_COMPARISON_DELTA;
        }

        public byte R { get; private set; }
        public byte G { get; private set; }
        public byte B { get; private set; }

        public Color(
                byte r, 
                byte g, 
                byte b) {

            R = r;
            G = g;
            B = b;
        }
    }
}
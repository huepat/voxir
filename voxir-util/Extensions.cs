using OpenTK.Mathematics;
using System;
using System.Collections.Generic;
using System.Linq;

namespace HuePat.VoxIR {
    public static class Extensions {
        private const double EPSILON = 10E-5;

        public static int Abs(this int value) {
            return Math.Abs(value);
        }

        public static double Abs(this double value) {
            return Math.Abs(value);
        }

        public static double Ceil(this double value) {
            return Math.Ceiling(value);
        }

        public static double Floor(this double value) {
            return Math.Floor(value);
        }

        public static double Round(this double value) {
            return Math.Round(value);
        }

        public static double Squared(this double value) {
            return value * value;
        }

        public static double Sqrt(this double value) {
            return Math.Sqrt(value);
        }

        public static double Cos(this double value) {
            return Math.Cos(value);
        }

        public static double Sin(this double value) {
            return Math.Sin(value);
        }

        public static double DegreeToRadian(this double degree) {
            return degree * Math.PI / 180.0;
        }

        public static double RadianToDegree(this double radian) {
            return radian * 180.0 / Math.PI;
        }

        public static double GetVoxelSizeDiagonal(this double resolution) {
            return 2.0.Sqrt() * resolution;
        }

        public static int GetDistanceInVoxels(
                this double metricValue,
                double resolution) {

            return (int)(metricValue / resolution).Ceil();
        }

        public static int GetAreaInVoxels(
                this double metricValue,
                double resolution) {

            return (int)(metricValue / resolution.Squared()).Ceil();
        }

        public static bool IsDirectionDiagonal(this (int, int) direction) {
            return direction.Item1.Abs() + direction.Item2.Abs() == 2;
        }

        public static bool ApproximateEquals(
                this double d1,
                double d2) {

            return (d2 - d1).Abs() < EPSILON;
        }

        public static bool ApproximateEquals(
                this Vector3d vector,
                Vector3d otherVector) {

            return vector.X.ApproximateEquals(otherVector.X) 
                && vector.Y.ApproximateEquals(otherVector.Y) 
                && vector.Z.ApproximateEquals(otherVector.Z);
        }

        public static double DistanceTo(
                this Vector3d v1,
                Vector3d v2) {

            return (v2 - v1).Length;
        }

        public static T[] Copy<T>(
                this T[] array) {

            T[] copy = new T[array.Length];

            for (int i = 0; i < array.Length; i++) {
                copy[i] = array[i];
            }

            return copy;
        }

        public static void AddRange<T>(
                this HashSet<T> set,
                IEnumerable<T> values) {

            foreach (T value in values) {
                set.Add(value);
            }
        }

        public static void Remove<T>(
                this HashSet<T> set,
                IEnumerable<T> values) {

            foreach (T value in values) {
                set.Remove(value);
            }
        }

        public static void RemoveAt<T>(
                this List<T> list,
                IEnumerable<int> indices) {

            foreach (int i in indices.OrderDescending()) {
                list.RemoveAt(i);
            }
        }

        public static IEnumerable<T> Order<T>(
                this IEnumerable<T> values) {

            return values.OrderBy(value => value);
        }

        public static IEnumerable<T> OrderDescending<T>(
                this IEnumerable<T> values) {

            return values.OrderByDescending(value => value);
        }

        public static void GetMinMax(
                this IEnumerable<Vector3d> vectors,
                out Vector3d min,
                out Vector3d max) {

            min = new Vector3d(
                double.MaxValue, 
                double.MaxValue, 
                double.MaxValue);
            max = new Vector3d(
                double.MinValue, 
                double.MinValue, 
                double.MinValue);

            foreach (Vector3d vector in vectors) {
                if (vector.X < min.X) {
                    min.X = vector.X;
                }
                if (vector.X > max.X) {
                    max.X = vector.X;
                }
                if (vector.Y < min.Y) {
                    min.Y = vector.Y;
                }
                if (vector.Y > max.Y) {
                    max.Y = vector.Y;
                }
                if (vector.Z < min.Z) {
                    min.Z = vector.Z;
                }
                if (vector.Z > max.Z) {
                    max.Z = vector.Z;
                }
            }
        }

        public static void BucketIncrement<T>(
                this Dictionary<T, int> dictionary,
                T key) {

            if (!dictionary.ContainsKey(key)) {
                dictionary.Add(key, 0);
            }
            dictionary[key]++;
        }

        public static void BucketAdd<T>(
                this Dictionary<T, int> dictionary,
                T key,
                int value) {

            if (!dictionary.ContainsKey(key)) {
                dictionary.Add(key, 0);
            }
            dictionary[key] += value;
        }

        public static void BucketAdd<T>(
                this Dictionary<T, int> dictionary,
                Dictionary<T, int> dictionary2) {

            foreach (T key in dictionary2.Keys) {
                if (!dictionary.ContainsKey(key)) {
                    dictionary.Add(key, 0);
                }
                dictionary[key] += dictionary2[key];
            }
        }

        public static IEnumerable<TValue> UnwrapValues<TKey, TValue>(
                this Dictionary<TKey, List<TValue>> dictionary) {

            return dictionary
                .Values
                .SelectMany(values => values);
        }

        public static void BucketAdd<TKey, TValue>(
                this Dictionary<TKey, List<TValue>> dictionary,
                TKey key,
                TValue value) {

            if (!dictionary.ContainsKey(key)) {
                dictionary.Add(
                    key, 
                    new List<TValue>());
            }
            dictionary[key].Add(value);
        }

        public static void BucketAdd<TKey, TValue>(
                this Dictionary<TKey, HashSet<TValue>> dictionary,
                TKey key,
                TValue value) {

            if (!dictionary.ContainsKey(key)) {
                dictionary.Add(
                    key, 
                    new HashSet<TValue>());
            }
            dictionary[key].Add(value);
        }

        public static void BucketAdd<TKey, TValue>(
                this Dictionary<TKey, List<TValue>> dictionary1,
                Dictionary<TKey, List<TValue>> dictionary2) {

            foreach (TKey key in dictionary2.Keys) {
                if (!dictionary1.ContainsKey(key)) {
                    dictionary1.Add(
                        key, 
                        new List<TValue>());
                }
                dictionary1[key].AddRange(dictionary2[key]);
            }
        }

        public static void BucketAdd<TKey, TValue>(
                this Dictionary<TKey, HashSet<TValue>> dictionary1,
                Dictionary<TKey, HashSet<TValue>> dictionary2) {

            foreach (TKey key in dictionary2.Keys) {
                if (!dictionary1.ContainsKey(key)) {
                    dictionary1.Add(
                        key, 
                        new HashSet<TValue>());
                }
                dictionary1[key].AddRange(dictionary2[key]);
            }
        }

        public static void ForEach<T>(
                this IEnumerable<T> values,
                Action<T> callback) {

            foreach (T value in values) {
                callback(value);
            }
        }

        public static List<T> WhereMin<T, U>(
                this IEnumerable<T> objects,
                Func<T, U> valueExtractor) where U : IComparable {

            if (!objects.Any()) {
                return new List<T>();
            }

            bool first = true;
            int testResult;
            List<T> resultObjects = new List<T> { objects.First() };
            U candidateValue = valueExtractor(resultObjects[0]);
            U testValue;

            foreach (T @object in objects) {
                testValue = valueExtractor(@object);
                testResult = testValue.CompareTo(candidateValue);
                if (testResult < 0) {
                    candidateValue = testValue;
                    resultObjects = new List<T> {
                        @object
                    };
                }
                else if (!first && testResult == 0) {
                    resultObjects.Add(@object);
                }
                first = false;
            }

            return resultObjects;
        }

        public static List<T> WhereMax<T, U>(
                this IEnumerable<T> objects,
                Func<T, U> valueExtractor) where U : IComparable {

            if (!objects.Any()) {
                return new List<T>();
            }

            bool first = true;
            int testResult;
            List<T> resultObjects = new List<T> { objects.First() };
            U candidateValue = valueExtractor(resultObjects[0]);
            U testValue;

            foreach (T @object in objects) {
                testValue = valueExtractor(@object);
                testResult = testValue.CompareTo(candidateValue);
                if (testResult > 0) {
                    candidateValue = testValue;
                    resultObjects = new List<T> {
                        @object
                    };
                }
                else if (!first && testResult == 0) {
                    resultObjects.Add(@object);
                }
                first = false;
            }

            return resultObjects;
        }

        public static int Median(
                this IEnumerable<int> values) {

            return (int)values
                .Select(value => (double)value)
                .Median()
                .Round();
        }

        public static double Median(
                this IEnumerable<double> values) {

            if (!values.Any()) {
                return 0.0;
            }

            List<double> sorted = values
                .OrderBy(value => value)
                .ToList();

            if (sorted.Count % 2 == 0) {
                return ((sorted[sorted.Count / 2] + sorted[sorted.Count / 2 - 1]) / 2.0);
            }

            return sorted[sorted.Count / 2];
        }

        public static Vector3d Multiply(
                this Matrix3d matrix,
                Vector3d vector) {

            return new Vector3d(
                matrix[0, 0] * vector.X + matrix[0, 1] * vector.Y + matrix[0, 2] * vector.Z,
                matrix[1, 0] * vector.X + matrix[1, 1] * vector.Y + matrix[1, 2] * vector.Z,
                matrix[2, 0] * vector.X + matrix[2, 1] * vector.Y + matrix[2, 2] * vector.Z);
        }
    }
}
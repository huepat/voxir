using System.Collections.Generic;
using System.Linq;

namespace HuePat.VoxIR.NormalGridDetermination {
    class VerticalAdjacencyGraph {
        private class Node {
            public HashSet<int> Above { get; private set; }
            public HashSet<int> Below { get; private set; }

            public Node() {

                Above = new HashSet<int>();
                Below = new HashSet<int>();
            }

            public void Add(Node node) {

                Above.AddRange(node.Above);
                Below.AddRange(node.Below);
            }
        }

        private class Edge {
            public int Coverage { get; private set; }
            public double MeanDistance { get; private set; }

            public Edge(int distance) {

                MeanDistance = distance;
                Coverage = 1;
            }

            public void Add(int distance) {

                MeanDistance = MeanDistance + (distance - MeanDistance) / (Coverage + 1);
                Coverage++;
            }

            public void Add(Edge edge) {

                int totalCoverage;

                totalCoverage = Coverage + edge.Coverage;
                MeanDistance = (double)Coverage / totalCoverage * MeanDistance
                    + (double)edge.Coverage / totalCoverage * edge.MeanDistance;
                Coverage = totalCoverage;
            }
        }

        private Dictionary<int, Node> nodes;
        private Dictionary<(int, int), Edge> edges;

        public VerticalAdjacencyGraph() {

            nodes = new Dictionary<int, Node>();
            edges = new Dictionary<(int, int), Edge>();
        }

        public void Add(
                int lowerSegmentId,
                int upperSegmentId,
                int distance) {

            AddNodes(
                lowerSegmentId,
                upperSegmentId);
            
            AddEdge(
                (
                    lowerSegmentId,
                    upperSegmentId
                ),
                distance);
        }

        public void Add(
                VerticalAdjacencyGraph verticalAdjacencyGraph) {

            Add(verticalAdjacencyGraph.nodes);
            Add(verticalAdjacencyGraph.edges);
        }

        public HashSet<int> GetAbove(
                int lowerSegmentId) {

            return nodes[lowerSegmentId].Above;
        }

        public HashSet<int> GetBelow(
                int upperSegmentId) {

            return nodes[upperSegmentId].Below;
        }

        public double GetDistance(
                int lowerSegmentId,
                int upperSegmentId) {

            return edges[(
                lowerSegmentId,
                upperSegmentId)].MeanDistance;
        }

        public int GetCoverage(
                int lowerSegmentId,
                int upperSegmentId) {

            return edges[(
                lowerSegmentId,
                upperSegmentId)].Coverage;
        }

        public int GetMaxCoverage(
                int segmentId) {

            List<int> coverages = new List<int>();

            coverages.AddRange(
                nodes[segmentId]
                    .Below
                    .Select(segmentIdBelow => GetCoverage(
                            segmentIdBelow,
                            segmentId)));

            coverages.AddRange(
                nodes[segmentId]
                    .Above
                    .Select(segmentIdAbove => GetCoverage(
                            segmentId,
                            segmentIdAbove)));

            return coverages.Max();
        }

        public IEnumerable<int> GetMainSegmentIdsBelow(
                int segmentId) {

            HashSet<int> segmentIdsAbove;

            return nodes[segmentId]
                .Below
                .Where(segmentIdBelow => {

                    segmentIdsAbove = nodes[segmentIdBelow].Above;

                    if (!segmentIdsAbove.Contains(segmentId)) {
                        return false;
                    }

                    return segmentIdsAbove
                        .WhereMax(segmentIdAbove => edges[(segmentIdBelow, segmentIdAbove)].Coverage)
                        .Contains(segmentId);
                });
        }

        public IEnumerable<int> GetMainSegmentIdsAbove(
                int segmentId,
                bool ignoreOutside = false) {

            HashSet<int> segmentIdsBelow;

            return nodes[segmentId]
                .Above
                .Where(segmentIdAbove => {

                    if (ignoreOutside
                            && segmentIdAbove == NormalGridDetermination.UPPER_OUTSIDE_ID) {

                        return false;
                    }

                    segmentIdsBelow = nodes[segmentIdAbove].Below;

                    if (!segmentIdsBelow.Contains(segmentId)) {
                        return false;
                    }

                    return segmentIdsBelow
                        .Where(segmentIdBelow => !ignoreOutside
                            || segmentIdBelow != NormalGridDetermination.LOWER_OUTSIDE_ID)
                        .WhereMax(segmentIdBelow => edges[(segmentIdBelow, segmentIdAbove)].Coverage)
                        .Contains(segmentId);
                });
        }

        public IEnumerable<int> GetMainSegmentIdsAbove(
                IEnumerable<int> segmentIds,
                int minDistance) {

            return segmentIds
                .SelectMany(segmentId => GetMainSegmentIdsAbove(segmentId, true)
                    .Where(segmentIdAbove => edges[(segmentId, segmentIdAbove)].MeanDistance >= minDistance));
        }

        private void AddNodes(
                int lowerSegmentId,
                int upperSegmentId) {

            if (!nodes.ContainsKey(lowerSegmentId)) {

                nodes.Add(
                    lowerSegmentId,
                    new Node());
            }

            if (!nodes.ContainsKey(upperSegmentId)) {

                nodes.Add(
                    upperSegmentId,
                    new Node());
            }

            nodes[lowerSegmentId].Above.Add(upperSegmentId);
            nodes[upperSegmentId].Below.Add(lowerSegmentId);
        }

        private void AddEdge(
                (int, int) edgeId,
                int distance) {

            if (edges.ContainsKey(edgeId)) {
                edges[edgeId].Add(distance);
            }
            else {
                edges.Add(
                    edgeId,
                    new Edge(distance));
            }
        }

        private void Add(
                Dictionary<int, Node> nodes) {

            foreach (int segmentId in nodes.Keys) {

                if (this.nodes.ContainsKey(segmentId)) {

                    this.nodes[segmentId].Add(
                        nodes[segmentId]);
                }
                else {
                    this.nodes.Add(
                        segmentId,
                        nodes[segmentId]);
                }
            }
        }

        private void Add(
                Dictionary<(int, int), Edge> edges) {

            foreach ((int, int) edgeId in edges.Keys) {

                if (this.edges.ContainsKey(edgeId)) {

                    this.edges[edgeId].Add(
                        edges[edgeId]);
                }
                else {
                    this.edges.Add(
                        edgeId,
                        edges[edgeId]);
                }
            }
        }
    }
}
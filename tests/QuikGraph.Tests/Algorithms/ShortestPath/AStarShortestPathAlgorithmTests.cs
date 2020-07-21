using System;
using System.Collections.Generic;
using JetBrains.Annotations;
using NUnit.Framework;
using QuikGraph.Algorithms;
using QuikGraph.Algorithms.Observers;
using QuikGraph.Algorithms.ShortestPath;
using static QuikGraph.Tests.Algorithms.AlgorithmTestHelpers;

namespace QuikGraph.Tests.Algorithms.ShortestPath
{
    /// <summary>
    /// Tests for <see cref="AStarShortestPathAlgorithm{TVertex,TEdge}"/>.
    /// </summary>
    [TestFixture]
    internal class AStartShortestPathAlgorithmTests : ShortestPathAlgorithmTestsBase
    {
        #region Test helpers

        private static void RunAStarAndCheck<TVertex, TEdge>(
            [NotNull] IVertexAndEdgeListGraph<TVertex, TEdge> graph,
            [NotNull] TVertex root)
            where TEdge : IEdge<TVertex>
        {
            var distances = new Dictionary<TEdge, double>();
            foreach (TEdge edge in graph.Edges)
                distances[edge] = graph.OutDegree(edge.Source) + 1;

            var algorithm = new AStarShortestPathAlgorithm<TVertex, TEdge>(
                graph,
                e => distances[e],
                v => 0.0);

            algorithm.InitializeVertex += vertex =>
            {
                Assert.AreEqual(GraphColor.White, algorithm.VerticesColors[vertex]);
            };

            algorithm.DiscoverVertex += vertex =>
            {
                Assert.AreEqual(GraphColor.Gray, algorithm.VerticesColors[vertex]);
            };

            algorithm.FinishVertex += vertex =>
            {
                Assert.AreEqual(GraphColor.Black, algorithm.VerticesColors[vertex]);
            };

            var predecessors = new VertexPredecessorRecorderObserver<TVertex, TEdge>();
            using (predecessors.Attach(algorithm))
                algorithm.Compute(root);

            Assert.IsNotNull(algorithm.Distances);
            Assert.AreEqual(graph.VertexCount, algorithm.Distances.Count);

            Verify(algorithm, predecessors);
        }

        private static void Verify<TVertex, TEdge>(
            [NotNull] AStarShortestPathAlgorithm<TVertex, TEdge> algorithm,
            [NotNull] VertexPredecessorRecorderObserver<TVertex, TEdge> predecessors)
            where TEdge : IEdge<TVertex>
        {
            // Verify the result
            var visitedGraph = algorithm.VisitedGraph as IVertexAndEdgeListGraph<TVertex, TEdge>;
            foreach (TVertex vertex in visitedGraph.Vertices)
            {
                if (!predecessors.VerticesPredecessors.TryGetValue(vertex, out TEdge predecessor))
                    continue;
                if (predecessor.Source.Equals(vertex))
                    continue;
                Assert.AreEqual(
                    algorithm.TryGetDistance(vertex, out double currentDistance),
                    algorithm.TryGetDistance(predecessor.Source, out double predecessorDistance));
                Assert.GreaterOrEqual(currentDistance, predecessorDistance);
            }
        }

        #endregion

        [Test]
        public void Constructor()
        {
            Func<int, double> Heuristic = v => 1.0;
            Func<Edge<int>, double> Weights = e => 1.0;

            var graph = new AdjacencyGraph<int, Edge<int>>();
            var algorithm = new AStarShortestPathAlgorithm<int, Edge<int>>(graph, Weights, Heuristic);
            AssertAlgorithmProperties(algorithm, graph, Heuristic, Weights);

            algorithm = new AStarShortestPathAlgorithm<int, Edge<int>>(graph, Weights, Heuristic, DistanceRelaxers.CriticalDistance);
            AssertAlgorithmProperties(algorithm, graph, Heuristic, Weights, DistanceRelaxers.CriticalDistance);

            algorithm = new AStarShortestPathAlgorithm<int, Edge<int>>(null, graph, Weights, Heuristic, DistanceRelaxers.CriticalDistance);
            AssertAlgorithmProperties(algorithm, graph, Heuristic, Weights, DistanceRelaxers.CriticalDistance);

            #region Local function

            void AssertAlgorithmProperties<TVertex, TEdge>(
                AStarShortestPathAlgorithm<TVertex, TEdge> algo,
                IVertexListGraph<TVertex, TEdge> g,
                Func<TVertex, double> heuristic = null,
                Func<TEdge, double> eWeights = null,
                IDistanceRelaxer relaxer = null)
                where TEdge : IEdge<TVertex>
            {
                AssertAlgorithmState(algo, g);
                Assert.IsNull(algo.VerticesColors);
                if (heuristic is null)
                    Assert.IsNotNull(algo.CostHeuristic);
                else
                    Assert.AreSame(heuristic, algo.CostHeuristic);
                if (eWeights is null)
                    Assert.IsNotNull(algo.Weights);
                else
                    Assert.AreSame(eWeights, algo.Weights);
                Assert.IsNull(algo.Distances);
                if (relaxer is null)
                    Assert.IsNotNull(algo.DistanceRelaxer);
                else
                    Assert.AreSame(relaxer, algo.DistanceRelaxer);
            }

            #endregion
        }

        [Test]
        public void Constructor_Throws()
        {
            // ReSharper disable ObjectCreationAsStatement
            // ReSharper disable AssignNullToNotNullAttribute
            var graph = new AdjacencyGraph<int, Edge<int>>();

            Func<int, double> Heuristic = v => 1.0;
            Func<Edge<int>, double> Weights = e => 1.0;

            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, Weights, Heuristic));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(graph, null, Heuristic));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(graph, Weights, null));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(graph, null, null));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, null, Heuristic));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, Weights, null));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, null, Heuristic));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, null, null));

            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, Weights, Heuristic, DistanceRelaxers.CriticalDistance));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(graph, null, Heuristic, DistanceRelaxers.CriticalDistance));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(graph, Weights, null, DistanceRelaxers.CriticalDistance));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(graph, Weights, Heuristic, null));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(graph, Weights, null, null));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(graph, null, Heuristic, null));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(graph, null, null, DistanceRelaxers.CriticalDistance));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, Weights, Heuristic, null));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, Weights, null, DistanceRelaxers.CriticalDistance));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, null, Heuristic, DistanceRelaxers.CriticalDistance));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, Weights, null, null));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, null, Heuristic, null));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, null, null, DistanceRelaxers.CriticalDistance));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, null, null, null));

            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, null, Weights, Heuristic, DistanceRelaxers.CriticalDistance));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, graph, null, Heuristic, DistanceRelaxers.CriticalDistance));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, graph, Weights, null, DistanceRelaxers.CriticalDistance));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, graph, Weights, Heuristic, null));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, graph, Weights, null, null));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, graph, null, Heuristic, null));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, graph, null, null, DistanceRelaxers.CriticalDistance));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, null, Weights, Heuristic, null));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, null, Weights, null, DistanceRelaxers.CriticalDistance));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, null, null, Heuristic, DistanceRelaxers.CriticalDistance));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, null, Weights, null, null));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, null, null, Heuristic, null));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, null, null, null, DistanceRelaxers.CriticalDistance));
            Assert.Throws<ArgumentNullException>(
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(null, null, null, null, null));
            // ReSharper restore AssignNullToNotNullAttribute
            // ReSharper restore ObjectCreationAsStatement
        }

        #region Rooted algorithm

        [Test]
        public void TryGetRootVertex()
        {
            var graph = new AdjacencyGraph<int, Edge<int>>();
            var algorithm = new AStarShortestPathAlgorithm<int, Edge<int>>(graph, edge => 1.0, vertex => 0.0);
            TryGetRootVertex_Test(algorithm);
        }

        [Test]
        public void SetRootVertex()
        {
            var graph = new AdjacencyGraph<int, Edge<int>>();
            var algorithm = new AStarShortestPathAlgorithm<int, Edge<int>>(graph, edge => 1.0, vertex => 0.0);
            SetRootVertex_Test(algorithm);
        }

        [Test]
        public void SetRootVertex_Throws()
        {
            var graph = new AdjacencyGraph<TestVertex, Edge<TestVertex>>();
            var algorithm = new AStarShortestPathAlgorithm<TestVertex, Edge<TestVertex>>(graph, edge => 1.0, vertex => 0.0);
            SetRootVertex_Throws_Test(algorithm);
        }

        [Test]
        public void ClearRootVertex()
        {
            var graph = new AdjacencyGraph<int, Edge<int>>();
            var algorithm = new AStarShortestPathAlgorithm<int, Edge<int>>(graph, edge => 1.0, vertex => 0.0);
            ClearRootVertex_Test(algorithm);
        }

        [Test]
        public void ComputeWithoutRoot_Throws()
        {
            var graph = new AdjacencyGraph<int, Edge<int>>();
            ComputeWithoutRoot_NoThrows_Test(
                graph,
                () => new AStarShortestPathAlgorithm<int, Edge<int>>(graph, edge => 1.0, vertex => 0.0));
        }

        [Test]
        public void ComputeWithRoot()
        {
            var graph = new AdjacencyGraph<int, Edge<int>>();
            graph.AddVertex(0);
            var algorithm = new AStarShortestPathAlgorithm<int, Edge<int>>(graph, edge => 1.0, vertex => 0.0);
            ComputeWithRoot_Test(algorithm);
        }

        [Test]
        public void ComputeWithRoot_Throws()
        {
            var graph = new AdjacencyGraph<TestVertex, Edge<TestVertex>>();
            ComputeWithRoot_Throws_Test(
                () => new AStarShortestPathAlgorithm<TestVertex, Edge<TestVertex>>(graph, edge => 1.0, vertex => 0.0));
        }

        #endregion

        #region Shortest path algorithm

        [Test]
        public void TryGetDistance()
        {
            var graph = new AdjacencyGraph<int, Edge<int>>();
            graph.AddVertex(1);
            var algorithm = new AStarShortestPathAlgorithm<int, Edge<int>>(graph, edge => 1.0, vertex => 0.0);
            TryGetDistance_Test(algorithm);
        }

        [Test]
        public void TryGetDistance_Throws()
        {
            var graph = new AdjacencyGraph<TestVertex, Edge<TestVertex>>();
            var algorithm = new AStarShortestPathAlgorithm<TestVertex, Edge<TestVertex>>(graph, edge => 1.0, vertex => 0.0);
            TryGetDistance_Throws_Test(algorithm);
        }

        #endregion

        [Test]
        public void GetVertexColor()
        {
            var graph = new AdjacencyGraph<int, Edge<int>>();
            graph.AddVerticesAndEdge(new Edge<int>(1, 2));

            var algorithm = new AStarShortestPathAlgorithm<int, Edge<int>>(graph, edge => 1.0, vertex => 0.0);
            algorithm.Compute(1);

            Assert.AreEqual(GraphColor.Black, algorithm.GetVertexColor(1));
            Assert.AreEqual(GraphColor.Black, algorithm.GetVertexColor(2));
        }

        [Test]
        public void GetVertexColor_Throws()
        {
            var graph = new AdjacencyGraph<int, Edge<int>>();
            graph.AddVertex(0);
            var algorithm = new AStarShortestPathAlgorithm<int, Edge<int>>(graph, edge => 1.0, vertex => 0.0);
            algorithm.Compute(0);

            // ReSharper disable once ReturnValueOfPureMethodIsNotUsed
            Assert.Throws<VertexNotFoundException>(() => algorithm.GetVertexColor(1));
        }

        [Test]
        [Category(TestCategories.LongRunning)]
        public void AStar()
        {
            foreach (AdjacencyGraph<string, Edge<string>> graph in TestGraphFactory.GetAdjacencyGraphs_SlowTests())
            {
                foreach (string root in graph.Vertices)
                    RunAStarAndCheck(graph, root);
            }
        }

        [Test]
        public void AStar_Throws()
        {
            var edge12 = new Edge<int>(1, 2);
            var edge23 = new Edge<int>(2, 3);
            var edge34 = new Edge<int>(3, 4);

            var negativeWeightGraph = new AdjacencyGraph<int, Edge<int>>();
            negativeWeightGraph.AddVerticesAndEdgeRange(new[]
            {
                edge12, edge23, edge34
            });

            var algorithm = new AStarShortestPathAlgorithm<int, Edge<int>>(
                negativeWeightGraph,
                e =>
                {
                    if (e == edge12)
                        return 12.0;
                    if (e == edge23)
                        return -23.0;
                    if (e == edge34)
                        return 34.0;
                    return 1.0;
                },
                v => 0.0);
            Assert.Throws<NegativeWeightException>(() => algorithm.Compute(1));
        }
    }
}
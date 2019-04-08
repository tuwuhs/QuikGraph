#if SUPPORTS_SERIALIZATION
using System;
#endif
using System.Collections.Generic;
#if SUPPORTS_CONTRACTS
using System.Diagnostics.Contracts;
#endif

namespace QuickGraph.Predicates
{
#if SUPPORTS_SERIALIZATION
    [Serializable]
#endif
    public sealed class FilteredEdgeListGraph<TVertex, TEdge, TGraph>
        : FilteredImplicitVertexSet<TVertex, TEdge, TGraph>
        , IEdgeListGraph<TVertex, TEdge>
        where TGraph : IEdgeListGraph<TVertex, TEdge>
        where TEdge : IEdge<TVertex>
    {
        public FilteredEdgeListGraph(
            TGraph baseGraph,
            VertexPredicate<TVertex> vertexPredicate,
            EdgePredicate<TVertex, TEdge> edgePredicate
            )
            : base(baseGraph, vertexPredicate, edgePredicate)
        {}

        public bool IsVerticesEmpty
        {
            get
            {
                foreach (var v in this.BaseGraph.Vertices)
                    if (this.VertexPredicate(v))
                        return false;
                return true;
            }
        }

        public int VertexCount
        {
            get
            {
                int count = 0;
                foreach (var v in this.BaseGraph.Vertices)
                    if (this.VertexPredicate(v))
                        count++;
                return count;
            }
        }

        public IEnumerable<TVertex> Vertices
        {
            get
            {
                foreach (var v in this.BaseGraph.Vertices)
                    if (this.VertexPredicate(v))
                        yield return v;
            }
        }

        public bool IsEdgesEmpty
        {
            get
            {
                foreach (var edge in this.BaseGraph.Edges)
                    if (this.FilterEdge(edge))
                        return false;
                return true;
            }
        }

        public int EdgeCount
        {
            get
            {
                int count = 0;
                foreach (var edge in this.BaseGraph.Edges)
                    if (this.FilterEdge(edge))
                        count++;
                return count;
            }
        }

        public IEnumerable<TEdge> Edges
        {
            get
            {
                foreach (var edge in this.BaseGraph.Edges)
                    if (this.FilterEdge(edge))
                        yield return edge;
            }
        }

#if SUPPORTS_CONTRACTS
        [Pure]
#endif
        private bool FilterEdge(TEdge edge)
        {
            return this.VertexPredicate(edge.Source)
                        && this.VertexPredicate(edge.Target)
                        && this.EdgePredicate(edge);
        }

#if SUPPORTS_CONTRACTS
        [Pure]
#endif
        public bool ContainsEdge(TEdge edge)
        {
            return
                this.FilterEdge(edge) &&
                this.BaseGraph.ContainsEdge(edge);
        }
    }
}
import random
import time
import networkx as nx
import matplotlib.pyplot as plt

def gerar_grafo(n, tipo="denso"):
    G = {i: [] for i in range(n)}
    edges = []

    if tipo == "denso":
        for i in range(n):
            for j in range(i + 1, n):
                w = random.randint(1, 100)
                G[i].append((j, w))
                G[j].append((i, w))
                edges.append((w, i, j))
    elif tipo == "esparso":
        m = (n - 1) * 4
        nodes = list(range(n))
        random.shuffle(nodes)
        for i in range(n - 1):
            w = random.randint(1, 100)
            G[nodes[i]].append((nodes[i + 1], w))
            G[nodes[i + 1]].append((nodes[i], w))
            edges.append((w, nodes[i], nodes[i + 1]))
        while len(edges) < m:
            u, v = random.sample(range(n), 2)
            if all(nei != v for nei, _ in G[u]):
                w = random.randint(1, 100)
                G[u].append((v, w))
                G[v].append((u, w))
                edges.append((w, u, v))
    return G, edges

def prim(G):
    n = len(G)
    in_mst = [False] * n
    key = [float('inf')] * n
    parent = [-1] * n
    key[0] = 0

    for _ in range(n):
        u = min((k for k in range(n) if not in_mst[k]), key=lambda x: key[x])
        in_mst[u] = True

        for v, w in G[u]:
            if not in_mst[v] and w < key[v]:
                key[v] = w
                parent[v] = u

    mst_edges = [(parent[v], v, key[v]) for v in range(1, n)]
    return mst_edges

class UnionFind:
    def __init__(self, n):
        self.parent = list(range(n))
        self.rank = [0] * n

    def find(self, u):
        if self.parent[u] != u:
            self.parent[u] = self.find(self.parent[u])
        return self.parent[u]

    def union(self, u, v):
        ru, rv = self.find(u), self.find(v)
        if ru == rv:
            return False
        if self.rank[ru] < self.rank[rv]:
            self.parent[ru] = rv
        elif self.rank[ru] > self.rank[rv]:
            self.parent[rv] = ru
        else:
            self.parent[rv] = ru
            self.rank[ru] += 1
        return True

def kruskal(n, edges):
    uf = UnionFind(n)
    mst_edges = []
    for w, u, v in sorted(edges):
        if uf.union(u, v):
            mst_edges.append((u, v, w))
        if len(mst_edges) == n - 1:
            break
    return mst_edges

def iniciar():
    sizes = [10, 50, 100, 500, 2000]
    for n in sizes:
        print(f"\n--- n={n} ---")
        for tipo in ["denso", "esparso"]:
            G, edges = gerar_grafo(n, tipo)
            print(f"Grafo {tipo}, vértices={n}, arestas={len(edges)}")

            t0 = time.perf_counter()
            prim(G)
            t1 = time.perf_counter()
            kruskal(n, edges)
            t2 = time.perf_counter()

            print(f"Prim: {t1 - t0:.5f}s | Kruskal: {t2 - t1:.5f}s")

            if n in [10, 50, 100]:
                G, edges = gerar_grafo(n, tipo)
                mst_edges = kruskal(n, edges)
                desenhar(n, edges, mst_edges, f"Grafo {tipo}: {n} vértices")

def desenhar(n, edges, mst_edges, titulo="MST"):
    G = nx.Graph()
    G.add_nodes_from(range(n))
    for w, u, v in edges:
        G.add_edge(u, v, weight=w)

    mstG = nx.Graph()
    mstG.add_nodes_from(range(n))
    for u, v, w in mst_edges:
        mstG.add_edge(u, v, weight=w)

    pos = nx.spring_layout(G, seed=42)
    plt.figure(figsize=(6, 6))
    nx.draw(G, pos, with_labels=True, node_color="lightblue", edge_color="gray")
    nx.draw_networkx_edges(mstG, pos, edge_color="red", width=2)
    labels = nx.get_edge_attributes(G, "weight")
    nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
    plt.title(titulo)
    plt.show()

if __name__ == "__main__":
    iniciar()
from geopy.geocoders import Nominatim
import osmnx as ox
import heapq
import matplotlib.pyplot as plt

def pathing_route_dijkstra(G, start, target, weight="length"):
    dist = {node: float("inf") for node in G.nodes()}
    dist[start] = 0
    prev = {}
    pq = [(0, start)]

    while pq:   
        current_dist, u = heapq.heappop(pq)

        if u == target:
            path = []
            while u is not None:
                path.append(u)
                u = prev.get(u)
            return current_dist, list(reversed(path))

        if current_dist > dist[u]:
            continue

        for v in G.neighbors(u):
            edge_data = G.get_edge_data(u, v)
            if edge_data is None:
                continue

            w = min(ed.get(weight, 1) for ed in edge_data.values())
            new_dist = current_dist + w

            if new_dist < dist[v]:
                dist[v] = new_dist
                prev[v] = u
                heapq.heappush(pq, (new_dist, v))

    return float("inf"), []


class TripService:
    @staticmethod
    def define_trip(depot_address, orders_address, city_context="Rio de Janeiro, RJ, Brasil"):
        addresses = [depot_address] + orders_address

        print(f"1. Geocodificando endereços em {city_context}...")
        geolocator = Nominatim(user_agent="cheap_tracker", timeout=10)

        locations = []
        for address in addresses:
            search_query = address
            loc = geolocator.geocode(search_query)
            if loc:
                locations.append(loc)
                print(f"   [OK] Encontrado: {address.split(',')[0]},{address.split(',')[1]}")
            else:
                raise ValueError(f"Endereço não encontrado pelo Nominatim: {address}")

        print("2. Baixando o mapa da cidade...")
        G = ox.graph_from_place(city_context, network_type="drive")

        xs = [loc.longitude for loc in locations]
        ys = [loc.latitude for loc in locations]
        nodes = ox.distance.nearest_nodes(G, xs, ys)

        segment_paths = []
        total_distance = 0
        route_order = [0]
        remaining = list(range(1, len(nodes)))

        print("3. Calculando melhor rota (Vizinho Mais Próximo)...")

        while remaining:
            last = route_order[-1]
            candidates = []

            for r in remaining:
                dist, path = pathing_route_dijkstra(G, nodes[last], nodes[r])
                candidates.append((dist, r, path))

            candidates.sort(key=lambda x: x[0])
            chosen_distance, chosen_index, chosen_path = candidates[0]

            route_order.append(chosen_index)
            remaining.remove(chosen_index)
            total_distance += chosen_distance
            segment_paths.append(chosen_path)

        print("   Calculando retorno ao depósito...")
        chosen_distance, return_path = pathing_route_dijkstra(G, nodes[route_order[-1]], nodes[0])
        segment_paths.append(return_path)
        route_order.append(0)
        total_distance += chosen_distance

        print("\nOrdem de Parada da Rota:")
        for i, idx in enumerate(route_order):
            if i == 0:
                print(f"Partida: {addresses[idx]}")
            elif i == len(route_order) - 1:
                 print(f"Chegada (Retorno): {addresses[idx]}")
            else:
                print(f"{i}ª parada: {addresses[idx]}")

        print(f"Distância total da viagem: {round(total_distance/1000, 2)} km")

        fig, ax = ox.plot_graph(G, node_size=0, show=False, close=False, edge_color="#555555", bgcolor="black")
        fig.patch.set_facecolor('black')

        colors = ['cyan', 'orange', 'lime', 'magenta', 'yellow', 'white']
        for i, path in enumerate(segment_paths):
            c = colors[i % len(colors)]
            ox.plot_graph_route(G, path, ax=ax, node_size=0, route_linewidth=3, route_color=c, show=False, close=False)

        x_coords = [G.nodes[n]['x'] for n in nodes]
        y_coords = [G.nodes[n]['y'] for n in nodes]
        ax.scatter(x_coords, y_coords, s=100, c="red", zorder=5, edgecolors='white')

        for stop_number, node_index in enumerate(route_order[:-1]):
            node = nodes[node_index]
            x = G.nodes[node]['x']
            y = G.nodes[node]['y']
            ax.text(x, y, str(stop_number), fontsize=12, color="white", weight="bold", zorder=10)

        plt.title("Rota Otimizada em Campos dos Goytacazes", color="white")
        plt.gray()
        plt.show()

        return route_order
    

depot = "Estrada do Rio Grande, 3000, Taquara, Rio de Janeiro, RJ, 22723-220, Brasil"

orders = ["Rua Barata Ribeiro, 370, Copacabana, Rio de Janeiro, RJ, 22040-002, Brasil",
                  "Avenida Atlântica, 1702, Copacabana, Rio de Janeiro, RJ, 22021-000, Brasil",
                  "Rua do Catete, 153, Catete, Rio de Janeiro, RJ, 22220-000, Brasil",
                  "Rua Haddock Lobo, 195, Tijuca, Rio de Janeiro, RJ, 20260-142, Brasil",
                  "Avenida Presidente Vargas, 2000, Centro, Rio de Janeiro, RJ, 20071-003, Brasil",
                  "Rua Visconde de Pirajá, 550, Ipanema, Rio de Janeiro, RJ, 22410-002, Brasil"]

try:
    TripService.define_trip(depot, orders)
except Exception as e:
    print(f"Erro ao executar: {e}")
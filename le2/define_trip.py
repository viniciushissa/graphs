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
    def define_trip(depot_address, orders_address):
        addresses = [depot_address] + orders_address

        geolocator = Nominatim(user_agent="cheaptracker")

        locations = [geolocator.geocode(address) for address in addresses]

        for location, address in zip(locations, addresses):
            if location is None:
                raise ValueError(f"Address not found: {address}")

        G = ox.graph_from_place("Rio de Janeiro, Brasil", network_type="drive")

        xs = [loc.longitude for loc in locations]
        ys = [loc.latitude for loc in locations]
        nodes = ox.distance.nearest_nodes(G, xs, ys)

        segment_paths = []
        total_distance = 0
        route_order = [0]
        remaining = list(range(1, len(nodes)))

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

        # trocar para endereço do carrier
        chosen_distance, return_path = pathing_route_dijkstra(G, nodes[route_order[-1]], nodes[0])
        segment_paths.append(return_path)
        route_order.append(0)
        total_distance += chosen_distance

        print("\nRoute stop order:")
        for i, idx in enumerate(route_order):
            if i == 0:
                print(f"Departure: {addresses[idx]}")
            else: 
                print(f"{i}° stop: {addresses[idx]}")
        print(f"Distance traveled on the trip: {round(total_distance/1000, 1)} km")

        fig, ax = ox.plot_graph(G, node_size=0, show=False, close=False)

        for path in segment_paths:
            ox.plot_graph_route(G, path, ax=ax, node_size=0, route_linewidth=3, show=False, close=False)

        x = [G.nodes[n]['x'] for n in nodes]
        y = [G.nodes[n]['y'] for n in nodes]
        ax.scatter(x, y, s=120, c="red", zorder=5)

        for path in segment_paths:
            ox.plot_graph_route(G, path, ax=ax, node_size=0, route_linewidth=3, show=False, close=False)

        x = [G.nodes[n]['x'] for n in nodes]
        y = [G.nodes[n]['y'] for n in nodes]
        ax.scatter(x, y, s=120, c="red", zorder=5)

        for stop_number, node_index in enumerate(route_order):
            node = nodes[node_index]
            x = G.nodes[node]['x']
            y = G.nodes[node]['y']
            ax.text(x, y, str(stop_number), fontsize=11, color="yellow", weight="bold", zorder=10)

        plt.show()

        return route_order

orders_address = ["Rua Barata Ribeiro, 370, Copacabana, Rio de Janeiro, RJ, 22040-002, Brasil",
                  "Avenida Atlântica, 1702, Copacabana, Rio de Janeiro, RJ, 22021-000, Brasil",
                  "Rua do Catete, 153, Catete, Rio de Janeiro, RJ, 22220-000, Brasil",
                  "Rua Haddock Lobo, 195, Tijuca, Rio de Janeiro, RJ, 20260-142, Brasil",
                  "Avenida Presidente Vargas, 2000, Centro, Rio de Janeiro, RJ, 20071-003, Brasil",
                  "Rua Visconde de Pirajá, 550, Ipanema, Rio de Janeiro, RJ, 22410-002, Brasil"]

TripService.define_trip("Estrada do Rio Grande, 3000, Taquara, Rio de Janeiro, RJ, 22723-220, Brasil", orders_address)
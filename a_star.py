import pandas as pd
import numpy as np
import csv
import ast
import heapq
from math import cos, sin, asin, sqrt, inf, isnan, radians

class Node:
    def __init__(self, parent=None, latitude=None, longitude=None, station_name=None, trans_type=None, routes=[]):
        self.parent = parent
        self.lat = latitude
        self.long = longitude
        self.name = station_name
        self.type = trans_type
        self.routes = routes

        self.total_cost = 0 # total cost of node (aka f == g+h)
        self.dist_from_start = 0 # distance between current Node and starting Node (aka g)
        self.dist_from_end = 0 # heuristic: estimated distance (Haversine for now) from current Node to end Node (aka h)

    # overload equality operator
    def __eq__(self, other):
        return (self.lat == other.lat) and (self.long == other.long)

    # overload less than operator
    def __lt__(self, other):
        return self.total_cost < other.total_cost

    # overload greater than operator
    def __gt__(self, other):
        return self.total_cost > other.total_cost

def get_haversine_dist(lat1, long1, lat2, long2):
    """
    Get haversine distance (without 2r coefficient) of two stations based on their lat/long coordinates.
    station1 and station2 should be dicts w/ keys {name, long, lat, type}
    """
    EARTH_RADIUS_MILES = 3958.8

    lat1, long1, lat2, long2 = radians(lat1), radians(long1), radians(lat2), radians(long2)
    dist = asin(sqrt(pow(sin((lat2-lat1)/2),2) + cos(lat1)*cos(lat2)*pow(sin((long2-long1)/2),2)))
    
    return dist * EARTH_RADIUS_MILES * 2

def get_closest_station(coord, station_list, coord_type='coord'):
    '''
    Get closest station to the list of coordinates specified in the 'coord' argument.
    Coord can either be in the form ([latitude], [longitude]) or a station dict.
    '''
    if (coord_type != 'coord') and (coord_type != 'station'):
        print("coord_type argument can only be 'coord' or 'station'")
        return None

    best_station = {}
    best_dist = inf

    if coord_type == 'station':
        for station in station_list:
            station_lat = station['lat']
            station_long = station['long']
            if route_match(station['routes'], coord['routes'], input_type='routes') is False:
                continue
            dist_from_start = get_haversine_dist(coord['lat'], coord['long'], station_lat, station_long)
            if dist_from_start < best_dist:
                best_dist = dist_from_start
                current_station = station

    elif coord_type == 'coord':      
        for station in station_list:
            station_lat = station['lat']
            station_long = station['long']
            dist_from_start = get_haversine_dist(coord[0], coord[1], station_lat, station_long)
            if dist_from_start < best_dist:
#                 print('considering:', station)
#                 print('best_dist:', dist_from_start)
                best_dist = dist_from_start
                current_station = station

    return current_station

def route_match(station1, station2, input_type='station'):
    '''
    Determine if two stations are on the same route by checking their respective list of routes
    that they belong to.
    '''
    # FOR NOW: treat nan like they can transfer to any route
    # if type(routes1[0]) is float or type(routes2[0]) is float:
    #     return True
    if input_type == 'station':
        if station1['type'] != station2['type']:
            return True
        for route in station1['routes']:
            if route in station2['routes']:
                return True
        return False
    else: # station1 and station2 are lists of routes
        for route in station1:
            if route in station2:
                return True
        return False

def already_seached_route(station_routes, visited_routes_list):
    for route in station_routes:
        if route in visited_routes_list:
            return True
    return False

def backtrack(current_node):
    path = []
    visited_routes = []
    current = current_node
    while current is not None:
        path.append({'name': current.name, 'long': current.long, 'lat': current.lat, 'type': current.type, 'routes': current.routes})
        visited_routes += current.routes
        current = current.parent
    return path[::-1], list(set(visited_routes))

def a_star(start_coord, end_coord, station_list):
    print('beginning a* algo')

    start = get_closest_station(start_coord, station_list)
    end = get_closest_station(end_coord, station_list)

    print('START:', start)
    print('END:', end)

    # assume start and end are in the station dict format
    start_node = Node(None, start['lat'], start['long'], start['name'], start['type'], start['routes'])
    end_node = Node(None, end['lat'], end['long'], end['name'], end['type'], end['routes'])

    # initialize open and closed lists
    open_list = []
    closed_list = []
    heapq.heapify(open_list)
    heapq.heappush(open_list, start_node)

    outer_iterations = 0
    max_iterations = 100
    while len(open_list) > 0:
        outer_iterations += 1
        if outer_iterations > max_iterations:
            print("Cannot find optimal path. Re-trying algorithm with start and end station matching.")
            # find new start node
            new_eligible_stations = [s for s in station_list if route_match(s['routes'], end['routes'], input_type='routes')]
            start_candidate = get_closest_station(start_coord, new_eligible_stations)

            # measure distance from start_coord to new starting station in miles
            new_start_miles = get_haversine_dist(start_coord[0], start_coord[1], start_candidate['lat'], start_candidate['long'])
            print('new starting station:', start_candidate)
            print('this new station is', new_start_miles, 'miles away')

            # if new station is more than 1/3 miles away, switch mode of transportation
            switched_mode = False
            if new_start_miles > (1/3):
                print('Considering switching modes of transportation.')
                new_eligible_stations = [s for s in station_list if s['type'] is not start_candidate['type']]
                start_candidate = get_closest_station(start_coord, new_eligible_stations)
                print('new', start_candidate['type'], 'station:', start_candidate)
                new_start_miles = get_haversine_dist(start_coord[0], start_coord[1], start_candidate['lat'], start_candidate['long'])
                print('this new station is', new_start_miles, 'miles away')
                end_candidate = get_closest_station(end_coord, new_eligible_stations)
                # but only switch modes if this leads to less walking
                if get_haversine_dist(start_coord[0], start_coord[1], start_candidate['lat'], start_candidate['long']) <= (1/3)\
                and get_haversine_dist(end_coord[0], end_coord[1], end_candidate['lat'], end_candidate['long']) <= (1/3):
                    print('SWITCHING MODES OF TRANSPORTATION')
                    start = start_candidate
                    end = end_candidate
                    print('new start:', start)
                    print('new end:', end)
                    switched_mode = True

            new_start_node = Node(None, start['lat'], start['long'], start['name'], start['type'], start['routes'])
            if switched_mode is True:
                end_node = Node(None, end['lat'], end['long'], end['name'], end['type'], end['routes'])

            # if start_node is still the same, let the algorithm run its course
            if new_start_node == start_node:
                if max_iterations >= 1000:
                    print("Returning empty lists. The shortest route will require an unrealistic amount of transfers. "+
                           "Please try Kyle's fewest transfer code.")
                    return [], []
                print('Having a little trouble....Increasing maximum iterations to find route.')
                max_iterations = 1000

            # clear out the heap
            open_list = []
            heapq.heapify(open_list)
            heapq.heappush(open_list, new_start_node)

            # reset outer_iterations
            outer_iterations = 0

        current_node = heapq.heappop(open_list)
        # print('heap size', len(open_list))
        # if len(open_list) > 0:
            # print('first station in open_list', open_list[0].name)
        #     print('last station in open_list', open_list[-1].name)
        # print('current node:', current_node.name, current_node.lat, current_node.long, current_node.routes)
        # print('current node cost:', current_node.total_cost)
        # current_index = 0
        # for idx, item in enumerate(open_list):
        #     if item.total_cost < current_node.total_cost:
        #         current_node = item
        #         current_index = idx

        # pop current node off open list and add it to closed list
        # open_list.pop(current_index)
        closed_list.append(current_node)

        if current_node == end_node:
            print('reached end')
            return backtrack(current_node)

        # create children nodes
        children = []
        # Current definition of neighbors:
        # - all stations whose routes match at least one of the current station's routes
        # - the 3 closest stations to the current route where the above condition doesn't hold
        neighbors = [s for s in station_list if s['name'] != current_node.name and
        route_match(s['routes'], current_node.routes, input_type='routes')]
        # not_transferrable = [s for s in station_list if route_match(s['routes'], current_node.routes, input_type='routes') is False]
        # closest_3 = []
        # nt_dist_list = []
        # for i, station in enumerate(not_transferrable):
        #   cur_dist = get_haversine_dist(station['lat'], station['long'], current_node.lat, current_node.long)
        #   closest_dict = {'idx': i, 'dist': cur_dist}
        #   nt_dist_list.append(closest_dict)
        # nt_dist_list = sorted(nt_dist_list, key=lambda d: d['dist'])[:3]
        # for n in nt_dist_list:
        #   closest_3.append(not_transferrable[n['idx']])
        # neighbors += closest_3
        # print('neighbors:', len(neighbors))
        # print('iteration', outer_iterations)
        for station in neighbors:
            node_lat = station['lat']
            node_long = station['long']
            node_name = station['name']
            node_type = station['type']
            node_routes = station['routes']
            new_node = Node(current_node, node_lat, node_long, node_name, node_type, node_routes)
            children.append(new_node)

        # search children for next station
        for child in children:
            # do not search children already on the closed list
            if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                continue

            # update heuristic values for child
            child.dist_from_start = get_haversine_dist(child.lat, child.long, start_node.lat, start_node.long)
            child.dist_from_end = get_haversine_dist(child.lat, child.long, end_node.lat, end_node.long)
            child.total_cost = child.dist_from_start + child.dist_from_end

            # don't need to add child to open list again if it's already in there
            if len([open_node for open_node in open_list if child == open_node]) > 0:
                continue
            heapq.heappush(open_list, child)

def main():
    # read in data
    lat_long_data = pd.read_csv("closest_buses.csv")
    bus_route_data = pd.read_csv("BUS_ROUTE_ORDERS.csv")
    train_route_data = pd.read_csv("trains_stops_w_lines.csv")

    # reconcile spelling/letter case differences between datasets
    wrong_sp = sorted([t for t in train_route_data['NAME'] if t not in list(lat_long_data['train_station_name'])])
    right_sp = sorted(list(set([t for t in lat_long_data['train_station_name'] if t not in list(train_route_data['NAME'])])))
    for i, name in enumerate(wrong_sp):
        train_route_data.loc[train_route_data['NAME'] == name, 'NAME'] = right_sp[i]

    # join metro station names to their routes
    train_route_tojoin = train_route_data[["NAME", "MetroLine"]]
    train_route_tojoin.rename(columns={"NAME": "train_station_name"}, inplace=True)
    lat_long_data = lat_long_data.merge(train_route_tojoin, on='train_station_name', how='left')

    # put stations into a list to use for algorithm
    list_of_stations = []
    for idx, entry in lat_long_data.iterrows():
        train_station = {'name': entry[1], 'long': entry[2], 'lat': entry[3], 'type': 'train'}
        if train_station not in list_of_stations:
            list_of_stations.append(train_station)
    for idx, entry in bus_route_data.iterrows():
        bus_station = {'name': entry[2], 'long': entry[3], 'lat': entry[4], 'type': 'bus'}
        if bus_station not in list_of_stations:
            list_of_stations.append(bus_station)
    for station in list_of_stations:
        if station['type'] is 'train':
            routes = lat_long_data[lat_long_data['train_station_name'] == station['name']]['MetroLine']
            for item in routes:
                route_list = []
                for line in item.split(','):
                    route_list.append(line)
            station['routes'] = route_list
        elif station['type'] is 'bus':
            routes = bus_route_data[bus_route_data['Name'] == station['name']]['Routes']
            station['routes'] = ast.literal_eval(list(set(routes))[0])

    # some hardcoded location coordinates to test code with
    WASHINGTON_MONUMENT = (38.889484, -77.035278)
    ROSSLYN = (38.8940, -77.0752)
    ROSSLYN_STATION = (38.896595, -77.07146)
    JEFFERSON_MEM = (38.8814, -77.0365)
    COLLEGE_PARK = (38.9869, -76.9426)
    COLLEGE_PARK_STATION = (38.978606, -76.928886)
    DULLES = (38.9625, -77.4380)
    BRANCH_AVE = (38.8270, -76.9124)
    SHADY_GROVE = (39.1201, -77.1648)

    '''
    TEST CASES
    '''
    # College Park to DULLES
    # start1 = get_closest_station(COLLEGE_PARK, list_of_stations)
    # end1 = get_closest_station(DULLES, list_of_stations)
    # #new_station_list = [s for s in list_of_stations if route_match(s, start1)]
    # #path1, visited_routes1 = a_star(COLLEGE_PARK, DULLES, new_station_list)

    # path1, visited_routes1 = a_star(COLLEGE_PARK, DULLES, list_of_stations)
    # print(path1)
    # return

    # path2, visited_routes2 = a_star(COLLEGE_PARK, WASHINGTON_MONUMENT, list_of_stations)
    # print(path2)
    # return

    path3, visited_routes3 = a_star(BRANCH_AVE, SHADY_GROVE, list_of_stations)
    print(path3)
    return

    # if end1 in path1:
    #     return
    # while end1 not in path1:
    #     # print('stopped at:', path1[-1])
    #     # print('visited routes:', sorted(visited_routes1))
    #     new_start = get_closest_station((path1[-1]['lat'], path1[-1]['long']), [s for s in list_of_stations if s not in path1
    #         and already_seached_route(s['routes'], visited_routes1) is False])
    #     # print('TRANSFER TIME')
    #     # print('transferring to:', new_start)
    #     new_station_list = [s for s in list_of_stations if route_match(s, new_start) and s not in path1]
    #     new_path, new_visited_routes = a_star((new_start['lat'], new_start['long']), DULLES, new_station_list)
    #     visited_routes1 += new_visited_routes
    #     visited_routes1 = list(set(visited_routes1))
    #     path1 += new_path


    # print(path1)
    # H st nw + 9th st nw
    # print(get_haversine_dist(38.89974, -77.02416, WASHINGTON_MONUMENT[0], WASHINGTON_MONUMENT[1]) +\
    #   get_haversine_dist(38.89974, -77.02416, COLLEGE_PARK[0], COLLEGE_PARK[1]))
    # # K str nw + 4th st nw
    # print(get_haversine_dist(38.902454, -77.016391, WASHINGTON_MONUMENT[0], WASHINGTON_MONUMENT[1])+\
    #   get_haversine_dist(38.89974, -77.02416, COLLEGE_PARK[0], COLLEGE_PARK[1]))
    # # 13th st nw + I st nw
    # print(get_haversine_dist(38.901438, -77.030088, WASHINGTON_MONUMENT[0], WASHINGTON_MONUMENT[1])+\
    #   get_haversine_dist(38.89974, -77.02416, COLLEGE_PARK[0], COLLEGE_PARK[1]))
    # # 14th st + Jefferson
    # print(get_haversine_dist(38.888463, -77.031797, WASHINGTON_MONUMENT[0], WASHINGTON_MONUMENT[1])+\
    #   get_haversine_dist(38.89974, -77.02416, COLLEGE_PARK[0], COLLEGE_PARK[1]))
    # return

    # current test case
    # path2, visited_routes2 = a_star(COLLEGE_PARK, WASHINGTON_MONUMENT, list_of_stations)
    # # TODO: sort path by decreasing haversine distance
    # print(path2)

if __name__ == '__main__':
    main()

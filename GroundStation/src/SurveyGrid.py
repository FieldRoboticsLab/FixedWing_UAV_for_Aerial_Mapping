#Author: Bugra Buyukarslan
import math
import matplotlib.pyplot as plt
import numpy as np
from polylabel import polylabel
from shapely.geometry import Polygon, Point
import utm

show_animation = True

"""
coordinates = [[-35.360607, 149.164255], [-35.361346, 149.161181], [-35.366425, 149.161879], [-35.362628, 149.164488]]
"""

text_init = "QGC WPL 110\n"

NAV_TAKEOFF  = "{}	1	3	22	15.0	0.0	0.0	0.0	{}	{}	{}	1\n"

NAV_WAYPOINT = "{}	0	3	16	0.0	0.0	0.0	0.0	{}	{}	{}	1\n"

NAV_LAND     = "{}	0	3	21	0.0	0.0	0.0	0.0	{}	{}	0	1"

class SurveyGrid:
    def __init__(self):
        self.coords_list = None
        self.dist_corridors = 40
        self.degree = 80
        self.alt = 30

    def get_coords(self, coordinates):
        self.coords_list = coordinates
        print("-------******************-----------")
        print(self.coords_list)
        print("-------******************-----------")
        #print(self.coords)
        self.coords_list.append(self.coords_list[0])
        self.coords_list.append(self.coords_list[0])        


    def ref_zone_utm_from_latlon(self):
        coords = self.coords_list
        x, y, zone, ut = utm.from_latlon(coords[0][0],coords[0][1])                                   #Our origin - referance point to convert coordinates
        xy_coords_list = []

        for i in range(len(coords)):
            px , py, zone, ut = utm.from_latlon(coords[i][0],coords[i][1])                            # Convert mappint area cartesian from lat lon
            px -= x
            py -= y
            xy_coords_list.append((px,py))

        return xy_coords_list, zone, ut, x, y


    def get_wps_latlon(self):                                                                         # convert waypoints cartesian to lat lon
        waypoint_list_cartesian = self.get_intersection_list()
        ref_point, zone, ut ,x ,y = self.ref_zone_utm_from_latlon()
        waypoint_list_latlon = []

        for i in range(len(waypoint_list_cartesian)):
            to_lat = waypoint_list_cartesian[i][0] + x
            to_lon = waypoint_list_cartesian[i][1] + y
            lat, lon = utm.to_latlon(to_lat, to_lon, zone, ut)
            waypoint_list_latlon.append([lat,lon])

        for i in range(2,len(waypoint_list_latlon), 4):
            temp = waypoint_list_latlon[i]
            waypoint_list_latlon[i] = waypoint_list_latlon[i+1]
            waypoint_list_latlon[i+1] = temp
            
        print("--------------------------------------------")
        print(waypoint_list_latlon)
        print("--------------------------------------------")

        return waypoint_list_latlon
    
    def save_to_txt(self, file_name, waypoint_list=0):
        if waypoint_list == 0:
            waypoint_list = self.get_wps_latlon()
        file = open("./missions/ardupilot/{}".format(file_name)+".txt", "w")
        file.write(text_init)
        for _ in range(2):
            file.write(NAV_TAKEOFF.format(1, waypoint_list[0][0], waypoint_list[0][1], self.alt))
        
        for i in range(1, len(waypoint_list)-1):
            file.write(NAV_WAYPOINT.format(i+1, waypoint_list[i][0], waypoint_list[i][1], self.alt))
        
        file.write(NAV_LAND.format(len(waypoint_list), waypoint_list[-1][0], waypoint_list[-1][1]))

        file.close()
    
    def define_polygon(self):                                                                         # Return cartesian polygon (mapping area)
        self.cartesian_coords , zone, ut, x ,y = self.ref_zone_utm_from_latlon()

        return self.cartesian_coords


    def divide_coords_xy(self):                                                                       # Divide polygons coords to x-y
        coords = self.define_polygon()
        num_points = len(coords)
        self.x_list = []
        self.y_list = []

        for i in range(num_points -2):
            self.x_list.append(coords[i][0])
            self.y_list.append(coords[i][1])

        return self.x_list, self.y_list


    def is_inside(self,x,y,poly):                                                                     # Returns a point is inside or outside of a polygon
        n = len(poly)
        inside =False
        p1x,p1y = poly[0]

        for i in range(n+1):
            p2x,p2y = poly[i % n]
            if y > min(p1y,p2y):
                if y <= max(p1y,p2y):
                    if x <= max(p1x,p2x):
                        if p1y != p2y:
                            xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x,p1y = p2x,p2y

        return inside                                                                                  # Returns True or False

    
    def centroid_polylabel(self):                                                                      # Returns centroid point using polylabel algorithm.
        polygon = self.define_polygon()
        centroid_polylabel = polylabel([polygon])

        return centroid_polylabel


    def longest_straight(self):                                                                        # Returns longest straight in polygon (mapping area)
        lines = self.convert_polynodes_to_lines()
        length = 0

        for i in range(len(lines)):
            dx = float(lines[i][0][0]) - float(lines[i][1][0])
            dy = float(lines[i][0][1]) - float(lines[i][1][1])
            length += (math.sqrt(dx**2 + dy**2))

        return length


    def corridor_center_points(self):                                                                  # Returns center points to draw corridors.
        distance = self.dist_corridors
        centroid = self.centroid_polylabel()
        angle , reversed_angle = self.set_angle()                                                                                              
        angle += math.radians(90)                                                                      # Line centers will be vertical to the main axis
        reversed_angle += math.radians(90)                                                             # Line centers will be vertical to the main axis
        center_points_list = []
        center_points_list.append(centroid)
        top_list_size = int(((self.longest_straight())/2)/distance)

        # We add a line perpendicular to the centroid. One end of the added line is the centroid and the other is the center point of the corridors.
        # We have the distance between the corridors (line length), the centroid, and the right angle to find corridor centers.

        for i in range(top_list_size):
            endx = centroid[0] + (distance * math.cos(angle))
            endy = centroid[1] + (distance * math.sin(angle))
            distance += self.dist_corridors
            add = (endx, endy)
            center_points_list.append(add)

        center_points_list.reverse()
        distance = self.dist_corridors

        for a in range(top_list_size):
            endx = centroid[0] + (distance * math.cos(reversed_angle))
            endy = centroid[1] + (distance * math.sin(reversed_angle))
            distance += self.dist_corridors
            add = (endx, endy)
            center_points_list.append(add)

        return center_points_list                                                                     # Returns a list of corridor centers 


    def set_ax_lines(self):                                                                           # Using corridor centers and wind rotation, determines the corridor lines
        length = self.longest_straight()
        center_points_list = self.corridor_center_points()
        angle, reversed_angle = self.set_angle()
        length = length/2
        line_list = []

        for i in range(len(center_points_list)):
            endx = center_points_list[i][0] + (length * math.cos(angle))
            endy = center_points_list[i][1] + (length * math.sin(angle))
            mirrorx = center_points_list[i][0] + (length * math.cos(reversed_angle))
            mirrory = center_points_list[i][1] + (length * math.sin(reversed_angle))
            line_list.append(([endx, endy], [mirrorx, mirrory]))

        return line_list                                                                              # Returns the corridor lines.           


    def convert_polynodes_to_lines(self):                                                             # Convert elements to lines to use detect intersections.
        polygon = self.define_polygon()
        poly_lines = []

        for i in range(len(polygon)-2): 
            line = polygon[i], polygon[i+1]
            poly_lines.append(line)

        return poly_lines                                                                             # Returns the lines that creates the polygon.


    def line_intersection(self, line1, line2):                                                        # Function to detect intersection between 2 lines.
        Ax1 = line1[0][0]
        Ax2 = line1[1][0]
        Bx1 = line2[0][0]
        Bx2 = line2[1][0]

        Ay1 = line1[0][1]
        Ay2 = line1[1][1]
        By1 = line2[0][1]
        By2 = line2[1][1]

        d = (By2 - By1) * (Ax2 - Ax1) - (Bx2 - Bx1) * (Ay2 - Ay1)                                     # Analytical Geometry formulas.
        if d:
            uA = ((Bx2 - Bx1) * (Ay1 - By1) - (By2 - By1) * (Ax1 - Bx1)) / d
            uB = ((Ax2 - Ax1) * (Ay1 - By1) - (Ay2 - Ay1) * (Ax1 - Bx1)) / d
        else:
            return
        if not(0 <= uA <= 1 and 0 <= uB <= 1):
            return
        x = Ax1 + uA * (Ax2 - Ax1)
        y = Ay1 + uA * (Ay2 - Ay1)

        return (x,y)                                                                                  # Returns intersection points


    def get_intersection_list(self):                                                                  # Checks the polygon and line lists for ray intersection with each other.
        polygon = self.convert_polynodes_to_lines()
        lines = self.set_ax_lines()
        intersection_list = []

        for i in range(len(lines)):
            for a in range(len(polygon)):
                intersection_point = self.line_intersection(polygon[a],lines[i])
                intersection_list.append(intersection_point)

        intersection = []
        for val in intersection_list:                                                                 # Delete parallel (None) elements from intersection list
            if val != None:
                intersection.append(val)

        orianted_intersections = []
        for i in range(0,len(intersection),2):
            if i < len(intersection)-1:                
                if abs(intersection[i][1]) > abs(intersection[i+1][1]):
                    orianted_intersections.append(intersection[i])
                    orianted_intersections.append(intersection[i+1])
                else:
                    orianted_intersections.append(intersection[i+1])       
                    orianted_intersections.append(intersection[i])  
            else:
                orianted_intersections.append(intersection[i])

        if len(orianted_intersections)%2 == 1:
            del(orianted_intersections[-1])       

        return orianted_intersections                                                                 # Returns the list of ray intersections.


    def get_trajectory(self):                                                                         # Sort intersections to get suitable waypoint list
        intersections = self.get_intersection_list()
        trajectory_lines = []
        ind = 1
        for i in range(len(intersections)):
            if ind %4 == 1:
                indx = i
                indy = i+1
            elif ind %4 == 2:
                indx = i
                indy = i+2
            elif ind %4 == 3:
                indx = i+1
                indy = i
            else:
                indx = i-1
                indy = i+1
            if indx >= len(intersections):
                indx = 0
            if indy >= len(intersections):
                indy = 0
            trajectory_lines.append(((intersections[indx][0], intersections[indx][1]), (intersections[indy][0], intersections[indy][1])))
            ind +=1

        return trajectory_lines                                                                      # Returns trajectory lines to plot it.


    def get_waypoints(self):
        line_list = self.get_trajectory()

        waypoints_cartesian = []
        for i in range(len(line_list)):
            if i != len(line_list):
                waypoints_cartesian.append(line_list[i][0])
            else:
                waypoints_cartesian.append(line_list[i][0])
                waypoints_cartesian.append(line_list[i][1])

        return waypoints_cartesian


    def draw_intersections(self):                                                                    # Draws non-ray intersections(waypoints)
        intersections = self.get_intersection_list()

        for i in range(len(intersections)):
            plt.figure(1)
            plt.plot(intersections[i][0],intersections[i][1], "bx")


    def draw_test_polygon(self): 
        poly = self.define_polygon()                                                                  # Plots the polygon (mapping area)
        xs ,ys = zip(*poly)
        plt.figure(1)
        plt.plot(xs,ys) 


    def draw_ax_lines(self):                                                                          # Draws the corridor liens.
        lines_to_draw = self.set_ax_lines()

        for i in range(len(lines_to_draw)):
            plt.figure(1)
            plt.plot([lines_to_draw[i][0][0],lines_to_draw[i][1][0]] , [lines_to_draw[i][0][1],lines_to_draw[i][1][1]], "g")


    def draw_center_points(self):                                                                     # Draws corridor centers
        center_points = self.corridor_center_points()

        for i in range(len(center_points)):
            plt.figure(1)
            plt.plot(center_points[i][0],center_points[i][1], "r+")


    def draw_trajectory(self):
        lines_to_draw = self.get_trajectory()
        del(lines_to_draw[-1])

        for i in range(len(lines_to_draw)):
            plt.figure(1)
            plt.plot([lines_to_draw[i][0][0],lines_to_draw[i][1][0]] , [lines_to_draw[i][0][1],lines_to_draw[i][1][1]], "r")


    def set_angle(self):                                                                              # Wind direction. Will be requested from the user.
        degree = self.degree       
        angle = math.radians(degree)
        reversed_angle = math.radians(degree+180)
        
        return angle, reversed_angle

    def get_angle(self, angle):
        self.degree = angle

    def get_dist_corridros(self, dist):
        self.dist_corridors = dist

    def get_alt(self, alt):
        self.alt = alt
        
    def polygon_area(self):                                                                           # Will be calculated using mapping area
        corners = self.define_polygon()
        n = len(corners)
        area = 0.0

        for i in range(n):
            j = (i + 1) % n
            area += corners[i][0] * corners[j][1]
            area -= corners[j][0] * corners[i][1]
        area = abs(area) / 2.0

        print(area,"square meters")


    def total_trajectory_distance(self):                                                              # Will be calculated using trajectory and map scale
        trajectory = self.get_trajectory()

        dist = 0
        for i in range(len(trajectory)):
            x = trajectory[i][0][0] - trajectory[i][1][0]
            y = trajectory[i][0][1] - trajectory[i][1][1]
            dist += math.sqrt(x**2 + y**2)
        return dist


    def battery_req(self, velocity):
        dist = self.total_trajectory_distance()
        print(dist,"meters")
        flight_time = (dist / velocity) / 60                                                                 # For 15m/s cruise speed
        print("required flight time in minutes:", flight_time)
        return flight_time


def main():
    SG = SurveyGrid()
    
    coordinates = eval(input("Enter coordinates : "))
    SG.get_dist_corridros(20)
    SG.get_angle(80)
    SG.get_coords(coordinates)
    
    #SG.draw_test_polygon()
    #SG.draw_center_points()
    #SG.draw_ax_lines()
    #SG.draw_intersections()
    #SG.draw_trajectory()
    #SG.get_wps_latlon()
    SG.save_to_txt("test3.txt")
    #SG.polygon_area()
    #SG.total_trajectory_distance()
    #SG.battery_req()

    if show_animation:
        plt.show()

if __name__ == '__main__':
    main()

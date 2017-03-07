#!/usr/bin/env python
import rospy
import itertools
import numpy as np
from shapely.geometry import Polygon, LineString, Point
from human_trajectory.trajectories import OfflineTrajectories
from strands_navigation_msgs.msg import TopologicalMap
from util import get_soma_info
from scipy.spatial.distance import cosine, euclidean, correlation

######################## ADDED STATE NOWHERE
######################## REDUCING NUMBER OF NODES FOR BAYES NET PURPOSES
######################## MODIFYING M3.PY TO INCLUDE PATH VALIDATION
######################## SPLITTING TOPOLOGICAL EDGES
######################## CONNECTING WAYPOINTS AND REGIONS
######################## MAKING CHANGES FOR VALIDATING CHOSEN EDGE
######################## DIFFERENT LOGIC FOR PATH VALIDATION

class state_transitions: 
    def __init__(self, query):
       self.t_map = None
       self.regions = None
       self.soma_map = None
       self.no_points = 15
       self.edges_lines = dict()
       self.waypoints = dict()
       self.waypoints_regions = dict()
       self.edges_waypoints = dict()
       self.trajectories = []
       self.edges_points = dict()
       self.states_probs = dict()
       self.trajectory_state = dict()
       soma_config = 'cropped_cleaned_config'
       self.initialize(soma_config, query)
       self.states_traj_origins = dict()
       self.reduced_state_probs = dict() 
       
    def topo_map_cb(self, topo_map):
       self.t_map = topo_map
       
    def initialize(self, soma_config, query):
       # SOMA Regions in the form of a dictionary mapping to polygons
       print 'Working on SOMA Regions'
       self.regions, self.soma_map = get_soma_info(soma_config)
       #print self.regions
       
       # Trajectory Information
       print 'Working on Trajectories' 
       trajectories = OfflineTrajectories(query)
       for i in trajectories.traj.values():
           self.trajectories.append(i.get_trajectory_message())
    
       # Loading topological map info
       topo_sub = rospy.Subscriber("/topological_map", TopologicalMap, self.topo_map_cb, None, 10)
       rospy.loginfo("Getting information from /topological_map...")
       while self.t_map is None:
           print 'No data from topologocial map'
           rospy.sleep(0.1)
       topo_sub.unregister()
       
       # Loading info on waypoints and edges from topological map 
       print 'Working on Waypoints' 
       for wp in self.t_map.nodes:
           if 'ChargingPoint' not in wp.name:
               self.waypoints.update({wp.name : wp.pose.position})
           
       print 'Working on edges'
       for wp in self.t_map.nodes:
           if wp.name != 'ChargingPoint':
               f_point = [wp.pose.position.x, wp.pose.position.y]
               for edge in wp.edges:
                   node_name = edge.node
                   if node_name != 'ChargingPoint': 
                       node_position = self.waypoints[node_name]  
                       t_point = [node_position.x, node_position.y]
                       self.edges_lines.update({edge.edge_id : [f_point, t_point]})
       #print self.edges_lines            
       # Mapping regions to way points
       print 'Mapping regions and waypoints'
       for waypoint, position in self.waypoints.iteritems():
           p = Point(position.x, position.y)
           for regionid, polygon in self.regions.iteritems():
               if p.intersects(polygon):
                   self.waypoints_regions.update({waypoint : regionid})         
       # print self.waypoints_regions
    
       # Splitting Topological Edges in Smaller Parts 
       print 'Splitting Topological Edges'  
       for edge in self.edges_lines:
           #print edge ,'HAPPENING'
           x1 = self.edges_lines[edge][0][0]
           y1 = self.edges_lines[edge][0][1]
           x2 = self.edges_lines[edge][1][0]
           y2 = self.edges_lines[edge][1][1]
           deltax = (x2 - x1)/(self.no_points -1)         # Splitting edges such that we get 15 points in total
           deltay = (y2 - y1)/(self.no_points -1) 
           x = x1
           y = y1
           points = []
           
           for i in range(0,self.no_points):
               if (x2 - x1) == 0:
                   y = y + deltay
               else:
                   y = (y2-y1)/(x2-x1)*x + (y1*(x2-x1) - x1*(y2-y1))/(x2-x1)
               points.append([x,y])
               x = x + deltax         
           
           self.edges_points.update({edge : points})
       print 'TOTAL NUMBER OF EDGES',len(self.edges_points)
    
    # Mapping trajectories to edges to find state transitions
    def calculate_state_transitions(self):
      count = 0 
      for t in self.trajectories:
          count += 1
          print count
          #print t.uuid
          trajectory = t.trajectory
          min_dist = np.inf
          for path, e_p in self.edges_points.iteritems():
              dist = self.find_average_distance(trajectory, e_p)
              if dist < min_dist and self.valid_path(path, trajectory):                   
                  min_dist = dist
                  min_path = path
          #print min_dist, min_path
          # For keeping track of trajectory and states associated to it
          self.trajectory_state.update({ t : min_path})
          # For keeping track of no. of trajectories that originate from a point
          states = min_path.split('_', 1)
          print states
          # IF CONDITION FOR CALCULATING REDUCED STATE HALLWAY TRANSITIONS PROPERLY
          if (min_path != 'WayPoint9_WayPoint12' and min_path != 'WayPoint12_WayPoint9'):
              print 'After if condition min_path', min_path
              if states[0] not in self.states_traj_origins:
                  self.states_traj_origins.update({states[0] : 1.0})
              else:
                  self.states_traj_origins[states[0]] += 1
          # For keeping track of the number of state changes possible among given trajectories
          if min_path not in self.states_probs:
              self.states_probs.update({min_path : 1.0})
          else:
              self.states_probs[min_path] += 1
      # Calculating final state transition probabilities        
      for state in self.states_probs:
          x = state.split('_', 1)
          self.states_probs[state] = float(self.states_probs[state]/self.states_traj_origins[x[0]]) 
          #print state, self.states_probs[state]
      for edge in self.edges_points:
          if edge not in self.states_probs:
              #print edge, 0
              self.states_probs.update({edge : 0})  
      #print 'TOTAL NUMBER OF STATE TRANSITIONS ' ,len(self.states_probs) 
              
    # For returning to an external function      
    def get_trajectories_states(self):
       self.calculate_state_transitions()
       return self.trajectory_state, self.states_probs
    
    # Reducing the number of states for bayes network purposes
    def reducing_states(self):
       print 'Reducing the states by combining them'
       reduced_states = dict()
       for t in self.trajectory_state:
           state = self.trajectory_state[t].split('_',1)
           for s in range(0,len(state)):
               if (state[s] == 'WayPoint1' or state[s] == 'WayPoint5' or state[s]== 'WayPoint16' or state[s] == 'WayPoint18'): 
                   state[s] = 'WayUpstairs'
               elif (state[s] == 'WayPoint8' or state[s] == 'WayPoint15'):
                   state[s] = 'RobotLab'
               elif (state[s] == 'WayPoint9' or state[s] == 'WayPoint12'):
                   state[s] = 'Hallway'
               elif (state[s] == 'WayPoint7' or state[s] == 'WayPoint13'):
                   state[s] = 'VendingMachine'
           #print state
           # Inequality added to remove hallway to hallway self transitions
           if (state[0] != state[1]):
               reduced_states.update({t : state[0]+'_'+state[1]})
                   
       reduced_states_traj_origin = dict()
       #print 'Original', self.states_traj_origins
       for s in self.states_traj_origins:
           if ( s == 'WayPoint1' or s == 'WayPoint5' or s == 'WayPoint16' or s == 'WayPoint18' or s == 'WayPoint6'): 
               if 'Nowhere on LGF' not in reduced_states_traj_origin:
                   reduced_states_traj_origin.update({ 'Nowhere on LGF' : self.states_traj_origins[s]})
               else:
                   reduced_states_traj_origin['Nowhere on LGF'] += self.states_traj_origins[s] 
               if ( s == 'WayPoint1' or s == 'WayPoint5' or s == 'WayPoint16' or s == 'WayPoint18'): 
                   if 'WayUpstairs' not in reduced_states_traj_origin:
                       reduced_states_traj_origin.update({ 'WayUpstairs' : self.states_traj_origins[s]})
                   else:
                       reduced_states_traj_origin['WayUpstairs'] += self.states_traj_origins[s]
               else:
                   if 'WayPoint6' not in reduced_states_traj_origin:
                       reduced_states_traj_origin.update({ 'WayPoint6' : self.states_traj_origins['WayPoint6']})
                   
           elif (s == 'WayPoint15' or s == 'WayPoint8'):   
               if 'RobotLab' not in reduced_states_traj_origin:
                   reduced_states_traj_origin.update({ 'RobotLab' : self.states_traj_origins[s]})
               else:
                   reduced_states_traj_origin['RobotLab'] += self.states_traj_origins[s]
                   
           elif ( s == 'WayPoint9' or s == 'WayPoint12'):   
               if 'Hallway' not in reduced_states_traj_origin:
                   reduced_states_traj_origin.update({ 'Hallway' : self.states_traj_origins[s]})
               else:
                   reduced_states_traj_origin['Hallway'] += self.states_traj_origins[s]
                
           elif ( s == 'WayPoint7' or s == 'WayPoint13'):   
               if 'VendingMachine' not in reduced_states_traj_origin:
                   reduced_states_traj_origin.update({ 'VendingMachine' : self.states_traj_origins[s]})
               else:
                   reduced_states_traj_origin['VendingMachine'] += self.states_traj_origins[s]
              
           else:
               reduced_states_traj_origin.update({ s : self.states_traj_origins[s] })
           
                    
       #print reduced_states_traj_origin
       return reduced_states, reduced_states_traj_origin
       
    # Calculating probabilities for reduced states
    def get_reduced_states_probs(self):
       print 'Calculating reduced state probabilities'
       t_states, t_from_state = self.reducing_states()
       for t in t_states:
           if t_states[t] not in self.reduced_state_probs:
               self.reduced_state_probs.update({t_states[t] : 1})
               self.reduced_state_probs.update({})
           else:
               self.reduced_state_probs[t_states[t]] += 1
              
       for s in self.reduced_state_probs:
           states = s.split('_',1)
           #print 'before cal' , states
           self.reduced_state_probs[s] = float(self.reduced_state_probs[s]/t_from_state[states[0]])
           #print s 
       self.reduced_state_probs.update({ 'Nowhere on LGF_WayPoint6' : float(t_from_state['WayPoint6']/t_from_state['Nowhere on LGF'])})
       self.reduced_state_probs.update({'Nowhere on LGF_WayUpstairs':float(t_from_state['WayUpstairs']/t_from_state['Nowhere on LGF'])}) 
       return self.reduced_state_probs  
              
    # Calculation of distance between two edges.
    def find_average_distance(self,trajectory_points, edge_points):
       if len(trajectory_points) > self.no_points:
           factor = (len(trajectory_points) - 2)//(self.no_points -2) 
           index = []
           for k in range(0,(self.no_points -1)):
               index.append(factor*k) 
           index.append(len(trajectory_points)-1)       
           dist = 0
           for i in range(0, self.no_points):
               t_p = [trajectory_points[index[i]].pose.position.x, trajectory_points[index[i]].pose.position.y]
               dist += euclidean(np.asmatrix(t_p), np.asmatrix(edge_points[i]))
           return dist/self.no_points
       
       elif len(trajectory_points) < self.no_points:
           factor = (self.no_points -1) //(len(trajectory_points)-2)
           index = []
           for k in range(0,len(trajectory_points)-1):
               index.append(factor*k) 
           index.append((self.no_points -1))  
           dist = 0
           for i in range(0,len(trajectory_points)):
               t_p = [trajectory_points[i].pose.position.x, trajectory_points[i].pose.position.y]
               dist += euclidean(np.asmatrix(t_p), np.asmatrix(edge_points[index[i]]))
           return dist/len(trajectory_points)
           
       elif len(trajectory_points) == self.no_points:
           dist = 0
           for i in range(0,self.no_points):
               t_p = [trajectory_points[i].pose.position.x, trajectory_points[i].pose.position.y]
               dist += euclidean(np.asmatrix(t_p), np.asmatrix(edge_points[i]))
           return dist/self.no_points
           
    # For checking if the trajectory belongs to the edge under condsideration
    def valid_path(self, edge_name, trajectory):
       rs = edge_name.split('_',1)
       start_region = self.waypoints_regions[rs[0]]
       end_region = self.waypoints_regions[rs[1]]
       start_point = Point(trajectory[0].pose.position.x, trajectory[0].pose.position.y)
       end_point = Point(trajectory[-1].pose.position.x, trajectory[-1].pose.position.y)
       intersects_s = False 
       intersects_e = False
       
       for region_area in self.regions.values():
           intersects_s = intersects_s or start_point.intersects(region_area)
           
       for region_area in self.regions.values():
           intersects_e = intersects_e or end_point.intersects(region_area) 
           
       if (intersects_s and intersects_e):
           if start_point.intersects(self.regions[start_region]) and end_point.intersects(self.regions[end_region]):
               #print 'Start and End both lie in respective regions'
               return True
           elif start_point.intersects(self.regions[start_region]) and end_point.intersects(self.regions[start_region]):
               #print 'Start and End both lie in same region'
               return True
           elif start_point.intersects(self.regions[end_region]) and end_point.intersects(self.regions[end_region]):
               #print 'Start and End both lie in same region'
               return True
           else:
               return False
               
       elif (intersects_s and not intersects_e):
           #print 'End Point Lying In No Region'; 
           if start_point.intersects(self.regions[start_region]):
               return True
           else:
               return False
               
       elif (intersects_e and not intersects_s):
           #print 'Start Point Lying In No Region'
           if end_point.intersects(self.regions[end_region]):
               return True
           else:
               return False
                          
       else:
           #print 'Both Trajectory points Not In Any Region'                 
           return True
             
           

            
if __name__ == '__main__':
   try:
          rospy.init_node('trial' , anonymous = True)
          s_t = state_transitions(None)
          #soma_config = 'cropped_cleaned_config'
          #query = None
          #s_t.initialize(soma_config, query)
          print 'DONE INITIALISATION'
          print 'Starting Mapping to pathways procedure'
          s_t.calculate_state_transitions()
          print 'REDUCING STATES'
          probs = s_t.get_reduced_states_probs()
          f = open('markov_chain', 'w')
          for p in probs:
             print p, probs[p]
             value = str((p, probs[p]))
             f.write(value)
          print 'Number of non-zero transitions ', len(probs) 
   except rospy.ROSInterruptException:
      pass


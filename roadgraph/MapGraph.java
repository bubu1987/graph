/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;



import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.HashSet;
import java.util.function.Consumer;
import java.util.Comparator;
import java.util.PriorityQueue;

import geography.GeographicPoint;
import geography.RoadSegment;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	private Map<GeographicPoint,ArrayList<GeographicPoint>> adjListsMap;
	/*
	private Map<, String> roadNameMap;
	private Map<, String> roadTypeMap;
	private Map<, double> roadLengthMap;
	*/
	private Map<String,RoadSegment> roadInfoMap; 
//	private Map<GeographicPoint,Double> distanceToNode;
    private int edgeNum;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		adjListsMap = new HashMap<GeographicPoint,ArrayList<GeographicPoint>>();
		roadInfoMap = new HashMap<String,RoadSegment>();  
	    // map to store the distance from start to each node,used for dijkstra
	//	distanceToNode = new HashMap<GeographicPoint,Double>();
        edgeNum = 0;
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return adjListsMap.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		return adjListsMap.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		int numEdges = 0;
		for (GeographicPoint key : adjListsMap.keySet())
		{
			numEdges = numEdges + adjListsMap.get(key).size();
		}
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		adjListsMap.put(location, new ArrayList<GeographicPoint>());
		return adjListsMap.containsKey(location);
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length)  {
					//throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 2
		if (from == null || to == null || roadName == null || roadType == null || 
			length < 0 || !adjListsMap.containsKey(from) || !adjListsMap.containsKey(to))
		{
			throw new IllegalArgumentException(); 
		}
		// check whether the edge has been added, if added, ignore
		if (!adjListsMap.get(from).contains(to))
		{
			(adjListsMap.get(from)).add(to);
			roadInfoMap.put(from.toString()+to.toString(), new RoadSegment(from, to, new ArrayList<GeographicPoint>(), roadName, roadType, length));
			System.out.println(from.toString()+" "+to.toString()+" distance: "+length);
			edgeNum++;
        }
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		if (!adjListsMap.containsKey(start) || !adjListsMap.containsKey(goal))
		{
			System.out.println("start node or goal node is not in map.");
			return null;
		}
		
		if (start.distance(goal)==0)
		{
			System.out.println("start node is same as goal node.");
			return null;
		}
		
		
		// TODO: Implement this method in WEEK 2
		// the shortest path for return
		ArrayList<GeographicPoint> retPath = new ArrayList<GeographicPoint>();
		// record the temporary path during bfs
		ArrayList<GeographicPoint> tempPath = new ArrayList<GeographicPoint>();
		// set to store all visited nodes
		Set<GeographicPoint> visitedNodes = new HashSet<GeographicPoint>();
		// queue to store the nodes temporarily 
		java.util.Queue<GeographicPoint> nodeQueue = new java.util.LinkedList<GeographicPoint>();
		// map to store each visited node path
		Map<GeographicPoint,GeographicPoint> parentMap = new HashMap<GeographicPoint,GeographicPoint>();
		GeographicPoint next = new GeographicPoint(0,0); 
		
		//retPath.add(start);
		nodeQueue.add(start);
		visitedNodes.add(start);
		while (!nodeQueue.isEmpty())
		{
			next = nodeQueue.poll();
			
			tempPath.add(next);
			// Hook for visualization.  See writeup.
			nodeSearched.accept(next.getLocation());
			for (GeographicPoint node : adjListsMap.get(next))
			{
				// if node has not been visited do following; if visited, skip
				if (!visitedNodes.contains(node))
				{	
					if (node.distance(goal)==0)
					{
						parentMap.put(node, next);
						System.out.println("parent :"+next.toString()+" child "+node.toString());
						retPath.add(node);
						GeographicPoint parentNode = parentMap.get(node);
						retPath.add(0,parentNode);	
						while(parentNode.distance(start)!=0) 
						{
							parentNode = parentMap.get(parentNode);
							retPath.add(0,parentNode);	
						}
						return retPath;
					}
					else
					{
						parentMap.put(node, next);
						System.out.println("parent :"+next.toString()+" child "+node.toString());
						nodeQueue.add(node);
						visitedNodes.add(node);
					}	
				}
			}
			tempPath.remove(tempPath.size()-1);
		}
		// if reach here, means no path between start and goal, return null
		System.out.println("There is no route between start node and goal node.");
		return null;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
         System.out.println("edgeNum in this map : " + edgeNum);  
        
        // TODO: Implement this method in WEEK 3
		// check start and goal are inside the map and they are not same
		if (!adjListsMap.containsKey(start) || !adjListsMap.containsKey(goal))
		{
			System.out.println("start node or goal node is not in map.");
			return null;
		}
		
		if (start.distance(goal)==0)
		{
			System.out.println("start node is same as goal node.");
			return null;
		}

		int numNodeVisited = 0;
		// the shortest path for return
		ArrayList<GeographicPoint> retPath = new ArrayList<GeographicPoint>();
		// record the temporary path during bfs
		ArrayList<GeographicPoint> tempPath = new ArrayList<GeographicPoint>();
		// set to store all visited nodes
		Set<GeographicPoint> visitedNodes = new HashSet<GeographicPoint>();
		// map to store the distance from start to each node
		Map<GeographicPoint,Double> distanceToNode = new HashMap<GeographicPoint,Double>();
		// map to store each visited node path
		Map<GeographicPoint,GeographicPoint> parentMap = new HashMap<GeographicPoint,GeographicPoint>();
		
		class dijkstraComparator implements Comparator<GeographicPoint>
		{
			@Override
			public int compare(GeographicPoint x, GeographicPoint y)
			{
				if (distanceToNode.get(x) < distanceToNode.get(y))
				{
					return -1;
				}
				if (distanceToNode.get(x) > distanceToNode.get(y))
				{
					return 1;
				}
				return 0;
			}
		}
		Comparator<GeographicPoint> comparator = new dijkstraComparator();
		PriorityQueue<GeographicPoint> nodePriorityQueue = new PriorityQueue<GeographicPoint>(20, comparator);
		GeographicPoint current = new GeographicPoint(0,0); 
		
		// initialize the distance.
		for (GeographicPoint node : 	adjListsMap.keySet())
		{
			distanceToNode.put(node, Double.POSITIVE_INFINITY);
		}
		
		distanceToNode.put(start,(double) 0);
		nodePriorityQueue.add(start);
		
		while(!nodePriorityQueue.isEmpty())
		{
			current = nodePriorityQueue.poll();
			numNodeVisited++;
			tempPath.add(current);
			if (!visitedNodes.contains(current))
			{
				visitedNodes.add(current);
			
				if (current.distance(goal)==0)
				{
                      System.out.println("Total Distance in Dyjask: "+distanceToNode.get(current)); 
                      retPath.add(current);
					  GeographicPoint parentNode = parentMap.get(current);
					  retPath.add(0,parentNode);	
					  while(parentNode.distance(start)!=0) 
					  {
					     parentNode = parentMap.get(parentNode);
					     retPath.add(0,parentNode);	
					  }
					  System.out.println("Number of node visited in dijkstra: "+numNodeVisited);
					  return retPath;
               }
			   else	
			   {	
				   // go thru all neighbours 
					for (GeographicPoint next : adjListsMap.get(current))
					{
					// Hook for visualization.  See writeup.
					nodeSearched.accept(next.getLocation());
					   RoadSegment tmp = roadInfoMap.get(current.toString()+next.toString());
					   double distanceCurrentNext =tmp.getLength();
					   double distanceNext = distanceToNode.get(current)+distanceCurrentNext;
					   if (distanceNext < distanceToNode.get(next))
					   {   
						   distanceToNode.put(next, distanceNext);
						   parentMap.put(next, current);
						   nodePriorityQueue.add(next);
					   }
				     }
			   }	
			}     
		}
		System.out.println("There is no route between start node and goal node.");
    	System.out.println("Number of node visited in dijkstra: "+numNodeVisited);
		return null;
	}
	
	// wrapper class for roadInfoMap
/*
	public class NodePair 
	{
		private GeographicPoint start, end;
		public NodePair(GeographicPoint x, GeographicPoint y)
		{
			start = x;
			end   = y;
		}
		
		public GeographicPoint getStart()
		{
			return start;
		}
		public GeographicPoint getEnd()
		{
			return end;
		}
		
		public boolean equals(Object o)
		{
			return (this.start.equals(((NodePair) o).getStart()) && this.end.equals(((NodePair) o).getEnd()));
		}
		
	}
	
	public class nodeComparator implements Comparator<GeographicPoint>
	{
		@Override
		public int compare(GeographicPoint x, GeographicPoint y)
		{
			if (distanceToNode.get(x) < distanceToNode.get(y))
			{
				return -1;
			}
			if (distanceToNode.get(x) > distanceToNode.get(y))
			{
				return 1;
			}
			return 0;
		}
	}
*/	


	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		System.out.println("edgeNum in this map : " + edgeNum);  
        // TODO: Implement this method in WEEK 3
		// check start and goal are inside the map and they are not same
		if (!adjListsMap.containsKey(start) || !adjListsMap.containsKey(goal))
		{
			System.out.println("start node or goal node is not in map.");
			return null;
		}
		
		if (start.distance(goal)==0)
		{
			System.out.println("start node is same as goal node.");
			return null;
		}

		int numNodeVisited = 0;
		// the shortest path for return
		ArrayList<GeographicPoint> retPath = new ArrayList<GeographicPoint>();
		// record the temporary path during bfs
		ArrayList<GeographicPoint> tempPath = new ArrayList<GeographicPoint>();
		// set to store all visited nodes
		Set<GeographicPoint> visitedNodes = new HashSet<GeographicPoint>();
        // map to store the shortest distance between start and a node 
        Map<GeographicPoint,Double> distanceStartNode = new HashMap<GeographicPoint,Double>();
		// map to store the distance from start to goal but go through the specific node, the node went thru is the key  (start -> node -> goal)
		Map<GeographicPoint,Double> distanceStartNodeGoal = new HashMap<GeographicPoint,Double>();
		// map to store each visited node path
		Map<GeographicPoint,GeographicPoint> parentMap = new HashMap<GeographicPoint,GeographicPoint>();
      
		class AStarComparator implements Comparator<GeographicPoint>
		{
			@Override
			public int compare(GeographicPoint x, GeographicPoint y)
			{
				if (distanceStartNodeGoal.get(x) < distanceStartNodeGoal.get(y))
				{
					return -1;
				}
				if (distanceStartNodeGoal.get(x) > distanceStartNodeGoal.get(y))
				{
					return 1;
				}
				return 0;
			}
		}
		Comparator<GeographicPoint> comparator = new AStarComparator();
		PriorityQueue<GeographicPoint> nodePriorityQueue = new PriorityQueue<GeographicPoint>(20, comparator);
		GeographicPoint current = new GeographicPoint(0,0); 
		
		// initialize the distance.
		for (GeographicPoint node : adjListsMap.keySet())
		{
			distanceStartNodeGoal.put(node, Double.POSITIVE_INFINITY);
            distanceStartNode.put(node, Double.POSITIVE_INFINITY);
		}
		
		distanceStartNodeGoal.put(start,start.distance(goal));
		nodePriorityQueue.add(start);
		
		while(!nodePriorityQueue.isEmpty())
		{
			current = nodePriorityQueue.poll();
			numNodeVisited++;
			tempPath.add(current);
			if (!visitedNodes.contains(current))
			{
				visitedNodes.add(current);
				// go thru all neighbours 
				
				if (current.distance(goal)==0)
				{
					  System.out.println("Total Distance in AStar: "+distanceStartNodeGoal.get(current));
					  retPath.add(current);
					  GeographicPoint parentNode = parentMap.get(current);
					  retPath.add(0,parentNode);	
					  while(parentNode.distance(start)!=0) 
					  {
					     parentNode = parentMap.get(parentNode);
					     retPath.add(0,parentNode);	
					  }
					  System.out.println("Number of node visited in AStar: "+numNodeVisited);
					  return retPath;
               }
			   else
			   {		
					for (GeographicPoint next : adjListsMap.get(current))
					{
					   // Hook for visualization.  See writeup.
					   nodeSearched.accept(next.getLocation());
					   RoadSegment tmp = roadInfoMap.get(current.toString()+next.toString());
					   // distance between current and next
					   double distanceCurrentNext =tmp.getLength();
					   // distance between start and next thru current
					   double distanceCurrentGoal = current.distance(goal);
					   double distanceNext = distanceStartNodeGoal.get(current)-distanceCurrentGoal+distanceCurrentNext;
					   // distance between start and end thru next
					   double distanceToDest = distanceNext + next.distance(goal);
                       if (distanceToDest < distanceStartNodeGoal.get(next))
					   {   
						  // distanceStartNode.put(next, distanceNext);
                           distanceStartNodeGoal.put(next, distanceToDest);
						   parentMap.put(next, current);
						   nodePriorityQueue.add(next);
					   }
				    }
				}	
			}     
		}
		System.out.println("There is no route between start node and goal node.");
    	System.out.println("Number of node visited in AStar: "+numNodeVisited);
		return null;
	}

	
	

	public static void main(String[] args)
	{
		 MapGraph simpleTestMap = new MapGraph();
			GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
			
			GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
			GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
			
			System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
			List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
			List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
			
			
			MapGraph testMap = new MapGraph();
			GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
			
			// A very simple test using real data
			testStart = new GeographicPoint(32.869423, -117.220917);
			testEnd = new GeographicPoint(32.869255, -117.216927);
			System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
			testroute = testMap.dijkstra(testStart,testEnd);
			testroute2 = testMap.aStarSearch(testStart,testEnd);
			
			
			// A slightly more complex test using real data
			testStart = new GeographicPoint(32.8674388, -117.2190213);
			testEnd = new GeographicPoint(32.8697828, -117.2244506);
			System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
			testroute = testMap.dijkstra(testStart,testEnd);
			testroute2 = testMap.aStarSearch(testStart,testEnd);
	}

	
}

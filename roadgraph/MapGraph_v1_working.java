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
	private ArrayList<RoadSegment> roadInfoList; 
	
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		adjListsMap = new HashMap<GeographicPoint,ArrayList<GeographicPoint>>();
		roadInfoList = new ArrayList<RoadSegment>();  
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
			roadInfoList.add(new RoadSegment(from, to, new ArrayList<GeographicPoint>(), roadName, roadType, length));
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
					if (!visitedNodes.contains(node))
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
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

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
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest2.map", simpleTestMap);
		
//		GeographicPoint testStart = new GeographicPoint(7.0, 3.0);
		GeographicPoint testStart = new GeographicPoint(20.0, 21.0);
		GeographicPoint testEnd = new GeographicPoint(21.0, 20.0);
		
//		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		
		List<GeographicPoint> testroute = simpleTestMap.bfs(testStart,testEnd);
		if (testroute != null)
		{	
			for (int i=0; i<testroute.size(); i++)
			{
				System.out.print(testroute.get(i).toString()+" -> ");
			}
		}
		else
		{
			System.out.println("No route.");
		}
		//System.out.println(java.util.Arrays.toString(testroute.toArray()));
		//List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		//List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		/*
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
		*/
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}

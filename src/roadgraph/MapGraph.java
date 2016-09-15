/**
 * @author anand
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import geography.GeographicPoint;
import util.GraphLoader;

import java.util.*;
import java.util.function.Consumer;

/**
 * @author anand
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//Member Variables
	private int numVertices;
	private int numEdges;
	private List<MapNode> vertices;
	private HashMap<GeographicPoint,MapNode> nodes;

	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
        vertices=new ArrayList<>();
		nodes=new HashMap<>();
		numEdges=0;
		numVertices=0;
	}

	public static void main(String[] args) {
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
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);

		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart, testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart, testEnd);


		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);

		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart, testEnd);
		testroute2 = testMap.aStarSearch(testStart, testEnd);


		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart, testEnd);
		testroute2 = testMap.aStarSearch(testStart, testEnd);


		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);


		List<GeographicPoint> route = theMap.dijkstra(start, end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start, end);


	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		Set<GeographicPoint> pointSet=new HashSet<>();
		pointSet.addAll(nodes.keySet());
		return pointSet;

	}

	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
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
		if (nodes.containsKey(location)) {
			return false;
		}

		MapNode node=new MapNode(location);
		nodes.put(location,node);
		vertices.add(node);
		numVertices++;
		return true;
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
			String roadType, Double length) throws IllegalArgumentException {
		if(!checkNullEdge(from,to,roadName,roadType,length)){
			throw new IllegalArgumentException();
		}

		//Stores edge in a MapEdge object.
		MapEdge edge=new MapEdge(from,to,roadName,roadType,length);
		MapNode start=nodes.get(from);
		start.addEdge(edge);
		numEdges++;
	}

	/*
	*This function checks for null values in Input parameters.
	 */
	private boolean checkNullEdge(GeographicPoint from, GeographicPoint to, String roadName,
							   String roadType, Double length){
		if(from==null || to==null || roadName==null || roadType==null || length==null){
			return false;
		}
		if(length<0 || !nodes.containsKey(from) || !nodes.containsKey(to)){
			return false;
		}
		return true;
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
		MapNode startNode=nodes.get(start);
		MapNode goalNode=nodes.get(goal);
		HashMap<MapNode,MapNode> prevoius=new HashMap<>();
		LinkedList<GeographicPoint> shortestPath=new LinkedList<>();
		Deque<MapNode> queue=new ArrayDeque<>();
		queue.add(startNode);
		while (!queue.isEmpty()) {
			MapNode n=queue.removeFirst();
			nodeSearched.accept(n.getLocation());
			n.setVisited(true);
			if(n==goalNode)
				break;
			for(MapEdge edge:n.getEdges()){
				MapNode mapNode=nodes.get(edge.getTo());
				if (mapNode.isVisited()){
					continue;
				}
				prevoius.put(mapNode,n);
				queue.add(mapNode);
			}
		}
		if (!prevoius.containsKey(goalNode)){
			return null;
		}
		MapNode pathNode=goalNode;
		while (prevoius.containsKey(pathNode)){
			shortestPath.addFirst(pathNode.getLocation());
			pathNode=prevoius.get(pathNode);
		}
		shortestPath.addFirst(start);
		return shortestPath;

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
		int count = 0;
		LinkedList<GeographicPoint> shortestPath = new LinkedList<>();
		HashMap<MapNode, MapNode> previous = new HashMap<>();
		PriorityQueue<CompareNode> queue = new PriorityQueue<>();
		CompareNode current = new CompareNode(nodes.get(start), 0);
		queue.add(current);

		while (!queue.isEmpty() && current.location != nodes.get(goal)) {
			current = queue.poll();
			while (current.location.isVisited())
				current = queue.poll();
			previous.put(current.location, current.previous);
			current.location.setVisited(true);
			nodeSearched.accept(current.location.getLocation());
			for (MapEdge e : current.location.getEdges()) {
				if (nodes.get(e.getTo()).isVisited()) {
					continue;
				}

				queue.add(new CompareNode(nodes.get(e.getTo()), current.distance + e.getLength(), current.location));
				count++;
			}
		}

		// Hook for visualization.  See writeup.


		MapNode pathNode = nodes.get(goal);
		while (pathNode != null) {
			shortestPath.addFirst(pathNode.getLocation());
			pathNode = previous.get(pathNode);
		}
		for (MapNode node : nodes.values()) {
			node.setVisited(false);
		}
		System.out.println(count);
		return shortestPath;
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
		int count = 0;
//		HashMap<GeographicPoint> store=new HashSet<>();
		LinkedList<GeographicPoint> shortestPath = new LinkedList<>();
		HashMap<MapNode, MapNode> previous = new HashMap<>();
		PriorityQueue<CompareNode> queue = new PriorityQueue<>();
		CompareNode current = new CompareNode(nodes.get(start), 0);
		queue.add(current);

		while (!queue.isEmpty() && current.location != nodes.get(goal)) {
			current = queue.poll();
			count++;
			while (current.location.isVisited()) {
				current = queue.poll();
				count++;
			}
			previous.put(current.location, current.previous);
			current.location.setVisited(true);
			nodeSearched.accept(current.location.getLocation());
			for (MapEdge e : current.location.getEdges()) {
				if (nodes.get(e.getTo()).isVisited()) {
					continue;
				}

				queue.add(new CompareNode(nodes.get(e.getTo()), e.getTo().distance(goal) + e.getLength(), current.location));

//				count++;
			}
		}

		// Hook for visualization.  See writeup.


		MapNode pathNode = nodes.get(goal);
		while (pathNode != null) {
			shortestPath.addFirst(pathNode.getLocation());
			pathNode = previous.get(pathNode);
		}
		for (MapNode node : nodes.values()) {
			node.setVisited(false);
		}
		System.out.println(count);
		return shortestPath;
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
	}

	private class CompareNode implements Comparable<CompareNode> {
		public MapNode location;
		public double distance;
		public MapNode previous;

		public CompareNode(MapNode location, double distance) {
			this.location = location;
			this.distance = distance;
		}

		public CompareNode(MapNode location, double distance, MapNode previous) {
			this.location = location;
			this.distance = distance;
			this.previous = previous;
		}


		@Override
		public int compareTo(CompareNode o) {
			return Double.compare(this.distance, o.distance);
		}
	}
	
}

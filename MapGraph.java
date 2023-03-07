
import java.util.*;

/**
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections
 *
 */
public class MapGraph {

	private int numVertices, numEdges;
	private Map<GeographicPoint,MapNode>vertexMap;
	private Map<MapNode,MapNode>parentMap;

	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph()
	{
		// TODO: Implement
		vertexMap = new HashMap<GeographicPoint,MapNode>();
		parentMap = new HashMap<MapNode,MapNode>();
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() //---------done??--------------
	{
		//TODO: Implement this method
		return numVertices;
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()//------------done??---------------
	{
		//TODO: Implement this method

		return vertexMap.keySet();
	}

	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()//----------done??---------
	{
		//TODO: Implement this method
		return numEdges;
	}

	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)//------------done?------------
	{
		// TODO: Implement this method
		if(vertexMap.containsKey(location)){
			return false;
		}
		MapNode lol = new MapNode();
		lol.gp = location;
		lol.edges = new ArrayList<>();
		vertexMap.put(location, lol );

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
						String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method
		//if (vertexMap.get(from) == null || vertexMap.get(to) == null || roadName == null || roadType == null || length < 0 || vertexMap.containsKey(from) || vertexMap.containsKey(to)){
		if (from == null || to == null || roadName == null || roadType == null || length < 0 || !vertexMap.containsKey(from) || !vertexMap.containsKey(to)){
			throw new IllegalArgumentException("what the scallop");  //problems with this if statement right here^^^
		}
		MapEdge edge = new MapEdge(from, to, roadName, roadType, length);
		if(vertexMap.get(from) == null){
			System.out.println("wow");
		}
		vertexMap.get(from).edges.add(edge);
		numEdges++;




	}

	/** Find the path from start to goal using breadth first search
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start,
									 GeographicPoint goal) // ------------- needs work---------(check slides
	{
		// TODO: Implement this method
		if(vertexMap.get(start) == null || vertexMap.get(goal) == null){
			throw new NullPointerException("THIS IS NULL");
		}
		HashSet<MapNode> done = new HashSet<MapNode>(); //visited
		List<GeographicPoint> ram = new ArrayList<>();//our answer
		HashMap<MapNode,MapNode> father = new HashMap<>();//parents
		Queue<MapNode> lol = new LinkedList<MapNode>();//queue
		lol.add(vertexMap.get(start));
		while(!lol.isEmpty()){
			MapNode hold = lol.poll();
			List<MapNode> sol = new ArrayList<MapNode>();
			for(int i = 0; i<hold.edges.size(); i++){ //grabs all the node neighbors
				sol.add(vertexMap.get(hold.edges.get(i).end));
			}

			for(int i  =0; i<sol.size(); i++){
				if(!done.contains(hold.edges)){
					done.add(sol.get(i));
					father.put(hold,sol.get(i));
					lol.add(sol.get(i));
				}
			}

			ram.add(hold.gp);
			if(hold.gp == goal){
				return ram;
			}
		}













		return ram;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start,
										  GeographicPoint goal)//------------needs work2---------
	{
		// TODO: Implement this method in part two
		PriorityQueue<MapNode> pq = new PriorityQueue<>();
		HashSet<MapNode> visited = new HashSet<>();
		HashMap<GeographicPoint, MapNode> parent = new HashMap<>();
		List<GeographicPoint> ram = new ArrayList<>();
		List<Double> infinity = new ArrayList<>();
		MapNode first = new MapNode();//possible change
		first.gp = start;

		pq.add(first);
		while(!pq.isEmpty()){
			MapNode hold =pq.poll();
			List<GeographicPoint> wth= new ArrayList<>();
			for(int i = 0; i<hold.edges.size(); i++){
				wth.add(hold.edges.get(i).end); //grabs neigbors
			}
			if(!visited.contains(hold)){
				visited.add(hold);
			}
			if(first == vertexMap.get(goal)){
				return null; // change to return list
			}
			for(int i = 0; i<wth.size(); i++){
				if(hold.gp.distance(wth.get(i)) >0){
					//compare
				}
			}
		}

		return null;
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
											 GeographicPoint goal)//--------------needs work2-------------
	{
		// TODO: Implement this method in part two

		return null;
	}

	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("simpletest.map", theMap);
		System.out.println("DONE.");
		System.out.println("Num nodes: " + theMap.getNumVertices());
		System.out.println("Num edges: " + theMap.getNumEdges());
		System.out.println(theMap.bfs(new GeographicPoint(1.0,1.0), new GeographicPoint(8,-1)));

		// uncomment for part 2
		//System.out.println(theMap.dijkstra(new GeographicPoint(1.0,1.0), new GeographicPoint(8,-1)));
		//System.out.println(theMap.aStarSearch(new GeographicPoint(1.0,1.0), new GeographicPoint(8,-1)));

	}

}
class MapNode implements Comparable<MapNode>{

	GeographicPoint gp;
	List<MapEdge>edges;
	// add other instance variables/constructors/methods as you deem necessary

	public int compareTo(MapNode other){
		return -1;
	}

}
class MapEdge{
	GeographicPoint start;
	GeographicPoint end;
	String streetName,roadType;
	double distance;
	public MapEdge(GeographicPoint start, GeographicPoint end, String streetName, String roadType, double distance){
		this.start = start;
		this.end = end;
		this.streetName = streetName;
		this.roadType = roadType;
		this.distance = distance;
	}
	public String toString(){
		return "" + start + " " + end;
	}
}



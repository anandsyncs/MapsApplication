package roadgraph;

import geography.GeographicPoint;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by anand on 30/08/16.
 */

public class MapNode {
    //Member Variables
    private List<MapEdge> edges;
    private GeographicPoint point;
    //Keeps track of node's visited status.
    private boolean visited;

    //Constructor to initialize member variables.
    public MapNode(GeographicPoint geographicPoint){
        edges=new ArrayList<>();
        point=geographicPoint;
    }

    public boolean addEdge(MapEdge edge){
        if(edges.contains(edge)){
            return false;
        }
        edges.add(edge);
        return true;
    }

    //Getters and Setters
    public GeographicPoint getLocation(){
        return new GeographicPoint(point.getX(),point.getY());
    }

    public List<MapEdge> getEdges(){
        return new ArrayList<>(edges);
    }

    public boolean isVisited(){
        return visited;
    }

    public void setVisited(boolean b){
        this.visited=b;
    }
}

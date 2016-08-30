package roadgraph;

import geography.GeographicPoint;

/**
 * Created by anand on 30/08/16.
 */

/*
 *This class stores individual edges with proper getters and setters.
 */
public class MapEdge {
    //Member Variables
    private GeographicPoint from;
    private GeographicPoint to;
    private double length;
    private String roadName;
    private String roadType;

    //Constructor to initialize memeber variables.
    public MapEdge(GeographicPoint from,GeographicPoint to, String roadName,String roadType, double length){
        this.from=from;
        this.to=to;
        this.length=length;
        this.roadName=roadName;
        this.roadType=roadType;
    }

    //Getters and Setters
    public GeographicPoint getFrom(){
        return from;
    }

    public GeographicPoint getTo(){
        return to;
    }

    public double getLength(){
        return length;
    }

    public String getRoadName(){
        return roadName;
    }

    public String getRoadType(){
        return  roadType;
    }
}

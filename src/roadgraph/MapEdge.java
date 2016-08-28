package roadgraph;

import geography.GeographicPoint;

/**
 * Created by anand on 30/08/16.
 */
public class MapEdge {
    private GeographicPoint from;
    private GeographicPoint to;
    private double length;
    private String roadName;
    private String roadType;

    public MapEdge(GeographicPoint from,GeographicPoint to, String roadName,String roadType, double length){
        this.from=from;
        this.to=to;
        this.length=length;
        this.roadName=roadName;
        this.roadType=roadType;
    }
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

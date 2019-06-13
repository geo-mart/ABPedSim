package hcu.csl.agentbasedmodeling;

import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.vividsolutions.jts.geom.Geometry;

import de.fhg.ivi.crowdsimulation.simulation.mentalmodel.WayFindingModel;
import de.fhg.ivi.crowdsimulation.simulation.objects.Pedestrian;
import de.fhg.ivi.crowdsimulation.simulation.objects.WayPoint;
import math.geom2d.Vector2D;

/**
 * Class to convert Agent data to {@link Pedestrian}. Methods are often similar to those of the
 * {@link Pedestrian} class.
 *
 * @author Martin Knura
 *
 */
public class PedestrianAgent
{
    /**
     * Uses the object logger for printing specific messages in the console.
     */
    private static final Logger logger = LoggerFactory.getLogger(PedestrianAgent.class);

    /**
     * Identification of this {@link Pedestrian}. Required for {@link Cloneable} interface.
     */
    private int                 id;

    /**
     * Starting x, y position as {@link Vector2D} of the {@link Pedestrian}. This updated, when
     * {@link #setWayPoints(List)} is called.
     */
    private Vector2D            initialPositionVector;

    /**
     * Current approximated x, y position as {@link Vector2D} of the {@link Pedestrian}.
     */
    public Vector2D             currentPositionVector;

    /**
     * {@link List} object, which contains all {@link WayPoint}
     */
    private List<Geometry>      wayPoints;

    /**
     * class name of the {@link WayFindingModel}
     */
    private String              wayFindingModelName;

    /**
     * Creates a new {@link PedestrianAgent} object, from which a {@link Pedestrian} can be built.
     *
     * @param id
     * @param initialPositionX x component of the position of a {@link Pedestrian} at the time of
     *            its creation
     * @param initialPositionY y component of the position of a {@link Pedestrian} at the time of
     *            its creation
     * @param wayPoints a {@link List} of {@link Geometry} objects indicating the {@link Geometry}
     *            to be visited
     * @param wayFindModel a String of the class name of the {@link WayFindingModel}
     */
    public PedestrianAgent(int id, double initialPositionX, double initialPositionY,
        List<Geometry> wayPoints, String wayFindModel)
    {
        this.setKey(id);
        this.initialPositionVector = new Vector2D(initialPositionX, initialPositionY);
        this.setCurrentPosition(new Vector2D(initialPositionX, initialPositionY));
        this.setWayPoints(wayPoints);
        this.setWayFindingModelName(wayFindModel);
    }

    /**
     * Sets the current x, y position as {@link Vector2D} of the {@link Pedestrian}.
     *
     * @param currentPosition denotes the approximated x, y position as {@link Vector2D} of the
     *            {@link PedestrianAgent}
     */

    /**
     * @param currentPosition {@link Vector2D}
     */
    public void setCurrentPosition(Vector2D currentPosition)
    {
        this.currentPositionVector = currentPosition;
    }

    /**
     * @return list of the {@link Geometry} of the pedestrian waypoints
     */
    public List<Geometry> getWayPoints()
    {
        return wayPoints;
    }

    /**
     * @return {@link Vector2D}
     */
    public Vector2D getCurrentPosition()
    {
        return this.currentPositionVector;
    }

    /**
     * @param wayPoints list of the {@link Geometry} of the pedestrian waypoints
     */
    public void setWayPoints(List<Geometry> wayPoints)
    {
        this.wayPoints = wayPoints;
    }

    /**
     * @return String of Mental model classname
     */
    public String getWayFindingModelName()
    {
        return wayFindingModelName;
    }

    /**
     * @param wayFindingModel String of Mental model classname
     */
    public void setWayFindingModelName(String wayFindingModel)
    {
        this.wayFindingModelName = wayFindingModel;
    }

    /**
     * @return key id as int
     */
    public int getKey()
    {
        return id;
    }

    /**
     * @param id key
     */
    public void setKey(int id)
    {
        this.id = id;
    }

}

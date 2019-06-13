package de.fhg.ivi.crowdsimulation.simulation.objects;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.ml.clustering.Clusterable;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.vividsolutions.jts.geom.Geometry;

import de.fhg.ivi.crowdsimulation.simulation.forcemodel.ForceModel;
import de.fhg.ivi.crowdsimulation.simulation.mentalmodel.FollowWayPointsMentalModel;
import de.fhg.ivi.crowdsimulation.simulation.mentalmodel.WayFindingModel;
import math.geom2d.Vector2D;

/**
 * A {@link Pedestrian} is initially represented through his/her initial x- and y-position, initial
 * desired velocity, maximum desired velocity, starting time and a {@link List} of {@link WayPoint},
 * which defines his/her route in dependence to the {@link WayFindingModel}.
 * <p>
 * Farther this class calculates the forces which have an impact on the movement behavior of a
 * {@link Pedestrian}. These forces are:
 *
 * <li>intrinsic forces of each {@link Pedestrian}, see
 * {@link Pedestrian#intrinsicForce(Vector2D, Vector2D, long, ForceModel)}</li>
 * <li>an extrinsic force resulting of the repulsion between {@link Pedestrian} and
 * {@link Boundary}, see {@link Pedestrian#interactBoundaries(Vector2D, List, ForceModel)}</li>
 * <li>an extrinsic force resulting of the repulsion between a {@link Pedestrian} and another
 * {@link Pedestrian}, see {@link Pedestrian#interactPedestrians(Vector2D, List, ForceModel)}</li>
 *
 * <p>
 *
 * @author hahmann/meinert
 */
public class Pedestrian implements Clusterable, Cloneable
{

    /**
     * Uses the object logger for printing specific messages in the console.
     */
    private static final Logger logger = LoggerFactory.getLogger(Pedestrian.class);

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
    private Vector2D            currentPositionVector;

    /**
     * Current velocity as {@link Vector2D} of the {@link Pedestrian}.
     */
    private Vector2D            currentVelocityVector;

    /**
     * The desired velocity that this {@link Pedestrian} initially wants to reach (without being
     * "delayed") given in m/s.
     */
    private float               normalDesiredVelocity;

    /**
     * The maximal desired velocity the {@link Pedestrian} can reach (when being "delayed") given in
     * m/s.
     */
    private float               maximumDesiredVelocity;

    /**
     * Is the resulting force, of all acting forces, which denotes the {@link Pedestrian} movement.
     */
    private Vector2D            totalForce;

    /**
     * Force vector that results from the interaction of this {@link Pedestrian} with all
     * surrounding {@link Pedestrian} objects.
     */
    private Vector2D            forceInteractionWithPedestrians;

    /**
     * Force Vector that results from the interaction of this {@link Pedestrian} with all
     * surrounding {@link Boundary} objects.
     */
    private Vector2D            forceInteractionWithBoundaries;

    /**
     * Sum of all {@link #totalForce} extrinsic forces (currently
     * {@link #forceInteractionWithBoundaries} and {@link #forceInteractionWithPedestrians}) that
     * influence the movement of this {@link Pedestrian}.
     */
    private Vector2D            totalExtrinsicForces;

    /**
     * list of pedestrian positions to export a LineString of pedestrian movement
     *
     * @author Martin Knura
     */
    private List<Vector2D>      trajectory;

    /**
     * Object of the {@link WayFindingModel}
     */
    protected WayFindingModel   wayFindingModel;

    /**
     * Creates a new {@link Pedestrian} object and initializes the variables the {@link Pedestrian}
     * needs to know to realize his/her movement.
     *
     * @param id
     * @param initialPositionX x component of the position of a {@link Pedestrian} at the time of
     *            its creation
     * @param initialPositionY y component of the position of a {@link Pedestrian} at the time of
     *            its creation
     * @param initialDesiredVelocity the desired velocity which a {@link Pedestrian} initially wants
     *            to reach (without being "delayed") given in m/s
     * @param maximumDesiredVelocity the maximal desired velocity a {@link Pedestrian} can reach
     *            (when being "delayed") given in m/s
     * @param startTime the time when a {@link Pedestrian} starts moving towards its destination -
     *            this should be updated in the cases, when this {@link Pedestrian} does not
     *            immediately start moving after it has been created (TODO: currently, the start
     *            time is not updated, when the simulation is started, but it should be, also if new
     *            waypoints are loaded)
     * @param wayPoints a {@link List} of {@link Geometry} objects indicating the {@link WayPoint}
     *            to be visited
     */
    public Pedestrian(int id, double initialPositionX, double initialPositionY,
        float initialDesiredVelocity, float maximumDesiredVelocity, long startTime,
        List<WayPoint> wayPoints)
    {
        this.id = id;
        this.initialPositionVector = new Vector2D(initialPositionX, initialPositionY);
        this.wayFindingModel = new FollowWayPointsMentalModel(wayPoints,
            this.initialPositionVector);
        this.setCurrentPosition(new Vector2D(initialPositionX, initialPositionY));
        this.setNormalDesiredVelocity(initialDesiredVelocity);
        this.setMaximumDesiredVelocity(maximumDesiredVelocity);
        // this.setCurrentVelocity(normalizedDirectionVector.times(initialDesiredVelocity));
        this.setCurrentVelocity(
            wayFindingModel.getNormalizedDirectionVector().times(initialDesiredVelocity));
        wayFindingModel.setStartTime(startTime);
        this.trajectory = new ArrayList<>();
        this.addPositionToTrajectory();
    }

    /**
     * ***SAME AS ABOVE, BUT WITH ADDITIONAL STRING OF WAYFINDING MODEL FOR FURTHER IMPLEMENTATION
     * OF AGENTS***
     *
     * Creates a new {@link Pedestrian} object and initializes the variables the {@link Pedestrian}
     * needs to know to realize his/her movement.
     *
     * @param id
     * @param initialPositionX x component of the position of a {@link Pedestrian} at the time of
     *            its creation
     * @param initialPositionY y component of the position of a {@link Pedestrian} at the time of
     *            its creation
     * @param initialDesiredVelocity the desired velocity which a {@link Pedestrian} initially wants
     *            to reach (without being "delayed") given in m/s
     * @param maximumDesiredVelocity the maximal desired velocity a {@link Pedestrian} can reach
     *            (when being "delayed") given in m/s
     * @param startTime the time when a {@link Pedestrian} starts moving towards its destination -
     *            this should be updated in the cases, when this {@link Pedestrian} does not
     *            immediately start moving after it has been created (TODO: currently, the start
     *            time is not updated, when the simulation is started, but it should be, also if new
     *            waypoints are loaded)
     * @param wayPoints a {@link List} of {@link Geometry} objects indicating the {@link WayPoint}
     *            to be visited
     */
    // Constructor für Übergabe mit WayFindingModel als String
    public Pedestrian(int id, double initialPositionX, double initialPositionY,
        float initialDesiredVelocity, float maximumDesiredVelocity, long startTime,
        List<WayPoint> wayPoints, String wayFindModel)
    {
        this.id = id;
        this.initialPositionVector = new Vector2D(initialPositionX, initialPositionY);
        this.wayFindingModel = new FollowWayPointsMentalModel(wayPoints,
            this.initialPositionVector);
        this.setCurrentPosition(new Vector2D(initialPositionX, initialPositionY));
        this.setNormalDesiredVelocity(initialDesiredVelocity);
        this.setMaximumDesiredVelocity(maximumDesiredVelocity);
        // this.setCurrentVelocity(normalizedDirectionVector.times(initialDesiredVelocity));
        this.setCurrentVelocity(
            wayFindingModel.getNormalizedDirectionVector().times(initialDesiredVelocity));
        wayFindingModel.setStartTime(startTime);
        this.trajectory = new ArrayList<>();
        this.addPositionToTrajectory();
    }

    /**
     * Implementation of {@link Cloneable} interface. Creates new instances of
     * {@link #currentPositionVector}, {@link #currentVelocityVector}.
     *
     * Does not create deep clones of force vectors, {@link #initialPositionVector},
     * {@link #wayFindingModel}
     *
     * @see java.lang.Object#clone()
     */
    @Override
    public Pedestrian clone()
    {
        try
        {
            Pedestrian clone = (Pedestrian) super.clone();
            clone.setCurrentPosition(this.getCurrentPosition().clone());
            clone.setCurrentVelocity(this.getCurrentVelocity().clone());
            clone.setNormalDesiredVelocity(this.getNormalDesiredVelocity());
            clone.setMaximumDesiredVelocity(this.getMaximumDesiredVelocity());
            return clone;
        }
        catch (CloneNotSupportedException e)
        {
            logger.error("Pedestrian.clone(), ", e);
        }
        return null;
    }

    /**
     * Tests if two Pedestrians equal each other using {@link #getId()} method
     *
     * @see java.lang.Object#equals(java.lang.Object)
     */
    @Override
    public boolean equals(Object obj)
    {
        if (obj == null)
        {
            return false;
        }
        if ( !Pedestrian.class.isAssignableFrom(obj.getClass()))
        {
            return false;
        }
        final Pedestrian other = (Pedestrian) obj;
        if ((this.id == 0) ? (other.getId() != 0) : this.id != other.getId())
        {
            return false;
        }
        return true;
    }

    /**
     * Generates a hash value of this {@link Pedestrian}.
     *
     * @return the hash value of this pedestrian
     *
     * @see java.lang.Object#hashCode()
     */
    @Override
    public int hashCode()
    {
        return super.hashCode();
    }

    /**
     * Gets the identification of this {@link Pedestrian}. Required for {@link Cloneable} interface.
     *
     * @return the identification of this {@link Pedestrian}
     */
    public int getId()
    {
        return id;
    }

    /**
     * Gets the {@link #currentPositionVector} as {@link Clusterable}.
     *
     * @see org.apache.commons.math3.ml.clustering.Clusterable#getPoint()
     */
    @Override
    public double[] getPoint()
    {
        return new double[] { currentPositionVector.x(), currentPositionVector.y() };
    }

    /**
     * Gets the {@link #initialPositionVector}.
     *
     * @return the {@link #initialPositionVector}
     */
    public Vector2D getInitialPositionVector()
    {
        return initialPositionVector;
    }

    /**
     * Gets the {@link #currentPositionVector} of a {@link Pedestrian}.
     *
     * @return the {@link #currentPositionVector}
     */
    public Vector2D getCurrentPosition()
    {
        return currentPositionVector;
    }

    /**
     * Sets the current x, y position as {@link Vector2D} of the {@link Pedestrian}.
     *
     * @param currentPosition denotes the approximated x, y position as {@link Vector2D} of the
     *            {@link Pedestrian}
     */
    public void setCurrentPosition(Vector2D currentPosition)
    {
        this.currentPositionVector = currentPosition;
    }

    /**
     * Gets the {@link #currentVelocityVector}.
     *
     * @return the {@link #currentVelocityVector}
     */
    public Vector2D getCurrentVelocity()
    {
        return currentVelocityVector;
    }

    /**
     * Adds current position to trajectory
     */
    public void addPositionToTrajectory()
    {

        if ( !(Double.isNaN(this.getCurrentPosition().x())
            || Double.isNaN(this.getCurrentPosition().y())))
        {
            this.trajectory.add(new Vector2D(currentPositionVector.x(), currentPositionVector.y()));
        }
    }

    /**
     * @return trajectory of the {@link Pedestrian}
     */
    public List<Vector2D> getTrajectory()
    {

        return this.trajectory;
    }

    /**
     * @return WKT-String of trajectory
     *
     * @author Martin Knura
     */
    public String trajectory2WKT()
    {
        String wktTyp = new String("LINESTRING (");
        String wktCoords = new String();
        int indexCoord = 0;
        for (Vector2D p : trajectory)
        {
            if (indexCoord % 10 == 0)
            {
                wktCoords = wktCoords + new String(String.valueOf(p.x()));
                wktCoords = wktCoords + new String(" ");
                wktCoords = wktCoords + new String(String.valueOf( -p.y()));
                wktCoords = wktCoords + new String(",");
            }
            indexCoord++ ;
        }

        wktCoords = wktCoords.substring(0, wktCoords.length() - 1);
        String wktEnd = new String(")");

        return wktTyp + wktCoords + wktEnd;
    }

    /**
     * @return GEOJSON-String of trajectory
     *
     * @author Martin Knura
     */
    public String trajectory2JSON()
    {

        String wktTyp = new String("    { \"type\": \"Feature\",\n" + "        \"geometry\": {\n"
            + "          \"type\": \"LineString\",\n"
            + "          \"coordinates\": [\r\n          ");
        String wktCoords = new String();
        int indexCoord = 0;
        for (Vector2D p : trajectory)
        {
            if (indexCoord % 10 == 0)
            {

                wktCoords = wktCoords + new String("[");
                wktCoords = wktCoords + new String(String.valueOf(p.x()));
                wktCoords = wktCoords + new String(", ");
                wktCoords = wktCoords + new String(String.valueOf( -p.y()));
                wktCoords = wktCoords + new String("]");
                wktCoords = wktCoords + new String(",");
            }
            indexCoord++ ;
        }

        wktCoords = wktCoords.substring(0, wktCoords.length() - 1);

        String wktEnd = new String("\r\n          ]\r\n        }\r\n     }");
        return wktTyp + wktCoords + wktEnd;
    }

    /**
     * @return GEOJSON-String of trajectory in a single line for CityScope transfer
     *
     * @author Martin Knura
     */
    public String trajectory2JSONLine()
    {

        String wktTyp = new String("{\"type\":\"Feature\",\"geometry\":{"
            + "\"type\":\"LineString\"," + "\"coordinates\":[");
        String wktCoords = new String();
        int indexCoord = 0;
        for (Vector2D p : trajectory)
        {
            if (indexCoord % 10 == 0)
            {

                wktCoords = wktCoords + new String("[");
                wktCoords = wktCoords + new String(String.valueOf(p.x()));
                wktCoords = wktCoords + new String(", ");
                wktCoords = wktCoords + new String(String.valueOf( -p.y()));
                wktCoords = wktCoords + new String("]");
                wktCoords = wktCoords + new String(",");
            }
            indexCoord++ ;
        }

        wktCoords = wktCoords.substring(0, wktCoords.length() - 1);

        String wktEnd = new String("]}}");
        return wktTyp + wktCoords + wktEnd;
    }

    /**
     * @return GEOJSON-String of current position
     *
     * @author Martin Knura
     */
    public String currentPosition2JSON()
    {

        String wktTyp = new String("    { \"type\": \"Feature\",\n" + "        \"geometry\": {\n"
            + "          \"type\": \"Point\",\n" + "          \"coordinates\": [");
        String wktCoords = new String(String.valueOf(currentPositionVector.x()));
        wktCoords = wktCoords + new String(", ");
        wktCoords = wktCoords + new String(String.valueOf( -currentPositionVector.y()));
        String wktEnd = new String("]\r\n        }\r\n     }");
        return wktTyp + wktCoords + wktEnd;
    }

    /**
     * @return GEOJSON-String of current position in a single line for CityScope transfer
     *
     * @author Martin Knura
     */
    public String currentPosition2JSONLine()
    {

        String wktTyp = new String("{\"type\":\"Feature\"," + "\"geometry\":{"
            + "\"type\":\"Point\"," + "\"coordinates\": [");
        String wktCoords = new String(String.valueOf(currentPositionVector.x()));
        wktCoords = wktCoords + new String(", ");
        wktCoords = wktCoords + new String(String.valueOf( -currentPositionVector.y()));
        String wktEnd = new String("]}}");
        return wktTyp + wktCoords + wktEnd;
    }

    /**
     * Sets the {@link #currentVelocityVector}, which denotes the approximated velocity of the
     * {@link Pedestrian}.
     *
     * @param currentVelocity denotes the approximated velocity of the {@link Pedestrian}
     */
    public void setCurrentVelocity(Vector2D currentVelocity)
    {
        this.currentVelocityVector = currentVelocity;
    }

    /**
     * Sets the {@link #normalDesiredVelocity} that this {@link Pedestrian} wants to reach (without
     * being "delayed") given in m/s. Cf. Helbing et al (2005) p. 11.
     *
     * @param normalDesiredVelocity the velocity that this {@link Pedestrian} wants to reach
     *            (without being "delayed")
     */
    public void setNormalDesiredVelocity(float normalDesiredVelocity)
    {
        if (normalDesiredVelocity < 0)
            normalDesiredVelocity = 0;
        this.normalDesiredVelocity = normalDesiredVelocity;
    }

    /**
     * Gets the {@link #normalDesiredVelocity} that this {@link Pedestrian} wants to reach (without
     * being "delayed") given in m/s. Cf. Helbing et al (2005) p. 11.
     *
     * @return the velocity that this {@link Pedestrian} wants to reach (without being "delayed")
     */
    public float getNormalDesiredVelocity()
    {
        return this.normalDesiredVelocity;
    }

    /**
     * Gets the {@link #maximumDesiredVelocity}.
     *
     * @return the {@link #maximumDesiredVelocity}
     */
    public float getMaximumDesiredVelocity()
    {
        return maximumDesiredVelocity;
    }

    /**
     * Sets the @link #maximumDesiredVelocity}, i.e. the velocity that this {@link Pedestrian} can
     * reach (in case "delayed") given in m/s.
     *
     * @param maximumDesiredVelocity the maximum velocity the {@link Pedestrian} can reach
     */
    public void setMaximumDesiredVelocity(float maximumDesiredVelocity)
    {
        if (maximumDesiredVelocity < 0)
            maximumDesiredVelocity = 0;
        this.maximumDesiredVelocity = maximumDesiredVelocity;
    }

    /**
     * ***Method not necessary anymore***
     *
     * Sets the {@link List} of {@link WayPoint} objects this Pedestrian should follow.
     *
     * @param wayPoints {@link List} of {@link Geometry} objects that indicate the direction in
     *            which the {@link Pedestrian} goes
     */
    // public void setWayPoints(List<WayPoint> wayPoints)

    /**
     * Gets the current force vector that results from the interaction of this {@link Pedestrian}
     * with all surrounding {@link Pedestrian} objects.
     *
     * Please note: this methods only returns the value of the force, without re-computing it. For
     * re-computing {@link #getForces(Vector2D, Vector2D, long, List, List, ForceModel)} needs to be
     * called.
     *
     * @return the current force vector for Pedestrian-Pedestrian interaction.
     */
    public Vector2D getForceInteractionWithPedestrians()
    {
        return forceInteractionWithPedestrians;
    }

    /**
     * Gets the current force Vector that results from the interaction of this {@link Pedestrian}
     * with all surrounding {@link Boundary} objects
     *
     * Please note: this methods only returns the value of the force, without re-computing it. For
     * re-computing {@link #getForces(Vector2D, Vector2D, long, List, List, ForceModel)} needs to be
     * called.
     *
     * @return the current force Vector for Pedestrian-Boundary interaction.
     */
    public Vector2D getForceInteractionWithBoundaries()
    {
        return forceInteractionWithBoundaries;
    }

    /**
     * Gets the Sum of all extrinsic forces (i.e. {@link #forceInteractionWithBoundaries} and
     * {@link #forceInteractionWithPedestrians}) that influence the movement of this
     * {@link Pedestrian}.
     *
     * Please note: this methods only returns the value of the force, without re-computing it. For
     * re-computing {@link #getForces(Vector2D, Vector2D, long, List, List, ForceModel)} needs to be
     * called.
     *
     * @return the sum of all extrinsic forces from Pedestrian-Pedestrian and Pedestrian-Boundary
     *         interaction.
     */
    public Vector2D getTotalExtrinsicForces()
    {
        return totalExtrinsicForces;
    }

    /**
     * Gets the {@link WayFindingModel} object.
     *
     * @return the {@link WayFindingModel} object2
     */
    public WayFindingModel getMentalModel()
    {
        return wayFindingModel;
    }

    /**
     * Computes and updates acceleration that is needed to reach the desired velocity and go in the
     * desired direction.
     *
     * @param currentPosition denotes the approximated x, y position as {@link Vector2D} of the
     *            {@link Pedestrian}
     * @param currentVelocity the approximated velocity of the {@link Pedestrian}
     * @param currentTime current system time in milliseconds
     * @param forceModel the kind of movement calculation which is denoted by the {@link ForceModel}
     *
     * @return the {@link Vector2D} vector which resulting from the self-driven velocity and
     *         direction of pedestrian
     */
    private Vector2D intrinsicForce(Vector2D currentPosition, Vector2D currentVelocity,
        long currentTime, ForceModel forceModel)
    {
        float averageVelocityOnDesiredRoute = normalDesiredVelocity;

        // at the beginning no average velocity can be computed due to division by zero
        if (currentTime != wayFindingModel.getStartTime())
        {
            averageVelocityOnDesiredRoute = wayFindingModel
                .getAverageVelocityOnRoute(currentPosition, currentTime, false);
        }

        // the actual force computation
        Vector2D resultingForce = forceModel.intrinsicForce(currentPosition, currentVelocity,
            wayFindingModel.getNormalizedDirectionVector(), averageVelocityOnDesiredRoute,
            normalDesiredVelocity, maximumDesiredVelocity);

        return resultingForce;
    }

    /**
     * Computes and returns the force resulting from pedestrian-pedestrian interaction.
     *
     * @param currentPosition denotes the approximated x, y position as {@link Vector2D} of the
     *            {@link Pedestrian}
     * @param pedestrians the {@link List} object of all {@link Pedestrian} which interact with the
     *            current {@link Pedestrian}
     * @param forceModel object of the {@link ForceModel}
     *
     * @return the {@link Vector2D} vector which describes the force resulting of
     *         pedestrian-pedestrian interaction
     */
    private Vector2D interactPedestrians(Vector2D currentPosition, List<Pedestrian> pedestrians,
        ForceModel forceModel)
    {
        Vector2D resultingForce = new Vector2D(0, 0);

        for (Pedestrian pedestrian : pedestrians)
        {
            if ( !this.equals(pedestrian))
            {
                resultingForce = resultingForce.plus(forceModel.interactPedestrian(currentPosition,
                    wayFindingModel.getNormalizedDirectionVector(), pedestrian));

            }
        }
        return resultingForce;
    }

    /**
     * Computes and returns the force resulting from pedestrian-boundary interaction.
     *
     * @param currentPosition denotes the approximated x, y position as {@link Vector2D} of the
     *            {@link Pedestrian}
     * @param boundaries {@link List} that contains all {@link Boundary} objects
     * @param forceModel object of the {@link ForceModel}
     *
     * @return the {@link Vector2D} vector which describes the force resulting of
     *         pedestrian-geometry interaction
     */
    private Vector2D interactBoundaries(Vector2D currentPosition, List<Boundary> boundaries,
        ForceModel forceModel)
    {
        Vector2D resultingForce = new Vector2D(0, 0);

        if (boundaries != null && !boundaries.isEmpty())
        {
            for (Boundary boundary : boundaries)
            {
                // original calculation method
                resultingForce = resultingForce.plus(forceModel.interactBoundary(currentPosition,
                    wayFindingModel.getNormalizedDirectionVector(), boundary));
            }
        }

        return resultingForce;
    }

    /**
     * Computes and adds the resulting forces out of all forces which influence the behavior of the
     * {@link Pedestrian}s. Part of this forces are the own forces to go with a certain velocity and
     * the two repulsive forces in dependency of the {@code boundaries} and other
     * {@link Pedestrian}s.
     *
     * @param currentTime is the current system time stamp
     * @param pedestrians is a {@link List} which contains all {@link Pedestrian}s
     * @param boundaries {@link List} that contains all {@link Boundary} objects
     * @param forceModel object of the {@link ForceModel}
     *
     * @return the resulting force out of all involved forces as an {@link Vector2D}
     */
    public Vector2D getForces(long currentTime, List<Pedestrian> pedestrians,
        List<Boundary> boundaries, ForceModel forceModel)
    {
        return this.getForces(getCurrentPosition(), getCurrentVelocity(), currentTime, pedestrians,
            boundaries, forceModel);
    }

    /**
     * Computes and adds the resulting forces out of all forces which influence the behavior of the
     * {@link Pedestrian}s. Part of this forces are the own forces to go with a certain velocity and
     * the two repulsive forces in dependency of the {@code boundaries} and other
     * {@link Pedestrian}s.
     *
     * @param currentPosition describes the current position of a {@link Pedestrian}
     * @param currentVelocity the current velocity of the {@link Pedestrian}
     * @param currentTime is the current system time stamp
     * @param pedestrians is a {@link List} which contains all {@link Pedestrian}s
     * @param boundaries {@link List} that contains all {@link Boundary} objects
     * @param forceModel object of the {@link ForceModel}
     *
     * @return the resulting force out of all involved forces as an {@link Vector2D}
     */
    public Vector2D getForces(Vector2D currentPosition, Vector2D currentVelocity, long currentTime,
        List<Pedestrian> pedestrians, List<Boundary> boundaries, ForceModel forceModel)
    {
        // "self-interaction" to reach desired velocity
        Vector2D intrinsicForce = intrinsicForce(currentPosition, currentVelocity, currentTime,
            forceModel);

        // interaction with other pedestrians
        forceInteractionWithPedestrians = interactPedestrians(currentPosition, pedestrians,
            forceModel);

        // interaction with boundaries
        forceInteractionWithBoundaries = interactBoundaries(currentPosition, boundaries,
            forceModel);

        // total extrinsic forces
        totalExtrinsicForces = forceInteractionWithPedestrians.plus(forceInteractionWithBoundaries);

        // total acceleration on current pedestrian
        totalForce = intrinsicForce.plus(totalExtrinsicForces);

        return totalForce;
    }

    /**
     * @return {@link WayFindingModel}
     */
    public WayFindingModel getWayFindingModel()
    {
        return wayFindingModel;
    }

    /**
     * @param wayFindingModel {@link WayFindingModel}
     */
    public void setWayFindingModel(WayFindingModel wayFindingModel)
    {
        this.wayFindingModel = wayFindingModel;
    }
}

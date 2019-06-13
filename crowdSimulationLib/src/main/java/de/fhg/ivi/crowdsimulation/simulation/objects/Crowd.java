package de.fhg.ivi.crowdsimulation.simulation.objects;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.atomic.AtomicInteger;

import org.apache.commons.math3.ml.clustering.Cluster;
import org.apache.commons.math3.ml.clustering.Clusterable;
import org.apache.commons.math3.ml.clustering.DBSCANClusterer;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.geotools.graph.path.Path;
import org.geotools.graph.structure.Node;
import org.opensphere.geometry.algorithm.ConcaveHull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.vividsolutions.jts.algorithm.ConvexHull;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.MultiPoint;
import com.vividsolutions.jts.geom.Point;

import de.fhg.ivi.crowdsimulation.simulation.CrowdSimulator;
import de.fhg.ivi.crowdsimulation.simulation.forcemodel.ForceModel;
import de.fhg.ivi.crowdsimulation.simulation.numericintegration.NumericIntegrator;
import de.fhg.ivi.crowdsimulation.simulation.numericintegration.RungeKuttaIntegrator;
import de.fhg.ivi.crowdsimulation.simulation.numericintegration.SemiImplicitEulerIntegrator;
import de.fhg.ivi.crowdsimulation.simulation.numericintegration.SimpleEulerIntegrator;
import de.fhg.ivi.crowdsimulation.simulation.tools.GeometryTools;
import de.fhg.ivi.crowdsimulation.simulation.tools.MathTools;
import hcu.csl.agentbasedmodeling.PedestrianAgent;

/**
 * A {@link Crowd} consists of a {@link List} of {@link Pedestrian} objects.
 * <p>
 * The crowd is initialized with two velocities - {@link Crowd#meanNormalDesiredVelocity} and
 * {@link Crowd#meanMaximumDesiredVelocity}. It is assumed that the normal walking velocity and the
 * maximum walking velocity of all {@link Pedestrian}s are Gaussian distributed.
 * <p>
 * When creating the {@link Crowd}, for each pedestrian a normal and a maximum walking speed is
 * selected from Gaussian distribution defined by the velocities and standard deviation. Moreover a
 * {@link List} of {@link WayPoint}s defines the direction, in which the crowd and hence all
 * {@link Pedestrian}s will walk as well as the applied {@link ForceModel} for pedestrian movement
 * are set in this method.
 * <p>
 * Besides multiple {@link Crowd}s can be created, in order to represent multiple groups of
 * {@link Pedestrian}s with different characteristics.
 * <p>
 * This class also encapsulates all objects / methods that are relevant to compute the outlines
 * (clustered or non-clustered, convex or concave) of the given {@link List} of {@link Pedestrian}
 * objects
 *
 * @author hahmann/meinert
 */
public class Crowd
{
    /**
     * Uses the object logger for printing specific messages in the console.
     */
    private static final Logger logger                                           = LoggerFactory
        .getLogger(Crowd.class);

    /**
     * {@link List} object, which contains all created {@link Pedestrian}
     */
    private List<Pedestrian>    pedestrians;

    /**
     * {@link Geometry} object, which denotes a outline of all {@link Pedestrian}
     */
    private ArrayList<Geometry> crowdOutlines;

    /**
     * Crowd ID
     */
    private String              crowdIdentifyer;

    /**
     * Can be one of the following methods of numerical mathematics to compute {@link Pedestrian}
     * movement - Simple Euler {@link SimpleEulerIntegrator}, Semi Implicit Euler
     * {@link SemiImplicitEulerIntegrator} or Runge Kutta {@link RungeKuttaIntegrator}.
     */
    private NumericIntegrator   numericIntegrator;

    /**
     * {@link ForceModel} objects, which represents the pedestrian modeling approach.
     */
    private ForceModel          forceModel;

    /**
     * {@link List} of {@link Pedestrian}s, which contains a full copy of all {@link Pedestrian}s of
     * all {@link Crowd}s (including the {@link Pedestrian}s of this {@link Crowd}). This must be
     * updated before any call of {@link #moveCrowd(long, double)}
     */
    private List<Pedestrian>    allPedestrians;

    /**
     * {@link Geometry} object, which contains the geometric union of all {@link Boundary} objects
     * of {@link CrowdSimulator}
     */
    private List<Boundary>      boundaries;

    /**
     * {@link Geometry} object, which contains the geometric union of all {@link Boundary} objects
     * of {@link CrowdSimulator}
     */
    private Geometry            unionOfAllBoundaries;

    /**
     * Thread Pool for parallelization of Pedestrian movement computation
     */
    private ExecutorService     threadPool;

    /**
     * Default Average normal velocity, i.e. the average normal walking velocity of a
     * {@link Pedestrian}. This is the velocity that a pedestrian would choose for walking, when not
     * being delayed. The velocity is given in m/s. Cf. Helbing et al (2005) p. 11.
     */
    public static float         defaultMeanNormalDesiredVelocity                 = 1.2f;

    /**
     * Default Standard deviation of {@link Crowd#defaultMeanNormalDesiredVelocity}, i.e. the
     * standard deviation of the average normal walking velocity, which is used to pick a normal
     * walking velocity for each {@link Pedestrian} from the resulting Gaussian distribution.
     * Outliers that would not be within the 95% of all values are avoided. The value is given in
     * m/s. Cf. Helbing et al (2005) p. 11.
     */
    public static float         defaultStandardDeviationOfNormalDesiredVelocity  = 0.3f;

    /**
     * Default average maximum velocity, i.e. the average maximum walking velocity of all
     * pedestrians. This is used to derive the maximum velocity that an individual
     * {@link Pedestrian} is mable to walk, e.g. when being delayed. The velocity is given in m/s.
     * The value is defined in Helbing, Molnar (1995) p. 4284. It is 30% above
     * {@link Crowd#defaultMeanNormalDesiredVelocity}. Unfortunately there is no value for that in
     * Helbing et al (2005).
     */
    public static float         defaultMeanMaximumDesiredVelocity                = 1.3f
        * defaultMeanNormalDesiredVelocity;

    /**
     * Default standard deviation of {@link Crowd#defaultMeanMaximumDesiredVelocity}, i.e. the
     * standard deviation of the maximum walking velocity, which is used to compensate for delays by
     * each {@link Pedestrian} from the resulting Gaussian distribution. Outliers that would not be
     * within the 95% of all values are avoided. The value is given in m/s.
     * <p>
     * This value is chosen arbitrarily. In Helbing, Molnar (1995) as well as in Helbing et al
     * (2005) there is no value defined.
     */
    public static float         defaultStandardDeviationOfMaximumDesiredVelocity = 0.3f;

    /**
     * Default value, if the {@link Pedestrian} objects are clustered into groups using
     * {@link DBSCANClusterer} before the crowd outlines are computed
     */
    public static boolean       defaultIsClusteringCrowdOutlines                 = false;

    /**
     * Default value, if the calculation of the crowd outlines uses a {@link ConvexHull} or a
     * {@link ConcaveHull} algorithm
     */
    public static boolean       defaultIsCrowdOutlineConvex                      = false;

    /**
     * Average normal velocity, i.e. the average normal walking velocity of all {@link Pedestrian}s.
     * From this the velocity that a pedestrian would choose for walking (when not being delayed) is
     * derived. The velocity is given in m/s.
     */
    private float               meanNormalDesiredVelocity;

    /**
     * Standard deviation of {@link Crowd#meanNormalDesiredVelocity}, i.e. the standard deviation of
     * the average normal walking velocity, which is used to pick a normal walking velocity for each
     * {@link Pedestrian} of this Crowd from the resulting Gaussian distribution. Outliers that
     * would not be within the 95% of all values are avoided.
     */
    private float               standardDeviationOfNormalDesiredVelocity;

    /**
     * Average maximum velocity, i.e. the average maximum walking velocity of all pedestrians. From
     * this the maximum velocity that an individual {@link Pedestrian} is able to walk (e.g. when
     * being delayed) is derived. The velocity is given in m/s.
     */
    private float               meanMaximumDesiredVelocity;

    /**
     * Standard deviation of {@link Crowd#meanMaximumDesiredVelocity}, i.e. the standard deviation
     * of the maximum walking velocity, which is used to compensate for delays by each
     * {@link Pedestrian} from the resulting Gaussian distribution. Outliers that would not be
     * within the 95% of all values are avoided. The value is given in m/s.
     */
    private float               standardDeviationOfMaximumDesiredVelocity;

    /**
     * Indicates, if the {@link Pedestrian} objects are clustered into groups using
     * {@link DBSCANClusterer} before the crowd outlines are computed
     */
    private boolean             isClusteringCrowdOutlines;

    /**
     * Indicates, if calculation of the crowd outline use a {@link ConvexHull} or a
     * {@link ConcaveHull} algorithm
     */
    private boolean             isCrowdOutlineConvex;

    /**
     * One out of two parameters of the {@link DBSCANClusterer}.
     * <p>
     * This parameter denotes the maximum radius of the neighborhood, in which a cluster can be
     * build up.
     */
    private static final double epsilon                                          = 10d;

    /**
     * One out of two parameters of the {@link DBSCANClusterer}.
     * <p>
     * MinPts denotes the minimal number of points, which is necessary to build up a cluster.
     */
    private static final int    minPts                                           = 4;

    /**
     * Parameter for the form of the concave hull.
     * <p>
     * For further informations look into Duckham (2008).
     *
     * @see <a href=
     *      "http://geosensor.net/papers/duckham08.PR.pdf">http://geosensor.net/papers/duckham08.PR.pdf</a>
     */
    private double              crowdOutlineThreshold                            = 20;

    /**
     * Indicates, if the {@link #crowdOutlines} should be updated each time
     * {@link #updateCrowdOutline(Geometry)} is called. Usually, it is unnecessary to do so, if the
     * crowd outline is not visible in the Graphical User Interface, since it is currently unused
     * elsewhere.
     */
    private boolean             isUpdatingCrowdOutline                           = true;

    /**
     * {@link RoutingNetwork} for pedestrian wayfinding
     *
     * @author Martin Knura
     */
    private RoutingNetwork      network;

    /**
     * Creates a new {@link Crowd} object
     *
     * @param id the id of this {@link Crowd}
     * @param numericIntegrator object, which represents on of the following classes: Simple Euler
     *            {@link SimpleEulerIntegrator}, Semi Implicit Euler
     *            {@link SemiImplicitEulerIntegrator} or Runge Kutta {@link RungeKuttaIntegrator}
     * @param forceModel the {@link ForceModel} objects, which represents the pedestrian modeling
     *            approach
     *
     * @param boundaries the {@link List} of existing {@link Boundary} objects
     * @param unionOfAllBoundaries the geometric union of all {@link Boundary} objects as a single
     *            {@link Geometry} objects
     * @param threadPool the Thread Pool for parallelization of Pedestrian movement computation
     * @param rn RoutingNetwork for pedestrian wayfinding
     */
    public Crowd(String id, NumericIntegrator numericIntegrator, ForceModel forceModel,
        List<Boundary> boundaries, Geometry unionOfAllBoundaries, ExecutorService threadPool,
        RoutingNetwork rn)
    {
        crowdIdentifyer = id;
        this.numericIntegrator = numericIntegrator;
        this.forceModel = forceModel;
        this.boundaries = boundaries;
        this.unionOfAllBoundaries = unionOfAllBoundaries;
        this.threadPool = threadPool;

        this.meanNormalDesiredVelocity = defaultMeanNormalDesiredVelocity;
        this.standardDeviationOfNormalDesiredVelocity = defaultStandardDeviationOfNormalDesiredVelocity;
        this.meanMaximumDesiredVelocity = defaultMeanMaximumDesiredVelocity;
        this.standardDeviationOfMaximumDesiredVelocity = defaultStandardDeviationOfMaximumDesiredVelocity;
        this.isClusteringCrowdOutlines = defaultIsClusteringCrowdOutlines;
        this.isCrowdOutlineConvex = defaultIsCrowdOutlineConvex;
        this.network = rn;

        pedestrians = new ArrayList<>();
        crowdOutlines = new ArrayList<>();

    }

    /**
     * Gets a {@link List} of all {@link Pedestrian} objects belonging to this {@link Crowd}
     *
     * @return the {@link Pedestrian} object as an {@link List}.
     */
    public List<Pedestrian> getPedestrians()
    {
        // return java.util.Collections.unmodifiableList(pedestrians);
        return getPedestrians(false);
    }

    /**
     * Gets a {@link List} of all {@link Pedestrian} objects belonging to this {@link Crowd}. If
     * {@code clone} parameter is set to {@code true}, a new {@link ArrayList} is returned
     * containing cloned {@link Pedestrian} objects
     *
     * @param clone if {@code true} a new {@link ArrayList} is returned containing cloned
     *            {@link Pedestrian} objects otherwise {@link #pedestrians} is returned directly
     * @return the list of all {@link Pedestrian} objects contained in this Crowd
     */
    public List<Pedestrian> getPedestrians(boolean clone)
    {
        if (clone)
        {
            List<Pedestrian> pedestriansDeepCopy = new ArrayList<>();
            for (Pedestrian pedestrian : pedestrians)
            {
                pedestriansDeepCopy.add(pedestrian.clone());
            }
            return pedestriansDeepCopy;
        }
        return pedestrians;
    }

    /**
     * This create a new list of {@link Pedestrian} objects belonging to this {@link Crowd}.
     * Furthermore the initial velocity parameters of all {@link Pedestrian}s are set.
     *
     * @param pedestrianPositions a list of the start positions of {@link Pedestrian} objects. Each
     *            list element will translate into a new {@link Pedestrian} object
     * @param startTime the time of the creation of the {@link Pedestrian}s of this {@link Crowd}
     */
    // public void setPedestrians(HashMap<Integer, Coordinate> pedestrianPositions, long startTime)

    /**
     * This create a new list of {@link Pedestrian} objects belonging to this {@link Crowd}.
     * Furthermore the initial velocity parameters of all {@link Pedestrian}s are set.
     *
     * All routings between {@link WayPoint}s are processed in this method.
     *
     * @param agents list of {@link PedestrianAgent}s
     * @param startTime the time of the creation of the {@link Pedestrian}s of this {@link Crowd}
     *
     * @author Martin Knura, original method setPedestrians(HashMap<Integer, Coordinate>
     *         pedestrianPositions, long startTime) from Meinert/Hahmann
     */
    public void setPedestriansFromAgents(List<PedestrianAgent> agents, long startTime)
    {
        // removes pedestrians if they are already existing
        if (pedestrians != null && !pedestrians.isEmpty())
            pedestrians.clear();

        RoutingNetwork routing = network;

        for (PedestrianAgent agent : agents)
        {
            // System.out.println(agent.getKey());
            // System.out.println("WP1: " + agent.getWayPoints());
            List<WayPoint> generatedWaypoints = new ArrayList<>();
            for (int i = 0; i < agent.getWayPoints().size(); i++ )
            {
                logger.trace("setWayPoints(), number of waypoints=" + agent.getWayPoints().size());
                if (i == 0)
                {
                    Coordinate pedestrianPosition = new Coordinate(agent.getCurrentPosition().x(),
                        -(agent.getCurrentPosition().y()));
                    // System.out.println("1: " + agent.getWayPoints().get(i).getCoordinate());

                    Point start = GeometryTools.coordinateToPoint(pedestrianPosition);
                    Point target = GeometryTools.coordinateToPoint(
                        new Coordinate(agent.getWayPoints().get(i).getCoordinate().x,
                            -(agent.getWayPoints().get(i).getCoordinate().y)));

                    // System.out.println("start: " + start + " target: " + target);

                    Collection<Point> neighborStart = routing.nearestNeighbor(1, start);
                    Collection<Point> neighborTarget = routing.nearestNeighbor(1, target);

                    // System.out.println("NN: " + neighborStart);

                    Iterator<Point> poiA = neighborStart.iterator();
                    Coordinate startNode = poiA.next().getCoordinate();
                    Node a = routing.getNode(startNode);

                    Iterator<Point> poiB = neighborTarget.iterator();
                    Node b = routing.getNode(poiB.next().getCoordinate());

                    if ( !(a.toString().equalsIgnoreCase(b.toString())))
                    {
                        Path path = routing.calculatePath(a, b);
                        if (path != null)
                        {
                            // System.out.println(
                            // a.toString() + " -> " + b.toString() + " PATH: " + path.toString());

                            List<WayPoint> wayPoints = routing.pathToWayPointList(path, start, i,
                                unionOfAllBoundaries, forceModel);
                            generatedWaypoints.addAll(wayPoints);
                            // System.out.println("waypoints: " + wayPoints);
                        }
                        else
                        {
                            // System.out.println("ELSE X");

                            Point startReal = GeometryTools.coordinateToPoint(new Coordinate(
                                start.getCoordinate().x, -(start.getCoordinate().y)));
                            agent.getWayPoints().set(i, startReal);
                            // System.out.println("WPX: " + agent.getWayPoints());

                            Coordinate targetJava = new Coordinate(start.getCoordinate().x,
                                -(start.getCoordinate().y));
                            Coordinate startNodeJava = new Coordinate(startNode.x, -(startNode.y));

                            WayPoint x = new WayPoint(targetJava, startNodeJava, i,
                                unionOfAllBoundaries, forceModel, false);
                            generatedWaypoints.add(x);

                        }

                    }
                    else
                    {
                        Coordinate targetJava = new Coordinate(target.getCoordinate().x,
                            -(target.getCoordinate().y));
                        Coordinate startNodeJava = new Coordinate(startNode.x, -(startNode.y));

                        WayPoint noPath = new WayPoint(targetJava, startNodeJava, i,
                            unionOfAllBoundaries, forceModel, false);
                        generatedWaypoints.add(noPath);
                    }

                    WayPoint wayPoint = new WayPoint(agent.getWayPoints().get(i).getCoordinate(),
                        target.getCoordinate(), i, unionOfAllBoundaries, forceModel, false);

                    generatedWaypoints.add(wayPoint);

                }
                else
                {

                    Point start = GeometryTools.coordinateToPoint(
                        new Coordinate(agent.getWayPoints().get(i - 1).getCoordinate().x,
                            -(agent.getWayPoints().get(i - 1).getCoordinate().y)));
                    Point target = GeometryTools.coordinateToPoint(
                        new Coordinate(agent.getWayPoints().get(i).getCoordinate().x,
                            -(agent.getWayPoints().get(i).getCoordinate().y)));

                    Collection<Point> neighborStart = routing.nearestNeighbor(1, start);
                    Collection<Point> neighborTarget = routing.nearestNeighbor(1, target);

                    Iterator<Point> poiA = neighborStart.iterator();
                    Coordinate startNode = poiA.next().getCoordinate();
                    Node a = routing.getNode(startNode);

                    Iterator<Point> poiB = neighborTarget.iterator();
                    Node b = routing.getNode(poiB.next().getCoordinate());

                    if ( !(a.toString().equalsIgnoreCase(b.toString())))
                    {
                        Path path = routing.calculatePath(a, b);
                        if (path != null)
                        {

                            // System.out.println(a.toString() + " -> " + b.toString() + " PATH:
                            // " + path.toString());

                            List<WayPoint> wayPoints = routing.pathToWayPointList(path, start, i,
                                unionOfAllBoundaries, forceModel);
                            generatedWaypoints.addAll(wayPoints);
                        }
                        else
                        {
                            // System.out.println("ELSE Y");
                            Point startReal = GeometryTools.coordinateToPoint(new Coordinate(
                                start.getCoordinate().x, -(start.getCoordinate().y)));
                            agent.getWayPoints().set(i, startReal);
                            // System.out.println("WPY: " + agent.getWayPoints());

                            Coordinate targetJava = new Coordinate(start.getCoordinate().x,
                                -(start.getCoordinate().y));
                            Coordinate startNodeJava = new Coordinate(startNode.x, -(startNode.y));

                            WayPoint y = new WayPoint(targetJava, startNodeJava, i,
                                unionOfAllBoundaries, forceModel, false);
                            generatedWaypoints.add(y);
                        }
                    }
                    else
                    {
                        Coordinate targetJava = new Coordinate(target.getCoordinate().x,
                            -(target.getCoordinate().y));
                        Coordinate startNodeJava = new Coordinate(startNode.x, -(startNode.y));

                        WayPoint noPath = new WayPoint(targetJava, startNodeJava, i,
                            unionOfAllBoundaries, forceModel, false);
                        generatedWaypoints.add(noPath);
                    }

                    WayPoint wayPoint = new WayPoint(agent.getWayPoints().get(i).getCoordinate(),
                        target.getCoordinate(), i, unionOfAllBoundaries, forceModel, false);
                    generatedWaypoints.add(wayPoint);

                }
            }

            // System.out.println(agent.getKey() + ": " + agent.getCurrentPosition().x() + " "
            // + agent.getCurrentPosition().y() + " PATH: " + generatedWaypoints);

            Pedestrian pedestrian = new Pedestrian(agent.getKey(), agent.getCurrentPosition().x(),
                agent.getCurrentPosition().y(),
                MathTools.getRandomGaussianValue(meanNormalDesiredVelocity,
                    standardDeviationOfNormalDesiredVelocity),
                MathTools.getRandomGaussianValue(meanMaximumDesiredVelocity,
                    standardDeviationOfMaximumDesiredVelocity),
                startTime, generatedWaypoints);

            pedestrian.getMentalModel().updateNormalizedDirectionVector(
                pedestrian.getCurrentPosition(), startTime, boundaries,
                pedestrian.getNormalDesiredVelocity());
            pedestrians.add(pedestrian);

            logger.trace("setPedestrians(), " + pedestrian.getInitialPositionVector());
            logger.trace("setPedestrians(), " + startTime);
        }

        updateCrowdOutline(unionOfAllBoundaries);
    }

    /**
     * Gets an Array of {@link Geometry} that represent the outlines from clustered points -
     * {@link #crowdOutlines}.
     *
     * @return the {@link ConvexHull} {@link Geometry} of all {@link Pedestrian}
     */
    @SuppressWarnings("unchecked")
    public List<Geometry> getCrowdOutlines()
    {
        if (crowdOutlines == null)
            return null;
        return (ArrayList<Geometry>) crowdOutlines.clone();
    }

    /**
     * Gets the {@link #meanNormalDesiredVelocity}.
     *
     * @return the {@link #meanNormalDesiredVelocity}.
     */
    public float getMeanNormalDesiredVelocity()
    {
        return meanNormalDesiredVelocity;
    }

    /**
     * Sets the mean of the velocities that all {@link Pedestrian} want to reach (without being
     * "delayed") given in m/s. Cf. Helbing et al (2005) p. 11 and subsequently updates the normal
     * desired velocities of all {@link Pedestrian}
     *
     * @param meanNormalDesiredVelocity the mean of the velocities that all {@link Pedestrian} want
     *            to reach (without being "delayed")
     */
    public void setMeanNormalDesiredVelocity(float meanNormalDesiredVelocity)
    {
        this.meanNormalDesiredVelocity = meanNormalDesiredVelocity;

        for (Pedestrian pedestrian : getPedestrians())
        {
            float normalDesiredVelocity = MathTools.getRandomGaussianValue(
                meanNormalDesiredVelocity, standardDeviationOfNormalDesiredVelocity);
            pedestrian.setNormalDesiredVelocity(normalDesiredVelocity);
        }
    }

    /**
     * Gets the {@link #standardDeviationOfNormalDesiredVelocity} based on the
     * {@link #meanNormalDesiredVelocity}.
     *
     * @return the {@link #standardDeviationOfNormalDesiredVelocity}
     */
    public float getStandardDeviationOfNormalDesiredVelocity()
    {
        return standardDeviationOfNormalDesiredVelocity;
    }

    /**
     * Sets the standard deviation of the velocities that all {@link Pedestrian} want to reach
     * (without being "delayed") given in m/s. Cf. Helbing et al (2005) p. 11 and subsequently
     * updates the normal desired velocities of all {@link Pedestrian}
     *
     * @param standardDeviationOfNormalDesiredVelocity the standard deviation of the velocities that
     *            all {@link Pedestrian} want to reach (without being "delayed")
     */
    public void setStandardDeviationOfNormalDesiredVelocity(
        float standardDeviationOfNormalDesiredVelocity)
    {
        this.standardDeviationOfNormalDesiredVelocity = standardDeviationOfNormalDesiredVelocity;
        for (Pedestrian pedestrian : getPedestrians())
        {
            float normalDesiredVelocity = MathTools.getRandomGaussianValue(
                meanNormalDesiredVelocity, standardDeviationOfNormalDesiredVelocity);
            pedestrian.setNormalDesiredVelocity(normalDesiredVelocity);
        }
    }

    /**
     * Gets the {@link #meanMaximumDesiredVelocity}.
     *
     * @return the {@link #meanMaximumDesiredVelocity}
     */
    public float getMeanMaximumDesiredVelocity()
    {
        return meanMaximumDesiredVelocity;
    }

    /**
     * Sets the mean of the velocities that all {@link Pedestrian} can reach (in case "delayed")
     * given in m/s. Cf. Helbing et al (2005) p. 11 and subsequently updates the maximum desired
     * velocities of all {@link Pedestrian}
     *
     * @param meanMaximumDesiredVelocity the mean of the velocities that all {@link Pedestrian} can
     *            reach (in case of being "delayed")
     */
    public void setMeanMaximumDesiredVelocity(float meanMaximumDesiredVelocity)
    {
        this.meanMaximumDesiredVelocity = meanMaximumDesiredVelocity;
        for (Pedestrian pedestrian : getPedestrians())
        {
            float maximumDesiredVelocity = MathTools.getRandomGaussianValue(
                meanMaximumDesiredVelocity, standardDeviationOfMaximumDesiredVelocity);
            pedestrian.setMaximumDesiredVelocity(maximumDesiredVelocity);
        }
    }

    /**
     * Gets the {@link #standardDeviationOfMaximumDesiredVelocity} based on the
     * {@link #meanMaximumDesiredVelocity}.
     *
     * @return the {@link #standardDeviationOfMaximumDesiredVelocity}
     */
    public float getStandardDeviationOfMaximumVelocity()
    {
        return standardDeviationOfMaximumDesiredVelocity;
    }

    /**
     * Sets the standard deviation of the mean of the maximum velocities that all {@link Pedestrian}
     * can reach (in case of being "delayed") given in m/s. Cf. Helbing et al (2005) p. 11 and
     * subsequently updates the maximum velocities of all {@link Pedestrian}
     *
     * @param standardDeviationOfMaximumVelocity the standard deviation of the mean of the maximum
     *            velocities that all {@link Pedestrian} can reach (in case of being "delayed")
     */
    public void setStandardDeviationOfMaximumVelocity(float standardDeviationOfMaximumVelocity)
    {
        this.standardDeviationOfMaximumDesiredVelocity = standardDeviationOfMaximumVelocity;
        for (Pedestrian pedestrian : getPedestrians())
        {
            float maximumDesiredVelocity = MathTools.getRandomGaussianValue(
                meanMaximumDesiredVelocity, standardDeviationOfMaximumDesiredVelocity);
            pedestrian.setMaximumDesiredVelocity(maximumDesiredVelocity);
        }
    }

    /**
     * Tests if the {@link Pedestrian} objects are clustered into groups using
     * {@link DBSCANClusterer} before the crowd outlines are computed.
     *
     * @return {@code true} for clustering = yes, {@code false} for clustering = no
     */
    public boolean isClusteringCrowdOutlines()
    {
        return isClusteringCrowdOutlines;
    }

    /**
     * Sets if the {@link Pedestrian} objects are clustered into groups using
     * {@link DBSCANClusterer} before the crowd outlines are computed. {@code true} for clustering =
     * yes, {@code false} for clustering = no
     *
     * @param isClusteringCrowdOutlines Sets if the {@link Pedestrian} objects are clustered into
     *            groups using {@link DBSCANClusterer} before the crowd outlines are computed.
     *            {@code true} for clustering = yes, {@code false} for clustering = no
     */
    public void setClusteringCrowdOutlines(boolean isClusteringCrowdOutlines)
    {
        this.isClusteringCrowdOutlines = isClusteringCrowdOutlines;
    }

    /**
     * Tests, if calculation of the crowd outline uses a {@link ConvexHull} or a {@link ConcaveHull}
     * algorithm.
     *
     * @return {@code true} if {@link ConvexHull} is used, if {@code false} {@link ConcaveHull} is
     *         used
     */
    public boolean isCrowdOutlineConvex()
    {
        return this.isCrowdOutlineConvex;
    }

    /**
     * Set, if calculation of the crowd outline uses a {@link ConvexHull} or a {@link ConcaveHull}
     * algorithm.
     *
     * @param isCrowdOutlineConvex Indicates, if calculation of the crowd outline use a
     *            {@link ConvexHull} or a {@link ConcaveHull} algorithm. If {@code true} the
     *            {@link ConvexHull} is used, if {@code false} {@link ConcaveHull} is used
     */
    public void setCrowdOutlineConvex(boolean isCrowdOutlineConvex)
    {
        this.isCrowdOutlineConvex = isCrowdOutlineConvex;
    }

    /**
     * Update {@link #crowdOutlines} based on the current position of the {@code pedestrians} minus
     * {@code unionOfAllBoundaries} (if available). Return immediately without any
     * computation/update if {@link #isUpdatingCrowdOutline} is set to {@code false}
     *
     * @param allBoundaries {@link Geometry} object, which contains the geometric union of all
     *            {@link Boundary} objects.
     */
    public void updateCrowdOutline(Geometry allBoundaries)
    {
        // updates already existing pedestrians with the boundaries
        if (pedestrians == null)
            return;

        if ( !isUpdatingCrowdOutline)
            return;

        if (isClusteringCrowdOutlines)
        {
            DBSCANClusterer<Clusterable> dbscan = new DBSCANClusterer<>(epsilon, minPts);
            List<Cluster<Clusterable>> clusters = dbscan
                .cluster(new ArrayList<Clusterable>(pedestrians));
            ArrayList<Geometry> tempCrowdOutlines = new ArrayList<>();
            for (int i = 0; i < clusters.size(); i++ )
            {
                Cluster<Clusterable> cluster = clusters.get(i);
                List<Clusterable> points = cluster.getPoints();
                Coordinate[] coordinates = new Coordinate[points.size()];
                for (int j = 0; j < points.size(); j++ )
                {
                    coordinates[j] = new Coordinate(points.get(j).getPoint()[0],
                        points.get(j).getPoint()[1]);
                }
                MultiPoint multiPointCluster = JTSFactoryFinder.getGeometryFactory()
                    .createMultiPoint(coordinates);

                Geometry crowdOutline = GeometryTools.createOutline(multiPointCluster,
                    allBoundaries, isCrowdOutlineConvex, crowdOutlineThreshold);
                tempCrowdOutlines.add(crowdOutline);
            }
            crowdOutlines = tempCrowdOutlines;
        }
        else
        {
            // gets position of all pedestrians as MultiPoint object
            MultiPoint multiPoint = JTSFactoryFinder.getGeometryFactory()
                .createMultiPoint(GeometryTools.getCoordinatesFromPedestrians(pedestrians));
            Geometry crowdOutline = GeometryTools.createOutline(multiPoint, allBoundaries,
                isCrowdOutlineConvex, crowdOutlineThreshold);
            ArrayList<Geometry> tempCrowdOutlines = new ArrayList<>();
            tempCrowdOutlines.add(crowdOutline);
            crowdOutlines = tempCrowdOutlines;
        }
    }

    /**
     * Toggles, if the {@link #crowdOutlines} should be updated each time
     * {@link #updateCrowdOutline(Geometry)} is called. Usually, it is unnecessary to do so, if the
     * crowd outline is not visible in the Graphical User Interface, since {@link #crowdOutlines}
     * are currently unused elsewhere.
     *
     * @param isUpdatingCrowdOutline
     */
    public void setUpdatingCrowdOutline(boolean isUpdatingCrowdOutline)
    {
        this.isUpdatingCrowdOutline = isUpdatingCrowdOutline;
    }

    /**
     * Gets the {@link #crowdIdentifyer}.
     *
     * @return the {@link #crowdIdentifyer}
     */
    public String getCrowdIdentifyer()
    {
        return crowdIdentifyer;
    }

    /**
     * Gets the {@link #forceModel}
     *
     * @return the {@link #forceModel}
     */
    public ForceModel getForceModel()
    {
        return forceModel;
    }

    /**
     * Sets the {@link #numericIntegrator} to {@code NumericIntegrator}
     *
     * @param numericIntegrator the {@link NumericIntegrator} should be one of
     *            {@link SimpleEulerIntegrator}, {@link SemiImplicitEulerIntegrator},
     *            {@link RungeKuttaIntegrator}
     */
    public void setNumericIntegrator(NumericIntegrator numericIntegrator)
    {
        synchronized (this.numericIntegrator)
        {
            this.numericIntegrator = numericIntegrator;
        }
    }

    /**
     * Sets the {@link List} of {@link Pedestrian}s, which contains a full copy of all
     * {@link Pedestrian}s of all {@link Crowd}s (including the {@link Pedestrian}s of this
     * {@link Crowd}). This must be updated before any call of {@link #moveCrowd(long, double)}
     *
     * @param allPedestrians {@link List} of {@link Pedestrian}, which contains a full copy of all
     *            {@link Pedestrian}s of all {@link Crowd}s
     */
    public void setAllPedestrians(List<Pedestrian> allPedestrians)
    {
        this.allPedestrians = allPedestrians;
    }

    /**
     * Sets {@link #boundaries} object as a {@link List} of {@link Geometry}s.
     *
     * @param boundaries object denotes all {@link Geometry} implements in a {@link List}.
     */
    public void setBoundaries(List<Boundary> boundaries)
    {
        this.boundaries = boundaries;
    }

    /**
     * Computes the {@link Geometry} object, which contains the geometric union of all
     * {@link Boundary} objects of this {@link CrowdSimulator}
     *
     * @param unionOfAllBoundaries object is the {@link GeometryCollection} of all {@link Boundary}
     *            objects
     */
    public void setUnionOfAllBoundaries(Geometry unionOfAllBoundaries)
    {
        this.unionOfAllBoundaries = unionOfAllBoundaries;
    }

    /**
     * Method for GeoJSON file export of the current pedestrian positions
     *
     * @author Martin Knura
     */
    public void printCurrentPeds()
    {
        String newline = System.getProperty("line.separator");

        StringBuilder json = new StringBuilder();

        json.append("{ \"type\": \"FeatureCollection\",\r\n" + "\t\"features\": [");
        json.append(newline);

        String start = "";

        for (Pedestrian pede : this.getPedestrians())
        {
            json.append(start);
            json.append(newline);
            json.append(pede.currentPosition2JSONLine());
            start = ",";

        }

        json.append(newline);
        json.append(" ]\r\n}");
        String result = json.toString();

        System.out.println(result);

        return;

    }

    /**
     * Method for GeoJSON export of the current pedestrian positions in one line via
     * System.out.println
     *
     * @author Martin Knura
     */
    public void printCurrentPedsInLine()
    {

        StringBuilder json = new StringBuilder();

        json.append("{\"type\":\"FeatureCollection\"," + "\"features\":[");

        String start = "";

        for (Pedestrian pede : this.getPedestrians())

        {

            if ( !(Double.isNaN(pede.getCurrentPosition().x())
                || Double.isNaN(pede.getCurrentPosition().y())))
            {
                json.append(start);
                json.append(pede.currentPosition2JSONLine());
                start = ",";

            }

        }

        json.append("]}");
        String result = json.toString();

        System.out.println(result);

        return;

    }

    /**
     * Computes the number of {@link Pedestrian} per m².
     *
     * @return the {@link Double} value of persons / m²
     */
    public double getCrowdDensity()
    {
        double density = 0d;
        List<Geometry> tempCrowdOutlines = getCrowdOutlines();
        if (tempCrowdOutlines != null && pedestrians != null)
        {
            double totalCrowdArea = 0;
            for (Geometry crowdOutline : tempCrowdOutlines)
            {
                if (crowdOutline != null)
                    totalCrowdArea += crowdOutline.getArea();
            }
            if (totalCrowdArea == 0)
                density = -1;
            else
                density = MathTools.round(pedestrians.size() / totalCrowdArea, 2);
        }
        return density;
    }

    /**
     * Gets the size of this crowd, i.e. the number of {@link Pedestrian} objects that belong to
     * this crowd
     *
     * @return the number of {@link Pedestrian} objects belonging to this {@link Crowd}
     */
    public int getSize()
    {
        return pedestrians.size();
    }

    /**
     * Calls one of the {@link SemiImplicitEulerIntegrator} or {@link SimpleEulerIntegrator} or
     * {@link RungeKuttaIntegrator} for calculation of the velocity and position of all
     * {@link Pedestrian} objects of this {@link Crowd} in the next time step.
     *
     * Important: {@link #setAllPedestrians(List)} needs to be called before this method
     *
     * @param time the current time stamp in simulated time (given in milliseconds)
     * @param simulationUpdateInterval the time between this call of the method and the last one
     *            (i.e. the time between 2 consecutive simulation steps given in seconds)
     * 
     *            Addition of @author Martin Knura: Adding current position to trajectory of
     *            {@link Pedestrian}, print currentPedsInLine for CityScope
     */
    public void moveCrowd(long time, double simulationUpdateInterval)
    {

        AtomicInteger movedPedestrians = new AtomicInteger(0);
        for (Pedestrian pedestrian : pedestrians)
        {
            if (threadPool != null)
            {
                // ensure that all threads for moving a single pedestrian are finished before this
                // method is left to remain consistency
                threadPool.submit(new Runnable()
                    {
                        @Override
                        public void run()
                        {
                            pedestrian.getMentalModel().updateNormalizedDirectionVector(
                                pedestrian.getCurrentPosition(), time, boundaries,
                                pedestrian.getNormalDesiredVelocity());
                            numericIntegrator.move(time, simulationUpdateInterval, pedestrian,
                                allPedestrians, boundaries, forceModel);
                            movedPedestrians.getAndIncrement();
                        }
                    });
            }
            else
            {
                pedestrian.getMentalModel().updateNormalizedDirectionVector(
                    pedestrian.getCurrentPosition(), time, boundaries,
                    pedestrian.getNormalDesiredVelocity());
                numericIntegrator.move(time, simulationUpdateInterval, pedestrian, allPedestrians,
                    boundaries, forceModel);
                movedPedestrians.getAndIncrement();
                logger.trace("moveCrowd(), " + pedestrian.getCurrentPosition());
            }
            pedestrian.addPositionToTrajectory();
        }
        logger.trace("moveCrowd(), -----------------------------");

        long waitForThreadsToFinish = System.currentTimeMillis();
        while (movedPedestrians.get() < pedestrians.size())
        {
            try
            {
                // wait 0.1 milliseconds for pedestrian threads to finish
                Thread.sleep(0, 100000);
            }
            catch (InterruptedException e)
            {
                logger.debug("Crowd.movePedestrians(), sleep interrupted", e);
            }
            long threadProcessingTime = System.currentTimeMillis() - waitForThreadsToFinish;
            // inform user, if threads are not finishing within reasonable time (2 seconds)
            if (threadProcessingTime > 10000)
                logger.info("movePedestrians(), threads take too long too finish, totalTime="
                    + threadProcessingTime / 1000 + "s, moved peds=" + movedPedestrians.get()
                    + ", total peds=" + pedestrians.size());
        }
        updateCrowdOutline(unionOfAllBoundaries);
        printCurrentPedsInLine();
    }

    /**
     * Sets {@link #crowdOutlines} and the {@link ArrayList} of all {@link Pedestrian}s to
     * {@code null}
     */
    public void clear()
    {
        crowdOutlines = null;
        // wayPoints.clear();
        pedestrians.clear();
    }

}

package de.fhg.ivi.crowdsimulation.simulation;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;

import org.geotools.geometry.jts.JTSFactoryFinder;
import org.geotools.geometry.jts.ReferencedEnvelope;
import org.opengis.referencing.crs.CoordinateReferenceSystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;

import de.fhg.ivi.crowdsimulation.simulation.forcemodel.ForceModel;
import de.fhg.ivi.crowdsimulation.simulation.forcemodel.HelbingBuznaModel;
import de.fhg.ivi.crowdsimulation.simulation.numericintegration.NumericIntegrator;
import de.fhg.ivi.crowdsimulation.simulation.numericintegration.RungeKuttaIntegrator;
import de.fhg.ivi.crowdsimulation.simulation.numericintegration.SemiImplicitEulerIntegrator;
import de.fhg.ivi.crowdsimulation.simulation.numericintegration.SimpleEulerIntegrator;
import de.fhg.ivi.crowdsimulation.simulation.objects.Boundary;
import de.fhg.ivi.crowdsimulation.simulation.objects.Crowd;
import de.fhg.ivi.crowdsimulation.simulation.objects.Grid;
import de.fhg.ivi.crowdsimulation.simulation.objects.Pedestrian;
import de.fhg.ivi.crowdsimulation.simulation.objects.RoutingNetwork;
import de.fhg.ivi.crowdsimulation.simulation.objects.WayPoint;
import de.fhg.ivi.crowdsimulation.simulation.tools.GeometryTools;
import hcu.csl.agentbasedmodeling.PedestrianAgent;
import math.geom2d.Vector2D;

/**
 * This class serves as container for the simulation, which means that all simulation objects will
 * be managed in this class. This includes the {@link Boundary}, {@link Pedestrian}, {@link Grid},
 * {@link Crowd} and {@link WayPoint} object.
 * <p>
 * Furthermore the default calibration of the used implementation of the Force Model (see
 * {@link ForceModel} and the algorithm of the numerical integration method, which is necessary to
 * dissolve the Force Model, (see {@link NumericIntegrator} is set in this class.
 * <p>
 * Also this class invokes methods for calculating an union of all {@link Boundary}s, the outline
 * around a {@link Crowd}, an {@link Envelope} around all objects and sets a
 * {@link CoordinateReferenceSystem}. Generally all object classes were set or created here.
 * Especially for the {@link WayPoint} object the {@code wayPointVertical} and
 * {@code axisBetweenWayPoints} are calculated.
 * <p>
 * Furthermore there are some validation methods for the crowd simulator. For example the class
 * checks whether all necessary objects are loaded (this means {@link Boundary}s, {@link WayPoint}s
 * and {@link Pedestrian}s) or two specific conditions in case of the relation between
 * {@link Pedestrian}s and {@link WayPoint}s are fulfilled.
 * <p>
 * This class implements {@link Runnable} and thus contains the actual simulation loop.
 *
 * @author hahmann/meinert
 */
public class CrowdSimulator implements Runnable
{

    /**
     * Uses the object logger for printing specific messages in the console.
     */
    private static final Logger       logger                          = LoggerFactory
        .getLogger(CrowdSimulator.class);

    /**
     * Time between 2 consecutive Iteration/Update steps. Given in seconds.
     */
    private double                    simulationUpdateInterval        = 0.0f;

    /**
     * Time between 2 consecutive Iteration/Update steps. Given in nanoseconds.
     */
    // private long simulationUpdateIntervalNanos = 0;

    /**
     * Average time between two consecutive Iteration/Update steps. Given in seconds.
     */
    private double                    averageSimulationUpdateInterval = 0.0f;

    /**
     * Total number of simulation steps calculated in this {@link CrowdSimulator}
     */
    private int                       totalSimulationSteps            = 0;

    /**
     * An storage for the time stamp when the simulation was started. Given in milliseconds.
     */
    private long                      startSimulationTime;

    /**
     * Variable which stores the last time than the simulation was executed. At the start it is set
     * to null. The parameter is given in milliseconds.
     */
    private long                      lastSimulationTime              = 0;

    /**
     * The minimum interval between two consecutive refreshes in milliseconds
     */
    private int                       refreshInterval                 = 1;

    /**
     * The time difference between the {@code startSimulationTime} and the
     * {@code lastSimulationTime}.
     */
    private long                      simulatedTime;

    /**
     * Boolean parameter which denotes whether the thread for the simulation is running or not.
     */
    private boolean                   simulationThreadRunning         = false;

    /**
     * A "Clock" which can speed up (or speed down) the simulation.
     */
    private FastForwardClock          fastForwardClock;

    /**
     * {@link Long} number which denotes how much the simulation is speed up or down
     */
    private int                       fastForwardFactor               = 1;

    /**
     * An {@link Integer} storage for the set {@code fastForwardFactor}
     */
    private int                       savedFastForwardFactor          = 0;

    /**
     * {@code true}, if simulation computations are currently in progress, {@code false} otherwise
     */
    private boolean                   isSimulatingInProgress;

    /**
     * Can be one of the following methods of numerical mathematics to compute {@link Pedestrian}
     * movement - Simple Euler {@link SimpleEulerIntegrator}, Semi Implicit Euler
     * {@link SemiImplicitEulerIntegrator} or Runge Kutta {@link RungeKuttaIntegrator}.
     */
    private NumericIntegrator         numericIntegrator;

    /**
     * {@link ForceModel} objects, which represents the pedestrian modeling approach.
     */
    private ForceModel                forceModel;

    /**
     * {@link List} object, which contains {@link Boundary} objects for pedestrian-boundary
     * interaction
     */
    private List<Boundary>            boundaries;

    /**
     * {@link Geometry} object, which contains the geometric union of all {@link Boundary} objects
     * of {@link CrowdSimulator}
     */
    private Geometry                  unionOfAllBoundaries;

    /**
     * {@link List} object, which contains all {@link WayPoint}
     */
    private List<WayPoint>            wayPoints;

    /**
     * The Bounding Box of the {@link #boundaries} and the {@link #wayPoints} and the initial
     * positions of all {@link Pedestrian}s contained in the {@link #crowds}
     */
    private Envelope                  boundingBox;

    /**
     * {@link List} of {@link Crowd}s, which represents all crowds participating in this simulation
     * and also encapsulate all {@link Pedestrian} objects.
     */
    private List<Crowd>               crowds;

    /**
     * {@link Integer} to create a unique id for every crowd, which has been created.
     */
    private int                       crowdId                         = 0;

    /**
     * {@link Grid} object
     */
    private Grid                      grid;

    /**
     * Object for saving a coordinate reference system of {@link Geometry}s.
     */
    private CoordinateReferenceSystem crs;

    /**
     * Thread Pool for parallelization of Pedestrian movement computation
     */
    private ExecutorService           threadPool;

    /**
     * {@link RoutingNetwork} for pedestrian wayfinding
     * 
     * @author Martin Knura
     */
    private RoutingNetwork            network;

    /**
     * Constructor.
     * <p>
     * Calls {@link #init()}, defines the type of the {@link NumericIntegrator} and of the
     * {@link ForceModel}.
     */
    public CrowdSimulator()
    {
        this(null);
    }

    /**
     * Constructor.
     * <p>
     * Calls {@link #init()}, defines the type of the {@link NumericIntegrator} and of the
     * {@link ForceModel}.
     *
     * @param threadPool allows parallel processing of the movement calculation of the
     *            {@link Pedestrian}. If {@code null} no parallelization is applied
     */
    public CrowdSimulator(ExecutorService threadPool)
    {
        init();
        fastForwardClock = new FastForwardClock();
        numericIntegrator = new SemiImplicitEulerIntegrator();
        forceModel = new HelbingBuznaModel();
        this.threadPool = threadPool;
    }

    /**
     * Gets the time between 2 consecutive simulation steps.
     *
     * @return the simulation interval given in seconds.
     */
    public double getSimulationUpdateInterval()
    {
        return simulationUpdateInterval;
    }

    /**
     * Gets the average time between 2 consecutive simulation steps.
     *
     * @return the average simulation interval given in seconds.
     */
    public double getAverageSimulationUpdateInterval()
    {
        return averageSimulationUpdateInterval;
    }

    /**
     * Gets the time, as unix timestamp, for the start timepoint when the simulation was computed.
     *
     * @return the time in milliseconds
     */
    public long getStartSimulationTime()
    {
        return startSimulationTime;
    }

    /**
     * Sets the time, as unix timestamp, for the start timepoint when the simulation was computed.
     * The time is given in milliseconds.
     *
     * @param startSimulationTime the starting time of the simulation
     */
    public void setStartSimulationTime(long startSimulationTime)
    {
        this.startSimulationTime = startSimulationTime;
    }

    /**
     * Gets the {@link #lastSimulationTime} in milliseconds.
     *
     * @return the {@link #lastSimulationTime} in milliseconds.
     */
    public long getLastSimulationTime()
    {
        return lastSimulationTime;
    }

    /**
     * Sets the time, as unix timestamp, for the last timepoint when the simulation was computed.
     * The time is given in milliseconds.
     *
     * @param lastSimulationTime the last timepoint of the simulation
     */
    public void setLastSimulationTime(long lastSimulationTime)
    {
        this.lastSimulationTime = lastSimulationTime;
    }

    /**
     * Gets the minimum interval between two refreshes in milliseconds.
     *
     * @return the minimum interval between two refreshes in milliseconds
     */
    public int getRefreshInterval()
    {
        return refreshInterval;
    }

    /**
     * Calculates the time the simulation is running, which results of the difference between
     * {@link #lastSimulationTime} and {@link #startSimulationTime}.
     *
     * @return the time the simulation is running in milliseconds
     */
    public long getSimulatedTimeSpan()
    {
        this.simulatedTime = this.lastSimulationTime - this.startSimulationTime;
        return simulatedTime;
    }

    /**
     * Gets {@code true} if simulationThread is running.
     *
     * @return if the thread is running (true) or not (false)
     */
    public boolean isSimulationThreadRunning()
    {
        return simulationThreadRunning;
    }

    /**
     * Sets {@link #simulationThreadRunning} true, if {@code simulationThreadRunning} is
     * {@code true}.
     *
     * @param simulationThreadRunning {@link Boolean} value if the thread is running (true) or not
     *            (false)
     */
    public void setSimulationThreadRunning(boolean simulationThreadRunning)
    {
        this.simulationThreadRunning = simulationThreadRunning;

        logger.trace("setSimulationThreadRunning(), " + simulationThreadRunning);
    }

    /**
     * Gets an object of the {@link FastForwardClock}.
     *
     * @return Object of a "Clock" which can speed up (or speed down) the simulation.
     */
    public FastForwardClock getFastForwardClock()
    {
        return fastForwardClock;
    }

    /**
     * Gets the {@link #fastForwardFactor}.
     *
     * @return denotes the number with which the simulation is speed up or down
     */
    public double getFastForwardFactor()
    {
        return fastForwardFactor;
    }

    /**
     * Sets the factor with which the simulation is speed up or down.
     *
     * @param fastForwardFactor the speed up/down factor as number
     */
    public void setFastForwardFactor(int fastForwardFactor)
    {
        this.fastForwardFactor = fastForwardFactor;
    }

    /**
     * Sets the {@link #fastForwardFactor} to the current value of {@link #savedFastForwardFactor}.
     *
     */
    public void restoreFastForwardFactor()
    {
        fastForwardFactor = savedFastForwardFactor;
    }

    /**
     * Saves the {@link #fastForwardFactor} into the variable {@link #savedFastForwardFactor}.
     */
    public void saveFastForwardFactor()
    {
        savedFastForwardFactor = fastForwardFactor;
    }

    /**
     * Pause the simulation after by setting the {@link #fastForwardFactor} to zero
     *
     */
    public void pauseSimulation()
    {
        if (getFastForwardFactor() > 0)
        {
            saveFastForwardFactor();
            setFastForwardFactor(0);
        }

    }

    /**
     * Resumes the simulation after the simulation was paused by restoring the fastForwardFactor
     * from before the simulation was paused
     */
    public void resumeSimulation()
    {
        if (getFastForwardFactor() == 0)
        {
            restoreFastForwardFactor();
        }
    }

    /**
     * Checks, if rendering is currently in progress.
     *
     * @return {@code true}, if simulation computations are currently in progress, {@code false}
     *         otherwise
     */
    public boolean isSimulatingInProgress()
    {
        return isSimulatingInProgress;
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
            for (Crowd crowd : crowds)
            {
                crowd.setNumericIntegrator(numericIntegrator);
            }
        }
    }

    /**
     * Tests which type of {@link #numericIntegrator} is currently used by the
     * {@link CrowdSimulator}
     *
     * @return the {@link NumericIntegrator}, one of {@link SimpleEulerIntegrator},
     *         {@link SemiImplicitEulerIntegrator}, {@link RungeKuttaIntegrator}
     */
    public NumericIntegrator getNumericIntegrator()
    {
        return this.numericIntegrator;
    }

    /**
     * Gets an object of the {@link ForceModel}.
     *
     * @return an object of the {@link ForceModel}
     */
    public ForceModel getForceModel()
    {
        return forceModel;
    }

    /**
     * Gets the complete {@link List} of {@link Boundary} objects.
     *
     * @return the complete {@link List} of {@link Boundary} objects.
     */
    public List<Boundary> getBoundaries()
    {
        return boundaries;
    }

    /**
     * Gets the {@link Geometry} object, which contains the geometric union of all {@link Boundary}
     * objects of {@link CrowdSimulator}
     *
     * @return the geometric union of all {@link Boundary} objects
     */
    public Geometry getUnionOfAllBoundaries()
    {
        return unionOfAllBoundaries;
    }

    /**
     * Invokes setCollectionOfAllBoundaries() to create {@link GeometryCollection} of all boundaries
     * once.
     * <p>
     * Otherwise the method updateCrowdOutline is invoked if
     * {@link CrowdSimulator#unionOfAllBoundaries} isn't empty.
     * <p>
     * Also the {@link Pedestrian}s are updated with the newly loaded boundaries in form of
     * {@link CrowdSimulator#unionOfAllBoundaries}.
     *
     * @param boundaries object denotes all {@link Geometry} implements in a {@link List}.
     */
    public void setBoundaries(List<Boundary> boundaries)
    {
        this.boundaries = boundaries;
        setAllBoundaries(boundaries);

        // update and intersect crowd outline with new boundaries
        for (Crowd crowd : crowds)
        {
            crowd.updateCrowdOutline(unionOfAllBoundaries);
            crowd.setBoundaries(boundaries);
        }
    }

    /**
     * Computes the {@link Geometry} object, which contains the geometric union of all
     * {@link Boundary} objects of this {@link CrowdSimulator}
     *
     * @param boundaryCollection object is the {@link GeometryCollection} of all {@link Boundary}
     *            objects
     */
    private void setUnionOfAllBoundaries(GeometryCollection boundaryCollection)
    {
        unionOfAllBoundaries = boundaryCollection.union();

        // Sets unionOfAllBoundaries in crowd
        for (Crowd crowd : crowds)
        {
            crowd.setUnionOfAllBoundaries(unionOfAllBoundaries);
        }
    }

    /**
     * Gets the complete {@link List} of {@link WayPoint} objects.
     *
     * @return the complete {@link List} of {@link WayPoint} objects.
     */
    public List<WayPoint> getWayPoints()
    {
        return wayPoints;
    }

    /**
     * Sets all waypoints in the class {@link WayPoint}.
     * <p>
     * For that the vertical of a waypoint and the axis between two waypoints are computed. This,
     * the x and y {@link Coordinate}, the {@link WayPoint#radiusOfPerpendicularLine} and an
     * identification are handed over to the {@link WayPoint}.
     * <p>
     * Also the {@link Pedestrian}s are updated with the newly loaded {@link WayPoint}s.
     *
     * @param wayPointGeometries object as a {@link List} of {@link Coordinate}s which describe
     *            their position.
     */
    // public void setWayPoints(List<Geometry> wayPointGeometries)

    /**
     * Gets a copy of the current bounding box of this {@link CrowdSimulator}. The bounding box is
     * guaranteed to contain all {@link Boundary} objects and {@link WayPoint} objects and before
     * the Simulation is running also all {@link Pedestrian} objects. After the simulation has
     * started it is not guaranteed anymore that all {@link Pedestrian} objects are contained.
     *
     * @return a copy of the current bounding box of this {@link CrowdSimulator}.
     */
    public Envelope getBoundingBox()
    {
        return new Envelope(boundingBox);
    }

    /**
     * Creates a bounding box as outline of the loaded {@link Geometry}, which means
     * {@link Boundary}s and {@link WayPoint}s.
     *
     * @param geometries {@link List} of {@link Geometry} objects
     */
    public void setBoundingBox(List<Geometry> geometries)
    {
        Envelope envelope = new Envelope();

        // sums up the bounding boxes of all geometries
        for (Geometry geometry : geometries)
        {
            envelope.expandToInclude(geometry.getEnvelopeInternal());
        }

        if (this.boundingBox == null)
        {
            this.boundingBox = envelope;
        }
        else
        {
            this.boundingBox.expandToInclude(envelope);
        }
        ReferencedEnvelope gridBounds = new ReferencedEnvelope(boundingBox, crs);
        this.grid = new Grid(gridBounds);
    }

    /**
     * Gets the {@link List} of {@link Crowd}s which includes all {@link Pedestrian}s and therefore
     * their movement. Also this encapsulates all objects / methods that are relevant to compute the
     * outlines (clustered/non-clustered, convex/concave) of the given List of {@link Pedestrian}
     * objects.
     *
     * @return the {@link #crowds} object of this {@link CrowdSimulator}, which represents a
     *         {@link List} of all {@link Crowd}s participating in this simulation.
     */
    public List<Crowd> getCrowds()
    {
        return crowds;
    }

    /**
     * Creates a new {@link Crowd} object with dependent elements of this crowd. This includes the
     * {@link Pedestrian}s which are contained by the {@link Crowd}, the id, the
     * {@link #numericIntegrator}, the {@link #forceModel}, the {@link #threadPool} of this crowd.
     *
     * @param pedestrians pedestrianPositions a {@link HashMap} of {@link Integer} (used as
     *            Pedestrian Ids) and {@link Coordinate}s which describe the starting positions
     *            (x,y) of the {@link Pedestrian}s
     */
    // public void addCrowd(HashMap<Integer, Coordinate> pedestrians)

    /**
     * Creates a new {@link Crowd} object with dependent elements of this crowd. This includes the
     * {@link Pedestrian}s which are contained by the {@link Crowd}, the id, the
     * {@link #numericIntegrator}, the {@link #forceModel}, the {@link #threadPool} of this crowd.
     *
     * @param pedestrians a list of {@link PedestrianAgent}
     * @author Martin Knura, original method addCrowd(HashMap<Integer, Coordinate> pedestrians) by
     *         Meinert/Hahmann
     */
    public void addAgentCrowd(List<PedestrianAgent> pedestrians)
    {
        long startTime = fastForwardClock.currentTimeMillis();

        // generate unique id for crowd
        crowdId++ ;
        String id = String.valueOf(crowdId);

        // create new crowd
        Crowd crowd = new Crowd(id, numericIntegrator, forceModel, boundaries, unionOfAllBoundaries,
            threadPool, network);
        crowd.setPedestriansFromAgents(pedestrians, startTime);
        crowds.add(crowd);
    }

    /**
     * Removes the given {@code crowd} from the list of all crowds in this {@link CrowdSimulator}.
     *
     * @param crowd the crowd to be removed
     */
    public void removeCrowd(Crowd crowd)
    {
        crowds.remove(crowd);
    }

    /**
     * Gets an object of the {@link Grid}.
     *
     * @return an object of the {@link Grid}
     */
    public Grid getGrid()
    {
        return grid;
    }

    /**
     * Gets the {@link #crs}.
     *
     * @return the {@link #crs}
     */
    public CoordinateReferenceSystem getCrs()
    {
        return crs;
    }

    /**
     * Sets the default coordinate reference system in dependence to the coordinate reference
     * systems of the imported data, e.g. {@link Boundary}s or {@link WayPoint}s.
     *
     * @param crs an identifier of a coordinate reference system
     * @throws CrowdSimulatorNotValidException
     */
    public void setCrs(CoordinateReferenceSystem crs) throws CrowdSimulatorNotValidException
    {
        if (crs == null && this.crs != null)
        {

        }
        else if (crs != null && this.crs == null)
        {
            this.crs = crs;
        }
        else if (crs != null && this.crs != null)
        {
            if ( !crs.equals(this.crs))
            {
                throw new CrowdSimulatorNotValidException(
                    "Datasets with different Coordinate Reference Systems are loaded, " + crs
                        + " != " + this.crs);
            }
        }
        this.crs = crs;
    }

    /**
     * @return an Object of {@link RoutingNetwork}
     */
    public RoutingNetwork getNetwork()
    {
        return network;
    }

    /**
     * @param r Object of {@link RoutingNetwork}
     */
    public void setNetwork(RoutingNetwork r)
    {
        this.network = r;
    }

    /**
     * Initializes {@link #crowds}, {@link #boundaries}, {@link #wayPoints} with new empty
     * {@link ArrayList} objects.
     */
    public void init()
    {
        boundaries = new ArrayList<>();
        wayPoints = new ArrayList<>();
        crowds = new ArrayList<>();
    }

    /**
     * Clears {@link ArrayList} objects {@link #boundaries}, {@link #wayPoints}, {@link #crowds}.
     */
    public void clear()
    {
        wayPoints.clear();
        boundaries.clear();
        crowds.clear();
        grid.clear();

        boundingBox = null;
        unionOfAllBoundaries = null;
    }

    /**
     * Converts listOfAllBoundaries into a single {@link Geometry} object and executes the method
     * {@link CrowdSimulator#setUnionOfAllBoundaries(GeometryCollection)}.
     * <p>
     * If {@code boundaries} are loaded after {@link WayPoint} or boundaries have been reloaded in
     * general, it's necessary that wayPoints are created again. Because {@code vertical} or
     * {@code axis} could have changed themselves.
     *
     * @param listOfAllBoundaries is the {@link List} object of all {@code boundaries}
     */
    private void setAllBoundaries(List<Boundary> listOfAllBoundaries)
    {
        GeometryCollection collectionOfAllBoundaries = GeometryTools
            .boundariesToGeometryCollection(listOfAllBoundaries);
        setUnionOfAllBoundaries(collectionOfAllBoundaries);

    }

    /**
     * First the method checks whether all necessary data, i.e. {@link WayPoint}s,
     * {@link Pedestrian}s and {@link CrowdSimulator#boundaries} is loaded.
     * <p>
     * Farther it will checked that the initial position of a {@link Pedestrian}s don't intersect
     * the initial position of any other {@link Pedestrian}, regardless of which {@link Crowd} the
     * {@link Pedestrian} is a part of. Nearly the same is checked in case of the {@link Pedestrian}
     * id. Here the id of a {@link Pedestrian} must be unique over all {@link Pedestrian}s of all
     * {@link Crowd}s.
     * <p>
     * Then this method checks two preconditions for a valid crowd simulator: first one is that the
     * all {@link Pedestrian}s must see the first {@link WayPoint}. The second one includes that
     * every {@link WayPoint} can see the {@link WayPoint} which lies next to it.
     * <p>
     *
     * @param crowdsToCheck the list of {@link Crowd} objects to be validated
     * @param wayPointsToCheck the list of {@link Geometry} objects representing the
     *            {@link WayPoint}s to be validated
     * @param boundariesToCheck the list of {@link Boundary} object to be validated
     *
     * @return {@code true} if all preconditions are fulfilled and all dates are loaded. Otherwise
     *         {@code false}.
     *
     * @throws CrowdSimulatorNotValidException, if one ore more preconditions are not fulfilled
     */
    public boolean validate(List<Crowd> crowdsToCheck, List<Geometry> wayPointsToCheck,
        List<Boundary> boundariesToCheck) throws CrowdSimulatorNotValidException
    {
        // checks whether required data is loaded
        boolean requiredDataLoaded = checkDataLoaded();

        // checks that there are no intersection between initial start positions of all pedestrians
        // in all crowds
        boolean distinctPedestrianPositions = checkDistinctPedestrianPositions(crowdsToCheck);

        // checks that pedestrian id's are unique over all crowds
        boolean uniquePedestrianIds = checkUniquePedestrianIds(crowdsToCheck);

        // checks whether the first precondition is fulfilled
        boolean pedestrianWayPointVisibility = checkPedestrianWayPointVisibility(crowdsToCheck,
            wayPointsToCheck, boundariesToCheck);

        // checks whether the second precondition is fulfilled
        boolean interWayPointVisibility = checkInterWayPointVisibility(wayPointsToCheck,
            GeometryTools.boundariesToGeometryCollection(boundariesToCheck).union());

        logger.debug("validate(), required data=" + requiredDataLoaded
            + ", pedestrian-waypoint-visibility=" + pedestrianWayPointVisibility
            + "inter-waypoint-visibility=" + interWayPointVisibility);

        // TODO: this solution is a little bit inconvenient for users:
        // in the case that both conditions are not fulfilled the user will only be informed
        // about Pedestrian - WayPoint visibility (and only afterwards about
        // Inter-WayPoint-Visibility)

        // if one of the preconditions isn't fulfilled the validation failed
        return requiredDataLoaded & pedestrianWayPointVisibility & interWayPointVisibility
            & distinctPedestrianPositions & uniquePedestrianIds;
    }

    public boolean validateAgents(List<Crowd> crowdsToCheck, List<Boundary> boundariesToCheck)
        throws CrowdSimulatorNotValidException
    {
        // checks whether required data is loaded
        boolean requiredDataLoaded = checkDataLoaded();

        // checks that there are no intersection between initial start positions of all pedestrians
        // in all crowds
        boolean distinctPedestrianPositions = checkDistinctPedestrianPositions(crowdsToCheck);

        // checks that pedestrian id's are unique over all crowds
        boolean uniquePedestrianIds = checkUniquePedestrianIds(crowdsToCheck);

        // checks whether the first precondition is fulfilled
        // boolean pedestrianWayPointVisibility = checkPedestrianWayPointVisibility(crowdsToCheck,
        // wayPointsToCheck, boundariesToCheck);
        boolean pedestrianWayPointVisibility = true;

        // checks whether the second precondition is fulfilled
        // boolean interWayPointVisibility = checkInterWayPointVisibility(wayPointsToCheck,
        // GeometryTools.boundariesToGeometryCollection(boundariesToCheck).union());
        boolean interWayPointVisibility = true;

        logger.debug("validate(), required data=" + requiredDataLoaded
            + ", pedestrian-waypoint-visibility=" + pedestrianWayPointVisibility
            + "inter-waypoint-visibility=" + interWayPointVisibility);

        // TODO: this solution is a little bit inconvenient for users:
        // in the case that both conditions are not fulfilled the user will only be informed
        // about Pedestrian - WayPoint visibility (and only afterwards about
        // Inter-WayPoint-Visibility)

        // if one of the preconditions isn't fulfilled the validation failed
        return requiredDataLoaded & pedestrianWayPointVisibility & interWayPointVisibility
            & distinctPedestrianPositions & uniquePedestrianIds;
    }

    /**
     * Checks whether the all {@link Pedestrian} of all given crowds ({@code crowdsToCheck} have
     * distinct position, i.e. there is no set of two (different) Pedestrians that are exactly at
     * the same position.
     *
     * @param crowdsToCheck {@link List} of {@link Crowd} objects to be checked
     * @return {@code true} if all {@link Pedestrian}s are at distinct positions
     * @throws CrowdSimulatorNotValidException if initial position of pedestrians are intersect
     */
    private boolean checkDistinctPedestrianPositions(List<Crowd> crowdsToCheck)
        throws CrowdSimulatorNotValidException
    {
        // Two loops over crowds to compare every crowd with every crowd
        for (Crowd crowd : crowdsToCheck)
        {
            for (Pedestrian pedestrian : crowd.getPedestrians())
            {
                for (Crowd otherCrowd : crowdsToCheck)
                {
                    for (Pedestrian otherPedestrian : otherCrowd.getPedestrians())
                    {
                        // if the pedestrian is compared with itself coordinate are allowed to be
                        // identical
                        if (pedestrian.equals(otherPedestrian))
                        {
                            continue;
                        }
                        Vector2D pedestrianPosition = pedestrian.getInitialPositionVector();
                        Vector2D otherPedestrianPosition = otherPedestrian
                            .getInitialPositionVector();

                        // TODO think about a radius around the Vector of the pedestrian position,
                        // because the pedestrians should not only be checked to not stand at
                        // exactly at the same position, but also not at nearly the same position.
                        // Using the pedestrian radius could be a good approach.
                        if (pedestrianPosition.equals(otherPedestrianPosition))
                        {
                            throw new CrowdSimulatorNotValidException(
                                "At least 2 pedestrians are at the same initial positions, which is not allowed");
                        }
                    }
                }
            }
        }
        return true;
    }

    /**
     * Checks whether the id the {@link Pedestrian}s of contained in all {@link Crowd}s in the given
     * list of crowds ({@code crowdsToCheck} are unique. If there is at least one pair of
     * pedestrians to have a common id an exception is thrown.
     *
     * @param crowdsToCheck {@link List} of all {@link Crowd} object to be checked
     * @return {@code true} if all {@link Pedestrian}s ids are unique
     * @throws CrowdSimulatorNotValidException if pedestrian ids are non-unique within all crowds
     */
    private boolean checkUniquePedestrianIds(List<Crowd> crowdsToCheck)
        throws CrowdSimulatorNotValidException
    {
        // Two loops over crowds to compare every crowd with every crowd
        for (Crowd crowd : crowdsToCheck)
        {
            for (Pedestrian pedestrian : crowd.getPedestrians())
            {
                int numberOfPedestriansWithSameId = 0;
                for (Crowd otherCrowd : crowdsToCheck)
                {
                    for (Pedestrian otherPedestrian : otherCrowd.getPedestrians())
                    {
                        if (pedestrian.getId() == otherPedestrian.getId())
                            numberOfPedestriansWithSameId++ ;
                    }
                }
                if (numberOfPedestriansWithSameId > 1)
                {
                    throw new CrowdSimulatorNotValidException(
                        "There are at least 2 pedestrians that have the same id. ids of pedestrians have to be unique.");
                }
            }
        }
        return true;
    }

    /**
     * Checks if all data is loaded. This includes data for the {@link WayPoint}s, the
     * {@link Pedestrian}s and the {@link CrowdSimulator#boundaries}.
     *
     * @return true if all data is loaded. Else {@link CrowdSimulatorNotValidException} is thrown.
     *
     * @throws CrowdSimulatorNotValidException if not all data aren't loaded
     *
     *             CHANGED by @author Martin Knura due to new {@link WayPoint} loading
     */
    private boolean checkDataLoaded() throws CrowdSimulatorNotValidException
    {
        if (boundaries == null || boundaries.size() == 0)
        {
            throw new CrowdSimulatorNotValidException("No Boundaries are loaded.");
        }
        if (crowds == null || crowds.isEmpty())
            throw new CrowdSimulatorNotValidException("No Pedestrians are loaded.");
        for (Crowd crowd : crowds)
        {
            if (crowd.getPedestrians() == null || crowd.getPedestrians().isEmpty())
            {
                throw new CrowdSimulatorNotValidException("At least one Crowd has no Pedestrians.");
            }
        }
        // if (wayPoints == null || wayPoints.size() == 0)
        // {
        // throw new CrowdSimulatorNotValidException("No WayPoints are loaded.");
        // }

        return true;
    }

    /**
     * Checks if all {@link Pedestrian}s can see the first {@link WayPoint}.
     *
     * @param crowdsToCheck the list of {@link Crowd} objects to be validated
     * @param wayPointsToCheck the list of {@link Geometry} objects representing the
     *            {@link WayPoint}s to be validated
     * @param boundariesToCheck the list of {@link Boundary} object to be validated
     *
     * @return {@code true} if every {@link Pedestrian} can see the first {@link WayPoint}. If one
     *         or more can't see it, the return value will be {@code false}.
     *
     * @throws CrowdSimulatorNotValidException, if the precondition is not fulfilled
     */
    private boolean checkPedestrianWayPointVisibility(List<Crowd> crowdsToCheck,
        List<Geometry> wayPointsToCheck, List<Boundary> boundariesToCheck)
        throws CrowdSimulatorNotValidException
    {
        for (Crowd crowd : crowdsToCheck)
        {
            if (wayPointsToCheck != null && boundariesToCheck != null && crowd != null
                && crowd.getPedestrians() != null)
            {
                Coordinate wayPointPosition = wayPointsToCheck.get(0).getCoordinate();

                for (Pedestrian pedestrian : crowd.getPedestrians())
                {
                    Coordinate pedestrianPosition = new Coordinate(
                        pedestrian.getCurrentPosition().x(), pedestrian.getCurrentPosition().y());
                    Coordinate[] coordinates = new Coordinate[] { pedestrianPosition,
                        wayPointPosition };
                    LineString lineString = JTSFactoryFinder.getGeometryFactory()
                        .createLineString(coordinates);
                    boolean visible = !lineString.intersects(unionOfAllBoundaries);
                    if ( !visible)
                    {
                        throw new CrowdSimulatorNotValidException(
                            "Not all Pedestrians see the first WayPoint, i.e. there is at least one Pedestrian, which has a boundary object on the direct line of sight between itself and the first WayPoint. The input data needs to be changed so that this condition is fulfilled.");
                    }
                }
            }
        }

        return true;
    }

    /**
     * Checks if every {@link WayPoint} can see the {@link WayPoint} which lying next to it.
     *
     * @param wayPointsToCheck the list of {@link Geometry} objects representing the
     *            {@link WayPoint}s to be validated
     * @param boundariesToCheck the list of {@link Boundary} object to be validated
     *
     * @return {@code true} if every {@link WayPoint} can see the {@link WayPoint} which lying next
     *         to it. If one or more can't see their neighbor, the return value will be
     *         {@code false}.
     *
     * @throws CrowdSimulatorNotValidException, if the precondition is not fulfilled
     */
    private boolean checkInterWayPointVisibility(List<Geometry> wayPointsToCheck,
        Geometry boundariesToCheck) throws CrowdSimulatorNotValidException
    {
        if (wayPointsToCheck != null && boundariesToCheck != null)
        {
            GeometryFactory factory = JTSFactoryFinder.getGeometryFactory();

            for (int i = 0; i < (wayPointsToCheck.size() - 1); i++ )
            {
                Coordinate firstPoint = wayPointsToCheck.get(i).getCoordinate();
                Coordinate secondPoint = wayPointsToCheck.get(i + 1).getCoordinate();
                Coordinate[] coordinates = new Coordinate[] { firstPoint, secondPoint };

                LineString lineString = factory.createLineString(coordinates);

                boolean visible = !lineString.intersects(boundariesToCheck);
                if ( !visible)
                {
                    throw new CrowdSimulatorNotValidException(
                        "Not every waypoint can see its neighboring waypoint, i.e. there is at least one pair of consecutive WayPoints that have a boundary element on the direct line of sight between them. The input data needs to be changed so that this condition is fulfilled.");
                }
            }
        }

        return true;
    }

    /**
     * Implements run() of the interface {@link Runnable}. The Thread can process In fact that means
     * that it's possible to send instructions to the thread (if the thread is running).
     *
     * @see java.lang.Runnable#run()
     */
    @Override
    public void run()
    {
        startSimulationTime = fastForwardClock.currentTimeMillis(fastForwardFactor);
        // startSimulationTime = fastForwardClock.nanoTime(fastForwardFactor);

        while (simulationThreadRunning)
        {
            logger.trace("CrowdSimulator.run(), fastForwardFactor=" + fastForwardFactor);

            long currentTime = fastForwardClock.currentTimeMillis(fastForwardFactor);
            // long currentTime = fastForwardClock.nanoTime(fastForwardFactor);

            // if (simulationUpdateIntervalNanos < refreshInterval)
            // {
            // try
            // {
            // long sleepInterval = refreshInterval - simulationUpdateIntervalNanos;
            // long sleepIntervalMs = TimeUnit.NANOSECONDS.toMillis(sleepInterval);
            // long sleepIntervalremainingNanos = (sleepInterval) % 1000000;
            // Thread.sleep(sleepIntervalMs, (int) sleepIntervalremainingNanos);
            // }
            // catch (InterruptedException e)
            // {
            // logger.trace("MapPanel.run(), sleep interrupted", e);
            // }
            // }
            // don't refresh more than 1 time per second
            if (currentTime - lastSimulationTime < refreshInterval)
                continue;
            isSimulatingInProgress = true;
            moveCrowds(currentTime);
            isSimulatingInProgress = false;
            lastSimulationTime = currentTime;
        }
    }

    /**
     * Moves all {@link Pedestrian} objects of the {@link Crowd} belonging to this
     * {@link CrowdSimulator}. Updating the {@link Grid} with new aggregate values of
     * {@link Pedestrian} object count per cell is invoked as well.
     *
     * @param currentTime the current timestamp (unix timestamp) in simulation time in milliseconds
     */
    public void moveCrowds(long currentTime)
    {
        if (crowds != null && !crowds.isEmpty())
        {
            // collect all pedestrians from all crowds and make copies of them before any
            // crowd/pedestrian is moved
            List<Pedestrian> allPedestrians = new ArrayList<>();
            for (Crowd crowd : crowds)
            {
                allPedestrians.addAll(crowd.getPedestrians(true));
            }

            // iterate over all crowds to move all crowds
            for (Crowd crowd : crowds)
            {
                if (lastSimulationTime == 0)
                {
                    return;
                }
                // this.simulationUpdateIntervalNanos = currentTime - lastSimulationTime;
                // this.simulationUpdateInterval = simulationUpdateIntervalNanos / 1_000_000_000d;
                this.simulationUpdateInterval = (currentTime - lastSimulationTime) / 1000d;
                this.totalSimulationSteps++ ;
                this.averageSimulationUpdateInterval = (averageSimulationUpdateInterval
                    * (totalSimulationSteps - 1) + this.simulationUpdateInterval)
                    / totalSimulationSteps;

                crowd.setAllPedestrians(allPedestrians);
                crowd.moveCrowd(currentTime, simulationUpdateInterval);
                logger.trace("moveCrowd(), " + currentTime);
                logger.trace("moveCrowd(), " + crowd.getPedestrians().size());
                logger.trace("moveCrowd(), " + forceModel);
                logger.trace("moveCrowd(), " + numericIntegrator);
                logger.trace("moveCrowd(), " + simulationUpdateInterval);
                logger.trace("moveCrowd(), " + crowd.getPedestrians(true).size());
                logger.trace("moveCrowd(), " + boundaries.size());
                logger.trace("moveCrowd(), " + unionOfAllBoundaries);
                logger.trace("moveCrowd(), " + threadPool.isShutdown());

                grid.update(crowd.getPedestrians(), currentTime);

                logger.trace("CrowdSimulator.movePedestrians(), ");
            }
        }
    }
}

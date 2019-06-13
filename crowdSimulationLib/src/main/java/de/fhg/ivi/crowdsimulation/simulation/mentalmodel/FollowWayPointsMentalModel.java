package de.fhg.ivi.crowdsimulation.simulation.mentalmodel;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.geotools.geometry.jts.JTSFactoryFinder;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LineSegment;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.MultiLineString;

import de.fhg.ivi.crowdsimulation.simulation.numericintegration.NumericIntegrationTools;
import de.fhg.ivi.crowdsimulation.simulation.objects.Boundary;
import de.fhg.ivi.crowdsimulation.simulation.objects.Pedestrian;
import de.fhg.ivi.crowdsimulation.simulation.objects.WayPoint;
import de.fhg.ivi.crowdsimulation.simulation.tools.GeometryTools;
import de.fhg.ivi.crowdsimulation.simulation.tools.MathTools;
import math.geom2d.Vector2D;

/**
 * This class is necessary for the way finding algorithm of each {@link Pedestrian}. Note that this
 * class inherits from the {@link WayFindingModel}, which is the super class of the way finding
 * processing.
 * <p>
 * This class knows the position and sequence of the {@link WayPoint}s, which define the route of
 * every {@link Pedestrian}s.
 * <p>
 * Furthermore here exists some validations, which make sure that the way finding processing works
 * well, especially if a {@link Pedestrian} can't see his next {@link WayPoint}, caused through the
 * concealment of {@link Boundary}s. In such case there are methods which proof if there is a view
 * axis from a {@link Pedestrian} to his/her next {@link WayPoint} and methods to move the
 * {@link Pedestrian} back to a position where the {@link WayPoint} is visible. The last thing can
 * be solved through the movement to the axis between the {@link WayPoint}, which is an integral
 * part of the {@link WayPoint}s, or through the movement to the last known point, where a
 * {@link Pedestrian} will directly see his/her next {@link WayPoint}.
 *
 * TODO maybe introduce the {@link Pedestrian}, to which this {@link WayFindingModel} belongs to as
 * a class variable here so that the {@link WayFindingModel} can access the
 * {@link Pedestrian#getCurrentPosition()} method
 *
 * @author hahmann/meinert
 */
public class FollowWayPointsMentalModel extends WayFindingModel
{

    /**
     * Uses the object logger for printing specific messages in the console.
     */
    private static final Logger logger                              = LoggerFactory
        .getLogger(FollowWayPointsMentalModel.class);

    /**
     * Distance, given in meter, which the pedestrian went between the
     * {@code currentDestinationWayPoint} and the last {@code visitedWayPoint}.
     */
    private float               currentDistanceOnRoute;

    /**
     * {@link List} of {@link Geometry} objects that indicate the direction in which the
     * {@link Pedestrian} goes
     */
    private List<WayPoint>      wayPoints;

    /**
     * {@link List} of {@link Coordinate} which have already been visited by the {@link Pedestrian}
     */
    private List<Coordinate>    visitedWayPoints;

    /**
     * The current destination in form of a {@link WayPoint} of this {@link Pedestrian}.
     */
    private WayPoint            currentDestinationWayPoint;

    /**
     * Describes the calculated nearest Coordinate on a {@link WayPoint#getWayPointVertical()} from
     * the point of view of a {@link Pedestrian}.
     */
    private Vector2D            targetCoordinateVector;

    /**
     * Describes the resolution value in which the {@link WayPoint#getWayPointVertical()} is
     * investigated in the search for an alternative target point
     */
    public static final float   searchResolutionAlternativePosition = WayPoint.radiusOfPerpendicularLine
        / 20.0f;

    /**
     * Defines the last time a {@link Pedestrian} has got a new orientation. Given in milliseconds.
     */
    private long                lastOrientationUpdateTime           = 0;

    /**
     * Defines the last position a {@link Pedestrian} has got a new orientation.
     */
    private Vector2D            lastOrientationUpdatePostion        = null;

    /**
     * Defines the last time a {@link Pedestrian} has got a new course. Given in milliseconds.
     */
    private long                lastCourseUpdateTime                = Long.MIN_VALUE;

    /**
     * Defines the last position a {@link Pedestrian} has got a new course.
     */
    private Vector2D            lastCourseUpdatePostion             = null;

    /**
     * Defines the maximum distance threshold, after which a {@link Pedestrian} checks for a new
     * orientation/course at the latest. Given in meters.
     */
    private final static float  orientationCourseUpdateDistance     = 5f;

    /**
     * Defines the time interval threshold, after which a {@link Pedestrian} checks for a new
     * orientation/course at the latest. Given in milliseconds.
     */
    private final static int    orientationCourseUpdateInterval     = 5000;

    /**
     * Maximum allowed course deviation. If the difference between the actual course and the
     * required course is above this threshold,
     * {@link #updateNormalizedDirectionVector(Vector2D, long, List, float)} should be called.
     *
     * Hint: 1 degree = 0.0175 radians, 5 degree = 0.0873 radians
     */
    public static final float   maximumAllowedCourseDeviation       = 0.0175f;

    /**
     * Starting x, y position as {@link Vector2D} of the {@link Pedestrian}. This updated, when
     * {@link #setWayPoints(List)} is called.
     */
    private Vector2D            initialPositionVector;

    /**
     * {@code true} if this {@link Pedestrian} has passed its {@link #currentDestinationWayPoint}
     * during its last move
     */
    private boolean             hasPassedWayPoint                   = false;

    /**
     * Constructor. Initializes an {@link ArrayList} and a {@link Vector2D}. Set class variables.
     *
     * @param wayPoints a {@link List} of all {@link WayPoint}s
     * @param initialPosition the position of a {@link Pedestrian} at his/her time of creation
     */
    public FollowWayPointsMentalModel(List<WayPoint> wayPoints, Vector2D initialPosition)
    {
        this.normalizedDirectionVector = new Vector2D(0, 0);
        this.visitedWayPoints = new ArrayList<>();
        // to avoid NullPointerExceptions, which could happen if the pedestrians are loaded before
        // the wayPoints
        if (wayPoints != null && !wayPoints.isEmpty())
        {
            this.currentDestinationWayPoint = wayPoints.get(0);
        }
        this.wayPoints = wayPoints;
        this.initialPositionVector = initialPosition;
    }

    /**
     * Sets the {@link #initialPositionVector} of this Mental Model
     *
     * @param initialPositionVector the initial position vector
     */
    public void setInitialPositionVector(Vector2D initialPositionVector)
    {
        this.initialPositionVector = initialPositionVector;
    }

    /**
     * Sets the {@code Pedestrian#wayPoints}.
     *
     * @param wayPoints {@link List} of {@link Geometry} objects that indicate the direction in
     *            which the {@link Pedestrian} goes
     */
    public void setWayPoints(List<WayPoint> wayPoints)
    {
        // this computes a new normlizedDirectionVector if wayPoints are loaded
        needsOrientation = true;

        // clears the list of visited wayPoints if a new list of waypoints has been loaded
        visitedWayPoints.clear();

        this.wayPoints = wayPoints;
        if (wayPoints != null && !wayPoints.isEmpty())
            currentDestinationWayPoint = wayPoints.get(0);
    }

    /**
     * Gets he current destination in form of a {@link WayPoint} of this {@link Pedestrian}.
     *
     * @return he current destination in form of a {@link WayPoint} of this {@link Pedestrian}.
     */
    public WayPoint getCurrentDestinationWayPoint()
    {
        return currentDestinationWayPoint;
    }

    /**
     * Gets the {@link #targetCoordinateVector}.
     *
     * @return the {@link #targetCoordinateVector}
     */
    public Vector2D getTargetCoordinateVector()
    {
        return targetCoordinateVector;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Vector2D getNormalizedDirectionVector()
    {
        return this.normalizedDirectionVector;
    }

    /**
     * Computes and updates the total distance which a {@link Pedestrian} has moved, since leaving
     * it's initial position.
     *
     * @param currentPositionVector the current position of a {@link Pedestrian}
     *
     * @return the {@link Float} distance the pedestrian is moved since his initial position
     */
    private float getTotalDistanceOnRoute(Vector2D currentPositionVector)
    {
        Coordinate currentPositionCoordinate = new Coordinate(currentPositionVector.x(),
            currentPositionVector.y());
        Coordinate lastVisitedWayCoordinate;
        if (visitedWayPoints.size() == 0)
        {
            lastVisitedWayCoordinate = new Coordinate(initialPositionVector.x(),
                initialPositionVector.y());
        }
        else
        {
            lastVisitedWayCoordinate = visitedWayPoints.get(visitedWayPoints.size() - 1);
        }

        float tempTotalDistanceOnRoute = 0;

        // sum of all already visited line segments
        if (visitedWayPoints.size() > 0)
        {
            for (int i = 0; i < visitedWayPoints.size(); i++ )
            {
                LineSegment lineSegment;
                if (i == 0)
                {
                    lineSegment = new LineSegment(
                        new Coordinate(initialPositionVector.x(), initialPositionVector.y()),
                        visitedWayPoints.get(i));
                }
                else
                {
                    lineSegment = new LineSegment(visitedWayPoints.get(i - 1),
                        visitedWayPoints.get(i));
                }
                // tempTotalDistanceOnRoute += lineSegment.getLength();
                tempTotalDistanceOnRoute += MathTools.distance(lineSegment.p0, lineSegment.p1);

            }
        }

        // part of the current segment
        if (currentDestinationWayPoint != null)
        {
            LineSegment lineSegment = new LineSegment(lastVisitedWayCoordinate,
                currentDestinationWayPoint);
            Coordinate currentPositionOnLineSegment = lineSegment
                .project(currentPositionCoordinate);

            double distanceOnCurrentSegment = MathTools.distance(currentPositionOnLineSegment,
                lastVisitedWayCoordinate);

            if (lineSegment.projectionFactor(currentPositionCoordinate) < 0)
            {
                distanceOnCurrentSegment = -distanceOnCurrentSegment;
            }
            tempTotalDistanceOnRoute += distanceOnCurrentSegment;

            currentDistanceOnRoute = (float) distanceOnCurrentSegment;
        }

        totalDistanceOnRoute = tempTotalDistanceOnRoute;

        return totalDistanceOnRoute;
    }

    /**
     * Checks, if the {@link Pedestrian} this {@link WayFindingModel} belongs to has passed its
     * {@link #currentDestinationWayPoint}. If so, the {@link #currentDestinationWayPoint} is added
     * to the {@link List} of {@link #visitedWayPoints}} and the next {@link WayPoint} is set as
     * {@link #currentDestinationWayPoint}. After that the method
     * {@link #computeNormalizedDirectionVector(Vector2D, long, List)} is invoked.
     * <p>
     * In case of the {@link Pedestrian} not having passed its {@link #currentDestinationWayPoint},
     * but one of the two re-orientation parameters {@link #needsOrientation} or
     * {@link #hasCourseDeviation} is {@code true} the
     * {@link #computeNormalizedDirectionVector(Vector2D, long, List)} is also invoked.
     *
     * @see de.fhg.ivi.crowdsimulation.simulation.mentalmodel.WayFindingModel#updateNormalizedDirectionVector(math.geom2d.Vector2D,
     *      long, java.util.List, float)
     */
    @Override
    public void updateNormalizedDirectionVector(Vector2D position, long time,
        List<Boundary> boundaries, float normalDesiredVelocity)
    {
        if (currentDestinationWayPoint == null || wayPoints == null || wayPoints.isEmpty())
        {
            return;
        }

        // updates direction if a waypoint has been reached
        if (hasPassedWayPoint)
            checkNextWayPoint(position);

        // check from time to time if the pedestrian get suspiciously slow
        if (getAverageVelocityOnRoute(position, time, true) < normalDesiredVelocity / 10
            && !isTargetVisible(position, currentDestinationWayPoint, boundaries)
            && !needsOrientation)
        {
            needsOrientation = true;
        }

        // updates the direction, in which the Pedestrian wants to go, if needed
        if (hasPassedWayPoint || hasCourseDeviation || needsOrientation)
            computeNormalizedDirectionVector(position, time, boundaries);

        if (hasPassedWayPoint)
        {
            hasPassedWayPoint = false;
        }
        // course should now be as required again
        if (hasCourseDeviation)
        {
            hasCourseDeviation = false;
        }
        return;
    }

    /**
     * Adds the {@link #currentDestinationWayPoint} to the list of visited WayPoints and checks, if
     * there is a next unvisited {@link WayPoint}. In this case the next WayPoint is set as the
     * {@link #currentDestinationWayPoint}
     */
    private void checkNextWayPoint(Vector2D position)
    {
        // TODO check if either axis or vertical of next waypoint would be visible and break, if not

        // add current destination way point to list of already visited waypoints
        visitedWayPoints.add(currentDestinationWayPoint);

        // this happens, if the pedestrian has reached the last waypoint
        if (visitedWayPoints.size() == wayPoints.size())
        {
            currentDestinationWayPoint = null;
        }
        else
        {
            currentDestinationWayPoint = getNextWayPoint();
        }
    }

    /**
     * Set whether this {@link Pedestrian} has passed its {@link #currentDestinationWayPoint} during
     * its last move
     *
     * @param hasPassedWayPoint {@code true} if this {@link Pedestrian} has passed its
     *            {@link #currentDestinationWayPoint} during its last move
     */
    @Override
    public void setHasPassedWayPoint(boolean hasPassedWayPoint)
    {
        this.hasPassedWayPoint = hasPassedWayPoint;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void checkWayPointPassing(Pedestrian pedestrian, Vector2D currentPosition,
        Vector2D updatedPosition)
    {
        // check if the current WayPoint has been passed
        if (currentDestinationWayPoint != null)
        {
            // before the first WayPoint no axisBetweenWayPoints exist
            if (currentDestinationWayPoint.getAxisBetweenWayPoints() == null)
            {
                setHasPassedWayPoint(NumericIntegrationTools.moveIntersectsGeometry(
                    currentDestinationWayPoint.getBoundingBox(),
                    currentDestinationWayPoint.getExtendedWayPointVertical(), currentPosition,
                    updatedPosition));
            }
            // else only check if a wayPoint has passed, if the pedestrian could be in the range of
            // a WayPoint
            else if (currentDestinationWayPoint.getAxisBetweenWayPoints() != null)
            {
                if (currentDistanceOnRoute > (currentDestinationWayPoint.getAxisLength()
                    - (2 * WayPoint.getRadiusOfPerpendicularLine())))
                {
                    setHasPassedWayPoint(NumericIntegrationTools.moveIntersectsGeometry(
                        currentDestinationWayPoint.getBoundingBox(),
                        currentDestinationWayPoint.getExtendedWayPointVertical(), currentPosition,
                        updatedPosition));
                }
            }
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void checkCourse(Pedestrian pedestrian, long timestamp)
    {
        // only check current course after a certain time/distance
        boolean needsCourseUpdate = false;
        try
        {
            needsCourseUpdate = timestamp - lastCourseUpdateTime > orientationCourseUpdateInterval
                || Math.abs(lastCourseUpdatePostion.x()
                    - pedestrian.getCurrentPosition().x()) > orientationCourseUpdateDistance
                || Math.abs(lastCourseUpdatePostion.y()
                    - pedestrian.getCurrentPosition().y()) > orientationCourseUpdateDistance;
        }
        // can happen, if lastCourseUpdatePostion is null (this is the case, if no waypoints are
        // set)
        catch (NullPointerException e)
        {
            needsCourseUpdate = false;
        }

        if (needsCourseUpdate)
        {
            // if there is no normalized direction vector (e.g. when Pedestrian is lost or when
            // simulation has finished), no angle can be computed
            if (normalizedDirectionVector != null
                && !MathTools.isZeroVector(normalizedDirectionVector)
                && targetCoordinateVector != null)
            {
                Vector2D requiredDirectionVector = targetCoordinateVector
                    .minus(pedestrian.getCurrentPosition());
                if ( !MathTools.isZeroVector(requiredDirectionVector))
                {
                    double actualCourse = MathTools.angle(normalizedDirectionVector);
                    double requiredCourse = MathTools.angle(requiredDirectionVector);

                    // check of the difference between the actual course and the required course to
                    // reach the current targetCoordinate is above threshold
                    if (Math.abs(actualCourse - requiredCourse) > maximumAllowedCourseDeviation)
                    {
                        hasCourseDeviation = true;
                        logger.trace("FollowWayPointsMentalModel.checkCourse(), deltaCourse="
                            + (actualCourse - requiredCourse));
                    }
                }
            }
        }
    }

    /**
     * Computes and updates the {@link Vector2D}, which is the normalized (i.e. length=1) target
     * vector of the {@link Pedestrian}, i.e. this vector indicates, in which direction this
     * Pedestrian currently wants to go. Usually this vector will point in the direction of the
     * {@link #currentDestinationWayPoint}.
     *
     * @param position for which the normalized direction vector should be updated
     * @param timestamp time when this method is called
     * @param boundaries {@link List} that contains all {@link Boundary} objects
     */
    private void computeNormalizedDirectionVector(Vector2D position, long timestamp,
        List<Boundary> boundaries)
    {
        // if current destination waypoint is null set target vector to (0, 0)
        if (currentDestinationWayPoint == null)
        {
            targetCoordinateVector = null;
            normalizedDirectionVector = new Vector2D(0, 0);
            needsOrientation = false;
            return;
        }

        Coordinate tempTargetCoordinate = null;
        // check, if the Pedestrian can still see its current target and re-use it
        if ((hasCourseDeviation || needsOrientation) && !hasPassedWayPoint)
        {
            // avoid null pointer exceptions at the beginning
            if (targetCoordinateVector != null)
            {
                Coordinate currentTargetCoordinate = new Coordinate(targetCoordinateVector.x(),
                    targetCoordinateVector.y());

                // always re-use the current target if it not a certain distance has been
                // passed since the last check
                if (Math
                    .abs(lastOrientationUpdatePostion.x()
                        - position.x()) < orientationCourseUpdateDistance
                    && Math.abs(lastOrientationUpdatePostion.y()
                        - position.y()) < orientationCourseUpdateDistance
                    && timestamp - lastOrientationUpdateTime < orientationCourseUpdateInterval)
                {
                    tempTargetCoordinate = currentTargetCoordinate;
                }
                // after a certain time or distance, check again, if target is still visible
                else if (isTargetVisible(position, currentTargetCoordinate, boundaries))
                {
                    tempTargetCoordinate = currentTargetCoordinate;
                }
            }
        }

        // if the existing target point could not be re-used (since it is not seen anymore or it has
        // been passed) or if the the pedestrian needs orientation and a certain distance / certain
        // time has been passed since the last check (whichever condition is fulfilled first)
        if (tempTargetCoordinate == null || (needsOrientation
            && (timestamp - lastOrientationUpdateTime > orientationCourseUpdateInterval
                || Math.abs(lastOrientationUpdatePostion.x()
                    - position.x()) > orientationCourseUpdateDistance
                || Math.abs(lastOrientationUpdatePostion.y()
                    - position.y()) > orientationCourseUpdateDistance)))
        {
            Coordinate currentPositionCoordinate = new Coordinate(position.x(), position.y());

            // System.out.println("current: " + currentDestinationWayPoint);

            // get nearest Coordinate on WayPointVertical from pedestrian position
            Coordinate nearestCoordinateOnWayPointVertical = GeometryTools.getNearestPoint(
                currentPositionCoordinate, currentDestinationWayPoint.getWayPointVertical());
            // System.out.println("near: " + nearestCoordinateOnWayPointVertical.toString());
            // checks if the nearestCoordinateOnWayPoint is visible for the pedestrian
            if (isTargetVisible(position, nearestCoordinateOnWayPointVertical, boundaries))

            // if (isTargetVisible(position, currentPositionCoordinate, boundaries))
            {
                tempTargetCoordinate = nearestCoordinateOnWayPointVertical;
                // tempTargetCoordinate = currentPositionCoordinate;
                needsOrientation = false;
            }
            else
            {
                Coordinate alternativeOnWayPointVertical = getAlternativeOnWayPointVertical(
                    position, boundaries);
                if (alternativeOnWayPointVertical == null)
                {
                    // try to head to axis between WayPoints
                    tempTargetCoordinate = getAlternativeOnWayPointAxis(position, boundaries);

                    // test, if this point is visible
                    if (tempTargetCoordinate == null)
                    {
                        // test, if the last orientation update position is visible
                        if (isTargetVisible(position,
                            new Coordinate(lastOrientationUpdatePostion.x(),
                                lastOrientationUpdatePostion.y()),
                            boundaries))
                        {
                            tempTargetCoordinate = new Coordinate(lastOrientationUpdatePostion.x(),
                                lastOrientationUpdatePostion.y());
                        }
                        else
                        {
                            // TODO: in this case the Pedestrian is completely lost...
                        }
                    }
                    needsOrientation = true;
                }
                else
                {
                    tempTargetCoordinate = alternativeOnWayPointVertical;
                    needsOrientation = false;
                }
            }
            if (timestamp != startTime)
                lastOrientationUpdateTime = timestamp;
            lastOrientationUpdatePostion = position;
        }
        if (tempTargetCoordinate != null)
        {
            targetCoordinateVector = new Vector2D(tempTargetCoordinate.x, tempTargetCoordinate.y);

            // this is the implementation of formula(1) of Helbing et al. (1995)
            Vector2D directionVector = targetCoordinateVector.minus(position);
            normalizedDirectionVector = MathTools.normalize(directionVector.x(),
                directionVector.y());
            // save time and position of last course update
            lastCourseUpdatePostion = position;
            lastCourseUpdateTime = timestamp;
        }
        else
        {
            targetCoordinateVector = null;
            normalizedDirectionVector = new Vector2D(0, 0);
        }
    }

    /**
     * Checks whether there is a direct line of sight between this {@link Pedestrian} at its
     * {@code currentPositionVector} and its current destination or whether this is blocked through
     * a {@code boundaries}.
     *
     * @param currentPositionVector the current position of a {@link Pedestrian}
     * @param nearestCoordinateOnWayPoint is the calculated nearest {@link Coordinate} on the
     *            {@code wayPointVertical}
     * @param boundaries {@link List} that contains all {@link Boundary} objects
     *
     * @return {@code true} if there is a direct line of sight between this {@link Pedestrian} and
     *         its {@code Pedestrian#currentDestinationWayPoint}, {@code false} otherwise
     */
    private boolean isTargetVisible(Vector2D currentPositionVector,
        Coordinate nearestCoordinateOnWayPoint, List<Boundary> boundaries)
    {
        Coordinate pedestrianPosition = new Coordinate(currentPositionVector.x(),
            currentPositionVector.y());

        // computes LineString between Pedestrian and next Waypoint
        Coordinate[] coordinates = new Coordinate[] { pedestrianPosition,
            nearestCoordinateOnWayPoint };
        LineString lineOfSight = JTSFactoryFinder.getGeometryFactory()
            .createLineString(coordinates);

        // checks if the Pedestrian sees the its next Waypoint.
        boolean lineOfSightIntersectsBoundary = false;
        if (boundaries != null)
        {
            Envelope boundingBoxLineOfSight = lineOfSight.getEnvelopeInternal();
            for (Boundary boundary : boundaries)
            {
                // quick check
                if (boundingBoxLineOfSight.intersects(boundary.getBoundingBox()))
                {
                    // accurate check
                    lineOfSightIntersectsBoundary = lineOfSight.intersects(boundary.getGeometry());
                    if (lineOfSightIntersectsBoundary)
                        break;
                }
            }
        }
        return !lineOfSightIntersectsBoundary;
    }

    /**
     * If this {@link Pedestrian} does not see its current target, this method tries to find an
     * alternative {@link WayPoint} which is seen by the {@link Pedestrian}s.
     * <p>
     * For that every {@code wayPointVertical} of the {@link WayPoint}, which is a
     * {@link LineString} or a {@link MultiLineString}, is divided into a specific amount of parts,
     * denoted by the {@link #searchResolutionAlternativePosition}}. For all this parts a coordinate
     * is generated, where the method tests if the {@link Pedestrian} can see them. If the
     * {@link Pedestrian} sees more than one, the distance between them will be computed and the
     * closest will be chosen.
     *
     * @param currentPositionVector the actual position of this {@link Pedestrian}
     * @param boundaries {@link List} that contains all {@link Boundary} objects
     *
     * @return {@link Coordinate} on the {@code wayPointVertical} which is visible for the
     *         {@link Pedestrian}
     */
    private Coordinate getAlternativeOnWayPointVertical(Vector2D currentPositionVector,
        List<Boundary> boundaries)
    {
        Map<Coordinate, Double> candidatePointsAndDistances = new HashMap<>();
        Coordinate pedestrianPosition = new Coordinate(currentPositionVector.x(),
            currentPositionVector.y());
        Coordinate coordinateOnVertical = null;

        if (currentDestinationWayPoint.getWayPointVertical() instanceof LineString)
        {
            LineString lineString = (LineString) currentDestinationWayPoint.getWayPointVertical();
            Coordinate startingCoordinate = new Coordinate(lineString.getStartPoint().getX(),
                lineString.getStartPoint().getY());
            Coordinate endingCoordinate = new Coordinate(lineString.getEndPoint().getX(),
                lineString.getEndPoint().getY());
            LineSegment lineSegment = new LineSegment(startingCoordinate, endingCoordinate);

            // counts in how much parts a line will be split for searching for an alternative
            // waypoint
            double maxIterations = lineSegment.getLength() / searchResolutionAlternativePosition;

            candidatePointsAndDistances = searchForAlternativePointsOnLineSegment(lineSegment,
                pedestrianPosition, maxIterations, boundaries, true);
        }
        else if (currentDestinationWayPoint.getWayPointVertical() instanceof MultiLineString)
        {
            int count = currentDestinationWayPoint.getWayPointVertical().getNumGeometries();

            for (int i = 0; i < count; i++ )
            {
                LineString lineString = (LineString) currentDestinationWayPoint
                    .getWayPointVertical()
                    .getGeometryN(i);
                Coordinate startingCoordinate = new Coordinate(lineString.getStartPoint().getX(),
                    lineString.getStartPoint().getY());
                Coordinate endingCoordinate = new Coordinate(lineString.getEndPoint().getX(),
                    lineString.getEndPoint().getY());
                LineSegment lineSegment = new LineSegment(startingCoordinate, endingCoordinate);

                // counts in how much parts a line will be split for searching for an alternative
                // waypoint
                double maxIterations = lineSegment.getLength()
                    / searchResolutionAlternativePosition;

                Map<Coordinate, Double> alternativePointsOnMultiLineSegments = searchForAlternativePointsOnLineSegment(
                    lineSegment, pedestrianPosition, maxIterations, boundaries, true);

                for (Entry<Coordinate, Double> alternativePoint : alternativePointsOnMultiLineSegments
                    .entrySet())
                {
                    candidatePointsAndDistances.put(alternativePoint.getKey(),
                        alternativePoint.getValue());
                }
            }
        }

        List<Map.Entry<Coordinate, Double>> sortedListOfCandidatePoints = GeometryTools
            .sortMap(candidatePointsAndDistances);

        if ( !sortedListOfCandidatePoints.isEmpty())
        {
            coordinateOnVertical = new Coordinate(sortedListOfCandidatePoints.get(0).getKey());
        }

        return coordinateOnVertical;
    }

    /**
     * Gets a {@link Coordinate} on the current WayPoint axis that is visible and as close as
     * possible to the next {@link WayPoint}
     *
     * @param position the current position
     * @param boundaries the list of {@link Boundary} objects
     * @return a {@link Coordinate} on the current {@link WayPoint} axis that is as close as
     *         possible to the next {@link WayPoint} or {@code null}, if no such {@link Coordinate}
     *         could be found.
     */
    private Coordinate getAlternativeOnWayPointAxis(Vector2D position, List<Boundary> boundaries)
    {
        LineSegment ls = null;
        Coordinate coordinateOnWayPointAxis = null;
        // fallback to axis between start position and first WayPoint - this should only
        // happen in front of the first waypoint, because there is no axis between
        // WayPoints already
        if (currentDestinationWayPoint.getAxisBetweenWayPoints() == null)
        {
            ls = new LineSegment(currentDestinationWayPoint,
                new Coordinate(initialPositionVector.x(), initialPositionVector.y()));
        }
        // axis between waypoints
        else
        {
            LineString currentAxis = currentDestinationWayPoint.getAxisBetweenWayPoints();
            ls = new LineSegment(
                new Coordinate(currentAxis.getStartPoint().getX(),
                    currentAxis.getStartPoint().getY()),
                new Coordinate(currentAxis.getEndPoint().getX(), currentAxis.getEndPoint().getY()));
        }
        double maxIterations = ls.getLength() / searchResolutionAlternativePosition;
        Map<Coordinate, Double> candidatePointsAndDistances = searchForAlternativePointsOnLineSegment(
            ls, new Coordinate(position.x(), position.y()), maxIterations, boundaries, false);
        List<Map.Entry<Coordinate, Double>> sortedListOfCandidatePoints = GeometryTools
            .sortMap(candidatePointsAndDistances);
        if ( !candidatePointsAndDistances.isEmpty())
        {
            coordinateOnWayPointAxis = new Coordinate(sortedListOfCandidatePoints.get(0).getKey());
        }

        return coordinateOnWayPointAxis;
    }

    /**
     * Computes a Map of {@link Coordinate}s and associated distances from the given
     * {@code pedestrianPosition} to some interpolated {@link Coordinate}s on the given
     * {@code lineSegment}.
     * <p>
     *
     * @param lineSegment denotes the wayPointVertical or a coherently part of the wayPointVertical
     * @param pedestrianPosition describes the current position of a {@link Pedestrian}
     * @param maxIterations counts the number of segments in which the {@link LineSegment} is split
     *            for the search for an alternative wayPoint.
     * @param boundaries {@link List} that contains all {@link Boundary} objects
     * @param distanceOriginPosition if {@code true} the distance between {@code pedestrianPosition}
     *            and the interpolated {@link Coordinate}s are computed, otherwise the distance
     *            between the start point of the segment and the interpolated Coordinate is computed
     *
     * @return the Map of interpolated {@link Coordinate}s and associated distances.
     */
    private Map<Coordinate, Double> searchForAlternativePointsOnLineSegment(LineSegment lineSegment,
        Coordinate pedestrianPosition, double maxIterations, List<Boundary> boundaries,
        boolean distanceOriginPosition)
    {
        Map<Coordinate, Double> candidatePointsAndDistances = new HashMap<>();
        Coordinate coordinateOnLineSegment;
        double distance = 0d;

        double lengthOfLineSegment = lineSegment.getLength();
        for (int i = 0; i <= maxIterations; i++ )
        {
            // creates values between 0 and 1 (including 0 and 1) which divides the lineSegment in
            // specific number of equal parts of equal length
            double incrementOnLineSegment = i * searchResolutionAlternativePosition
                / lengthOfLineSegment;
            // ensures that incrementOnLineSegment = 1 exist (otherwise it will be 0,9x)
            if (i == (int) maxIterations)
                incrementOnLineSegment = 1;

            coordinateOnLineSegment = lineSegment.pointAlong(incrementOnLineSegment);
            LineString lineOfSight = JTSFactoryFinder.getGeometryFactory()
                .createLineString(new Coordinate[] { pedestrianPosition, coordinateOnLineSegment });

            // only put into candidate points into map if there is no intersection with boundaries
            // (if boundaries exist)
            if (boundaries == null || boundaries.isEmpty())
            {
                if (distanceOriginPosition)
                    distance = MathTools.distance(coordinateOnLineSegment, pedestrianPosition);
                else
                    distance = MathTools.distance(coordinateOnLineSegment, lineSegment.p0);
                candidatePointsAndDistances.put(coordinateOnLineSegment, distance);
                if ( !distanceOriginPosition)
                    break;
            }
            else
            {
                Envelope boundingBoxOfLineString = lineOfSight.getEnvelopeInternal();
                boolean lineOfSightIntersectBoundaries = false;
                for (Boundary boundary : boundaries)
                {
                    if (boundingBoxOfLineString.intersects(boundary.getBoundingBox()))
                    {
                        lineOfSightIntersectBoundaries = lineOfSight
                            .intersects(boundary.getGeometry());
                    }
                    if (lineOfSightIntersectBoundaries)
                        break;
                }
                if ( !lineOfSightIntersectBoundaries)
                {
                    if (distanceOriginPosition)
                        distance = MathTools.distance(coordinateOnLineSegment, pedestrianPosition);
                    else
                        distance = MathTools.distance(coordinateOnLineSegment, lineSegment.p0);
                    candidatePointsAndDistances.put(coordinateOnLineSegment, distance);
                    if ( !distanceOriginPosition)
                        break;
                }
            }
        }

        return candidatePointsAndDistances;
    }

    /**
     * Gets the next {@link WayPoint}, which is not already contained in {@link #visitedWayPoints}
     *
     * @return the next {@link WayPoint}, which is not already contained in
     *         {@link #visitedWayPoints} or {@code null}, if either no {@link WayPoint} exists or
     *         all {@link WayPoint} are already contained in {@link #visitedWayPoints}
     */
    private WayPoint getNextWayPoint()
    {
        if (wayPoints != null && !wayPoints.isEmpty())
        {
            for (WayPoint wayPoint : wayPoints)
            {
                if ( !visitedWayPoints.contains(wayPoint))
                {
                    // System.out.println("NEXT: " + wayPoint);
                    return wayPoint;
                }
            }
            return null;
        }
        return null;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public float getAverageVelocityOnRoute(Vector2D position, long currentTime,
        boolean useCachedDistance)
    {
        float distance = useCachedDistance ? totalDistanceOnRoute
            : getTotalDistanceOnRoute(position);
        return distance / ((currentTime - getStartTime()) / 1000f);
    }
}

package de.fhg.ivi.crowdsimulation.simulation.mentalmodel;

import java.util.List;

import de.fhg.ivi.crowdsimulation.simulation.objects.Boundary;
import de.fhg.ivi.crowdsimulation.simulation.objects.Pedestrian;
import de.fhg.ivi.crowdsimulation.simulation.objects.WayPoint;
import math.geom2d.Vector2D;

/**
 * This class is the super class (abstract class) in case of the way finding algorithm of the
 * {@link Pedestrian}s.
 * <p>
 * If this class shall calculate a new way for the {@link Pedestrian}s, some preconditions must be
 * fulfilled. It must be {@code true} that a {@link Pedestrian} has extrinsic forces, needs
 * orientation generally or his/her last movement calculation is longer ago than 5 seconds or 5
 * meters in x-, y-direction.
 * <p>
 * Furthermore this class checks whether a {@link Pedestrian} has passed a {@link WayPoint},
 * calculates the normalized direction vector of the {@link Pedestrian} movement and calculates the
 * total distance on the route of the {@link Pedestrian}s since his/her starting point.
 *
 * @author hahmann/meinert
 */
public abstract class WayFindingModel
{

    /**
     * The {@link Vector2D} object of the normalized (i.e. length=1) direction vector, i.e. the
     * vector, in which direction the {@link Pedestrian} currently and intrinsically (i.e. assuming
     * no external influences) wants to go.
     */
    protected Vector2D normalizedDirectionVector;

    /**
     * Distance, given in meter, which the pedestrian went.
     */
    protected float    totalDistanceOnRoute;

    /**
     * {@code true} if this {@link Pedestrian} cannot find a target coordinate on the
     * {@code wayPointVertical} and thus tries to either head to the axis between the current and
     * the last waypoint or the last position where the next waypoint could be seen, and also in the
     * case of the start of the simulation. Else {@code false}.
     */
    protected boolean  needsOrientation   = true;

    /**
     * The time stamp when the {@link Pedestrian} starts moving towards its destination - this
     * should be updated, when this {@link Pedestrian} starts moving. Given in milliseconds.
     */
    protected long     startTime;

    /**
     * {@code true} if the actual course deviates from the required course to reach the current
     * WayPoints. Else {@code false}.
     */
    protected boolean  hasCourseDeviation = false;

    /**
     * Gets the current normalized direction vector, i.e. the direction vector this
     * {@link WayFindingModel} is currently heading towards, normalized to a length of without any
     * recalculation.
     *
     * @return the current normalized direction vector
     */
    public abstract Vector2D getNormalizedDirectionVector();

    /**
     * Gets the average velocity on the desired route at the current position and the current time
     *
     * @param position the current position
     * @param currentTime the current time
     * @param useCachedDistance if {@code true} the total distance on route is taken from cache,
     *            otherwise the total distance on route is forced to be calculated
     * @return the average velocity
     */
    public abstract float getAverageVelocityOnRoute(Vector2D position, long currentTime,
        boolean useCachedDistance);

    /**
     * Get whether this {@link Pedestrian} to {@link #needsOrientation}.
     *
     * @return {@code true} if this {@link Pedestrian} cannot find a {@link WayPoint} on the
     *         {@code wayPointVertical}, and must use an alternative wayPoint, and also in the case
     *         of the start of the simulation. Else {@code false}.
     */
    public boolean needsOrientation()
    {
        return this.needsOrientation;
    }

    /**
     * Set this {@link Pedestrian} to {@link #needsOrientation}.
     *
     * @param needsOrientation {@code true} if this {@link Pedestrian} cannot find a
     *            {@link WayPoint} on the {@code wayPointVertical}, and must use an alternative
     *            wayPoint, and also in the case of the start of the simulation. Else {@code false}.
     */
    public void setNeedsOrientation(boolean needsOrientation)
    {
        this.needsOrientation = needsOrientation;
    }

    /**
     * Get {@link #startTime}.
     *
     * @return the Unix time stamp when the {@link Pedestrian} started moving. Given in
     *         milliseconds.
     */
    public long getStartTime()
    {
        return this.startTime;
    }

    /**
     * Sets {@link #startTime}.
     *
     * @param startTime the Unix time stamp when the {@link Pedestrian} starts moving. Given in
     *            milliseconds.
     */
    public void setStartTime(long startTime)
    {
        this.startTime = startTime;
    }

    /**
     * Checks whether extrinsic forces impact a {@link Pedestrian}.
     *
     * @return {@code true} if extrinsic forces are not null. Else {@code false}
     */
    public boolean hasCourseDeviation()
    {
        return hasCourseDeviation;
    }

    /**
     * Derives the current normalized direction vector (i.e. the direction vector this mental model
     * is currently heading towards, normalized to a length of 1) for the given {@code position},
     * {@code time} and {@link Boundary} objects.
     *
     * @param position the {@link Vector2D} position at which the normalized direction vector should
     *            be derived
     * @param time the unix time stamp when this method is called (in simulation time)
     * @param boundaries {@link List} that contains all {@link Boundary} objects
     * @param normalDesiredVelocity the normal desired velocity of the pedestrian for checking, if
     *            the Pedestrian is not moving anymore
     */
    public abstract void updateNormalizedDirectionVector(Vector2D position, long time,
        List<Boundary> boundaries, float normalDesiredVelocity);

    /**
     * Set {@code true} if a {@link Pedestrian} has passed a specific {@link WayPoint}.
     *
     * @param hasPassedWayPoint {@code true} if the {@link Pedestrian} has passed a specific
     *            {@link WayPoint}. Else {@code false}
     *
     * @see de.fhg.ivi.crowdsimulation.simulation.mentalmodel.FollowWayPointsMentalModel#setHasPassedWayPoint(boolean)
     */
    public abstract void setHasPassedWayPoint(boolean hasPassedWayPoint);

    /**
     * Checks if the current {@link WayPoint} has been passed by the given {@code pedestrian}. This
     * depends on the current position, {@code currentPosition}, of the {@link Pedestrian} and
     * his/her next position, {@code updatedPosition}.
     *
     * @param pedestrian object
     * @param currentPosition the current position of the {@code pedestrian}
     * @param updatedPosition the next position of the {@code pedestrian}
     */
    public abstract void checkWayPointPassing(Pedestrian pedestrian, Vector2D currentPosition,
        Vector2D updatedPosition);

    /**
     * Checks if the actual course (i.e. angle of {@link #normalizedDirectionVector}) deviates from
     * the required course (i.e. angle between current position of the pedestrian and its current
     * target point) is above a threshold and mark this as such.
     *
     * @param pedestrian the pedestrian to be checked for course deviation
     * @param timestamp the current timestamp
     */
    public abstract void checkCourse(Pedestrian pedestrian, long timestamp);
}

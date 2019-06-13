package de.fhg.ivi.crowdsimulation.simulation.forcemodel;

import com.vividsolutions.jts.geom.Geometry;

import de.fhg.ivi.crowdsimulation.simulation.objects.Boundary;
import de.fhg.ivi.crowdsimulation.simulation.objects.Pedestrian;
import math.geom2d.Vector2D;

/**
 * The ForceModel represents a force-based approach for modeling microscopic pedestrian simulation.
 * It is a mixture of empirical parameters and Newton's 2nd law of motion. The original model or
 * Social Force Model is based on the publication Helbing and Molnár (1995). Because of the
 * empirical parameters, there are different possible solutions.
 * <p>
 * Currently, it is possible to choose between the classical Helbing model {@link HelbingBuznaModel}
 * (Helbing et al., 2005) and Johansson model {@link HelbingJohanssonModel} (Johansson et al.,
 * 2007). The difference between the models lies in the assumed force fields of the pedestrians and
 * the force parameters (i.e. strength of force field and size). Helbing assumes a circular field
 * and Johansson an elliptical field.
 * <p>
 * Further modeling approaches are under construction e.g. a model of Moussaïd {@link MoussaidModel}
 * (Moussaïd et al., 2009).
 *
 * @author hahmann/meinert
 */
public abstract class ForceModel
{
    /**
     * Maximum distance within the interaction between two {@link Pedestrian} is regarded (to
     * accelerate simulation). Given in meters.
     */
    protected float           maxPedestrianInteractionDistance;

    /**
     * Maximum distance within the interaction between a pedestrian and a boundary is regarded (to
     * accelerate simulation). Given in meters.
     */
    protected float           maxBoundaryInteractionDistance;

    /**
     * Minimal force threshold which is assumed to have an effect on the {@link Pedestrian}.
     *
     * <li>if this threshold is too small, too it will slow down performance, since too many
     * calculations for Pedestrian-Pedestrian and Pedestrian-Boundary interaction need be done that
     * actually have no significant effect on the pedestrians' movement
     * <li>if this threshold is too high, the simulation will get inaccurate, i.e. the Pedestrian
     * will only react to close boundaries / pedestrians we it gets very near to them
     */
    public static final float limitResultingForce = 0.01f;

    /**
     * Gets the acceleration that is needed to reach the desired velocity and go in the desired
     * direction.
     *
     * @param currentPosition the current x, y position as {@link Vector2D} of the
     *            {@link Pedestrian}
     * @param currentVelocity the current velocity of the {@link Pedestrian}
     * @param normalizedDirectionVector the direction {@link Vector2D} in which the
     *            {@link Pedestrian} wants to walk.
     * @param averageVelocityOnRoute averageVelocityOnRoute the velocity of the {@link Pedestrian}
     *            in dependence to his/her current traveled distance and the past time.
     * @param actualDesiredVelocity the current velocity of the {@link Pedestrian}
     * @param maximumDesiredVelocity the maximal desired velocity the {@link Pedestrian}
     *
     * @return the {@link Vector2D} force component that is needed to reach the desired velocity in
     *         the desired direction.
     */
    public abstract Vector2D intrinsicForce(Vector2D currentPosition, Vector2D currentVelocity,
        Vector2D normalizedDirectionVector, float averageVelocityOnRoute,
        float actualDesiredVelocity, float maximumDesiredVelocity);

    /**
     * Computes the force resulting from pedestrian-pedestrian interaction. Checks, if the distance
     * between {@code currentPosition} and the {@link Pedestrian} is smaller than
     * {@link #getMaxPedestrianInteractionDistance()} before.
     *
     * @param currentPosition denotes the approximated x, y position as {@link Vector2D} of the
     *            {@link Pedestrian}
     * @param normalizedDirectionVector the direction {@link Vector2D} in which the
     *            {@link Pedestrian} wants to walk.
     * @param pedestrian the {@link Pedestrian} object
     *
     * @return the {@link Vector2D} vector which resulting from the interaction of the current
     *         {@link Pedestrian} with another {@code pedestrian}
     */
    public abstract Vector2D interactPedestrian(Vector2D currentPosition,
        Vector2D normalizedDirectionVector, Pedestrian pedestrian);

    /**
     * Computes the force resulting from pedestrian-boundary interaction. Checks, if the distance
     * between {@code currentPosition} and the {@link Pedestrian} is smaller than
     * {@link #getMaxPedestrianInteractionDistance()} before.
     *
     * @param currentPosition denotes the approximated x, y position as {@link Vector2D} of the
     *            {@link Pedestrian}
     * @param normalizedDirectionVector the direction {@link Vector2D} in which the
     *            {@link Pedestrian} wants to walk.
     * @param boundary the {@link Geometry} object
     *
     * @return the {@link Vector2D} vector which resulting from the interaction of the current
     *         {@link Pedestrian} with a {@code geometry}
     */
    public abstract Vector2D interactBoundary(Vector2D currentPosition,
        Vector2D normalizedDirectionVector, Boundary boundary);

    /**
     * Gets the radius of a {@link Pedestrian}. Given in meters.
     *
     * @return the radius of a {@link Pedestrian}. Given in meters.
     */
    public abstract float getPedestrianRadius();

    /**
     * Computes the distance, in which a {@link Pedestrian} interacts with a {@link Boundary}. The
     * calculation formula is the formula of the interact() converted to the distance.
     * <p>
     * This distance depends on given constants and the resulting force
     * {@link ForceModel#limitResultingForce}. It is assumed that the force effect on the
     * pedestrians is negligible below this value.
     *
     * @return distance, given in meters, in which the {@link Pedestrian} interacts with a
     *         {@link Boundary}
     */
    public abstract float getMaxBoundaryInteractionDistance();

    /**
     * Computes the distance, in which a {@link Pedestrian} interacts with another
     * {@link Pedestrian}. The calculation formula is the formula of the
     * {@link #interactPedestrian(Vector2D, Vector2D, Pedestrian)} converted to the distance.
     * <p>
     * This distance depends on given constants and the resulting force
     * {@link ForceModel#limitResultingForce}. It is assumed that the force effect on the
     * pedestrians is negligible below this value.
     *
     * @return distance, given in meters, in which the {@link Pedestrian} interacts with another
     *         {@link Pedestrian}
     */
    protected abstract float getMaxPedestrianInteractionDistance();
}

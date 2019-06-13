package de.fhg.ivi.crowdsimulation.simulation.forcemodel;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;

import de.fhg.ivi.crowdsimulation.simulation.objects.Boundary;
import de.fhg.ivi.crowdsimulation.simulation.objects.Pedestrian;
import de.fhg.ivi.crowdsimulation.simulation.tools.GeometryTools;
import de.fhg.ivi.crowdsimulation.simulation.tools.MathTools;
import math.geom2d.Vector2D;

/**
 * Implements formulas for the 3 forces described in the Social Forces model for pedestrian dynamics
 * by Helbing and Molnár (1995), Helbing et al. (2005):
 * <li>Pedestrian-Pedestrian interaction</li>
 * <li>Pedestrian-Boundary (obstacle) interaction</li>
 * <li>Intrinsic Force of a Pedestrian to move towards a certain target at a desired speed</li>
 * <p>
 * The Helbing Model requires 8 parameters: A1, B1, A2, B2 for pedestrian-pedestrian interaction and
 * A1, B1, A2, B2 for pedestrian-boundary (obstacle) interaction that need to be implemented by
 * sub-classes
 *
 * @see <a href=
 *      "https://journals.aps.org/pre/abstract/10.1103/PhysRevE.51.4282">https://journals.aps.org/pre/abstract/10.1103/PhysRevE.51.4282</a>
 * @see <a href=
 *      "https://pubsonline.informs.org/doi/abs/10.1287/trsc.1040.0108">https://pubsonline.informs.org/doi/abs/10.1287/trsc.1040.0108</a>
 *
 * @author hahmann/meinert
 *
 */
public abstract class HelbingModel extends ForceModel
{

    /**
     * Uses the object logger for printing specific messages in the console.
     */
    private static final Logger logger = LoggerFactory.getLogger(HelbingModel.class);

    /**
     * Computes the force resulting from pedestrian-pedestrian interaction if the {@link Pedestrian}
     * is under the {@code Pedestrian#getMaxPedestrianInteractionDistance()} to other
     * {@link Pedestrian}.
     *
     * @param currentPosition denotes the approximated x, y position as {@link Vector2D} of the
     *            {@link Pedestrian}
     * @param pedestrian the {@link Pedestrian} object
     *
     * @return the {@link Vector2D} vector which resulting from the interaction of the current
     *         {@link Pedestrian} with another {@code pedestrian}
     */
    @Override
    public Vector2D interactPedestrian(Vector2D currentPosition, Vector2D normalizedDirectionVector,
        Pedestrian pedestrian)
    {
        Vector2D forceDelta = new Vector2D(0, 0);

        // quick check, if the other pedestrian is roughly within the interaction distance of
        // this pedestrian
        {
            if (Math.abs(pedestrian.getCurrentPosition().x()
                - currentPosition.x()) > getMaxPedestrianInteractionDistance())
            {
                return forceDelta;
            }
            if (Math.abs(pedestrian.getCurrentPosition().y()
                - currentPosition.y()) > getMaxPedestrianInteractionDistance())
            {
                return forceDelta;
            }
        }

        // exact check if a pedestrian is within the interaction distance of this pedestrian
        double distanceSquared = MathTools
            .normSquared(currentPosition.minus(pedestrian.getCurrentPosition()));

        // 2 pedestrians lying exactly on top of each - this should be a rare case
        if (currentPosition.equals(pedestrian.getCurrentPosition()))
            distanceSquared = Double.MIN_VALUE;

        // TODO: perhaps quick check if other pedestrian is behind this pedestrian:
        // ep12 = distp12.Normalized();
        // double tmpv = ped1->GetV().ScalarProduct(ep12); // < v^0_i , e_ij >
        // double ped2IsBehindv = (tmpv <= 0) ? 0 : 1;

        // compare square product of distances to gain some performance
        if (distanceSquared < getMaxPedestrianInteractionDistance()
            * getMaxPedestrianInteractionDistance())
        {
            double distance = MathTools
                .norm(currentPosition.minus(pedestrian.getCurrentPosition()));

            // 2 pedestrians lying exactly on top of each - this should be a rare case
            if (currentPosition.equals(pedestrian.getCurrentPosition()))
                distance = Double.MIN_VALUE;

            // calculation of nVector
            Vector2D nVector = (currentPosition.minus(pedestrian.getCurrentPosition())
                .times(1 / distance));

            // calculation of phi
            double phi = 0;
            if (getParameterPedestrianA1() != 0)
            {
                double cos = nVector.times( -1).dot(normalizedDirectionVector);
                // TODO: Look up table to speed up in case of HelbingJohanssonModel
                phi = Math.acos(cos);
            }

            forceDelta = interact(pedestrian.getCurrentPosition(), nVector,
                getParameterPedestrianA1(), getParameterPedestrianB1(), getParameterPedestrianA2(),
                getParameterPedestrianB2(), (getPedestrianRadius() * 2), distance, phi);
        }
        return forceDelta;
    }

    /**
     * Computes the force resulting from pedestrian-geometry interaction if the {@link Pedestrian}
     * is under the {@code Pedestrian#getMaxBoundaryInteractionDistance()} to {@link Geometry}.
     *
     * @param currentPosition denotes the approximated x, y position as {@link Vector2D} of the
     *            {@link Pedestrian}
     * @param normalizedDirectionVector the direction {@link Vector2D} in which the
     *            {@link Pedestrian} walks.
     * @param boundary the {@link Geometry} object
     *
     * @return the {@link Vector2D} vector which resulting from the interaction of the current
     *         {@link Pedestrian} with a {@code geometry}
     */
    @Override
    public Vector2D interactBoundary(Vector2D currentPosition, Vector2D normalizedDirectionVector,
        Boundary boundary)
    {
        Vector2D forceDelta = new Vector2D(0, 0);

        // bounding box based check, if the boundary is roughly within the interaction distance of
        // this pedestrian
        Envelope boundingBox = boundary.getBoundingBox();
        if ( !GeometryTools.isInside(currentPosition, boundingBox))
        {
            return forceDelta;
        }

        Coordinate currentPositionCoordinate = new Coordinate(currentPosition.x(),
            currentPosition.y());
        Coordinate relevantInteractionCoordinate = GeometryTools
            .getNearestPoint(currentPositionCoordinate, boundary.getGeometry());

        double distanceSquared = MathTools.distanceSquared(currentPositionCoordinate,
            relevantInteractionCoordinate);

        // a pedestrian lying exactly on top of a boundary - this should be a rare case
        if (currentPositionCoordinate.equals(relevantInteractionCoordinate))
            distanceSquared = Double.MIN_VALUE;

        // compare distance squared to gain some performance
        if (distanceSquared < getMaxBoundaryInteractionDistance()
            * getMaxBoundaryInteractionDistance())
        {
            // actual distance
            double distance = MathTools.distance(currentPositionCoordinate,
                relevantInteractionCoordinate);

            // a pedestrian lying exactly on top of a boundary - this should be a rare case
            if (currentPositionCoordinate.equals(relevantInteractionCoordinate))
                distance = Double.MIN_VALUE;

            // calculation of nVector
            Vector2D vectorInteractingObject = new Vector2D(relevantInteractionCoordinate.x,
                relevantInteractionCoordinate.y);
            Vector2D nVector = (currentPosition.minus(vectorInteractingObject)).times(1 / distance);

            // TODO Because of the following condition, the parameter phi is not used at the moment.
            // The reason for that is that the phi-calculation needs to be proven against its
            // correctness and usefulness (compare with Helbing et al. 2005).

            // calculation of phi - phi is the dot product of the normalized direction vectors
            double phi = 0;
            if (getParameterBoundaryA1() != 0)
            {
                Vector2D eAlpha = normalizedDirectionVector;
                Vector2D eBeta = GeometryTools.getEVectorOfBoundary(currentPositionCoordinate,
                    boundary.getGeometry());
                // TODO LUT based method for Math.acos()
                phi = Math
                    .acos((eAlpha.dot(eBeta) / (MathTools.norm(eAlpha) * MathTools.norm(eBeta))));
            }

            logger.trace("interactBoundary(), phi: " + phi);

            forceDelta = interact(
                new Vector2D(relevantInteractionCoordinate.x, relevantInteractionCoordinate.y),
                nVector, getParameterBoundaryA1(), getParameterBoundaryB1(),
                getParameterBoundaryA2(), getParameterBoundaryB2(), getPedestrianRadius(), distance,
                phi);
        }

        return forceDelta;
    }

    /**
     * Computes the force resulting of pedestrian-pedestrian interaction or pedestrian-geometry
     * interaction.
     *
     * @param currentPosition denotes the approximated x, y position as {@link Vector2D} of the
     *            {@link Pedestrian}
     * @param nVector the normalized {@link Vector2D} pointing from pedestrian or geometry to the
     *            current {@link Pedestrian}
     * @param parameterA1 the parameter denotes the strength of the interaction (it's a given
     *            constant)
     * @param parameterB1 the parameter denotes the range of the repulsive interaction (it's a given
     *            constant)
     * @param parameterA2 the parameter denotes the strength of the interaction (it's a given
     *            constant)
     * @param parameterB2 the parameter denotes the range of the repulsive interaction (it's a given
     *            constant)
     * @param interactionRadius radius of {@link Pedestrian} or boundaries
     * @param distance describes the distance between the {@link Coordinate} and the current
     *            {@link Pedestrian}
     * @param phi dot product of the normalized direction vector
     *
     * @return the {@link Vector2D} vector resulting from the interaction of the current
     *         {@link Pedestrian} with another {@code pedestrian} or a {@code geometry}
     */
    private Vector2D interact(Vector2D currentPosition, Vector2D nVector, double parameterA1,
        double parameterB1, double parameterA2, double parameterB2, double interactionRadius,
        double distance, double phi)
    {
        Vector2D forceDelta = nVector
            .times(parameterA2 * Math.exp((interactionRadius - distance) / parameterB2));

        if (parameterA1 != 0)
        {
            // TODO: Look up table for cos to speed up in case of HelbingJohanssonModel
            forceDelta = nVector
                .times(parameterA1 * Math.exp((interactionRadius - distance) / parameterB1))
                .times(getLambda() + (1 - getLambda()) * (1 + Math.cos(phi)) / 2)
                .plus(forceDelta);
        }

        return forceDelta;
    }

    /**
     * Gets the acceleration that is needed to reach the desired velocity and go in the desired
     * direction.
     *
     * @param currentPosition the current x, y position as {@link Vector2D} of the
     *            {@link Pedestrian}
     * @param currentVelocity the current velocity of the {@link Pedestrian}
     * @param normalizedDirectionVector the direction {@link Vector2D} in which the
     *            {@link Pedestrian} walks.
     * @param averageVelocityOnRoute the velocity of the {@link Pedestrian} in dependence to his/her
     *            current traveled distance and the past time.
     * @param actualDesiredVelocity the current velocity of the {@link Pedestrian}
     * @param maximumDesiredVelocity the maximal desired velocity of the {@link Pedestrian}.
     *
     * @return the {@link Vector2D} force component that is needed to reach the desired velocity in
     *         the desired direction.
     */
    @Override
    public Vector2D intrinsicForce(Vector2D currentPosition, Vector2D currentVelocity,
        Vector2D normalizedDirectionVector, float averageVelocityOnRoute,
        float actualDesiredVelocity, float maximumDesiredVelocity)
    {
        // this is the implementation of formula (5) and (6) of Helbing et al. (2005)
        float currentDesiredVelocity = averageVelocityOnRoute
            + (1f - averageVelocityOnRoute / actualDesiredVelocity) * maximumDesiredVelocity;

        // the actual force computation
        Vector2D resultingForce = normalizedDirectionVector.times(currentDesiredVelocity)
            .minus(currentVelocity)
            .times(1 / getTau());

        return resultingForce;
    }

    /**
     * Gets the strength parameter A1 of the acceleration resulting from the interaction of a
     * {@link Pedestrian} with another {@link Pedestrian} given in m/s�.
     *
     * @return the strength parameter A1 of the acceleration resulting from the interaction of a
     *         {@link Pedestrian} with another {@link Pedestrian} given in m/s�
     */
    public abstract float getParameterPedestrianA1();

    /**
     * Gets the B2 Parameter of the interaction range with another {@link Pedestrian}. Given in
     * meters. Parameterizes the private zone of a {@link Pedestrian}.
     *
     * @return the B2 Parameter of the interaction range. Given in meters.
     */
    public abstract float getParameterPedestrianB1();

    /**
     * Gets the strength parameter A2 of the acceleration resulting from the interaction of a
     * {@link Pedestrian} with another {@link Pedestrian}. Given in m/s�. cf. Helbing et al (2005)
     * p. 12.
     *
     * @return the strength parameter A1 of the acceleration resulting from the interaction of a
     *         {@link Pedestrian} with another {@link Pedestrian}. Given in m/s�
     */
    public abstract float getParameterPedestrianA2();

    /**
     * Gets the B2 Parameter of the interaction range with another {@link Pedestrian}. Given in
     * meters. Parameterizes the private zone of a {@link Pedestrian}.
     *
     * @return the B2 Parameter of the interaction range. Given in meters.
     */
    public abstract float getParameterPedestrianB2();

    /**
     * Gets the strength parameter A1 of the acceleration resulting from the interaction of a
     * {@link Pedestrian} with a {@link Boundary} given in m/s�.
     *
     * @return the strength parameter A1 of the acceleration resulting from the interaction of a
     *         {@link Pedestrian} with a {@link Boundary} given in m/s�
     */
    public abstract float getParameterBoundaryA1();

    /**
     * Gets the B2 Parameter of the interaction range with a {@link Boundary}. Given in meters.
     * Parameterizes the comfort zone of a {@link Pedestrian}.
     *
     * @return the B2 Parameter of the interaction range. Given in meters.
     */
    public abstract float getParameterBoundaryB1();

    /**
     * Gets the strength parameter A2 of the acceleration resulting from the interaction of a
     * {@link Pedestrian} with a {@link Boundary}. Given in m/s�. cf. Helbing et al (2005) p. 12.
     *
     * @return the strength parameter A1 of the acceleration resulting from the interaction of a
     *         {@link Pedestrian} with a {@link Boundary}. Given in m/s�
     */
    public abstract float getParameterBoundaryA2();

    /**
     * Gets the B2 Parameter of the interaction range with a {@link Boundary}. Given in meters.
     * Parameterizes the comfort zone of a {@link Pedestrian}
     *
     * @return the B2 Parameter of the interaction range. Given in meters.
     */
    public abstract float getParameterBoundaryB2();

    /**
     * Gets the "relaxation time". Deviations of the actual velocity from the desired velocity due
     * to disturbances (by obstacles or avoidance maneuvers) are corrected within the so-called
     * �relaxation time� "Relaxation time" of a {@link Pedestrian}. Given in seconds. Cf. Helbing
     * (2005), p. 11
     *
     * @return the "Relaxation time" of a {@link Pedestrian}. Given in seconds.
     */
    public abstract float getTau();

    /**
     * Gets the parameter that takes into account the anisotropic character of pedestrian
     * interactions, as the situation in front of a pedestrian has a larger impact on his or her
     * behavior than things happening behind. Cf. Helbing (2005), p. 13.
     *
     * @return the parameter that takes into account the anisotropic character of pedestrian
     *         interactions
     */
    public abstract float getLambda();

    /**
     * Gets the radius of a {@link Pedestrian}. Given in meters.
     *
     * @return the radius of a {@link Pedestrian}. Given in meters.
     */
    @Override
    public float getPedestrianRadius()
    {
        return 0.3f;
    }

    /**
     * Computes the distance, in which a {@link Pedestrian} interacts with another
     * {@link Pedestrian}. The calculation formula is the formula of the interact() converted to the
     * distance.
     * <p>
     * This distance depends on given constants and the resulting force limitResultingForce. This
     * variable is set to 0.01 because at this limit value is the force change on the pedestrians
     * negligible.
     *
     * @return distance, given in meters, in which the {@link Pedestrian} interacts with another
     *         {@link Pedestrian}
     *
     * @see de.fhg.ivi.crowdsimulation.simulation.forcemodel.ForceModel#getMaxPedestrianInteractionDistance()
     */
    @Override
    public float getMaxPedestrianInteractionDistance()
    {
        // do not repeat calculation, if value is already set
        if (maxPedestrianInteractionDistance != 0)
            return maxPedestrianInteractionDistance;

        maxPedestrianInteractionDistance = getMaxInteractionDistance(getParameterPedestrianA2(),
            getParameterPedestrianB2());

        return maxPedestrianInteractionDistance;
    }

    /**
     * Computes the distance, in which a {@link Pedestrian} interacts with a {@link Boundary}. The
     * calculation formula is the formula of the interact() converted to the distance.
     * <p>
     * This distance depends on given constants and the resulting force limitResultingForce. This
     * variable is set to 0.01 because at this limit, it is assumed that the force effect on the
     * pedestrians negligible.
     *
     * @return distance, given in meters, in which the {@link Pedestrian} interacts with a
     *         {@link Boundary}
     *
     * @see de.fhg.ivi.crowdsimulation.simulation.forcemodel.ForceModel#getMaxBoundaryInteractionDistance()
     */
    @Override
    public float getMaxBoundaryInteractionDistance()
    {
        // do not repeat calculation, if value is already set
        if (maxBoundaryInteractionDistance != 0)
            return maxBoundaryInteractionDistance;

        maxBoundaryInteractionDistance = getMaxInteractionDistance(getParameterBoundaryA2(),
            getParameterBoundaryB2());

        return maxBoundaryInteractionDistance;
    }

    /**
     * Calculates the maximal distance in which the interaction between a {@link Pedestrian} and a
     * {@link Boundary} or another {@link Pedestrian} has an appreciable effect.
     *
     * @param parameterA2 the strength parameter of the acceleration resulting from the interaction
     *            of a {@link Pedestrian} with a {@link Boundary} or another {@link Pedestrian}.
     * @param parameterB2 interaction range between a {@link Pedestrian} and another
     *            {@link Pedestrian} or a {@link Boundary}. Parameterizes the private zone of a
     *            {@link Pedestrian}.
     *
     * @return distance, given in meters, in which the {@link Pedestrian} interacts with a
     *         {@link Boundary} or another {@link Pedestrian}
     */
    private float getMaxInteractionDistance(float parameterA2, float parameterB2)
    {
        // TODO: this needs to be more generic in case of HelbingBuzna and HelbingJohansson class.
        // However, the second one is not yet fully implemented and this method will not work in
        // case of A2=0. Parameters, A1 and B1 should also be considered (but there are not used to
        // use, at the moment).
        return getPedestrianRadius()
            - parameterB2 * (float) Math.log((limitResultingForce / parameterA2));
    }
}

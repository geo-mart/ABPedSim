package de.fhg.ivi.crowdsimulation.simulation.numericintegration;

import java.util.List;

import org.geotools.geometry.jts.JTSFactoryFinder;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LineString;

import de.fhg.ivi.crowdsimulation.simulation.objects.Boundary;
import de.fhg.ivi.crowdsimulation.simulation.objects.Pedestrian;
import de.fhg.ivi.crowdsimulation.simulation.tools.GeometryTools;
import de.fhg.ivi.crowdsimulation.simulation.tools.MathTools;
import math.geom2d.Vector2D;

/**
 * This class is used to validate the calculation of the used numeric integration class. This could
 * be one out of the classes {@link SimpleEulerIntegrator}, {@link SemiImplicitEulerIntegrator} or
 * {@link RungeKuttaIntegrator}.
 * <p>
 * The validation is split into three parts:
 * <li>Validation of the velocity of the {@link Pedestrian}s</li>
 * <li>Validation of the movement, represented through the position, of the {@link Pedestrian}s</li>
 * <li>Validation whether the {@link Pedestrian} movement intersects a {@link Geometry}</li>
 * <p>
 *
 * @author hahmann/meinert
 */
public class NumericIntegrationTools
{

    /**
     * Uses the object logger for printing specific messages in the console.
     */
    private static final Logger logger     = LoggerFactory.getLogger(NumericIntegrationTools.class);

    /**
     * mode for checking crossing in
     * {@link #moveRelatesGeometry(Envelope, Geometry, Vector2D, Vector2D, byte)}
     */
    private static final byte   CROSSES    = 0;

    /**
     * mode for checking intersecting in
     * {@link #moveRelatesGeometry(Envelope, Geometry, Vector2D, Vector2D, byte)}
     */
    private static final byte   INTERSECTS = 1;

    /**
     * Checks whether the current velocity of a {@link Pedestrian} is bigger than the maximum
     * desired velocity, which could be defined as a speed limit for pedestrians. If the
     * {@code updatedVelocity} is bigger than the maximal, the velocity is computed new under the
     * influence of the maximum desired velocity.
     *
     * @param updatedVelocity is the current velocity of a {@link Pedestrian}
     * @param pedestrian an object of the {@link Pedestrian}
     *
     * @return the updated velocity of a {@link Pedestrian}
     */
    public static Vector2D getValidatedVelocity(Pedestrian pedestrian, Vector2D updatedVelocity)
    {
        // check if updatedVelocity is bigger than maximal desired velocity
        double updatedVelocitySquared = MathTools.normSquared(updatedVelocity);

        // compare square products to gain some performance
        if (updatedVelocitySquared > pedestrian.getMaximumDesiredVelocity()
            * pedestrian.getMaximumDesiredVelocity())
        {

            // TODO test why there are some more true results trough the new if condition than by
            // the old one
            // if (updatedVeloAsDouble > pedestrian.getMaximumDesiredVelocity())
            logger.trace("calculate(), updatedVelocityOld: " + updatedVelocity);

            updatedVelocity = MathTools.normalize(updatedVelocity.x(), updatedVelocity.y())
                .times(pedestrian.getMaximumDesiredVelocity());

            logger.trace("calculate(), vMax: " + pedestrian.getMaximumDesiredVelocity());
            // logger.trace("calculate(), updatedVelocityNew: " + updatedVeloAsDouble);
            logger.trace("calculate(), new velocity: " + updatedVelocity);
        }

        return updatedVelocity;
    }

    /**
     * Checks whether the {@link Pedestrian} movement crosses any {@link Geometry}s of
     * {@link Boundary}s. This would mean that the {@link Pedestrian} would pass a wall during this
     * move. If this would be the case, the {@code oldPosition} is returned, otherwise the
     * {@code newPosition}
     *
     * @param pedestrian an object of the {@link Pedestrian}
     * @param boundaries {@link List} that contains all {@link Boundary} objects
     * @param oldPosition current position of the {@link Pedestrian}
     * @param newPosition next position of the {@link Pedestrian}, after the move
     *
     * @return the validated position of the {@link Pedestrian} (either {@code oldPosition} or
     *         {@code newPosition}).
     */
    public static Vector2D validateMove(Pedestrian pedestrian, List<Boundary> boundaries,
        Vector2D oldPosition, Vector2D newPosition)
    {
        if (boundaries != null && !boundaries.isEmpty())
        {
            for (Boundary boundary : boundaries)
            {
                // this move would cross a boundary
                if (moveCrossesGeometry(boundary.getBoundingBox(), boundary.getGeometry(),
                    oldPosition, newPosition))
                {
                    logger.trace(
                        "NumericIntegrator.validateMove(), move intersects boundary, oldPosition="
                            + oldPosition + ", newPosition=" + newPosition);
                    pedestrian.getMentalModel().setNeedsOrientation(true);
                    return oldPosition;
                }
            }
        }

        return newPosition;
    }

    /**
     * Checks, whether a move from {@code oldPosition} to {@code newPosition} crosses the
     * {@link Geometry} {@code geometry}. A quick check is performed using {@code boundingBox}
     * before an accurate check.
     *
     * @param boundingBox the bounding box of the given {@code geometry}
     * @param geometry the geometry of a {@link Boundary}
     * @param oldPosition current position of the {@link Pedestrian}
     * @param newPosition next position of the {@link Pedestrian}, after the current time step
     *
     * @return {@code true}, if a move from {@code oldPosition} to {@code newPosition} intersects
     *         with the {@link Geometry} {@code geometry}, {@code false} otherwise
     */
    public static boolean moveCrossesGeometry(Envelope boundingBox, Geometry geometry,
        Vector2D oldPosition, Vector2D newPosition)
    {
        return moveRelatesGeometry(boundingBox, geometry, oldPosition, newPosition, CROSSES);
    }

    /**
     * Checks, whether a move from {@code oldPosition} to {@code newPosition} intersects with the
     * {@link Geometry} {@code geometry}. A quick check is performed using {@code boundingBox}
     * before an accurate check.
     *
     * @param boundingBox the bounding box of the given {@code geometry}
     * @param geometry the geometry of a {@link Boundary}
     * @param oldPosition current position of the {@link Pedestrian}
     * @param newPosition next position of the {@link Pedestrian}, after the current time step
     *
     * @return {@code true}, if a move from {@code oldPosition} to {@code newPosition} intersects
     *         with the {@link Geometry} {@code geometry}, {@code false} otherwise
     */
    public static boolean moveIntersectsGeometry(Envelope boundingBox, Geometry geometry,
        Vector2D oldPosition, Vector2D newPosition)
    {
        return moveRelatesGeometry(boundingBox, geometry, oldPosition, newPosition, INTERSECTS);
    }

    /**
     * Checks, whether a move from {@code oldPosition} to {@code newPosition} relates with the
     * {@link Geometry} {@code geometry}. A quick check is performed using {@code boundingBox}
     * before an accurate check.
     *
     * @param boundingBox the bounding box of the given {@code geometry}
     * @param geometry the geometry of a {@link Boundary}
     * @param oldPosition current position of the {@link Pedestrian}
     * @param newPosition next position of the {@link Pedestrian}, after the current time step
     * @param mode {@link #INTERSECTS} for testing intersection {@link #CROSSES} for testing
     *            crossing
     *
     * @return {@code true}, if a move from {@code oldPosition} to {@code newPosition} intersects
     *         with the {@link Geometry} {@code geometry}, {@code false} otherwise
     */
    private static boolean moveRelatesGeometry(Envelope boundingBox, Geometry geometry,
        Vector2D oldPosition, Vector2D newPosition, byte mode)
    {
        if ( !(GeometryTools.intersects(oldPosition, newPosition, boundingBox)))
            return false;
        LineString path = JTSFactoryFinder.getGeometryFactory()
            .createLineString(new Coordinate[] { new Coordinate(oldPosition.x(), oldPosition.y()),
                new Coordinate(newPosition.x(), newPosition.y()) });

        switch (mode)
        {
            case CROSSES:
                return path.crosses(geometry);
            case INTERSECTS:
                return path.intersects(geometry);
            default:
                return path.intersects(geometry);
        }
    }

}

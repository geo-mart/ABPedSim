package de.fhg.ivi.crowdsimulation.simulation.objects;

import java.awt.Shape;
import java.util.ArrayList;
import java.util.List;

import org.geotools.geometry.jts.JTSFactoryFinder;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.Point;

import de.fhg.ivi.crowdsimulation.simulation.forcemodel.ForceModel;
import de.fhg.ivi.crowdsimulation.simulation.forcemodel.HelbingModel;
import de.fhg.ivi.crowdsimulation.simulation.mentalmodel.FollowWayPointsMentalModel;
import de.fhg.ivi.crowdsimulation.simulation.mentalmodel.WayFindingModel;
import de.fhg.ivi.crowdsimulation.simulation.tools.GeometryTools;

/**
 * A {@link WayPoint} is represented through a {@link Coordinate} on a 2-dimensional level.
 * <p>
 * The {@link WayPoint}s are the target points of the {@link Pedestrian} movement. The sequence in
 * which the {@link WayPoint}s are loaded, which is represented through the {@link #id}, describes
 * also the sequence in which the pedestrian will pass all {@link WayPoint}s.
 *
 * @author hahmann/meinert
 */
public class WayPoint extends Coordinate
{
    /**
     * Uses the object logger for printing specific messages in the console.
     */
    private static final Logger logger                    = LoggerFactory.getLogger(WayPoint.class);

    /**
     * default serial version ID
     */
    private static final long   serialVersionUID          = 1L;

    /**
     * The "radius" which determines the half of the length of the perpendicular line computed for
     * {@link WayPoint#getWayPointVertical()}.
     * <p>
     * This denotes also the threshold in which the currentDestinationWayPoint of a
     * {@link Pedestrian} is set as visited.
     */
    public static final float   radiusOfPerpendicularLine = 4.0f;

    /**
     * The ratio for computing the extended "radius" of the perpendicular line
     */
    public static final float   extensionRatio            = 1.2f;

    /**
     * {@link Geometry} which stores vertical lines which goes through {@code wayPoints}.
     */
    private Geometry            wayPointVertical;

    /**
     * {@link Geometry} which stores vertical lines which goes through {@code wayPoints} extended by
     * {@link #extensionRatio}
     */
    private Geometry            extendedWayPointVertical;

    /**
     * Defines the size of a buffer, which will be computed around the
     * {@link #extendedWayPointVertical}. The size gets its value from
     * {@link FollowWayPointsMentalModel#searchResolutionAlternativePosition}, because this
     * describes the interval on a {@link WayPoint#wayPointVertical} where alternative
     * {@link WayPoint}s for the {@link Pedestrian} were computed.
     * <p>
     * It's used for, that {@link Pedestrian} can check off their {@link WayPoint} also in the near
     * of the {@link WayPoint} instead only directly on the {@link #wayPointVertical}. Given in
     * meters.
     */
    private double              bufferSizeOfVertical      = FollowWayPointsMentalModel.searchResolutionAlternativePosition;

    /**
     * The Bounding Box of {@link #wayPointVertical}
     */
    private Envelope            boundingBox;

    /**
     * {@link LineString} which stores the connecting line to a previous {@link WayPoint}.
     * <p>
     * The background of this is, that the {@link Pedestrian} could use a reference line to get back
     * to the route to the next {@link WayPoint}.
     */
    private LineString          axisBetweenWayPoints;

    /**
     * The length of the {@link #axisBetweenWayPoints}.
     */
    private float               axisLength;

    /**
     * Denotes a specific number for a {@link WayPoint} which distinguishes them from other
     * {@link WayPoint} objects.
     */
    private int                 id;

    /**
     * Creates a new {@link WayPoint}, which inherits from {@link Coordinate}.
     * <p>
     * Otherwise the constructor hands over the passed variables to global ones and creates
     * {@link Shape}s for the {@link WayPoint}, the {@link #wayPointVertical}, the
     * {@link #axisBetweenWayPoints} and a circle around the {@link WayPoint}.
     *
     * @param c {@link Coordinate} of the {@link WayPoint}
     * @param lastWayPointCoordinate the {@link Coordinate} of the last {@link WayPoint} before this
     *            {@link WayPoint}
     * @param id of this {@link WayPoint}
     * @param unionOfAllBoundaries merged {@link Geometry} of all {@link Boundary} objects
     * @param forceModel object of the {@link ForceModel}
     * @param hasAxis {@code true} if this {@link WayPoint} has an axis to the previous
     *            {@link WayPoint}
     */
    public WayPoint(Coordinate c, Coordinate lastWayPointCoordinate, int id,
        Geometry unionOfAllBoundaries, ForceModel forceModel, boolean hasAxis)
    {
        super(c);

        this.id = id;
        createCompleteWayPoint(lastWayPointCoordinate, id, unionOfAllBoundaries, forceModel,
            hasAxis);
        this.boundingBox = extendedWayPointVertical.getEnvelopeInternal();

        if (axisBetweenWayPoints != null)
            axisLength = (float) axisBetweenWayPoints.getLength();

        logger.trace("Waypoint(), " + "testlog");
    }

    /**
     * Gets the {@link #radiusOfPerpendicularLine}.
     *
     * @return the {@link #radiusOfPerpendicularLine}
     */
    public static float getRadiusOfPerpendicularLine()
    {
        return radiusOfPerpendicularLine;
    }

    /**
     * Gets the {@link Geometry} of the {@link #wayPointVertical}.
     *
     * @return the {@link #wayPointVertical}
     */
    public Geometry getWayPointVertical()
    {
        return wayPointVertical;
    }

    /**
     * Gets the {@link Geometry} of the {@link #extendedWayPointVertical}.
     *
     * @return the {@link #extendedWayPointVertical}
     */
    public Geometry getExtendedWayPointVertical()
    {
        return extendedWayPointVertical;
    }

    /**
     * Gets the Bounding Box of {@link #wayPointVertical}
     *
     * @return he Bounding Box of {@link #wayPointVertical}
     */
    public Envelope getBoundingBox()
    {
        return boundingBox;
    }

    /**
     * Gets a {@link LineString}, which represents the {@link #axisBetweenWayPoints}.
     *
     * @return the {@link #axisBetweenWayPoints}
     */
    public LineString getAxisBetweenWayPoints()
    {
        return axisBetweenWayPoints;
    }

    /**
     * Gets the length of the {@link #axisBetweenWayPoints}.
     *
     * @return the length of the {@link #axisBetweenWayPoints}
     */
    public float getAxisLength()
    {
        return axisLength;
    }

    /**
     * Gets the Id of this {@link WayPoint}.
     *
     * @return the id of this {@link WayPoint}
     */
    public int getId()
    {
        return id;
    }

    /**
     * Helper method to create a new {@link WayPoint} object including vertical and axis. These
     * objects are necessary for the way finding algorithm, which is implemented in the
     * {@link WayFindingModel} respectively {@link FollowWayPointsMentalModel}.
     *
     * @param lastWayPointCoordinate the {@link Coordinate} of the last {@link WayPoint} before this
     *            {@link WayPoint}
     * @param i the pointer into the {@link List} {@code coordinateList}
     * @param unionOfAllBoundaries merged {@link Geometry} of all {@link Boundary} objects
     * @param forceModel object of the {@link ForceModel}
     * @param hasAxis {@code true} if this {@link WayPoint} has an axis to the previous
     *            {@link WayPoint}
     */
    private void createCompleteWayPoint(Coordinate lastWayPointCoordinate, int i,
        Geometry unionOfAllBoundaries, ForceModel forceModel, boolean hasAxis)
    {
        // part for calculation of the vertical
        wayPointVertical = computeWayPointVertical(lastWayPointCoordinate, this,
            unionOfAllBoundaries, forceModel, false);
        extendedWayPointVertical = computeWayPointVertical(lastWayPointCoordinate, this,
            unionOfAllBoundaries, forceModel, true);

        // part for calculation of the axis
        if (hasAxis)
        {
            axisBetweenWayPoints = computeAxisBetweenWayPoints(this, lastWayPointCoordinate);
        }
        else
        {
            axisBetweenWayPoints = null;
        }
    }

    /**
     * Computes a perpendicular line {@link Geometry} at the position of the {@code wayPoints},
     * which is stored in the {@link ArrayList} {@code verticalLines}.
     * <p>
     * This method also tests whether there are no perpendicular lines which are far away from the
     * {@code wayPoints}.
     *
     * @param currentWayPoint contains the {@link Coordinate} of the current {@link WayPoint}
     * @param nextWayPoint contains the {@link Coordinate} of the next {@link WayPoint} after the
     *            {@code currentWayPoint}
     * @param extend if {@code true} an extended version of the vertical is computed
     * @param unionOfAllBoundaries merged {@link Geometry} of all {@link Boundary} objects
     * @param forceModel object of the {@link ForceModel}
     *
     * @return the {@link Geometry} of a vertical through every {@link WayPoint} in dependence of
     *         the location of {@link WayPoint}s lying side by side
     */
    private Geometry computeWayPointVertical(Coordinate currentWayPoint, Coordinate nextWayPoint,
        Geometry unionOfAllBoundaries, ForceModel forceModel, boolean extend)
    {
        double width = extend ? radiusOfPerpendicularLine * extensionRatio
            : radiusOfPerpendicularLine;

        // calculates vertical with nextWayPoint and currentWayPoint + the predefined width
        Geometry perpendicularLine = GeometryTools.getPerpendicularLine(nextWayPoint,
            currentWayPoint, width);

        // enlarge extended waypoint verticals so that the waypoint can be marked as crossed even
        // before the actual waypoint vertical is crossed
        if (extend)
            perpendicularLine = perpendicularLine.buffer(bufferSizeOfVertical);

        // proofs whether perpendicular line intersects boundaries
        if (unionOfAllBoundaries != null)
        {
            // use buffer so that target points on WayPointVerticals that are not too close to
            // boundaries
            double boundaryBuffer = 0;
            if (forceModel instanceof HelbingModel)
            {
                // allow an arm's length space to the boundaries
                boundaryBuffer = 2 * ((HelbingModel) forceModel).getPedestrianRadius();
            }
            if ( !extend && boundaryBuffer > 0)
                perpendicularLine = perpendicularLine
                    .difference(unionOfAllBoundaries.buffer(boundaryBuffer));
            else
                perpendicularLine = perpendicularLine.difference(unionOfAllBoundaries);
        }
        return perpendicularLine;
    }

    /**
     * If the simulation is initialized or there are any changes which affect the location of the
     * {@link WayPoint}, the {@code axisBetweenTwoPoints} will calculated.
     * <p>
     * For this the line between two {@link WayPoint} which lying next to each other is computed and
     * stored in a {@link List} of {@link LineString}.
     *
     * @param currentWayPoint contains the {@link Coordinate} of the current {@link WayPoint}
     * @param otherWayPoint contains the {@link Coordinate} of the next {@link WayPoint} after the
     *            {@code currentWayPoint}
     *
     * @return a {@link LineString}, which is the axis between two {@link WayPoint} lying next to
     *         each other.
     */
    private LineString computeAxisBetweenWayPoints(Coordinate currentWayPoint,
        Coordinate otherWayPoint)
    {
        GeometryFactory factory = JTSFactoryFinder.getGeometryFactory();

        Point firstPoint = factory
            .createPoint(new Coordinate(currentWayPoint.x, currentWayPoint.y));
        Point lastPoint = factory.createPoint(new Coordinate(otherWayPoint.x, otherWayPoint.y));

        Coordinate[] coordinates = new Coordinate[] {
            new Coordinate(firstPoint.getX(), firstPoint.getY()),
            new Coordinate(lastPoint.getX(), lastPoint.getY()) };

        LineString lineString = JTSFactoryFinder.getGeometryFactory().createLineString(coordinates);

        return lineString;
    }

    /**
     * Gets this WayPoint as a {@link Geometry} object
     *
     * @return the {@link Geometry} representation of this {@link WayPoint}
     */
    public Geometry toGeometry()
    {
        return GeometryTools.coordinateToPoint(this);
    }
}

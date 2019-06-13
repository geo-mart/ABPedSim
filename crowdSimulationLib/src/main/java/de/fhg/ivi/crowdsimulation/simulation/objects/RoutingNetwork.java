package de.fhg.ivi.crowdsimulation.simulation.objects;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;

import org.geotools.feature.FeatureCollection;
import org.geotools.feature.FeatureIterator;
import org.geotools.graph.build.feature.FeatureGraphGenerator;
import org.geotools.graph.build.line.LineStringGraphGenerator;
import org.geotools.graph.path.DijkstraShortestPathFinder;
import org.geotools.graph.path.Path;
import org.geotools.graph.structure.Edge;
import org.geotools.graph.structure.Graph;
import org.geotools.graph.structure.Node;
import org.geotools.graph.traverse.standard.DijkstraIterator;
import org.geotools.graph.traverse.standard.DijkstraIterator.EdgeWeighter;
import org.opengis.feature.simple.SimpleFeature;
import org.opengis.feature.simple.SimpleFeatureType;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.io.ParseException;
import com.vividsolutions.jts.io.WKTReader;

import de.fhg.ivi.crowdsimulation.simulation.forcemodel.ForceModel;
import de.fhg.ivi.crowdsimulation.simulation.tools.GeometryTools;
import hcu.csl.agentbasedmodeling.KdTree;

/**
 * Class for handling a network for routing between {@link WayPoint}s of the {@link Pedestrian}s
 * After the network is created, the {@link Pedestrian}s can use the
 * {@link DijkstraShortestPathFinder} to get the shortest path to the target node.
 *
 * Methods to find nearest neighbor node of the {@link Pedestrian} and converting the calculated
 * path to a list of {@link WayPoint}s are also implemented.
 *
 * @author Martin Knura
 *
 */
public class RoutingNetwork
{

    /**
     * {@link Graph}, imported class from geotools package
     */
    private Graph                      networkGraph;

    /**
     * given input data is a LineString-shp, so the {@link Graph} is generated from LineStrings
     */
    private LineStringGraphGenerator   lineStringGen;

    /**
     * GraphGenerator, implementing the {@link LineStringGraphGenerator} data
     */
    private FeatureGraphGenerator      featureGen;

    /**
     * {@link DijkstraShortestPathFinder} for the shortest path calculation with the algorithm of
     * Dijkstra
     */
    private DijkstraShortestPathFinder shortestDijkstraPathFinder;

    // private AStarShortestPathFinder shortestAStarPathFinder;

    /**
     * Defines the weighting for calculating {@link DijkstraShortestPathFinder} as the length of the
     * egde
     */
    private EdgeWeighter               weighter;

    /**
     * List of {@link Point}s representing all nodes of the {@link Graph} for calculating the
     * nearest neighbor from current pedestrian position
     */
    private ArrayList<Point>           pointList;

    /**
     * {@link KdTree} for fastest search of nearest neighbor node
     */
    private KdTree<Point>              kdTree;

    /**
     * Initializing and calculating the {@link Graph} from the input LineStrings. Setting list of
     * all nodes as {@link Point}s using their Coordinates, initializing a new kdTree with all nodes
     * for fast search.
     *
     * @param fCollection input LineStrings as FeatureCollection
     */
    public RoutingNetwork(FeatureCollection<SimpleFeatureType, SimpleFeature> fCollection)
    {

        this.networkGraph = generateGraph(fCollection);
        // this.shortestDijkstraPathFinder = new DijkstraShortestPathFinder(graph, start, weighter);
        this.setDistanceEdgeWeighter();
        this.setPointList();
        this.kdTree = new KdTree<>(this.pointList);

    }

    /**
     * Setting start node and calculating.
     *
     * @param start {@link Node} from which the route starts
     */
    public void setDijkstraPathFinder(Node start)
    {

        this.shortestDijkstraPathFinder = new DijkstraShortestPathFinder(this.networkGraph, start,
            this.weighter);
        shortestDijkstraPathFinder.calculate();

    }

    /**
     * Setting distance as criteria for calculating shortest path.
     */
    public void setDistanceEdgeWeighter()
    {
        this.weighter = new DijkstraIterator.EdgeWeighter()
            {
                @Override
                public double getWeight(Edge e)
                {
                    SimpleFeature feature = (SimpleFeature) e.getObject();
                    Geometry geometry = (Geometry) feature.getDefaultGeometry();
                    return geometry.getLength();
                }
            };

    }

    /**
     * Creating a list with all {@link Point}s for nearest neighbor searches. Due to
     * {@link LineStringGraphGenerator} data, the {@link Coordinates}s are only stored as the
     * toString()-representation of the nodes.
     */
    public void setPointList()
    {
        Collection<Node> nodesForList = this.networkGraph.getNodes();
        this.pointList = new ArrayList<>();
        Iterator<Node> featuresForList = nodesForList.iterator();
        WKTReader readerWKT = new WKTReader();

        while (featuresForList.hasNext())
        {
            String geometry = featuresForList.next().getObject().toString();
            featuresForList.next().toString();

            try
            {
                Geometry nextGeom = readerWKT.read(geometry);
                if (nextGeom.getGeometryType().equalsIgnoreCase("Point"))
                {
                    Coordinate c = new Coordinate(nextGeom.getCoordinate().x,
                        (nextGeom.getCoordinate().y));
                    Point p = GeometryTools.coordinateToPoint(c);
                    this.pointList.add(p);
                }

            }
            catch (ParseException ex)
            {
                // Logger.error("loadGeometriesFromCsvFile(), ", ex);
            }
        }

        // System.out.println("#: " + pointList.size());
        // System.out.println("0: " + pointList.get(0).toString());

    }

    /**
     * Fast search of nearest neighbor by using the kdTree.
     *
     * @param neighborsToFind int value of the number of neighbors to find
     * @param p {@link Point} representation of the input value
     *
     * @return {@link Collection <Point>} of the nearest neighbors
     */
    public Collection<Point> nearestNeighbor(int neighborsToFind, Point p)
    {
        return this.kdTree.nearestNeighbourSearch(neighborsToFind, p);

    }

    /**
     * Calculating the shortest path with {@link DijkstraShortestPathFinder}.
     *
     * @param start {@link Node} to start from
     * @param destination target {@link Node}
     * @return {@link Path} as a list of {@link Node} IDs
     */
    public Path calculatePath(Node start, Node destination)
    {

        DijkstraShortestPathFinder pathFinder = new DijkstraShortestPathFinder(this.networkGraph,
            start, this.weighter);
        pathFinder.calculate();
        return pathFinder.getPath(destination);

    }

    /**
     * Adding features from collection to {@link FeatureGraphGenerator} and initializes
     * {@link Graph} object.
     *
     * @param fCollection input data from LineString shp file
     * @return {@link Graph} object
     */
    public Graph generateGraph(FeatureCollection<SimpleFeatureType, SimpleFeature> fCollection)
    {
        this.lineStringGen = new LineStringGraphGenerator();
        this.featureGen = new FeatureGraphGenerator(lineStringGen);

        try (FeatureIterator<SimpleFeature> features = fCollection.features())
        {
            while (features.hasNext())
            {
                SimpleFeature feature = features.next();
                featureGen.add(feature);
            }
        }

        return featureGen.getGraph();

    }

    /**
     * Getting the node at given {@link Coordinate}.
     *
     * @param c {@link Coordinate} of the node to get
     * @return {@link Node}
     */
    public Node getNode(Coordinate c)
    {

        return this.lineStringGen.getNode(c);

    }

    /**
     * @return {@link Graph} Object
     */
    public Graph getGraph()
    {
        return this.networkGraph;
    }

    /**
     * Creating a list of {@link WayPoint}s by converting the path nodes back to {@link Point}
     * objects and initializing the new {@link WayPoint}s
     *
     * @param path {@link Path} object to convert into list of {@link WayPoint}s
     * @param pointBefore last {@link Point} before starting point of the path in pedestrians
     *            {@link WayPoint} list
     * @param j index
     * @param boundaries {@link Geometry} of all boundaries
     * @param forcemodel
     * @return list of {@link WayPoint}s
     */
    public List<WayPoint> pathToWayPointList(Path path, Point pointBefore, int j,
        Geometry boundaries, ForceModel forcemodel)
    {

        List<WayPoint> generatedWaypoints = new ArrayList<>();

        WKTReader readerWKT = new WKTReader();
        Iterator<Node> pathiterator = path.iterator();
        List<Point> nodes = new ArrayList<>();

        while (pathiterator.hasNext())
        {
            Node pathelement = pathiterator.next();
            String geometry = pathelement.getObject().toString();
            try
            {
                Geometry nextGeom = readerWKT.read(geometry);
                Geometry nextGeomJava = GeometryTools.transformGeometry(nextGeom);

                if (nextGeomJava.getGeometryType().equalsIgnoreCase("Point"))
                {
                    Point p = GeometryTools.coordinateToPoint(nextGeomJava.getCoordinate());
                    nodes.add(0, p);
                    // System.out.println("GEO: " + p.toString());
                }
            }
            catch (ParseException ex)
            {
                // Logger.error("loadGeometriesFromCsvFile(), ", ex);
            }

        }

        for (int i = 0; i < nodes.size(); i++ )
        {

            if (i == 0)
            {
                WayPoint wayPoint = new WayPoint(nodes.get(i).getCoordinate(),
                    pointBefore.getCoordinate(), j + i, boundaries, forcemodel, false);
                generatedWaypoints.add(wayPoint);
            }
            else
            {
                WayPoint wayPoint = new WayPoint(nodes.get(i).getCoordinate(),
                    nodes.get(i - 1).getCoordinate(), j + i, boundaries, forcemodel, true);
                generatedWaypoints.add(wayPoint);
            }
        }

        return generatedWaypoints;

    }

}

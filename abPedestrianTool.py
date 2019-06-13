#!/usr/bin/python

import sys # for command line args handling
import subprocess
import os
import time
import random
import csv
import pyproj # for transformations

import fiona
from shapely.geometry import *

from autobahn.asyncio.component import Component
from autobahn.asyncio.component import run

#Layer zur Übergabe in die Simulation
#crowd = "tourist-group.shp"
boundaries = "data/boundaries.shp"  # shapefile with the boundaries of buildings, obstacles etc. as polygons
boundaries_cut = "data/boundaries_cut.shp"
gencrowd="data/pedestrians.shp"

# Layer mit ÖPNV-Haltestellen
bahn="data/subahn.shp"  # shapefile with point data of subway stations
bus="data/bus.shp"   # shapefile with point data of bus stations
rad="data/stadtrad.shp"   # shapefile with point data of bike stations
park="data/parkhaus.shp"   # shapefile with point data of parking lots
gateways="data/gateways.shp"

# Globale Default-Variablen
bbox_minx = float(565900.000)
bbox_miny = float(5933766.750)
bbox_maxx = float(566173.405)
bbox_maxy = float(5933998.937)
peds=30
target= 0
        
parameterSet = False     


def defineBorders(buildings, bbox):
    """Defines the borders of the simulation.

    Keyword arguments:
    buildings -- list of with buildings
    bbox -- shape of the bbox
    """
    boundariesInBBox = []
    for building in buildings:
        if (shape(building['geometry']).intersects(bbox.buffer(50))):
            boundariesInBBox.append(building)
    return boundariesInBBox


def findStations(bahnStations, busStations, radStations, parkStations, bbox, boundariesInBBox):
    """Search for all Gateways in given bbox, but not in buildings.

    Keyword arguments:
    bahnStations -- list of U- & S-Bahn stations
    busStations -- list of bus stations
    radStations -- list of stadtrad stations
    parkStations -- list of car parks
    bbox -- shape of the bbox
    boundariesInBBox -- Boundaries in bbox
    """
    
    stationsInBBox = []

    for feature in bahnStations:
        if (shape(feature['geometry']).intersects(bbox) ):
            stationsInBBox.append((feature, "Bahn"))
    for feature in busStations:
        if (shape(feature['geometry']).intersects(bbox)):
            stationsInBBox.append((feature, "Bus"))
    for feature in radStations:
        if (shape(feature['geometry']).intersects(bbox)):
            stationsInBBox.append((feature, "Rad"))
    for feature in parkStations:
        if (shape(feature['geometry']).intersects(bbox)):
            stationsInBBox.append((feature, "Auto"))

    for gebaeude in boundariesInBBox:
        for station in stationsInBBox:
                if (shape(station[0]['geometry']).intersects(shape(gebaeude['geometry']))):
                    stationsInBBox.remove(station)

    if (len(stationsInBBox) < 5):
        stationsInBBox = findStations(bahnStations, busStations, radStations, parkStations, bbox.buffer(100), boundariesInBBox)
    
    return stationsInBBox


# Faktorisierung: Wie werden die Fußgänger auf die einzelnen Verkehrsmittel verteilt?
def factoring(bahnPeds, busPeds, radPeds, autoPeds):
    """returns the probability to get a type of gateway.

    Keyword arguments:
    bahnPeds -- list of U- & S-Bahn stations in bbox
    busPeds -- list of bus stations in bbox
    radPeds -- list of stadtrad stations in bbox
    parkPeds -- list of car parks in bbox
    """
    bahnFactor = 0
    busFactor = 0
    radFactor = 0
    autoFactor = 0
        
    if len(bahnPeds) > 0:
        if len(busPeds) > 0:
            if len(radPeds) > 0:
                if len(autoPeds) > 0:
                    bahnFactor = 0.55
                    busFactor = 0.25
                    radFactor = 0.1
                    autoFactor = 0.1
                else:
                    bahnFactor = 0.6
                    busFactor = 0.3
                    radFactor = 0.1
            else:
                if len(autoPeds) > 0:
                    bahnFactor = 0.6
                    busFactor = 0.3
                    autoFactor = 0.1
                else:
                    bahnFactor = 0.7
                    busFactor = 0.3
        else:
            if len(radPeds) > 0:
                if len(autoPeds) > 0:
                    bahnFactor = 0.8
                    radFactor = 0.1
                    autoFactor = 0.1
                else:
                    bahnFactor = 0.9
                    radFactor = 0.1
            else:
                if len(autoPeds) > 0:
                    bahnFactor = 0.9
                    autoFactor = 0.1
                else:
                    bahnFactor = 1
    else:
        if len(busPeds) > 0:
            if len(radPeds) > 0:
                if len(autoPeds) > 0:
                    busFactor = 0.70
                    radFactor = 0.15
                    autoFactor = 0.15
                else:
                    busFactor = 0.75
                    radFactor = 0.25
            else:
                if len(autoPeds) > 0:
                    busFactor = 0.5
                    autoFactor = 0.5
                else:
                    busFactor = 1
        else:
            if len(radPeds) > 0:
                if len(autoPeds) > 0:
                    radFactor = 0.4
                    autoFactor = 0.6
                else:
                    radFactor = 1
            else:
                autoFactor = 1

    return bahnFactor, busFactor, radFactor, autoFactor


# Zufallspunkte um die Gateways herum   
def randomPoint(point, startPositions):
    """creates a random point around a given point with minimum distance to all points in list.

    Keyword arguments:
    point -- coordinates of the point
    startPositions -- list of points
    """
    
    x = random.uniform(point.x-5,point.x+5)
    y = random.uniform(point.y-5,point.y+5)
    p = Point(x,y)
    print(p)
    
    
    # damit Fußgänger nicht zu nah beieinander starten
    for pts in startPositions:
        a = Point(pts)
        if p.distance(a) < 0.3 :
            randomPoint(point, startPositions)
        else:
            return p 
    return p



def execute(session, command):
    """executes a given command in cmd

    Keyword arguments:
    session -- ongoing WAMP session
    command -- String of commandline agrs to execute
    """
    
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, encoding='utf8')

    timeStart = time.time()
    # Poll process for new output until finished
    while True:
        nextline = process.stdout.readline()
        if nextline == '' and process.poll() is not None:
            break
        
        #print(nextline)
        # Erstes Zeichen L
        if nextline != '' and nextline[0] == "L":
            lineStringJSON = nextline[1:]
            print(lineStringJSON)
            session.publish(u'<session name for LineString Visualization>', str(lineStringJSON))
            
        else:
        
            if (((time.time() - timeStart) % 0.25 ) <= 0.01):
                print(nextline)
                session.publish(u'<session name for Point Visualization>', str(nextline))

        sys.stdout.flush()

    output = process.communicate()[0]
    exitCode = process.returncode

    if (exitCode == 0):
        return output
    else:
        raise ProcessException(command, exitCode, output)


def defineStartVariables(bbox_minx, bbox_miny, bbox_maxx, bbox_maxy, peds, target):
    """defines the shp files to start the simulation.

    Keyword arguments:
    bbox_minx, bbox_miny, bbox_maxx, bbox_maxy -- extent of bbox
    peds -- number of pedestrians to simulate
    target -- coordinates of target point to reach
    """
    
    # get Boundary
    fileNameInput = boundaries
    fileNameOutput = boundaries_cut

    input_bbox = box(bbox_minx, bbox_miny, bbox_maxx, bbox_maxy)
    bbox = shape(input_bbox)

    with fiona.open(fileNameInput) as source:
        buildings = list(source)
    
    boundariesInBBox = defineBorders(buildings, bbox)
    

    schema = {
        'geometry': 'Polygon',
        'properties': {"id": "int"},
    }
    with fiona.open(fileNameOutput, 'w', 'ESRI Shapefile', schema) as c:
        i = 0
        for feature in boundariesInBBox:
            c.write({
                'geometry': mapping(shape(feature['geometry'])),
                'properties': {"id": i},
            })
            i=i+1

    #get Gateways
    fileNameInputBahn = bahn
    fileNameInputBus = bus
    fileNameInputStadtrad = rad
    fileNameInputParkhaus = park
    fileNameGateways = gateways

    # loading input data
    with fiona.open(fileNameInputBahn) as source1:
        bahnStations = list(source1)
    with fiona.open(fileNameInputBus) as source2:
        busStations = list(source2)
    with fiona.open(fileNameInputStadtrad) as source3:
        radStations = list(source3)
    with fiona.open(fileNameInputParkhaus) as source4:
        parkStations = list(source4)    

    stationsInBBox = findStations(bahnStations, busStations, radStations, parkStations, bbox, boundariesInBBox)
                    
    schema = {
        'geometry': 'Point',
        'properties': {"id": "int", "transport": "str"},
    }
    with fiona.open(fileNameGateways, 'w', 'ESRI Shapefile', schema) as c:
        i = 0
        for feature in stationsInBBox:
            c.write({
                'geometry': mapping(shape(feature[0]['geometry'])),
                'properties': {"id": i, "transport": feature[1]},
            })
            i=i+1

    # get Pedestrians
    fileOutputCrowd = gencrowd

    with fiona.open(fileNameGateways) as source:
        gatewaysList = list(source)
            
    busPeds = []
    bahnPeds = []
    radPeds = []
    autoPeds = []

    for feature in gatewaysList:
            if (feature['properties']['transport'] == "Bahn"):
                    bahnPeds.append(feature['geometry'])
            elif (feature['properties']['transport'] == "Bus"):
                    busPeds.append(feature['geometry'])
            elif (feature['properties']['transport'] == "Rad"):
                    radPeds.append(feature['geometry'])
            elif (feature['properties']['transport'] == "Auto"):
                    autoPeds.append(feature['geometry'])

    bahnFactor, busFactor, radFactor, autoFactor = factoring(bahnPeds, busPeds, radPeds, autoPeds)

    startPositions = []

    for i in range(peds):
        x= random.random()
        if (x < bahnFactor):
            pPos = random.randint(0, len(bahnPeds)-1)
            p = randomPoint(shape(bahnPeds[pPos]), startPositions)
            startPositions.append(p)
        elif (x < (bahnFactor+busFactor)):
            pPos = random.randint(0, len(busPeds)-1)
            p = randomPoint(shape(busPeds[pPos]), startPositions)
            startPositions.append(p)
        elif (x < (bahnFactor+busFactor+radFactor)):
            pPos = random.randint(0, len(radPeds)-1)
            p = randomPoint(shape(radPeds[pPos]), startPositions)
            startPositions.append(p)
        else:
            pPos = random.randint(0, len(autoPeds)-1)
            p = randomPoint(shape(autoPeds[pPos]), startPositions)
            startPositions.append(p)

    schema = {
        'geometry': 'Point',
        'properties': {"id": "int"},
    }

    with fiona.open(fileOutputCrowd, 'w', 'ESRI Shapefile', schema) as c:
        i = 0
        for feature in startPositions:
            c.write({
                'geometry': mapping(shape(feature)),
                'properties': {"id": i},
            })
            i=i+1   

    csvfile = "data/pedMission.csv"

    with open(csvfile, "w") as output:
        writer = csv.writer(output, lineterminator='\n')
        writer.writerow(['startWKT', 'mentalModel', 'wktWayPoints'])
        for val in startPositions:
            if target == 0:
                generatedMission = '[' + startPositions[random.randint(0, len(startPositions)-1)].wkt + ', ' + startPositions[random.randint(0, len(startPositions)-1)].wkt + ', ' + startPositions[random.randint(0, len(startPositions)-1)].wkt + ', ' + startPositions[random.randint(0, len(startPositions)-1)].wkt  +  ']'
            else:
                generatedMission = '[ POINT (' + str(target[0]) + ' '+ str(target[1])+') ]'

            writer.writerow([val.wkt, 'FollowWayPointsMentalModel', generatedMission])


comp = Component(
    transports=u"ws://..port../ws",
    realm=u"...realm...",
)

@comp.on_join
async def joined(session, details):
    print("session ready")     
    
    
    def startSimulationTarget(x):
        """Processing CityScope data. If global variable parameterSet is true, start simulation.
        """
        
        global parameterSet
        source_prj_pyproj = pyproj.Proj(init='epsg:3857')
        target_prj_pyproj = pyproj.Proj(init='epsg:25832')
        e, n = pyproj.transform(source_prj_pyproj, target_prj_pyproj, x[0], x[1])
        print(str(e) + ' ' + str(n))
        target = (e, n)
        print(parameterSet)
        if parameterSet == True:
            defineStartVariables(bbox_minx, bbox_miny, bbox_maxx, bbox_maxy, peds, target)
            startARGS = " data/boundaries_cut.shp data/pedestrians.shp data/pedMission.csv data/networkForGraphN.shp"
            command = "java -jar abpedsim.jar" + startARGS
            execute(session, command)
    
    def startSimulation(extend, ped, type):
        """processing CityScope data. If type == 0, starts the simulation, else set global variable parameterSet=true.

        Keyword arguments:
        extent -- extent of bbox
        ped -- number of pedestrians to simulate
        type -- type of simulation (0=standard mode, 1=target mode)
        """
        
        global parameterSet
        
        print("event received: " + str(extend) + " " + str(ped)+ " " + str(type))
        
        bbox_minx = float(extend[0])
        bbox_miny = float(extend[1])
        bbox_maxx = float(extend[2])
        bbox_maxy = float(extend[3])

        peds = ped
        
        if type == 0:
            defineStartVariables(bbox_minx, bbox_miny, bbox_maxx, bbox_maxy, peds, target)
            startARGS = " data/boundaries_cut.shp data/pedestrians.shp data/pedMission.csv data/networkForGraphN.shp"
            command = "java -jar abpedsim.jar" + startARGS
            execute(session, command)
        else:
            parameterSet = True
            print("Parameter set " +str(parameterSet))

        
    try:
        await session.subscribe(startSimulation, u'<session name for ABPedSim Start>')
        print("subscribed to topic: <session name for ABPedSim Start>")
    except Exception as e:
        print("could not subscribe to topic: {0}".format(e))
    
    try:
        await session.subscribe(startSimulationTarget, u'<session name for rcToXy Conversation>')
        print("subscribed to topic: rcToXy")
    except Exception as e:
        print("could not subscribe to topic: {0}".format(e))
   

if __name__ == "__main__":
    run([comp])


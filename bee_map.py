"""
Defines read_map(mapName,map) that reads in the map
"""

def read_beesoft_map(mapName):
    fp = open("../data/map/"+mapName,"r")
    if not fp:
        print "Error: Could not open",mapName
        return -1
    print "Reading map",mapName
    # Read metadata
    l = fp.readline()
    global_mapsize_x = int(l.rpartition(" ")[-1])
    l = fp.readline()
    global_mapsize_y = int(l.rpartition(" ")[-1])
    l = fp.readline()
    resolution = int(l.rpartition(" ")[-1])
    l =fp.readline()
    autoshifted_x = int(l.rpartition(" ")[-1])
    l =fp.readline()
    autoshifted_y = int(l.rpartition(" ")[-1])
    #print global_mapsize_x,global_mapsize_y,resolution,autoshifted_x,autoshifted_y
    l = fp.readline()
    l = fp.readline()
    size_y = int((l.partition(" ")[-1]).partition(" ")[-1])
    size_x = int(l.rpartition(" ")[-1])
    print "Map size",size_x,size_y

    # create map
    map = []
    for x in range(size_x):
        row = []
        l = fp.readline().rsplit(" ")[:-1]
        for y in range(size_y):
            prob = float(l[y])
            row.append(prob)
        map.append(row)
    return ([size_x,size_y],[global_mapsize_x,global_mapsize_y,resolution,autoshifted_x,autoshifted_y], map)

if __name__ =="__main__":
    read_beesoft_map("wean.dat")

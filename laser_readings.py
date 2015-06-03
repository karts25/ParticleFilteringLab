# misc utils for particle filter
import math

"""
returns laser beam as measured
"""
one_degree = math.radians(1)

"""
define a laser model
"""
p_hit = 0.95
sigma_hit = 0.2 # variance for measurement noise (Gaussian)
lambda_short = 0.8 # exp decay rate for p(short readings) 
p_max = 0.02
p_rand = 0.01
p_short = 0.05
z_max = 81.830 # from the data
z_max_eps = 200

def raytrace(pose,angle,measurement):
    theta_beam = pose[2] + angle
    ray = []
    d = 0
    cos_theta = math.cos(theta_beam)
    sin_theta = math.sin(theta_beam)
    while d <= measurement:
        x = pose[0] + d*cos_theta
        y = pose[1] + d*sin_theta
        ray.append((x,y))
        d += 20
    return ray

"""
returns ground truth observations using raytrace
"""

def groundtruth(pose,mapReader,graphicsHandler = None):
    d = 0.0
    theta = -math.pi/2
    true_measurements = []
    resolution = mapReader.map_params[2]
    while theta <= math.pi/2:
        d = 0
        cos_theta = math.cos(pose[2]+theta)
        sin_theta = math.sin(pose[2]+theta)
        while d <= z_max*100:
            x = pose[0] + d*cos_theta
            y = pose[1] + d*sin_theta
            #graphicsHandler.draw_point((x,y),'red',resolution)
            if x/resolution >= mapReader.mapsize[0] or y/resolution >= mapReader.mapsize[1] or mapReader.read(x,y) <= 0.3:
                break
            elif mapReader.read(x,y) == -1:
                d = -1 # denotes unknown space
                break
            d += 20
        #graphicsHandler.canvas.update_idletasks()
        #raw_input("drew one beam")
        true_measurements.append(d)
        theta += one_degree
    #graphicsHandler.canvas.update_idletasks()
    return true_measurements

def probability_reading(z,groundtruth):
    p = 1
    for i in range(len(z)):
        print "z",z[i],"groundtruth",groundtruth[i]
        p_beam = math.pow(probability_beam(z[i],groundtruth[i]),1.0/90.0)
        print "p_beam",p_beam
        p = p*p_beam
    #print "total p",p
    return p

def probability_beam(z,zexp):
    """
    Use beam range sensor model
    """
    z = z/100.0
    zexp = zexp/100.0
    # if we have a reading into an unknown region, just give that probability 1
    if z == -1 or zexp == -1:
        return 1
    # find p_hit
    b = sigma_hit*sigma_hit
    p = p_hit*(1/(math.sqrt(2*math.pi*b))) * math.exp(-0.5*(z-zexp)*(z-zexp)/b)
    print "p_hit",p
    # add random measurement
    p += p_rand*1/z_max
    print "+rand",p
    if z < zexp: # unexpected obstacles
        p += p_short*lambda_short*math.pow(math.e,-lambda_short*z)
        print "p_hit + short",p
    if z >= (z_max - z_max_eps)/100: # max range
        p += p_max*1/(z_max_eps/100)
    return p

if __name__ == "__main__":
    print "z 50,zexp 50",
    print probability_beam(50,50)
    print "z 321,zexp 260",
    print probability_beam(321,260)
    print "z 360 zexp 340",
    print probability_beam(360,340)
    print "z 40, zexp 50",
    print probability_beam(40,50)
    print "z 8123 zexp 50",
    print probability_beam(8123,50)

#!/usr/bin/python
"""
Implement particle filter to localize the bot
"""
import bee_map
import read_log
import sys,random
import graphics,laser_readings
import math
from Tkinter import *

one_degree = math.radians(1)

class ParticleFilter:
    def __init__(self,logfile,mapfile):
        self.logStreamer = read_log.LogStreamer(logfile)
        (mapsize,map_params,mymap) = bee_map.read_beesoft_map(mapfile)
        self.MapReader = MapReader(mapsize,map_params,mymap)
        self.N = 5 # number of particles
        self.particles = self.init_particles()
        # initialize graphics
        self.root = Tk()
        self.graphicsHandler = graphics.Graphics(self.root)
        self.graphicsHandler.draw_map(self.MapReader.map,self.MapReader.mapsize,self.MapReader.map_params[2])
        self.last_odom = None
        
    def init_particles(self):
        # initialize N particles in empty region
        particles = []
        weight = 1.0/self.N
        while len(particles) < self.N:
            #newparticle = [self.MapReader.mapsize[0]*self.MapReader.map_params[2]*random.random(),self.MapReader.mapsize[1]*self.MapReader.map_params[2]*random.random(),2*math.pi*random.random()]
            #newparticle = [4000 + 0.5*self.MapReader.mapsize[0]*self.MapReader.map_params[2]*random.random(),5000+0.2*self.MapReader.mapsize[1]*self.MapReader.map_params[2]*random.random(),2*math.pi*random.random()]
            newparticle = [4000 + 0.5*self.MapReader.mapsize[0]*self.MapReader.map_params[2]*random.random(),self.MapReader.mapsize[1]*self\
.MapReader.map_params[2]*random.random(),2*math.pi*random.random()]
            if(self.MapReader.read(newparticle[0],newparticle[1]))==1:
                particles.append([newparticle,weight])
        print "finished initializing"
        return particles

    def motion_update(self,reading):
        if self.last_odom == None:
            self.last_odom = reading
        old = self.last_odom
        new = reading
        [alpha1,alpha2,alpha3,alpha4] = [0.001,0.0005,0.0005,0.001]#[0.005,0.001,0.001,0.005]#[0.1,0.1,0.1,0.1]
        d_rot1 = math.atan2(new[1] - old[1],new[0]-old[0]) - old[2]
        d_trans = math.sqrt( (old[0] - new[0])*(old[0] - new[0]) + (old[1]-new[1])*(old[1]-new[1]))
        d_rot2 = new[2] - old[2] - d_rot1

        d_rot1_new = d_rot1 - random.gauss(0,alpha1*d_rot1 + alpha2*d_trans)
        d_trans_new = d_trans - random.gauss(0,alpha3*d_trans + alpha4*(d_rot1 + d_rot2))
        d_rot2_new  = d_rot2 - random.gauss(0,alpha1*d_rot2 + alpha2*d_trans)
        
        for particle,weight in self.particles:
            particle[0] = particle[0] + d_trans_new*math.cos(particle[2] + d_rot1_new)
            particle[1] = particle[1] + d_trans_new*math.sin(particle[2] + d_rot1_new)
            particle[2] = particle[2] + d_rot1_new + d_rot2_new
        self.last_odom = reading

    """
    def motion_update(self,reading):
        if self.last_odom == None:
            self.last_odom = reading
        incr = [new - old for new,old in zip(reading,self.last_odom)]
        print "incr is",incr
        for particle,weight in self.particles:
            costheta = math.cos(particle[2])
            sintheta = math.sin(particle[2])
            particle[0] = particle[0] + incr[0]*costheta - incr[1]*sintheta
            particle[1] = particle[1] + incr[1]*costheta + incr[0]*sintheta
            particle[2] = (particle[2] + incr[2])%(2*math.pi) 
        self.last_odom = reading
    """    

    def sensor_update(self,reading):
        print "in sensor update"
        pose_car_odom =  reading[0:3]
        self.motion_update(pose_car_odom)
        pose_laser_car = [laser-car for (car,laser) in zip(pose_car_odom,reading[3:6])]
        Z = 0 # normalizing factor
        count = 1
        for item in self.particles:
            #print "particle number",count
            particle = item[0]
            x_laser_global = particle[0] + pose_laser_car[0]*math.cos(particle[2]) - pose_laser_car[1]*math.sin(particle[2])
            y_laser_global = particle[1] + pose_laser_car[1]*math.cos(particle[2]) + pose_laser_car[0]*math.sin(particle[2])
            theta_laser_global = (particle[2] + pose_laser_car[2])%(2*math.pi)
            if self.MapReader.read(x_laser_global,y_laser_global) == 0:
                item[1] = 0.0
                continue

            beam_angle = -math.pi/2
            """
            if count == 5:
                handles = []
            
                for measurement in reading[6:-1]:
                    ray = laser_readings.raytrace((x_laser_global,y_laser_global,theta_laser_global),beam_angle,measurement)
                    handles += self.graphicsHandler.draw_points(ray,'red',self.MapReader.map_params[2])
                    beam_angle += one_degree
            """    
            groundtruth = laser_readings.groundtruth((x_laser_global,y_laser_global,theta_laser_global),self.MapReader,self.graphicsHandler)
            prob_reading = laser_readings.probability_reading(reading[6:-1],groundtruth)
            #prob_reading = math.exp(prob_reading)
            """
            if count == 5:
                print "p = ",prob_reading
                raw_input()xs
                self.graphicsHandler.erase_points(handles)
            """
            item[1] = prob_reading
            Z += prob_reading
            count += 1
            #self.graphicsHandler.erase_points(handles)
        # normalize weights
        for item in self.particles:
            item[1] = item[1]/Z
        print "done sensor update"
        #raw_input("hit enter")

    def resample(self):
        newparticles = []
        # arrange particles linearly
        cumparticles = []
        cumparticles.append(self.particles[0][1])
        weightsofar = self.particles[0][1]

        for i in range(1,self.N):
            weightsofar+=self.particles[i][1]
            cumparticles.append(weightsofar)
        
        # low variance resampler
        N_resample = int(self.N)
        r = float(random.random()*(1.0/N_resample))
        incr = float(1.0/N_resample)
        p = 0
        p_old = 0
        new_weight = 1.0/self.N
        while len(newparticles) < N_resample:
            for p in range(p_old,len(cumparticles)):
                #print "r",r,"cumparticles",cumparticles[p]
                if r <= cumparticles[p]:
                    pose = list(self.particles[p][0])
                    if p == p_old:
                        # we don't want same particle, so add noise
                        pose[0] += (random.random()*5.0 - 10)
                        pose[1] += (random.random()*5.0 - 10)
                        pose[2] = (pose[2] + random.random()*(math.pi/8))% (2*math.pi)
                    newparticles.append([pose,new_weight])
                    p_old = p
                    break
            r += incr
        #print "some particles"
        #for i in range(self.N):
        #    print "old",self.particles[i],"new",newparticles[i]
        print "resampled particles",len(newparticles)
        while (len(newparticles) < self.N):
            newparticle = [4000 + 0.5*self.MapReader.mapsize[0]*self.MapReader.map_params[2]*random.random(),5000+0.2*self.MapReader.mapsize[1]*self.MapReader.map_params[2]*random.random(),2*math.pi*random.random()]
            if(self.MapReader.read(newparticle[0],newparticle[1]))==1:
                newparticles.append([newparticle,new_weight])
        print "total particles",len(newparticles)
        self.particles = newparticles

    def test_sensor(self,reading,point):
        self.graphicsHandler.draw_point(point[0:2],'red',self.MapReader.map_params[2])
        raw_input("hit enter to draw reading")
        (x_laser_global,y_laser_global,theta_laser_global) = point
        handles = []
        beam_angle = -math.pi/2
        for measurement in reading[6:-1]:                                                                                                                                  
            ray = laser_readings.raytrace((x_laser_global,y_laser_global,theta_laser_global),beam_angle,measurement)                                                         
            handles += self.graphicsHandler.draw_points(ray,'red',self.MapReader.map_params[2])                                                                              
            beam_angle += one_degree
        raw_input("hit enter to draw ground truth")
        self.graphicsHandler.erase_points(handles)
        groundtruth = laser_readings.groundtruth(point,self.MapReader,self.graphicsHandler)
        print "len groundtruth",len(groundtruth)
        if True or self.firsttime:
            beam_angle = -math.pi/2
            for measurement in groundtruth:
                ray = laser_readings.raytrace((x_laser_global,y_laser_global,theta_laser_global),beam_angle,measurement)
                handles += self.graphicsHandler.draw_points(ray,'red',self.MapReader.map_params[2])
                beam_angle += one_degree
            raw_input("hit enter to print p")
            self.graphicsHandler.erase_points(handles)
            self.firsttime = False
        prob_reading = laser_readings.probability_reading(reading[6:-1],groundtruth)
        print prob_reading

    def run(self):
        print "in mainloop"
        # Get next line from log
        reading = self.logStreamer.getNext()
        raw_input("ready to go")
        i = 1
        self.firsttime = True
        lasercount = 0
        self.particles = [[[4000,4100,3*math.pi/2],1]]
        while reading:
            print "iteration",i
            #if i == 100:
            #    raw_input("hit enter")
            self.graphicsHandler.draw_particles(self.particles,'red',self.MapReader.map_params[2])
            #self.particles = [[[4000,4100,3*math.pi/2],1]]
            if reading[0] == 'O':
                self.motion_update(reading[1:])
            #elif reading[0] == 'L':
            #    self.sensor_update(reading[1:])
            #    self.resample()
            
            """
            if reading[0] == 'L' and  i >= 500:
                index = int(random.random()*self.N)
                #self.test_sensor(reading[1:],[4000,4100,3*math.pi/2])
                #self.test_sensor(reading[1:],[6600,6100,3*math.pi/2])
                self.test_sensor(reading[1:],[4700,4000,0.0])
            if reading[0] == 'L':
                lasercount += 1
            """
            reading = self.logStreamer.getNext()
            i += 1
            
        print "done"
        self.root.mainloop()

class MapReader:
    def __init__(self,mapsize,map_params,mymap):
        self.mapsize = mapsize
        self.map_params = map_params
        self.map = mymap
    
    def read(self,x,y):
        resolution = self.map_params[2]
        #print "checking map at",(x,y),"value is",self.map[int(x/resolution)][int(y/resolution)]
        return self.map[int(x/resolution)][int(y/resolution)]
    
def test_resample(myPF):
    myPF.N = 5
    myPF.particles = [[(100,100,0),0.3],[(20,20,0),0.5],[(200,200,0),0.01],[(500,500,0),0.09],[(600,600,0),0.1]]
    print "\n\n old particles\n",myPF.particles
    myPF.resample()
    print myPF.particles

def main(argv):
    logfile = argv[1]
    mapfile = argv[2]
    myPF = ParticleFilter(logfile,mapfile)
    #test_resample(myPF)
    myPF.run()

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print "Usage: python particlefilter.py <logfile> <mapfile>"
        exit(0)
    main(sys.argv)

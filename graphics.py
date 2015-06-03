"""
Handles visualization using Tkinter
"""

from Tkinter import *

class Graphics:
    def __init__(self,root):
        self.root = root
        self.root.title("Particle Filtering")
        self.currentparticles = None
        
    def draw_map(self,mymap,mapsize,map_resolution):
        size_x,size_y = mapsize
        self.canvas = Canvas(width=size_x, height=size_y, bg='white')
        self.canvas.pack(expand=YES, fill=BOTH)
        self.map = mymap
        self.mapsize = mapsize
        for x in range(size_y):
            row = mymap[x]
            for y in range(300,size_x-50):
                val = row[y]
                color = 'blue' if val == -1 else ("#%02x%02x%02x" % (val*255, val*255, val*255))
                self.draw_point((x*map_resolution,y*map_resolution),color,map_resolution)
        self.canvas.update_idletasks()

    def draw_point(self,point,color,map_resolution):
        x,y = point
        x = x/map_resolution
        y = self.mapsize[1] - y/map_resolution
        handle = self.canvas.create_rectangle(x,y,x,y,outline=color, fill=color)
        return handle

    def draw_points(self,points,color,map_resolution):
        handles = []
        for point in points:
            handles.append(self.draw_point(point,color,map_resolution))
        self.canvas.update_idletasks()
        return handles

    def erase_points(self,handles):
        for handle in handles:
            self.canvas.delete(handle)
        self.canvas.update_idletasks()

    def draw_particles(self,particles,color,map_resolution):
        #erase old particles
        if self.currentparticles!=None:
            for particle in self.currentparticles:
                self.canvas.delete(particle)
        # draw new particles
        newparticles = []
        for particle in particles:
            x,y,theta = particle[0]
            newparticles.append(self.draw_point((x,y),color,map_resolution))
        self.currentparticles = newparticles
        self.canvas.update_idletasks()
        #raw_input("drew particles")
        

if __name__ == "__main__":
    root = Tk()
    graphicsHandler = Graphics(root)
    graphicsHandler.draw_map([],[800,800],10)
    root.mainloop()

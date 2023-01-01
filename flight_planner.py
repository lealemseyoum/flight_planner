#!/bin/python3

import math
import numpy as np
import json

class NumpyEncoder(json.JSONEncoder):
    def default(self,obj):
        if isinstance(obj,np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self,obj)


class Camera:
    def __init__(self,**kwargs):
        """
        Initializes camera object with a minimal pin-hole camera model.
        
        Parameters:
            f:              focal length/principal distance
            image_size:     image size in number of pixels (x,y)
            sensor_size:    sensor size in mm  (x,y)
            pixel_size:     size of each pixel in metric uint
            fov:            field of view (hfov, vfov)
            dfov:           diagonal field of view   
        """
        self._parameters = kwargs
        
        if(self._parameters.get('sensor_size') == None):
            sensor_size = self._parameters.get('pixel_size')*np.array(self._parameters.get('image_size'))
            self._parameters['sensor_size'] = sensor_size

    @property
    def pixel_size(self):
        return self._parameters.get('pixel_size',None)

    @property
    def image_size(self):
        return self._parameters.get('image_size',None)

    @property
    def sensor_size(self):
        return self._parameters.get('sensor_size',None)

    @property
    def fov(self):
        return self._parameters.get('fov',None)

    @property
    def dfov(self):
        return self._parameters.get('dfov',None)


    @property
    def focal_length(self):
        return self._parameters.get('f',None)

    @property
    def name(self):
        return self._parameters.get('name',None)
    
    @property
    def max_fps(self):
        return self._parameters.get('max_fps',None)

    @max_fps.setter
    def fps(self,fps):
        self._parameters['fps'] = fps 

    @property
    def exposure(self):
        return self._parameters.get('exposure',None)
    
    @exposure.setter
    def exposure(self,exposure):
        fps = self._parameters.get('max_fps')
        if (fps != None) & (exposure < 1/fps): 
            self._parameters['exposure'] = exposure 
        elif fps == None:
            self.max_fps = 1/exposure
            self._parameters['exposure'] = exposure 
        else: 
            print("Exposure exceeds maximum fps")

            

        self._parameters['exposure'] = exposure

    @name.setter
    def name(self,name):
        self._parameters['name'] = name

    @focal_length.setter
    def focal_length(self,f):
        self._parameters['f'] = f

    @pixel_size.setter
    def pixel_size(self,pixel_size):
        self._parameters['pixel_size'] = pixel_size
        

    @image_size.setter
    def image_size(self,image_size):
        self._parameters['image_size'] = image_size
        

    @sensor_size.setter
    def sensor_size(self,sensor_size):
        self._parameters['sensor_size'] = sensor_size

    @fov.setter
    def fov(self,fov):
        self._parameters['fov'] = fov

    @dfov.setter
    def dfov(self,dfov):
        self._parameters['dfov'] = dfov


        

class FlightPlanner:
    def __init__(self,camera,**kwargs):
        """
        Initializes flighth planner object.

        Parameters:
            camera:     Camera object
            forward:    forward overlap between images in %
            side:       side overlap between images in %
        """
        self._cam = camera
        self._plan = kwargs

    def calculate_photo_area(self):
        area = self._plan.get('swath')[0] * self._plan.get('swath')[1]
        self._plan["ground_area"] = area

    def calculate_base(self):
        """
        Computes base distance between successive shots (m)
        """
        B = self._plan.get('swath')[1] * (1-(self._plan.get('forward_overlap')/100))
        self._plan['base_length'] = B    

    def calculate_strip_offset(self):
        """
        Computes offset between adjacent strips (m)
        """
        A = self._plan.get('swath')[0] * (1-(self._plan.get('side_overlap')/100))
        self._plan['strip_offset'] = A

    def scale_from_height(self,h):
        m = h/(self._cam.focal_length/1000)
        self._plan['photo_scale'] = m
        S = self._plan.get('photo_scale') * self._cam.sensor_size
        self._plan['swath'] = S

    def scale_from_gsd(self,gsd):
        S = (gsd/100)*self._cam.image_size
        m = S[0]/self._cam.sensor_size[0]
        self._plan['photo_scale'] = m
        self._plan['swath'] = S
    

    def calculate_gsd(self):
        S = self._plan.get('photo_scale')*self._cam.sensor_size[0]
        gsd = 100*S/self._cam.image_size[0]
        self._plan['gsd'] = gsd
        

    def calculate_height(self,gsd):
        h = self._plan.get('photo_scale')  *self._cam.focal_length/1000
        self._plan['height'] = h

    def calculate_blur(self,speed):
        """
        @TODO 
        Calculates pixel blur based on platform speed.

        Parameters:
            speed:  platform speed (m/s)
        """
        pass

    def compute(self):
        """
        Computes all flight parameters. 

        Parameters:

        Returns:
            plan (dictionary):   Flight parameters
        """

        # forward and side overlap are mandatory for the flight plan

        if(self._plan.get('forward_overlap') == None):
            self._plan['forward_overlap'] = 60

        if (self._plan.get('side_overlap') == None):
            self._plan['side_overlap'] = 40

        h = self._plan.get("height")     
        gsd = self._plan.get("gsd")


        if(h == None and gsd == None):
            #use default values, maximum height for A2 class flying
            h = 120
            self.gsd_from_height(h)
   
        elif (h == None):
            # gsd is given, estimate h
            self.scale_from_gsd(gsd)
            self.calculate_height(gsd)   
        
        elif (gsd == None):
            # h is given, estimate gsd
            self.scale_from_height(h)
            self.calculate_gsd()
        else:
            # both h and gsd are given, do sanity check
            pass
        
        self.calculate_base()
        self.calculate_strip_offset()
        self.calculate_photo_area()

    def write(self,format="yaml"):
        """
        Writes flight parameters in JSON format.

        Parameters:
            format:     Serializtion format

        """
        data = [self._cam.__dict__,self._plan]
        with open(self._cam.name + ".json","w") as jsonfile:
            json.dump(data, jsonfile,indent=4,cls=NumpyEncoder)

    @property
    def height(self):
        return self._plan.get('height',None)
    
    @height.setter
    def height(self,h):
        self._plan['height'] = h
        self.compute()

    @property
    def gsd(self):
        return self._plan.get("gsd",None)
    
    @gsd.setter
    def gsd(self,gsd):
        self._plan['gsd'] = gsd
        self.compute()

    @property 
    def side_overlap(self):
        return self._plan.get("side_overlap",None)

    @side_overlap.setter
    def side_overlap(self,so):
        self._plan['side_overlap'] = so
        self.compute()

    @property
    def forward_overlap(self):
        return self._plan.get("forward_overlap",None)

    @forward_overlap.setter
    def forward_overlap(self,fo):
        self._plan['forward_overlap'] = fo
        self.compute()

    
def main():
    pass

if __name__=="__main__":
    main()
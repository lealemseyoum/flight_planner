#!/bin/python3

import math
import numpy as np
import json

class NumpyEncoder(json.JSONEncoder):
    def default(self,obj):
        if isinstance(obj,np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self,obj)


"""
Collection of Photogrammetric flight planning functions.

Symbols:
    A   - Distance between flight lines/strips          (m)
    B   - Base distance between consecutive images      (m)
    h   - Flying height abobe ground                    (m)
    Z   - Ground height                                 (m)
    v   - Flying speed over ground                      (m/s)
    L   - Length of a strip/block                       (m)
    Q   - Width of block                                (m)
    m   - Photo scale number                            (unitless)
    f   - focal length/principal distance               (px)
    s   - image width                                   (px)
    S   - swath (image width on the gruond)             (m)
    cx  - x-coordinate of principal point               (px)
    cy  - y-coordinate of principal point

"""


def photo_scale_number(h,f ):
    return h/f


def image_side_ground(h,c,s):
    return s * photo_scale_number(h,c)

def base_in_photo(h,c,B):
    return B/photo_scale_number(h,c)

def flying_height_agl(c,m_b):
    return c*m_b

def absolute_flying_height(h,Z):
    return h+Z

def forward_overlap(h,c,s,B):
    m_b = photo_scale_number(h,c)
    S = image_side_ground(h,c,s)
    return (1-(B/S))*100

def side_overlap(h,c,s,A):
    S = image_side_ground(h,c,s)
    return (1-(A/S))*100

def photo_ground_area(h,c,s):
    m_b = photo_scale_number(h,c)
    return s^2 * m_b^2

def base_length_():
    pass

def strip_distance():
    pass

def models_in_stip():
    pass

def photogs_in_strip():
    pass

def strips_in_block():
    pass

def stereoscopic_model_area():
    pass

def time_interval():
    pass


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
    flir = Camera(f=3.98,pixel_size = 3.75e-6,image_size=(800,600),name="Flir")
    senop = Camera(f=3.28, pixel_size = 5.5e-6, image_size=(1024,1024),name="Senop")

    flir_plan = FlightPlanner(flir,height = 1.0,side_overlap=60,forward_overlap=80)
    senop_plan = FlightPlanner(senop,height =1.5, side_overlap=60, forward_overlap=80)

    flir_plan.compute()
    senop_plan.compute()

    print(flir_plan._cam.name,flir_plan._plan)
    print(senop_plan._cam.name,senop_plan._plan)

    flir_plan.write()
    senop_plan.write()
if __name__=="__main__":
    main()
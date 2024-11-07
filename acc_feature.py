#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys
import numpy as np
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error
from agents.navigation.constant_velocity_agent import ConstantVelocityAgent  # pylint: disable=import-error



import carla
import pygame
import random
import time
pygame.init()
pygame.font.init()
font=pygame.font.Font(None,36)
display = pygame.display.set_mode((800, 600), pygame.HWSURFACE | pygame.DOUBLEBUF)
pygame.display.set_caption("CARLA RGB Camera with LDW")
 

class HUD: 
    def __init__(self):
        self.font = pygame.font.Font(None, 36)
        self.speed = 0
        self.distance_traveled = 0
        self.start_time = None
        self.start_location =  carla.Location(x=44.055626, y=-60.723831, z=0.600000)
        self.lane_marks=None
        self.distance_gap=None
    def render(self, diautosplay):
        speed_surface = self.font.render(f"Speed: {self.speed:.2f} km/h", True, (0, 0, 0))
        display.blit(speed_surface, (10, 50))
        distance_surface = self.font.render(f"Distance: {self.distance_traveled:.2f} m", True, (0, 0, 0))
        display.blit(distance_surface, (10, 90))
        distance_gap_surface = self.font.render(f"Distance Gap: {self.distance_gap:.2f} m", True, (0, 0, 0))
        display.blit(distance_gap_surface, (10, 120))
        if self.lane_marks!=None:

            text_surface=self.font.render(self.lane_marks,True,(0,0,0))
            display.blit(text_surface,(0,0))
            lane_marks=None

def processing_image(image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    display.blit(surface,(0,0))
    


def main():
    actor_list = []

    # In this tutorial script, we are going to add a vehicle to the simulation
    # and let it drive in autopilot. We will also create a camera attached to
    # that vehicle, and save all the images generated by the camera to disk.
   
    try:
        # First of all, we need to create the client that will send the requests
        # to the simulator. Here we'll assume the simulator is accepting
        # requests in the localhost at port 2000.
        client = carla.Client('localhost', 2000)
        #client.start_recorder("/home/kpit/Desktop/KSIL/recording01.log")
        client.set_timeout(10.0)

        # Once we have a client we can retrieve the world that is currently
        # running.
        world = client.get_world()

        # The world contains the list blueprints that we can use for adding new
        # actors into the simulation.
        blueprint_library = world.get_blueprint_library()

        # Now let's filter all the blueprints of type 'vehicle' and choose one
        # at random.
        bp = random.choice(blueprint_library.filter('vehicle'))

        # A blueprint contains the list of attributes that define a vehicle's
        # instance, we can read them and modify some of them. For instance,
        # let's randomize its color.
        if bp.has_attribute('color'):
            color = random.choice(bp.get_attribute('color').recommended_values)
            bp.set_attribute('color', color)

        # Now we need to give an initial transform to the vehicle. We choose a
        # random transform from the list of recommended spawn points of the map.
        transform = random.choice(world.get_map().get_spawn_points())
        print(transform)
        spawn_points=world.get_map().get_spawn_points()
      
        # So let's tell the world to spawn the vehicle.
        vehicle = world.spawn_actor(bp, transform)

        # It is important to note that the actors we create won't be destroyed
        # unless we call their "destroy" function. If we fail to call "destroy"
        # they will stay in the simulation even after we quit the Python script.
        # For that reason, we are storing all the actors we create so we can
        # destroy them afterwards.
        actor_list.append(vehicle)

        print('created %s' % vehicle.type_id)

        
        transform = random.choice(world.get_map().get_spawn_points())
        destination_=carla.Location(x=-0.764156, y=24.613132, z=0.600000)
        agent=BasicAgent(vehicle,30)
        agent.set_destination(destination_)

        # Let's add now a "depth" camera attached to the vehicle. Note that the
        # transform we give here is now relative to the vehicle.
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_transform = carla.Transform(carla.Location(x=-5.5, z=2.4))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
        actor_list.append(camera)
        print('created %s' % camera.type_id)

        # Now we register the function that will be called each time the sensor
        # receives an image. In this example we are saving the image to disk
        # converting the pixels to gray-scale.
        cc = carla.ColorConverter.LogarithmicDepth
        camera.listen(processing_image)

        # Oh wait, I don't like the location we gave to the vehicle, I'm going
        # to move it a bit forward.
        

        # But the city now is probably quite empty, let's add a few more
        # vehicles.
   
        
        

        time.sleep(5)

        while True: 
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return 
            if agent.done():
                print("agent its destination")
                break
            controller=agent.run_step() 
            vehicle.apply_control(controller)
            pygame.display.flip()
       

    finally:

        print('destroying actors')
        #camera.destroy()
        for actor in actor_list:
            actor.destroy()
        print('done.')



if __name__ == '__main__':
    
    main()
    

import glob
import os
import sys
import time
import random
import pygame
import numpy as np
import math 

try:
   
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass



from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error
from agents.navigation.constant_velocity_agent import ConstantVelocityAgent  # pylint: disable=import-error

import carla
from carla import ColorConverter as cc

 
# Initialize Pygame
pygame.init()
pygame.font.init()
font=pygame.font.Font(None,36)
display = pygame.display.set_mode((800, 600), pygame.HWSURFACE | pygame.DOUBLEBUF)
pygame.display.set_caption("CARLA RGB Camera with ACC")



def display_text(text,x,y):
    text_surface=font.render(text,True,(255,0,0))
    display.blit(text_surface,(x,y))

class Hud: 
    def __init__(self): 
        self.ego_vehicle_speed=0.0 
        self.distance_to_target_vehicle=0.0 
        self.ego_vehicle_start_location=None
        self.target_vehicle_speed=0

    def update_ego_vehicle_speed(self,v):
        self.ego_vehicle_speed=(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))
    def update_target_vehicle_speed(self,v):
        self.target_vehicle_speed=(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))
    def Update_ego_vehicle_distance_to_target_vehicle(self, ego_vehicle,target_vehicle): 
       
        self.distance_to_target_vehicle=ego_vehicle.distance(target_vehicle)
    def render(self):
       
        text='Ego_vehicle_Speed   :   % 6.0f km/h' % self.ego_vehicle_speed
        ego_vehicle_speed=font.render(text,True,(255,255,255))
        display.blit(ego_vehicle_speed,(0,0))
        text='Target_vehicle_Speed:   % 6.0f km/h' % self.target_vehicle_speed
        target_vehicle_speed=font.render(text,True,(255,255,255))
        display.blit(target_vehicle_speed,(0,30))
        text='Target_vehicle_distance:   % 3.0f meters' % self.distance_to_target_vehicle
        distance_to_target=font.render(text,True,(255,255,255))
        display.blit(distance_to_target,(0,60))




def process_image(image):
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    display.blit(surface, (0, 0))
    

def main():
    
    actor_list=[]
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(60.0)
        world = client.get_world()
        hud=Hud()
        blueprint_library = world.get_blueprint_library()
        bp = random.choice(blueprint_library.filter('vehicle.audi.a2'))
        #adding the color to vehicle
        if bp.has_attribute('color'):
            color = random.choice(bp.get_attribute('color').recommended_values)
            print(color)
            bp.set_attribute('color', '255,0,0')
            
        
        #adding the vehicle in simulation with transform 
        #transform = carla.Transform(carla.Location(x=44.055626, y=-60.723831, z=0.600000), carla.Rotation(pitch=0.000000, yaw=-0.023438, roll=0.000000))
        #transform = random.choice(world.get_map().get_spawn_points())
        #transform=carla.Transform(carla.Location(x=266.282227, y=-169.140869, z=0.600000), carla.Rotation(pitch=0.000000, yaw=0.326125, roll=0.000000))
        #print(transform)

        #transform=carla.Transform(carla.Location(x=335.745270, y=10.223274, z=0.541373), carla.Rotation(pitch=0.000000, yaw=0.326125, roll=0.000000))
        #transform=carla.Transform(carla.Location(x=270.511414, y=-117.624405, z=0.481942), carla.Rotation(pitch=0.000000, yaw=0.924043, roll=0.000000))
        #transform=carla.Transform(carla.Location(x=-84.932762, y=16.474657, z=0.600000), carla.Rotation(pitch=0.000000, yaw=-179.840790, roll=0.000000))
        transform=carla.Transform(carla.Location(x=-504.859375, y=200.032288, z=0.281942), carla.Rotation(pitch=0.000000, yaw=89.866272, roll=0.000000))
        vehicle = world.spawn_actor(bp, transform)
        actor_list.append(vehicle)
        camera_bp = blueprint_library.find('sensor.camera.rgb')
            #position of camera 
        camera_transform = carla.Transform(carla.Location(x=-5.5, z=2.5))

        #adding camera to vehicle and placing into the simulation world
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
        camera.listen(lambda image: process_image(image))

        actor_list.append(camera)

        agent = BasicAgent(vehicle, 56)
        #agent.follow_speed_limits(True)
        #spawn_points = world.get_map().get_spawn_points()
        #destination =random.choice(world.get_map().get_spawn_points()).location
        
        destination=carla.Location(x=318.508087, y=10.203942, z=0.963416)

        agent.set_destination(destination)
        bp2= random.choice(blueprint_library.filter('vehicle.audi.a2'))

        if bp2.has_attribute('color'):
            color = random.choice(bp.get_attribute('color').recommended_values)
            print(color)
            bp2.set_attribute('color', '0,0,255')
        #transform=carla.Transform(carla.Location(x=290.282227, y=-117.140869, z=0.600000), carla.Rotation(pitch=0.000000, yaw=0.326125, roll=0.000000))
        #transform=carla.Transform(carla.Location(x=310.252289, y=-99.961891, z=-0.002624), carla.Rotation(pitch=0.000000, yaw=0.326125, roll=0.000000))
        #transform=carla.Transform(carla.Location(x=310.238739, y=-89.179764, z=-0.004482), carla.Rotation(pitch=0.200418, yaw=89.399689, roll=-0.056641))
        transform=carla.Transform(carla.Location(x=-504.859375, y=207.032288, z=0.281942), carla.Rotation(pitch=0.000000, yaw=89.866272, roll=0.000000))

        Lead_vehicle = world.spawn_actor(bp2, transform)
        actor_list.append(Lead_vehicle)
        agent2 = BasicAgent(Lead_vehicle,50)
        #agent2.follow_speed_limits(True)
        #destination=random.choice(world.get_map().get_spawn_points()).location
        #destination=carla.Location(x=-504.859375, y=270.032288, z=0.281942)
        agent2.set_destination(destination)


   
        start_time=time.time()
        while True : 
            
            for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return
            current_time=time.time()
            if agent.done() or agent2.done():
               
                print("The target has been reached, stopping the simulation")
                break
            v =vehicle.get_velocity()
            v2=Lead_vehicle.get_velocity()

            #print(vehicle.get_transform())
            hud.update_ego_vehicle_speed(v)
            hud.update_target_vehicle_speed(v2)
            hud.Update_ego_vehicle_distance_to_target_vehicle(vehicle.get_location(),Lead_vehicle.get_location())
            hud.render()
            pygame.display.flip()

            control = agent.run_step()
            control.manual_gear_shift = False
            vehicle.apply_control(control)
          
            control2 = agent2.run_step()
            control2.manual_gear_shift = False
            Lead_vehicle.apply_control(control2)

            world.tick()
    except Exception as e:
        print(e)

    finally:
        for actor in actor_list:
            if actor is not None:
                actor.destroy()


main()
        



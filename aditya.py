import glob
import os
import sys
import random
import pygame
import math
import numpy as np
9502342465
# Add CARLA egg file to the Python path to use CARLA API
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major, sys.version_info.minor, 
'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
    
import carla 

# Initialize Pygame
pygame.init()
display = pygame.display.set_mode((800, 600), pygame.HWSURFACE | pygame.DOUBLEBUF)
pygame.display.set_caption("CARLA ACC Test")

 
class HUD:
    def __init__(self):
        self.font = pygame.font.Font(None, 36)
        self.warning_message = None
        self.warning_time = 0
        self.a_x_for_trq = 0.0
        self.Trq_en = False
        self.Gap_Ctr_Enable = False
        self.Tgt_Dist_Gap = 0.0
        self.Tgt_time_Gap = 0.0
        self.ACC_Set_spd = 0.0
        self.Headway_Gap_set = 0.0
        self.Time_Gap_set = 0.0
        self.Headway_Gap = 0.0
 
    def set_warning(self, message):
        self.warning_message = message
        self.warning_time = pygame.time.get_ticks()
 
    def update_metrics(self, a_x_for_trq, Trq_en, Gap_Ctr_Enable, Tgt_Dist_Gap, Tgt_time_Gap, ACC_Set_spd, Headway_Gap_set, Time_Gap_set, Headway_Gap):
        self.a_x_for_trq = a_x_for_trq
        self.Trq_en = Trq_en
        self.Gap_Ctr_Enable = Gap_Ctr_Enable
        self.Tgt_Dist_Gap = Tgt_Dist_Gap
        self.Tgt_time_Gap = Tgt_time_Gap
        self.ACC_Set_spd = ACC_Set_spd
        self.Headway_Gap_set = Headway_Gap_set
        self.Time_Gap_set = Time_Gap_set
        self.Headway_Gap = Headway_Gap
 
    def render(self, display):
        if self.warning_message:
            current_time = pygame.time.get_ticks()
            if current_time - self.warning_time < 2000:
                warning_surface = self.font.render(self.warning_message, True, (255, 0, 0))
                display.blit(warning_surface, (10, 10))
            else:
                self.warning_message = None
 
        a_x_for_trq_surface = self.font.render(f"a_x_for_trq: {self.a_x_for_trq:.2f} mps2", True, (0, 0, 255))
        Trq_en_surface = self.font.render(f"Trq_en: {self.Trq_en}", True, (0, 0, 255))
        Gap_Ctr_Enable_surface = self.font.render(f"Gap_Ctr_Enable: {self.Gap_Ctr_Enable}", True, (0, 0, 255))
        Tgt_Dist_Gap_surface = self.font.render(f"Tgt_Dist_Gap: {self.Tgt_Dist_Gap:.2f} m", True, (0, 0, 255))
        Tgt_time_Gap_surface = self.font.render(f"Tgt_time_Gap: {self.Tgt_time_Gap:.2f} s", True, (0, 0, 255))
        ACC_Set_spd_surface = self.font.render(f"ACC_Set_spd: {self.ACC_Set_spd:.2f} km/h", True, (0, 0, 255))
        Headway_Gap_set_surface = self.font.render(f"Headway_Gap_set: {self.Headway_Gap_set:.2f} m", True, (0, 0, 255))
        Time_Gap_set_surface = self.font.render(f"Time_Gap_set: {self.Time_Gap_set:.2f} s", True, (0, 0, 255))
        Headway_Gap_surface = self.font.render(f"Headway_Gap: {self.Headway_Gap:.2f} m", True, (0, 0, 255))
 
        display.blit(a_x_for_trq_surface, (10, 50))
        display.blit(Trq_en_surface, (10, 80))
        display.blit(Gap_Ctr_Enable_surface, (10, 110))
        display.blit(Tgt_Dist_Gap_surface, (10, 140))
        display.blit(Tgt_time_Gap_surface, (10, 170))
        display.blit(ACC_Set_spd_surface, (10, 200))
        display.blit(Headway_Gap_set_surface, (10, 230))
        display.blit(Time_Gap_set_surface, (10, 260))
        display.blit(Headway_Gap_surface, (10, 290))
 
def destroy_actors(actor_list):
    for actor in actor_list:
        if actor is not None:
            try:
                if callable(getattr(actor, 'is_alive', None)) and actor.is_alive():
                    actor.destroy()
            except Exception as e:
                print(f"Failed to destroy actor {getattr(actor, 'id', 'unknown')}: {e}")
 
def safe_spawn_actor(blueprint, transform, world):
    try:
        actor = world.spawn_actor(blueprint, transform)
        if actor is None:
            print(f"Failed to spawn actor at {transform.location}.")
        return actor
    except Exception as e:
        print(f"Exception during spawning actor: {e}")
        return None
 
def process_image(image, hud):
    #global hud  # make hud a global variable
    image.convert(carla.ColorConverter.Raw)
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    display.blit(surface, (0, 0))
    hud.render(display)  # use the global hud variable
    pygame.display.flip()
 
def place_target_vehicle_in_front(ego_vehicle, distance, world):
    ego_transform = ego_vehicle.get_transform()
    yaw = ego_transform.rotation.yaw * math.pi / 180.0
    x_offset = distance * math.cos(yaw)
    y_offset = distance * math.sin(yaw)
    target_location = carla.Location(
        x=ego_transform.location.x + x_offset,
        y=ego_transform.location.y + y_offset,
        z=ego_transform.location.z
    )
    target_transform = carla.Transform(
        target_location,
        carla.Rotation(
            pitch=ego_transform.rotation.pitch,
            yaw=ego_transform.rotation.yaw,
            roll=ego_transform.rotation.roll
        )
    )
    return target_transform
 
def autonomous_following(ego_vehicle, target_vehicle, Trgt_Px_m, Trgt_Vx_mps, Trgt_status, Trgt_Spd_mps, Veh_spd_mps, Veh_PathRadius_m, Veh_accel_mps2, SysPwdMd):
    target_location = target_vehicle.get_location()
    ego_location = ego_vehicle.get_location()
    target_velocity = target_vehicle.get_velocity()
    ego_velocity = ego_vehicle.get_velocity()

    if Trgt_status:
        distance = Trgt_Px_m
        desired_distance = 10.0
        k_p_throttle = 1.0
        k_p_brake = 1.0
        k_p_steer = 0.5
        desired_speed = 30.0

        throttle = 0.0
        brake = 0.0
        steer = 0.0

        if distance > desired_distance:
            throttle = min(k_p_throttle * ((distance - desired_distance) / desired_distance), 1.0)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(k_p_brake * ((desired_distance - distance) / desired_distance), 1.0)

        speed_kmh = Veh_spd_mps * 3.6
        target_speed_kmh = Trgt_Vx_mps * 3.6

        # Calculate desired yaw
        target_yaw = math.atan2(target_location.y - ego_location.y, target_location.x - ego_location.x)
        ego_yaw = ego_vehicle.get_transform().rotation.yaw * math.pi / 180.0

        # Normalize angles
        yaw_diff = target_yaw - ego_yaw
        yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi

        # Calculate steering
        steer = max(min(k_p_steer * yaw_diff, 1.0), -1.0)

        control = carla.VehicleControl(throttle=throttle, brake=brake, steer=steer, hand_brake=False)
        ego_vehicle.apply_control(control)

        # Calculate the output signals
        a_x_for_trq = (throttle - brake) * desired_speed
        Trq_en = True if throttle > 0.0 or brake > 0.0 else False
        Gap_Ctr_Enable = True if distance > desired_distance else False
        Tgt_Dist_Gap = desired_distance
        Tgt_time_Gap = desired_distance / desired_speed
        ACC_Set_spd = speed_kmh
        Headway_Gap_set = desired_distance
        Time_Gap_set = desired_distance / desired_speed
        Headway_Gap = distance

        hud.update_metrics(a_x_for_trq, Trq_en, Gap_Ctr_Enable, Tgt_Dist_Gap, Tgt_time_Gap, ACC_Set_spd, Headway_Gap_set, Time_Gap_set, Headway_Gap)
 
def main():
    global hud  # make hud a global variable
    hud = HUD()
    actor_list = []
    hud.distance = 0.0
    prev_location = None
    Trgt_Px_m = 20
    Trgt_Vx_mps = 20/3.6
    Trgt_status = True
    Trgt_Spd_mps = [20/3.6] * 30
    Veh_spd_mps = 0
    Veh_PathRadius_m = 100
    Veh_accel_mps2 = 2
    SysPwdMd = True
    ACC_ECU_CMD = "FOLLOW"
 
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_global_distance_to_leading_vehicle(5)
        
        # Remove all existing vehicles
        all_vehicles = world.get_actors().filter('vehicle.*')
        for vehicle in all_vehicles:
            if vehicle is not None and vehicle.is_alive:
                vehicle.destroy()

        blueprint_library = world.get_blueprint_library()
        car_blueprints = blueprint_library.filter('vehicle.audi.a2*')
 
        # Spawn ego vehicle
        ego_bp = random.choice(car_blueprints)
        ego_transform = random.choice(world.get_map().get_spawn_points())
        ego_vehicle = safe_spawn_actor(ego_bp, ego_transform, world)
        if ego_vehicle:
            actor_list.append(ego_vehicle)
        ego_vehicle.set_autopilot(True, traffic_manager.get_port())
 
        # Add a Depth Map camera to the ego vehicle
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_transform = carla.Transform(carla.Location(x=-5.5, z=2.4))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle)
        if camera:
            actor_list.append(camera)
        camera.listen(lambda image: process_image(image, hud))
 
        # Spawn one target vehicle for ACC testing
        target_transform = place_target_vehicle_in_front(ego_vehicle, 20, world)
        print(f"Spawning target vehicle at: {target_transform.location}")
        target_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
        target_vehicle = safe_spawn_actor(target_bp, target_transform, world)
        if target_vehicle:
            actor_list.append(target_vehicle)
        target_vehicle.set_autopilot(True, traffic_manager.get_port())
 
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt
 
            if ACC_ECU_CMD == "FOLLOW":
                autonomous_following(ego_vehicle, target_vehicle, Trgt_Px_m, Trgt_Vx_mps, Trgt_status, Trgt_Spd_mps, Veh_spd_mps, Veh_PathRadius_m, Veh_accel_mps2, SysPwdMd)  

            # Update metrics for HUD
            current_location = ego_vehicle.get_location()
            if prev_location:
                distance_traveled = current_location.distance(prev_location)
                hud.distance += distance_traveled
            prev_location = current_location
 
            speed = ego_vehicle.get_velocity()
            speed_kmh = (speed.x ** 2 + speed.y ** 2 + speed.z ** 2) ** 0.5 * 3.6
            target_speed = target_vehicle.get_velocity()
          
            target_speed_kmh = (target_speed.x ** 2 + target_speed.y ** 2 + target_speed.z ** 2) ** 0.5 * 3.6
            Trgt_Px_m = ego_vehicle.get_location().distance(target_vehicle.get_location())
            hud.update_metrics(hud.a_x_for_trq, hud.Trq_en, hud.Gap_Ctr_Enable, hud.Tgt_Dist_Gap, hud.Tgt_time_Gap, speed_kmh, hud.Headway_Gap_set, hud.Time_Gap_set, Trgt_Px_m)
 
            # Check distances to all other vehicles
            desired_distance = 10
            warning_displayed = False
            for actor in actor_list:
                if actor.id != ego_vehicle.id and actor.type_id.startswith('vehicle'):
                    distance = ego_vehicle.get_location().distance(actor.get_location())
                    if distance <= (desired_distance + 5) and distance >= desired_distance:
                        if not warning_displayed:
                           hud.set_warning(f"VEHICLE {actor.id} DETECTED: Distance is less than 15 meters")
                           warning_displayed = True
                    if distance <= desired_distance and distance > (desired_distance - 5.0):
                        if not warning_displayed:
                            hud.set_warning(f"VEHICLE {actor.id} DETECTED: Maintain distance quickly")
                            warning_displayed = True
                    if distance <= 5.0:
                        if not warning_displayed:
                            hud.set_warning(f"Handbrake Applied VEHICLE {actor.id} is too close:")
                            warning_displayed = True
 
            world.tick()
            pygame.time.Clock().tick(30)
 
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print('Destroying actors...')
        destroy_actors(actor_list)
        pygame.quit()
        print('Done.')
 
if __name__ == '__main__':
    main()
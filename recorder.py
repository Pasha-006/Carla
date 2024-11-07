import pygame
from pygame_screen_recorder import Recorder
 
# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((640, 480))
pygame.display.set_caption("Pygame Recording with pygame-screen-recorder")
 
# Set up the recorder
recorder = Recorder("pygame_recording.mp4", screen)
 
# Start recording
recorder.start()
 
clock = pygame.time.Clock()
recording = True
fps = 30  # Frames per second for recording
 
while recording:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            recording = False
 
    # Example game content (change color)
    screen.fill((255, 0, 0))  # Red background for demo
    pygame.display.flip()
 
    clock.tick(fps)  # Maintain frame rate
 
# Stop the recording and save the video
recorder.stop()
 
pygame.quit()
print("Recording saved as pygame_recording.mp4")
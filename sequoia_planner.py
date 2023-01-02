import flight_planner as fp

'''
Example for calculating flight parameters for different flying heights
'''

# Height above ground level
height_agl = (60,90,120)

for h in height_agl:

    # Create a camera model object for Sequoia based on camera specifications
    sequoia = fp.Camera(f = 3.98, pixel_size =3.75e-6, image_size=(1280,960),name="sequoia")

    # Create a flight planner object with the camera object and some flight constraints
    sequoia_plan = fp.FlightPlanner(sequoia,height =h, side_overlap=85, forward_overlap=85)

    # Compute all flight parameters
    sequoia_plan.compute()

    # Write all camera and flight parameters to JSON file
    sequoia_plan.write()
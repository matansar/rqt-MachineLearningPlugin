import random
import time
import inspect, os
LOC_BOUND = 0

def Run_Scenario(scen_obj, number_scenarios):
    number_scenarios = int(number_scenarios)
    if number_scenarios > 0:
        run_once(scen_obj)

def run_once(scen_obj):
    source_x, source_y, angle = generate_params()
    run_one_scenario(scen_obj, source_x, source_y, angle)

def generate_params():
    import math
    source_x = random.uniform(-LOC_BOUND, LOC_BOUND)
    source_y = random.uniform(-LOC_BOUND, LOC_BOUND)
    angle = random.uniform(0, 0) #2 *	 math.pi)
    return source_x, source_y, angle

def run_one_scenario(scen_obj, source_x, source_y, angle):
    import Apartment_Scenario as aparts
    aparts.Run_Scenario(scen_obj, source_x, source_y, angle)
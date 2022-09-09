from solid import *
from solid.utils import *
import os
import toml
#import math sqrt
from math import sqrt

epsilon = 0.0000001

#creates two bushings, one with +bushing spacing on the sphere and the other with just the sphere
def create_bushing(bushing_diameter, height, bushing_sphere_diameter, bushing_spacing, bushing_retraction_distance,wall_thickness):
    #create the bushing, which is a cylinder with a sphere at the top
    #bushing = cylinder(d=knob_bushing_diameter,h=knob_diameter/2)
    #bushing += translate([0,0,-knob_diameter/2+knob_bushing_sphere_diameter/2])(sphere(d=knob_bushing_sphere_diameter))
    solid_bushing = cylinder(d=bushing_diameter,h=height+wall_thickness/2)
    #ensure that the bushing intersects with the clasp which is wall_thickness thick
    solid_bushing += translate([0,0,-wall_thickness/4])(solid_bushing)
    solid_bushing += translate([0,0,-height-wall_thickness/2+bushing_retraction_distance+bushing_spacing])(sphere(d=bushing_sphere_diameter))
    #create a bushing with space added
    space_bushing = cylinder(d=bushing_diameter+bushing_spacing,h=height)
    space_bushing += translate([0,0,-height-wall_thickness/2+bushing_retraction_distance+bushing_spacing])(sphere(d=bushing_sphere_diameter+bushing_spacing))

    return solid_bushing,space_bushing

def SteeringKnob(wall_thickness,wheel_diameter,knob_diameter,knob_bushing_diameter,knob_bushing_sphere_diameter,bushing_spacing,bushing_retraction_distance,snap_on_distance, zip_tie_width, zip_tie_height):
    #start coding!
    #TODO: assert that the bushing cant slip out of the knob

    steering_knob = None
    clasp = None
    #create the seering knob as a sphere with a hole in the middle
    steering_knob = sphere(d=knob_diameter)
    solid_bushing,space_bushing = create_bushing(knob_bushing_diameter,knob_diameter/2,knob_bushing_sphere_diameter,bushing_spacing, bushing_retraction_distance,wall_thickness)
    #subtract the bushing from the steering knob
    steering_knob -= space_bushing
    steering_knob += solid_bushing
    #rotate steering knob 180 degrees
    steering_knob = rotate([180,0,0])(steering_knob)

    #create a clasp that is a cylinder with a hole in it of diameter 
    clasp = cylinder(d=wheel_diameter+wall_thickness,h=knob_bushing_diameter/2+wall_thickness,center=True)
    clasp -= cylinder(d=wheel_diameter,h=knob_bushing_diameter/2+wall_thickness,center=True)
    clasp = rotate([90,0,90])(clasp)

    #remove the lower half of the clasp to create a semi-circular clasp
    upper_clasp = clasp \
    -translate([0,0,-wheel_diameter/2])\
       (cube([wheel_diameter+wall_thickness,wheel_diameter+wall_thickness,wheel_diameter],center=True)) 
    #remove the upper half of the clasp to create a semi-circular clasp
    lower_clasp = clasp \
    -translate([0,0,wheel_diameter/2])\
         (cube([wheel_diameter+wall_thickness,wheel_diameter+wall_thickness,wheel_diameter],center=True))
    #translate lower_clasp to be on the xy plane
    lower_clasp = translate([0,0,wheel_diameter/2 + wall_thickness/2])(lower_clasp)

    zip_tie_hole =cube([zip_tie_width,zip_tie_height,knob_diameter],center=True)
    #right
    upper_clasp -= translate([0,wheel_diameter/2 + wall_thickness/4,0])(zip_tie_hole)
    #left
    upper_clasp -= translate([0,-wheel_diameter/2 - wall_thickness/4,0])(zip_tie_hole)

    upper_clasp = translate([0,0,-wheel_diameter/2])(upper_clasp)

    #add the upper clasp to the solid_bushing
    steering_knob += translate([0,0,-knob_diameter/2-wall_thickness/2])(upper_clasp)
    

    zip_tie_hole_radius = wheel_diameter/2 - wall_thickness/4
    lower_clasp_snap_on_height = sqrt((wheel_diameter/2)**2 - zip_tie_hole_radius**2)
    #create a cube that is the snap on distance
    snap_on_cube = cube([zip_tie_width/2,zip_tie_height/2,lower_clasp_snap_on_height],center=True)
    snap_on_cube = translate([0,0,lower_clasp_snap_on_height/2+wheel_diameter/2+wall_thickness/4])(snap_on_cube)
    #create the snap on hinge which is a traingular-prism
    #create the left
    snap_on_hinge_left = rotate([-90,-90,90])(linear_extrude(height=zip_tie_width/2)(polygon(points=[[0,0],[zip_tie_width/2,0],[0,zip_tie_height/2]])))
    snap_on_hinge_left = translate([-zip_tie_width/4,zip_tie_height/4,0])(snap_on_hinge_left)
    #translate so its on top of snap_on_cube
    snap_on_hinge_left = translate([0,0,lower_clasp_snap_on_height+zip_tie_height/4+zip_tie_width/4])(snap_on_hinge_left)
    #translate it to center 
    snap_on_hinge_left = translate([zip_tie_width/2,0,0])(snap_on_hinge_left)
    #translate it up to snap_on_height
    snap_on_hinge_left = translate([0,0,lower_clasp_snap_on_height])(snap_on_hinge_left)
    snap_on_left = snap_on_cube + snap_on_hinge_left
    #translate to the left side of the zip tie hole so it has room to flex
    snap_on_left = translate([0,zip_tie_height/2,0])(snap_on_left)

    #create the right
    snap_on_hinge_right = rotate([-90,-90,-90])(linear_extrude(height=zip_tie_width/2)(polygon(points=[[0,0],[zip_tie_width/2,0],[0,zip_tie_height/2]])))
    snap_on_hinge_right = translate([zip_tie_width/4,-zip_tie_height/4,0])(snap_on_hinge_right)
    #translate so its on top of snap_on_cube
    snap_on_hinge_right = translate([0,0,lower_clasp_snap_on_height+zip_tie_height/4+zip_tie_width/4])(snap_on_hinge_right)
    #translate it to center
    snap_on_hinge_right = translate([-zip_tie_width/2,0,0])(snap_on_hinge_right)
    #translate it up to snap_on_height
    snap_on_hinge_right = translate([0,0,lower_clasp_snap_on_height])(snap_on_hinge_right)
    snap_on_right = snap_on_cube + snap_on_hinge_right
    #translate to the right side of the zip tie hole so it has room to flex
    snap_on_right = translate([0,-zip_tie_height/2,0])(snap_on_right)


    #add the snap on to the lower clasp on either side
    lower_clasp += translate([0,wheel_diameter/2 + wall_thickness/4,snap_on_distance/2])(snap_on_left)
    lower_clasp += translate([0,-wheel_diameter/2 - wall_thickness/4,snap_on_distance/2])(snap_on_right)

    return steering_knob,lower_clasp


def render_object(render_object, filename):
    """
    creates a .stl and .scad solution for the given solidpython OpenSCAD object
    PARAMETERS:
        render_object: the OpenSCAD object
        filename: a string for the file to be saved
    """
    scad_render_to_file(render_object, filename + ".scad", file_header="$fn=25;")
    # render with OpenSCAD
    print("Openscad is now rendering the solution..")
    os.system("openscad -o " + filename + ".stl " + filename + ".scad &")


if __name__ == "__main__":
    config = toml.load("configuration.toml")
    steering_knob, clasp = SteeringKnob(**config)
    render_object(steering_knob, "SteeringKnob")
    render_object(clasp, "Clasp")

    print("Solution saved to *.stl")

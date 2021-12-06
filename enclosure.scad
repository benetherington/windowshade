// BASE UNIT -- MM
$fa = 10;
$fs = 0.4;

use <Parametric_ball_chain_pulley/parametric_ball_pulley.scad>
use <gears/gears.scad>

NEMA8_COORDS = [
    [16, 0], [16,16],
    [ 0, 0], [ 0,16]
];
NEMA8_RADIUS = sqrt(16^2+16^2)/2;
M3_THREAD = 2.5;
M3_CLEARANCE = 3.4;
M2_THREAD = 1.6;
M2_CLEARANCE = 2.4;
M2_COUNTERSINK_HEAD_DIAMETER = 4;
M2_COUNTERSINK_HEAD_THICK = 1.2;
M2_COUNTERSINK_HEAD_ANGLE = 90;


/*-------------*\
  STEPPER MOTOR  
\*-------------*/
bodyWidth = 20;
bodyHeight = 20;
bodyLength = 33;
neckDiameter = 15;
neckThick = 1.5;
shaftRadius = 4/2;
shaftLenth = 10; // Measured from body, not neck! Protrusion is shorter.
socketWidth = 12;
socketLength = 8;
socketHeight = 2;
module Stepper() {
    // Centered on the base of the usable shaft length.
    // body
    difference() {
        translate([0, 0, neckThick+bodyLength/2])
        cube([bodyWidth, bodyHeight, bodyLength], center=true);
        // mounting holes. Approx 5mm deep? Not accurate.
        for (deg=[0:90:360]) { rotate(deg+45) {
            translate([NEMA8_RADIUS, 0, neckThick+2.4])
            cylinder(d=M2_THREAD, h=5, center=true);
        }}
    };
    // neck/collar thing
    translate([0, 0, neckThick/2])
    cylinder(d=neckDiameter, h=neckThick, center=true);
    // shaft
    translate([0, 0, -shaftLenth/2+neckThick])
    cylinder(r=shaftRadius, h=shaftLenth, center=true);
    // microblade socket
    translate([-socketWidth/2, bodyHeight/2, bodyLength+neckThick-socketLength])
    cube([socketWidth, socketHeight, socketLength]);
};


/*---*\
  HUB  
\*---*/
// hubDiameter = 19;
// hubThick = 5;
// hubMountRadius = 6.35;
// hubBore = 4;
// hubThreads = "M3";
// module PololuUniversalHub() {
//     difference() {
//         cylinder(d=hubDiameter, h=hubThick, center=true);
//         for (deg=[0:90:360]) { rotate(deg) {
//             translate([hubMountRadius, 0, 0])
//             cylinder(d=3, h=hubThick*3, center=true);
//         }}
//         cylinder(d=hubBore, h=hubThick*3, center=true);
//     }
// }
hubDiameter     = 32;
hubThick        = 8;
hubNeckDia      = 14;
hubNeckThick    = 2;
hubMountRadius  = sqrt(16^2+16^2)/2;
hubBore         = 4;
hubThreads = "M4x0.7";
module GBSonicHub() {
    difference() {
        union() {
            // body
            difference() {
                cylinder(d=hubDiameter, h=hubThick, center=true);
                for (deg=[0:90:360]) {
                    rotate(deg)
                    translate([hubMountRadius,0,-hubThick*0.55])
                    cylinder(d=4, h=hubThick*1.1);
                }
            };
            // neck
            translate([0,0,-hubThick+hubNeckThick])
            cylinder(d=hubNeckDia, h=hubNeckThick);
        }
        translate([0,0,-(hubThick-neckThick-0.1)])
        cylinder(d=hubBore*1.01, h=(hubThick+neckThick)*1.1);
    }
}

bearingThick = 7;
bearingHeight = 43;
bearingBodyWidth = 38;
bearingOD = 43;
bearingID = 32;
bearingMountDistance = 32;
bearingThreads = "M4x0.7";
module GBPillow() {
    difference() {
        union() {
            // body
            cube([bearingBodyWidth, bearingHeight, bearingThick], center=true);
            // OD
            cylinder(d=bearingOD, h=bearingThick-0.01, center=true);
        }
        // ID
        cylinder(d=bearingID, h=bearingThick*1.1, center=true);
        // Mounting threads
        rotate([90, 90, 0]){
            translate([0, bearingMountDistance/2, -bearingHeight*1.1/2])
            cylinder(d=4, h=bearingHeight*1.1);
            translate([0, -bearingMountDistance/2, -bearingHeight*1.1/2])
            cylinder(d=4, h=bearingHeight*1.1);
        }
        
    }
}

/*----*\
  CASE  
\*----*/
stepperMountThick = neckThick;
module StepperMount() {
    difference() {
        // BASE
        cube([bodyWidth, bodyHeight, stepperMountThick]);

        // NECK
        translate([bodyWidth/2, bodyHeight/2, stepperMountThick])
        cylinder(d=neckDiameter+1, h=stepperMountThick*3, center=true);

        // NEMA
        translate([bodyWidth/2, bodyHeight/2, stepperMountThick])
        for (deg=[0:90:360]) { rotate(deg+45) {
            translate([NEMA8_RADIUS, 0, -stepperMountThick+M2_COUNTERSINK_HEAD_THICK/2-0.01]) {
            cylinder(d1=M2_COUNTERSINK_HEAD_DIAMETER, d2=2,
                     h=M2_COUNTERSINK_HEAD_THICK, center=true);
            cylinder(d=M2_CLEARANCE, h=stepperMountThick*3, center=true);
            }
        }}
    }
}
module BottomCase() {
    StepperMount();
}
module TopCase() {
    
}


/*--------*\
  ASSEMBLE  
\*--------*/
stepperGap = 3-neckThick;

// rotate([90,0,0])
// translate([0,0,-neckThick-0.1])
// BottomCase();

color([0.3, 0.3, 0.3]) // dark grey
rotate([90,0,0])
Stepper();

// color([0.8, 0.8, 0.8]) // light grey
rotate([90,0,0])
translate([0,0,-(hubThick/2 + stepperGap)])
%GBSonicHub();

// color([0.85, 0.85, 0.85]) // light grey but aluminum
rotate([90,0,0])
translate([0,0,-(hubThick/2 + stepperGap)])
%GBPillow();

gearWidth = 4;
// rotate([90,0,0])
// translate([ bodyWidth/2,
//             bodyWidth/2,
//             -(gearWidth + stepperGap)])
// spur_gear(  modul=2,
//             tooth_number=20,
//             width=gearWidth,
//             bore=4,
//             pressure_angle=20);

// rotate([90,0,180])
// translate([ -bodyWidth/2,
//             bodyWidth/2,
//             getPulleyHeight() + stepperGap ])
// pulley();



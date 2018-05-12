$fn=50;

wall=0.8;
2wall=2*wall;
bw = 26;
bl = 48.5;
bh = 23;
br = 2.5;

*box();
*lid();
assembly();


module assembly()
{
	box();
	translate([0, 0, bh+10])
	rotate([180, 0, 0])
		lid();
}

module lid()
{
    linear_extrude(1)
    difference() {
        roundedSq(bw+2wall, bl+2wall, br);
        for(i=[0,1],j=[0,1])
        mirror([i, 0, 0])
        mirror([0, j, 0])
        translate([bw/2-br, bl/2-br, 0])
            circle(d=3);
    }
    linear_extrude(3)
    difference() {
        roundedSq(bw, bl, br);
        roundedSq(bw-2wall, bl-2wall, br);
    }
    linear_extrude(3)
    for(i=[0,1],j=[0,1])
    mirror([i, 0, 0])
    mirror([0, j, 0])
    translate([bw/2-br, bl/2-br, 0])
    difference() {
        circle(r=br);
        circle(d=3);
    }
}

module box()
{
    linear_extrude(1)
        roundedSq(bw+2wall, bl+2wall, br);
    translate([0, 0, 1])
    linear_extrude(bh/2)
    difference() {
        roundedSq(bw+2wall, bl+2wall, br);
        roundedSq(bw, bl, br);
        // Temp sensor exit
        translate([0, bl/2+2wall/2, 0])
            square([13.5, 2*2wall], center=true);
    }
    translate([0, 0, 1+bh/2])
    linear_extrude(bh/2)
    difference() {
        roundedSq(bw+2wall, bl+2wall, br);
        roundedSq(bw, bl, br);
        // Usb
        translate([0, -bl/2+2wall/2, 0])
            square([8, 2*2wall], center=true);
    }
    linear_extrude(bh-5)
    for(i=[0,1],j=[0,1])
    mirror([i, 0, 0])
    mirror([0, j, 0])
    translate([bw/2-br, bl/2-br, 0])
    difference() {
        circle(r=br);
        circle(d=1.5);
    }
    
    translate([0, bl/2+16/2, 0])
    linear_extrude(2)
    difference() {
        square([13.5, 16], center=true);
        *translate([0, 5, 0])
            circle(d=3);
    }
}


module roundedSq(w, l, r)
{
    hull()
    for(i=[0,1],j=[0,1])
    mirror([i, 0, 0])
    mirror([0, j, 0])
    translate([w/2-r, l/2-r, 0])
        circle(r=r);
}
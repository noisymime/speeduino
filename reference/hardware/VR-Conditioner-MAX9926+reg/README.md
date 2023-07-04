I hope this helps anyone interested.
Anyway, I've done what I can & it's now ready for many eye-balls.

Phill


### 2021-02-23
I have not made & tested this design but it is barely changed from the manufacrurers data sheets or other designs.
If you make & test it then please advise.

I'm new to KiCAD so I don't know how to do some things.  Such as how to have an 8-pin set of holes without a 3D chip sitting in it.
Or how to do an 8-pin header in the shape & pin count order of a chip.

### 2021-02-24
Changed version number to 3.5 so that it reflects relative cronology from previous design.
Renamed this from Notes.txt to README.md & corrected the date above.
Added Comments to schematic, inc. single-sided design allowing 'Letter' postage!
Updated BOM in schematic to reflect readily available parts found on OctoPart.

### 2021-04-22
Goal: give the left 6 passive components some space from the edge.
Re-route COUT1 from DIP.8 to SIL.4 in green so no signal traces are by the edge.
Shift right 5 components slightly right
Shift DIP, chip & hidden cap slightly right
Shift left 6 components slightly right
Re-route left 6 components to avoid side swapping, keeping the larger SMTs.
Ensure GNG plane encircles everything with a via in each corner.
Ensure all 'thin' traces are 0.20mm (up from 0.16)
Tidy with some symetry.

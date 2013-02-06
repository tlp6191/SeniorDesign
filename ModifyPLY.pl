#!/usr/bin/perl

#C:\Users\Taylor\Dropbox\programs\SeniorDesign>"C:\Program Files (x86)\VCG\MeshLab\meshlabserver.exe" -i ketchup2.ply -o meshlabtestout.ply -s DelaunayTriangulation.mlx
#print "HELLO WORLD!";
srand(0);
$rand=0;
$keeppercent=1;
while(@ARGV){
	$x=shift @ARGV;
	#Disabled for now:
	#if ($x=~m/-r/){	$rand=1; $randmax=shift @ARGV;} #What's the maximum percentage to shift a value.
	if ($x=~m/-p/){ $keeppercent=shift @ARGV; }
	if ($x=~m/-f/){ $file=shift @ARGV;  $fmode=1;}
	if ($x=~m/-s/){ $stripFaces=1;}
	
	#print "$_\n";
}
if($fmode!=1){ print "Please enter a file with the -f option\n";exit;}
open FILE, $file or die $!;
@plyfile=<FILE>;
@face=grep(/element face/, @plyfile);
#print @face;
$face[0]=~m/element face (\d+)/;
#print "\nThere are $1 faces";
$faceNumber=$1;
$plyLines=@plyfile;
if($stripFaces==1){
	@plyfile=@plyfile[0..$plyLines-$faceNumber-1];
	
	for (@plyfile) 
	{
	s/element face $faceNumber/element face 0/;
	}
}
@plyHeader;
FOO:while(@plyfile){
$x=shift @plyfile;
#$x=chomp $x;
	@plyHeader=(@plyHeader, $x); 
	if($x=~m/end_header/){
		last FOO;
	}
}



@plyPoints;
while(@plyfile){
$x=shift @plyfile;
#$x=chomp $x;
if(rand()<$keeppercent){
	@plyPoints=(@plyPoints, $x);
	#print "$x";
}
}
$vertices=@plyPoints;
for (@plyHeader) 
{
	s/element vertex \d+/element vertex $vertices/;
}
print @plyHeader;
print @plyPoints;
#print @plyfile;

#print @ARGV;
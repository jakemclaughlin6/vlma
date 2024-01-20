
run_alignment_sc_geo() 
{  
    ~/catkin_ws/devel/lib/vlma/vlma_main --map1_config_file $1 --map2_config_file $2 --offset_map2 true
}

run_alignment_sc() 
{ 
    ~/catkin_ws/devel/lib/vlma/vlma_main --map1_config_file $1 --map2_config_file $2 --phi 0.0 --offset_map2 true
}

run_alignment_geo() 
{ 
    ~/catkin_ws/devel/lib/vlma/vlma_main --map1_config_file $1 --map2_config_file $2 --psi 1000.0 --offset_map2 true
} 

run_alignment_none() 
{ 
    ~/catkin_ws/devel/lib/vlma/vlma_main --map1_config_file $1 --map2_config_file $2 --psi 1000.0 --phi 0.0 --offset_map2 true
}

SL1='/home/jake/projects/beam_robotics/mapping/vlma/config/StructuresLab/map1.json'
SL2='/home/jake/projects/beam_robotics/mapping/vlma/config/StructuresLab/map2.json'
SL3='/home/jake/projects/beam_robotics/mapping/vlma/config/StructuresLab/map3.json'
SL4='/home/jake/projects/beam_robotics/mapping/vlma/config/StructuresLab/map4.json'
SL5='/home/jake/projects/beam_robotics/mapping/vlma/config/StructuresLab/map5.json'

CB1='/home/jake/projects/beam_robotics/mapping/vlma/config/ConestogoBridge/map1.json'
CB2='/home/jake/projects/beam_robotics/mapping/vlma/config/ConestogoBridge/map2.json'
CB3='/home/jake/projects/beam_robotics/mapping/vlma/config/ConestogoBridge/map3.json'
CB4='/home/jake/projects/beam_robotics/mapping/vlma/config/ConestogoBridge/map4.json'


# echo "Running SC+Geo experiments: "
# echo "$SL1 and $SL2"
# run_alignment_geo $SL1 $SL2 >> /home/jake/results/vlma/SL1SL2_scgeo.txt

# echo "$SL1 and $SL3"
# run_alignment_geo $SL1 $SL3 >> /home/jake/results/vlma/SL1SL3_scgeo.txt

# echo "$SL1 and $SL4"
# run_alignment_geo $SL1 $SL4 >> /home/jake/results/vlma/SL1SL4_scgeo.txt

# echo "$SL2 and $SL3"
# run_alignment_geo $SL2 $SL3 >> /home/jake/results/vlma/SL2SL3_scgeo.txt

# echo "$SL2 and $SL4"
# run_alignment_geo $SL2 $SL4 >> /home/jake/results/vlma/SL2SL4_scgeo.txt

# echo "$SL3 and $SL4"
# run_alignment_geo $SL3 $SL4 >> /home/jake/results/vlma/SL3SL4_scgeo.txt


echo "Running SC+Geo experiments: "
echo "$CB1 and $CB2"
run_alignment_geo $CB1 $CB2 >> /home/jake/results/vlma/CB1CB2_scgeo.txt

# echo "$CB1 and $CB3"
# run_alignment_geo $CB1 $CB3 >> /home/jake/results/vlma/CB1CB3_scgeo.txt

# echo "$CB2 and $CB3"
# run_alignment_geo $CB2 $CB3 >> /home/jake/results/vlma/CB2CB3_scgeo.txt


# echo "Running Geo experiments: "
# echo "$SL1 and $SL2"
# run_alignment_geo $SL1 $SL2 >> /home/jake/results/vlma/SL1SL2_geo.txt

# echo "$SL1 and $SL3"
# run_alignment_geo $SL1 $SL3 >> /home/jake/results/vlma/SL1SL3_geo.txt

# echo "$SL1 and $SL4"
# run_alignment_geo $SL1 $SL4 >> /home/jake/results/vlma/SL1SL4_geo.txt

# echo "$SL2 and $SL3"
# run_alignment_geo $SL2 $SL3 >> /home/jake/results/vlma/SL2SL3_geo.txt

# echo "$SL2 and $SL4"
# run_alignment_geo $SL2 $SL4 >> /home/jake/results/vlma/SL2SL4_geo.txt

# echo "$SL3 and $SL4"
# run_alignment_geo $SL3 $SL4 >> /home/jake/results/vlma/SL3SL4_geo.txt



# echo "Running None experiments: "
# echo "$SL1 and $SL2"
# run_alignment_none $SL1 $SL2 >> /home/jake/results/vlma/SL1SL2_none.txt

# echo "$SL1 and $SL3"
# run_alignment_none $SL1 $SL3 >> /home/jake/results/vlma/SL1SL3_none.txt

# echo "$SL1 and $SL4"
# run_alignment_none $SL1 $SL4 >> /home/jake/results/vlma/SL1SL4_none.txt

# echo "$SL2 and $SL3"
# run_alignment_none $SL2 $SL3 >> /home/jake/results/vlma/SL2SL3_none.txt

# echo "$SL2 and $SL4"
# run_alignment_none $SL2 $SL4 >> /home/jake/results/vlma/SL2SL4_none.txt

# echo "$SL3 and $SL4"
# run_alignment_none $SL3 $SL4 >> /home/jake/results/vlma/SL3SL4_none.txt



# echo "Running SC experiments: "
# echo "$SL1 and $SL2"
# run_alignment_sc $SL1 $SL2 >> /home/jake/results/vlma/SL1SL2_sc.txt

# echo "$SL1 and $SL3"
# run_alignment_sc $SL1 $SL3 >> /home/jake/results/vlma/SL1SL3_sc.txt

# echo "$SL1 and $SL4"
# run_alignment_sc $SL1 $SL4 >> /home/jake/results/vlma/SL1SL4_sc.txt

# echo "$SL2 and $SL3"
# run_alignment_sc $SL2 $SL3 >> /home/jake/results/vlma/SL2SL3_sc.txt

# echo "$SL2 and $SL4"
# run_alignment_sc $SL2 $SL4 >> /home/jake/results/vlma/SL2SL4_sc.txt

# echo "$SL3 and $SL4"
# run_alignment_sc $SL3 $SL4 >> /home/jake/results/vlma/SL3SL4_sc.txt
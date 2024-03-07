
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

# SL1='/home/jake/projects/beam_robotics/mapping/vlma/config/StructuresLab/map1.json'
SL1='/home/jake/projects/beam_robotics/mapping/vlma/config/StructuresLab/map2.json'
SL2='/home/jake/projects/beam_robotics/mapping/vlma/config/StructuresLab/map3.json' # align to this one
SL3='/home/jake/projects/beam_robotics/mapping/vlma/config/StructuresLab/map4.json'
# SL5='/home/jake/projects/beam_robotics/mapping/vlma/config/StructuresLab/map5.json'

# CB1='/home/jake/projects/beam_robotics/mapping/vlma/config/ConestogoBridge/map1.json'
CB1='/home/jake/projects/beam_robotics/mapping/vlma/config/ConestogoBridge/map2.json' # align to this one
CB2='/home/jake/projects/beam_robotics/mapping/vlma/config/ConestogoBridge/map3.json'
CB3='/home/jake/projects/beam_robotics/mapping/vlma/config/ConestogoBridge/map4.json'


# echo "Running SC+Geo experiments: "
# echo "$SL2 and $SL3"
# run_alignment_sc_geo $SL1 $SL2 >> /home/jake/results/vlma/SL2SL3_scgeo.txt

# echo "$SL2 and $SL1"
# run_alignment_sc_geo $SL2 $SL1 >> /home/jake/results/vlma/SL2SL1_scgeo.txt

# echo "Running Geo experiments: "
# echo "$SL2 and $SL3"
# run_alignment_geo $SL1 $SL2 >> /home/jake/results/vlma/SL2SL3_geo.txt

# echo "$SL2 and $SL1"
# run_alignment_geo $SL2 $SL1 >> /home/jake/results/vlma/SL2SL1_geo.txt

# echo "Running SC experiments: "
# echo "$SL2 and $SL3"
# run_alignment_sc $SL1 $SL2 >> /home/jake/results/vlma/SL2SL3_sc.txt

# echo "$SL2 and $SL1"
# run_alignment_sc $SL2 $SL1 >> /home/jake/results/vlma/SL2SL1_sc.txt

echo "Running None experiments: "
echo "$SL2 and $SL3"
run_alignment_none $SL1 $SL2 >> /home/jake/results/vlma/SL2SL3_none.txt

# echo "$SL2 and $SL1"
# run_alignment_none $SL2 $SL1 >> /home/jake/results/vlma/SL2SL1_none.txt


# echo "Running SC+Geo experiments : "
# echo "$CB1 and $CB2"
# run_alignment_sc_geo $CB1 $CB2  >> /home/jake/results/vlma/CB1CB2_scgeo.txt

# echo "$CB1 and $CB3"
# run_alignment_sc_geo $CB1 $CB3 >> /home/jake/results/vlma/CB1CB3_scgeo.txt

# echo "Running Geo experiments : "
# echo "$CB1 and $CB2"
# run_alignment_geo $CB1 $CB2>> /home/jake/results/vlma/CB1CB2_geo.txt

# echo "$CB1 and $CB3"
# run_alignment_geo $CB1 $CB3 >> /home/jake/results/vlma/CB1CB3_geo.txt

# echo "Running SC experiments : "
# echo "$CB1 and $CB2"
# run_alignment_sc $CB1 $CB2  >> /home/jake/results/vlma/CB1CB2_sc.txt

# echo "$CB1 and $CB3"
# run_alignment_sc $CB1 $CB3  >> /home/jake/results/vlma/CB1CB3_sc.txt

# echo "Running None experiments  "
# echo "$CB1 and $CB2"
# run_alignment_none $CB1 $CB2  >> /home/jake/results/vlma/CB1CB2_none.txt

# echo "$CB1 and $CB3"
# run_alignment_none $CB1 $CB3  >> /home/jake/results/vlma/CB1CB3_none.txt
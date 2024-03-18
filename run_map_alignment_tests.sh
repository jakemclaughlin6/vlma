
run_alignment_sc_geo() 
{  
    ~/catkin_ws/devel/lib/vlma/vlma_main --map1_config_file $1 --map2_config_file $2 --offset_map2=true
}

run_alignment_sc() 
{ 
    ~/catkin_ws/devel/lib/vlma/vlma_main --map1_config_file $1 --map2_config_file $2 --phi 0.0 --offset_map2=true
}

run_alignment_geo() 
{ 
    ~/catkin_ws/devel/lib/vlma/vlma_main --map1_config_file $1 --map2_config_file $2 --psi 1000.0 --offset_map2=true
} 

run_alignment_none() 
{ 
    ~/catkin_ws/devel/lib/vlma/vlma_main --map1_config_file $1 --map2_config_file $2 --psi 1000.0 --phi 0.0 --offset_map2=true
}

run_alignment_vma() 
{ 
    ~/catkin_ws/devel/lib/vlma/vlma_main --map1_config_file $1 --map2_config_file $2 --filter_path_outliers=false --psi 1000.0 --phi 0.5 --offset_map2=true --use_lpr=false --use_sac=false --use_nonrigid=false
}

run_alignment_lma() 
{ 
    ~/catkin_ws/devel/lib/vlma/vlma_main --map1_config_file $1 --map2_config_file $2 --psi 0.4 --filter_path_outliers=false --offset_map2=true --use_lpr=true --use_sac=false --use_nonrigid=false
}


run_alignment_rigid() 
{  
    ~/catkin_ws/devel/lib/vlma/vlma_main --map1_config_file $1 --map2_config_file $2 --offset_map2=true --use_nonrigid=false --filter_path_outliers=false
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


# run_alignment_rigid $SL2 $SL1 >> /home/jake/results/vlma/SL2SL1_rigid.txt
# run_alignment_rigid $SL2 $SL3 >> /home/jake/results/vlma/SL2SL3_rigid.txt

# run_alignment_rigid $CB1 $CB2 >> /home/jake/results/vlma/CB1CB2_rigid.txt
run_alignment_sc_geo $CB1 $CB3 >> /home/jake/results/vlma/CB1CB3.txt


# run_alignment_vma $SL2 $SL1 >> /home/jake/results/vlma/SL2SL1_vma.txt
# run_alignment_vma $SL2 $SL3 >> /home/jake/results/vlma/SL2SL3_vma.txt
# run_alignment_lma $SL2 $SL1 >> /home/jake/results/vlma/SL2SL1_lma.txt
# run_alignment_lma $SL2 $SL3 >> /home/jake/results/vlma/SL2SL3_lma.txt

# run_alignment_vma $CB1 $CB2 >> /home/jake/results/vlma/CB1CB2_vma.txt
# run_alignment_vma $CB1 $CB3 >> /home/jake/results/vlma/CB1CB3_vma.txt
# run_alignment_lma $CB1 $CB2 >> /home/jake/results/vlma/CB1CB2_lma.txt
# run_alignment_lma $CB1 $CB3 >> /home/jake/results/vlma/CB1CB3_lma.txt

# echo "Running SC+Geo experiments: "
# echo "$SL2 and $SL3"
# run_alignment_sc_geo $SL2 $SL3 >> /home/jake/results/vlma/SL2SL3_scgeo.txt

# echo "$SL2 and $SL1"
# run_alignment_sc_geo $SL2 $SL1 >> /home/jake/results/vlma/SL2SL1_scgeo.txt

# echo "Running Geo experiments: "
# echo "$SL2 and $SL3"
# run_alignment_geo $SL2 $SL3 >> /home/jake/results/vlma/SL2SL3_geo.txt

# echo "$SL2 and $SL1"
# run_alignment_geo $SL2 $SL1 >> /home/jake/results/vlma/SL2SL1_geo.txt

# echo "Running SC experiments: "
# echo "$SL2 and $SL3"
# run_alignment_sc $SL2 $SL3 >> /home/jake/results/vlma/SL2SL3_sc.txt

# echo "$SL2 and $SL1"
# run_alignment_sc $SL2 $SL1 >> /home/jake/results/vlma/SL2SL1_sc.txt

# echo "Running None experiments: "
# echo "$SL2 and $SL3"
# run_alignment_none $SL2 $SL3 >> /home/jake/results/vlma/SL2SL3_none.txt

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